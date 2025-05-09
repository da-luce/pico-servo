#include "servo.h"
#include <stdio.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "pico/sync.h"

#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

// === the fixed point macros ========================================
// Fixed point to represent radian angles in [0, 2pi]
// 1 sign, 3 decimal bits (up to 6), 28 fractional bits
// TODO: make code that generates this...
typedef signed int fix28 ;

#define fix15_to_float(a) ((float)(a)/32768.0)
#define float_to_fix15(a) ((fix15)((a)*32768.0)) 

#define fix15_to_int(a) ((int)(a >> 15))
#define int_to_fix15(a) ((fix15)(a << 15))

#define char_to_fix15(a) (fix15)(((fix15)(a)) << 15)

#define abs_fix15(a) abs(a) 
#define mult_fix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define div_fix15(a,b) (fix15)( (((signed long long)(a)) << 15) / (b))

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// A clock div that allows for overclocking with PWM
#define CLKDIV 100.0f

#define MICRO 1e6 // Number of microsecs in a sec

static Servo* registered_servos[NUM_PWM_SLICES];

/* WARNING: this may not work for very high frequency clocks with low frequency servos
 */
void servo_init(Servo* servo)
{
    // Tell GPIO it is allocated to the PWM
    gpio_set_function(servo->gpio, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO
    unsigned int slice_num = pwm_gpio_to_slice_num(servo->gpio);
    unsigned int channel_num = pwm_gpio_to_channel(servo->gpio);

    servo->slice_num = slice_num;
    servo->channel_num = channel_num;

    // Set speed to somethign slow if unset (on the safe side)
    servo->sec_per_60 = 0.5f;

    // Given CLKDIV, figure out how many cycles are required to achieve servo.period_usec
    pwm_set_clkdiv(slice_num, CLKDIV) ;
    unsigned int wrap_val = (servo->clock_freq / CLKDIV) * servo->period_usec / MICRO;
    // Max unit16 is 65535, so we need a high clockdiv to allow overclocking
    pwm_set_wrap(slice_num, wrap_val);

    mutex_init(&servo->mutex);

    // Set the duty cycle to the starting angle
    servo_set_deg(servo, servo->start_angle_deg);

    // Track this servo
    registered_servos[servo->slice_num] = servo;

    // Start the pwm
    pwm_set_enabled(servo->slice_num, true);

    servo->current_angle = servo->start_angle_deg;
}

void set_rad(Servo* servo, float angle_rad)
{
    if (angle_rad < 0.0f)
    {
        angle_rad = 0.0f;
    } else if (angle_rad > M_PI)
    {
        angle_rad = M_PI;
    }

    // Set the duty cycle to achieve the angle
    // Map angle to duty cycle
    unsigned int duty_usec = angle_rad / M_PI * (servo->duty_max_usec - servo->duty_min_usec) + servo->duty_min_usec;
    unsigned int duty_val = (servo->clock_freq / CLKDIV) * duty_usec / MICRO;
    pwm_set_chan_level(servo->slice_num, servo->channel_num, duty_val); 

    servo->current_angle = angle_rad * 180.0 / M_PI;
}

void set_deg(Servo* servo, float angle_deg)
{
    float angle_rad = angle_deg * M_PI / 180.0f;
    set_rad(servo, angle_rad);
}

void servo_set_rad(Servo* servo, float angle_rad)
{
    // Wait until we can use the servo
    mutex_enter_blocking(&servo->mutex);

    set_rad(servo, angle_rad);

    // Release the lock
    mutex_exit(&servo->mutex);
}

void servo_set_deg(Servo* servo, float angle_deg)
{
    float angle_rad = angle_deg * M_PI / 180.0f;
    servo_set_rad(servo, angle_rad);
}

int time_ms(float distance, float sec_per_60)
{
    return (int) (distance / 60.0f * sec_per_60 * 1000);
}

void servo_set_rad_wait(Servo* servo, float angle_rad)
{
    float angle_deg = angle_rad * 180.0f / M_PI;
    int wait_ms = time_ms(fabsf(angle_deg - servo->current_angle), servo->sec_per_60);
    servo_set_rad(servo, angle_rad);
    sleep_ms(wait_ms);
}

void servo_set_deg_wait(Servo* servo, float angle_deg)
{
    float angle_rad = angle_deg * M_PI / 180.0f;
    servo_set_rad_wait(servo, angle_rad);
}

float ease_sin(float x)
{
    return sinf(x * M_PI / 2.0f);
}

float ease_in_quad(float x) {
    return x * x;
}

float ease_out_quad(float x) {
    return 1.0f - (1.0f - x) * (1.0f - x);
}

float ease_lin(float x)
{
    return x;
}

float ease_out_expo(float x) {
    return (x >= 1.0f) ? 1.0f : 1 - powf(2.0f, -10.0f * x);
}

float ease_in_expo(float x) {
    return (x <= 0.0f) ? 0.0f : powf(2.0f, 10.0f * (x - 1.0f));
}

float ease_in_out_sigmoid(float x) {
    // Sigmoid centered at 0.5, scaled to fit [0,1]
    float steepness = 10.0f; // Higher = steeper mid-curve
    float s = 1.0f / (1.0f + expf(-steepness * (x - 0.5f)));
    float min = 1.0f / (1.0f + expf(steepness / 2.0f));
    float max = 1.0f / (1.0f + expf(-steepness / 2.0f));
    return (s - min) / (max - min); // Normalize to [0,1]
}

void on_pwm_wrap() {

    // Find which PWM slice has fired
    int pwm = 0;
    for (int p = 0; p < NUM_PWM_SLICES; p++) {
        if (pwm_get_irq_status_mask() & (1 << p)) {
            pwm = p;
            break;
        }
    }

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm);

    // Get the servo associated with this PWM
    Servo* servo = registered_servos[pwm];

    // Determine how far we are into the motion
    servo->motion.current_time_us += servo->period_usec;
    if (servo->motion.current_time_us > servo->motion.duration_us)
    {
        // We have reached the end of the motion, no more interrupts
        pwm_set_irq_enabled(servo->slice_num, false);

        // Update the current angle
        servo->current_angle = servo->motion.end_deg;

        // Release the lock and exit
        mutex_exit(&servo->mutex);
        return;
    }

    // Otherwise, set the next angle
    float t = (float) servo->motion.current_time_us /  (float) servo->motion.duration_us;
    float angle = servo->motion.start_deg + servo->motion.ease_fn(t) * (servo->motion.end_deg - servo->motion.start_deg);
    set_deg(servo, angle);
}

float ease_out_wobble_pop(float x) {
    // Combines smoothstep curve with a sine wobble and overshoot
    float overshoot = 1.05f;
    float smooth = x * x * (3.0f - 2.0f * x);  // Smoothstep
    float wobble = sinf(8.0f * M_PI * x) * (1.0f - x) * 0.1f;  // Diminishing wobble
    return smooth * overshoot + wobble;
}

float ease_out_bounce(float x) {
    if (x < 1.0f / 2.75f) {
        return 7.5625f * x * x;
    } else if (x < 2.0f / 2.75f) {
        x -= 1.5f / 2.75f;
        return 7.5625f * x * x + 0.75f;
    } else if (x < 2.5f / 2.75f) {
        x -= 2.25f / 2.75f;
        return 7.5625f * x * x + 0.9375f;
    } else {
        x -= 2.625f / 2.75f;
        return 7.5625f * x * x + 0.984375f;
    }
}

float ease_in_bounce(float x) {
    return 1.0f - ease_out_bounce(1.0f - x);
}

void servo_set_deg_ease(Servo* servo, float angle_deg, unsigned int duration_us, float (*ease_fn)(float))
{
    // Wait until we can use the servo
    mutex_enter_blocking(&servo->mutex);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(servo->slice_num);
    pwm_set_irq_enabled(servo->slice_num, true);
    irq_set_exclusive_handler(PWM_DEFAULT_IRQ_NUM(), on_pwm_wrap);
    irq_set_enabled(PWM_DEFAULT_IRQ_NUM(), true);

    // Set the motion
    servo->motion.current_time_us = 0u;
    servo->motion.duration_us = duration_us;
    servo->motion.start_deg = servo->current_angle;
    servo->motion.end_deg = angle_deg;
    servo->motion.ease_fn = ease_fn;
}

void servo_set_deg_ease_wait(Servo* servo, float angle_deg, unsigned int duration_us, float (*ease_fn)(float))
{
    servo_set_deg_ease(servo, angle_deg, duration_us, ease_fn);

    // Sleep for duration
    sleep_ms(duration_us / 10000);

    // Since interrupts introduce additional overhead (~20 µs on the RP2350), spin on the mutex
    // to ensure the motion fully completes. This prevents premature exit in cases where:
    // (1) multiple ease_wait() calls are made in sequence, and
    // (2) there is no other code running afterward to absorb the timing error.
    // NOTE: I'm not 100% sure what is going on here, or if it is a Pico bug.
    mutex_enter_blocking(&servo->mutex);
    mutex_exit(&servo->mutex);
}