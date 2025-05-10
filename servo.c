#include "servo.h"

#include <stdio.h>
#include <math.h>
#include <assert.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "pico/sync.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef TO_RAD
#define TO_RAD(degrees) ((degrees) * M_PI / 180.0f)
#endif

#ifndef TO_DEG
#define TO_DEG(radians) ((radians) * 180.0f / M_PI )
#endif

#define CLKDIV          100.0f      // A clock div that allows for overclocking with PWM
#define MICRO_PER_SEC   1000000u    // Number of microsecs in a sec

static uint32_t sys_hz;
static Servo* registered_servos[NUM_PWM_SLICES];


/* Simple movement **********************************************************/


/* Private. Uprotected: sets the servo angle in radians without checking the
 * mutex.
 */
void set_rad(Servo* servo, float target_rad)
{
    if (target_rad < 0.0f)
    {
        target_rad = 0.0f;
    } else if (target_rad > servo->max_rad)
    {
        target_rad = servo->max_rad;
    }

    // Set the duty cycle to achieve the angle
    // Map angle to duty cycle
    unsigned int duty_usec = target_rad / servo->max_rad * (servo->duty_max_usec - servo->duty_min_usec) + servo->duty_min_usec;
    unsigned int duty_val = (sys_hz / CLKDIV) * duty_usec / MICRO_PER_SEC;
    pwm_set_chan_level(servo->slice_num, servo->channel_num, duty_val); 

    servo->current_angle = TO_DEG(target_rad);
}

/* Private. Uprotected: sets the servo angle in degrees without checking the
 * mutex.
 */
void set_deg(Servo* servo, float target_deg)
{
    set_rad(servo, TO_RAD(target_deg));
}

void servo_set_rad(Servo* servo, float target_rad)
{
    // Wait until we can use the servo
    mutex_enter_blocking(&servo->mutex);

    set_rad(servo, target_rad);

    // Release the lock
    mutex_exit(&servo->mutex);
}

void servo_set_deg(Servo* servo, float target_deg)
{
    servo_set_rad(servo, TO_RAD(target_deg));
}

int time_to_move_ms(float distance_deg, float sec_per_60_deg)
{
    return (int) (distance_deg / 60.0f * sec_per_60_deg * 1000);
}

void servo_set_rad_wait(Servo* servo, float target_rad)
{
    float target_deg = TO_DEG(target_rad);
    int wait_ms = time_to_move_ms(fabsf(target_deg - servo->current_angle), servo->sec_per_60);
    servo_set_rad(servo, target_rad);
    sleep_ms(wait_ms);
}

void servo_set_deg_wait(Servo* servo, float target_deg)
{
    servo_set_rad_wait(servo, TO_RAD(target_deg));
}


/* Easing functions ***********************************************************/


void handle_servo_irq_for_slice(int slice)
{
    // TODO: this could be made faster (fixed point arithmetic)

    // Get the servo associated with this slice
    Servo* servo = registered_servos[slice];

    // Determine how far we are into the motion
    servo->motion.current_time_us += servo->period_usec;
    if (servo->motion.current_time_us > servo->motion.duration_us)
    {
        // We have reached the end of the motion, no more interrupts
        pwm_set_irq_enabled(servo->slice_num, false);

        // Update the current angle
        servo->current_angle = servo->motion.end_deg;

        // Release the lock and exit
        // Mutex API is "non-IRQ" but supposodly, it's ok to release the lock in
        // an IRQ.
        // https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#group_mutex
        mutex_exit(&servo->mutex);
        return;
    }

    // Otherwise, set the next angle
    float t = (float) servo->motion.current_time_us /  (float) servo->motion.duration_us;
    float angle = servo->motion.start_deg + servo->motion.ease_fn(t) * (servo->motion.end_deg - servo->motion.start_deg);

    // IMPORTANT: Make sure to use the unprotected function call, otherwise we
    // spin here forever becuase we already have the mutex!
    set_deg(servo, angle);
}

void servo_on_pwm_wrap() {

    // Get the IRQ status mask
    uint32_t irq_status = pwm_get_irq_status_mask();

    // Check each slice and handle if its IRQ is set
    for (int slice = 0; slice < NUM_PWM_SLICES; ++slice) {

        // Don't do anything with PWM slices we aren't using
        bool slice_set = irq_status & (1u << slice);
        bool servo_using = registered_servos[slice] != NULL;
        if (slice_set && servo_using) {

            // Clear the interrupt flag that brought us here
            pwm_clear_irq(slice);

            // Handle this particular slice
            handle_servo_irq_for_slice(slice);
        }
    }
}

void servo_time_to_rad(Servo* servo, float target_rad, unsigned int duration_us, float (*ease_fn)(float))
{
    // Wait until we can use the servo
    mutex_enter_blocking(&servo->mutex);

    // Default to linear motion
    if (ease_fn == NULL) {
        ease_fn = ease_lin;
    }

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(servo->slice_num);
    pwm_set_irq_enabled(servo->slice_num, true);

    // Set the motion
    servo->motion.current_time_us = 0u;
    servo->motion.duration_us = duration_us;
    servo->motion.start_deg = servo->current_angle;
    servo->motion.end_deg = TO_DEG(target_rad);
    servo->motion.ease_fn = ease_fn;

    return;

    // The mutex will be freed by the interrupt handler once the motion has ended
}

void servo_time_to_deg(Servo* servo, float target_deg, unsigned int duration_us, float (*ease_fn)(float))
{
    servo_time_to_rad(servo, TO_RAD(target_deg), duration_us, ease_fn);
}

void servo_time_to_rad_wait(Servo* servo, float target_rad, unsigned int duration_us, float (*ease_fn)(float))
{
    servo_time_to_rad(servo, target_rad, duration_us, ease_fn);

    // Sleep for duration
    sleep_us(duration_us);

    // Since interrupts introduce additional overhead (~20 µs on the RP2350), spin on the mutex
    // to ensure the motion fully completes. I.e., if main exits, no further interrupts will
    // be handled. This is only a problem if this is the last code to run in the program. Even if not,
    // we want to avoid timing issues between mutiple sequential ease_wait calls
    mutex_enter_blocking(&servo->mutex);
    mutex_exit(&servo->mutex);
}
void servo_time_to_deg_wait(Servo* servo, float target_deg, unsigned int duration_us, float (*ease_fn)(float))
{
    servo_time_to_rad_wait(servo, TO_RAD(target_deg), duration_us, ease_fn);
}

void servo_speed_to_deg(Servo* servo, float target_deg, float deg_per_sec)
{
    // TODO: no mutex needed here?
    float angle_delta = fabsf(target_deg - servo->current_angle);
    unsigned int time_us = (unsigned int) (angle_delta / deg_per_sec) * MICRO_PER_SEC;

    servo_time_to_deg(servo, target_deg, time_us, ease_lin);
}


/* Init ***********************************************************************/


void servo_init(Servo* servo)
{
    // Tell GPIO it is allocated to the PWM
    gpio_set_function(servo->gpio, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO
    unsigned int slice_num = pwm_gpio_to_slice_num(servo->gpio);
    unsigned int channel_num = pwm_gpio_to_channel(servo->gpio);

    servo->slice_num = slice_num;
    servo->channel_num = channel_num;

    // Track this servo globally
    assert(registered_servos[servo->slice_num] == NULL);
    registered_servos[servo->slice_num] = servo;

    // Set defaults for unset fields
    if (!servo->sec_per_60)
    {
        // Stay on the safe side and use somethign slow if unset
        servo->sec_per_60 = 0.5f;
    }
    if (!servo->max_degrees)
    {
        servo->max_degrees = 180.0f;
    }
    servo->max_rad = TO_RAD(servo->max_degrees);
    assert(servo->max_degrees > 0.0f);
    assert(servo->max_degrees <= 360.0f);

    // Determine the wrap value of the PWM

    sys_hz = clock_get_hz(clk_sys);
    // Given CLKDIV, figure out how many cycles are required to achieve servo.period_usec
    pwm_set_clkdiv(slice_num, CLKDIV) ;
    unsigned int wrap_val = (sys_hz / CLKDIV) * servo->period_usec / MICRO_PER_SEC;
    // Max unit16 is 65535, so we need a high clockdiv to work with overclocking
    pwm_set_wrap(slice_num, wrap_val);

    // Initialize the servo mutex
    mutex_init(&servo->mutex);

    // Initialize IRQ for PWM globally
    static bool pwm_irq_initialized = false;
    if (!pwm_irq_initialized) {
        irq_set_exclusive_handler(PWM_DEFAULT_IRQ_NUM(), servo_on_pwm_wrap);
        irq_set_enabled(PWM_DEFAULT_IRQ_NUM(), true);
        pwm_irq_initialized = true;
    }

    // Disable IRQs for this slice, we will enable when we want to use during
    // a motion
    pwm_set_irq_enabled(slice_num, false);

    // Set the duty cycle to the starting angle and start
    servo_set_deg(servo, servo->start_angle_deg);
    pwm_set_enabled(servo->slice_num, true);
}


/* Easing functions ***********************************************************/


float ease_lin(float x)
{
    return x;
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

float ease_out_expo(float x) {
    return (x >= 1.0f) ? 1.0f : 1 - powf(2.0f, -10.0f * x);
}

float ease_in_expo(float x) {
    return (x <= 0.0f) ? 0.0f : powf(2.0f, 10.0f * (x - 1.0f));
}

/* IMPORTANT: this must be clamped!
 */
float ease_inverse_smoothstep(float x)
{
    if (x <= 0.0f) return 0.0f;
    if (x >= 1.0f) return 1.0f;
    float clamped = 1.0f - 2.0f * x;
    return 0.5f - sinf(asinf(clamped) / 3.0f);
}
