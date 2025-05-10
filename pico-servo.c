#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"

#include "servo.h"

#define SERVOA_GPIO                 22u
#define SERVOB_GPIO                 9u
#define START_ANGLE                 0.0f
#define SEC                         1000000u // One second in us, since we do everything in us

// MG90S constants
#define MG90S_FRAME_PERIOD_USEC     20000u  // 10 ms (running at 2 x 50 Hz)
#define MG90S_PULSE_WIDTH_MIN_USEC  500u    // 0.5 ms
#define MG90S_PULSE_WIDTH_MAX_USEC  2500u   // 2.5 ms
#define MG90S_SEC_PER_60            0.5f    // Setting this slower as I don't have a legit MG90S
#define MG90S_MAX_ANGLE             180.0f  // 180 degrees

Servo servoA = {
    .gpio               = SERVOA_GPIO,
    .period_usec        = MG90S_FRAME_PERIOD_USEC,
    .duty_min_usec      = MG90S_PULSE_WIDTH_MIN_USEC,
    .duty_max_usec      = MG90S_PULSE_WIDTH_MAX_USEC,
    .start_angle_deg    = START_ANGLE,
    .sec_per_60         = MG90S_SEC_PER_60,
    .max_degrees        = MG90S_MAX_ANGLE,
};

Servo servoB = {
    .gpio               = SERVOB_GPIO,
    .period_usec        = MG90S_FRAME_PERIOD_USEC,
    .duty_min_usec      = MG90S_PULSE_WIDTH_MIN_USEC,
    .duty_max_usec      = MG90S_PULSE_WIDTH_MAX_USEC,
    .start_angle_deg    = START_ANGLE,
    .sec_per_60         = MG90S_SEC_PER_60,
    .max_degrees        = MG90S_MAX_ANGLE,
};


void oscillate_to_center(Servo* servo)
{
    const float center = 90.0f;
    const float max_offset = 90.0f; // max deviation from center (i.e. from 90 to 0/180)
    const float min_offset = 1.0f;  // stop oscillation when within ±1 degree of center
    const float decay_step = 10.0f; // how much to reduce the offset per cycle
    const unsigned int decay_delay = 50000;
    unsigned int delay_us = 400000; // 200ms between moves

    float offset = max_offset;

    while (offset > min_offset)
    {
        float left = center - offset;
        float right = center + offset;

        // Clamp to servo limits
        if (left < 0.0f) left = 0.0f;
        if (right > 180.0f) right = 180.0f;

        servo_set_deg(servo, left);
        sleep_us(delay_us);

        servo_set_deg(servo, right);
        sleep_us(delay_us);

        offset -= decay_step;
        delay_us -= decay_delay;
    }

    // Finish at center
    servo_set_deg(servo, center);
    sleep_ms(250);
}


void oscillate_to_center_ease(Servo* servo)
{
    const float center = 90.0f;
    const float max_offset = 90.0f; // max deviation from center (i.e. from 90 to 0/180)
    const float min_offset = 1.0f;  // stop oscillation when within ±1 degree of center
    const float decay_step = 10.0f; // how much to reduce the offset per cycle
    const unsigned int decay_delay = 50000;
    unsigned int delay_us = 400000; // 200ms between moves

    float offset = max_offset;

    while (offset > min_offset)
    {
        float left = center - offset;
        float right = center + offset;

        // Clamp to servo limits
        if (left < 0.0f) left = 0.0f;
        if (right > 180.0f) right = 180.0f;

        servo_set_deg_ease(servo, left, delay_us, ease_sin);
        servo_set_deg_ease(servo, right, delay_us, ease_sin);

        offset -= decay_step;
        delay_us -= decay_delay;
    }

    // Finish at center
    servo_set_deg(servo, center);
    sleep_ms(250);
}


void test_sequence(Servo* servo)
{
    // Simple stepping sequence
    bool forward = true;
    for (float step = 20.0f; step >= 5.0f; step -= 5.0f)
    {
        float angle;
        int sleep_time = 20 * (step / 5.0f);
        if (forward)
        {
            angle = 0.0f;
            while (angle <= 180.0f)
            {
                servo_set_deg(servo, angle);
                angle += step;
                sleep_ms(sleep_time);
            }
        } else
        {
            angle = 180.0f;
            while (angle >= 0.0f)
            {
                servo_set_deg(servo, angle);
                angle -= step;
                sleep_ms(sleep_time);
            }

        }

        forward = !forward;
    }

    servo_set_deg(servo, 180.0f);
    sleep_ms(500);
    servo_set_deg(servo, 0.0f);
    sleep_ms(500);

    servo_set_deg_ease(servo, 180.0f, SEC, ease_out_bounce);
    servo_set_deg_ease(servo, 0.0f, SEC, ease_out_bounce);

    servo_set_deg_ease(servo, 180.0f, SEC, ease_in_expo);
    servo_set_deg_ease(servo, 0.0f, SEC, ease_inverse_smoothstep);
    servo_set_deg_ease(servo, 180.0f, SEC, ease_inverse_smoothstep);
    servo_set_deg_ease(servo, 0.0f, SEC, ease_out_expo); 

    // Keep the core active for the last movement
    // TODO: is this a bug on the Pico, should it not be in interrupt wait mode?
    sleep_ms(1000);
}

void test_non_blocking(Servo* servo)
{
    servo_set_deg_ease(servo, 0.0f, SEC, ease_lin);
    servo_set_deg_ease(servo, 180.0f, SEC, ease_lin);
}

void test_sequence_wait(Servo* servo)
{
    // Simple stepping sequence
    bool forward = true;
    for (float step = 20.0f; step >= 5.0f; step -= 5.0f)
    {
        float angle;
        if (forward)
        {
            angle = 0.0f;
            while (angle <= 180.0f)
            {
                servo_set_deg_wait(servo, angle);
                angle += step;
            }
        } else
        {
            angle = 180.0f;
            while (angle >= 0.0f)
            {
                servo_set_deg_wait(servo, angle);
                angle -= step;
            }

        }

        forward = !forward;
    }

    servo_set_deg_wait(servo, 180.0f);
    servo_set_deg_wait(servo, 0.0f);

    servo_set_deg_ease_wait(servo, 180.0f, SEC, ease_out_bounce);
    servo_set_deg_ease_wait(servo, 0.0f, SEC, ease_out_bounce);

    servo_set_deg_ease_wait(servo, 180.0f, SEC, ease_in_expo);
    servo_set_deg_ease_wait(servo, 0.0f, SEC, ease_inverse_smoothstep);
    servo_set_deg_ease_wait(servo, 180.0f, SEC, ease_inverse_smoothstep);
    servo_set_deg_ease_wait(servo, 0.0f, SEC, ease_out_expo); 
}

bool is_prime(int n) {
    if (n < 2) return false;
    for (int i = 2; i <= sqrt(n); i++) {
        if (n % i == 0) return false;
    }
    return true;
}

int find_nth_prime(int target) {
    int count = 0;
    int num = 2;

    while (count < target) {
        if (is_prime(num)) {
            count++;
        }
        num++;
    }

    return num - 1;
}

// Perform initialisation
int pico_led_init(void) {
    #if defined(PICO_DEFAULT_LED_PIN)
        // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
        // so we can use normal GPIO functionality to turn the led on and off
        gpio_init(PICO_DEFAULT_LED_PIN);
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
        return PICO_OK;
    #elif defined(CYW43_WL_GPIO_LED_PIN)
        // For Pico W devices we need to initialise the driver etc
        return cyw43_arch_init();
    #endif
}

// Turn the led on or off
void pico_set_led(bool led_on) {
    #if defined(PICO_DEFAULT_LED_PIN)
        // Just set the GPIO on or off
        gpio_put(PICO_DEFAULT_LED_PIN, led_on);
    #elif defined(CYW43_WL_GPIO_LED_PIN)
        // Ask the wifi "driver" to set the GPIO on or off
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
    #endif
}

int main() {

    stdio_init_all();
    pico_led_init();
    servo_init(&servoA);
    servo_init(&servoB);
    sleep_ms(500);

    servo_set_deg_ease(&servoA, 180.0f, SEC, ease_lin);
    servo_set_deg_ease(&servoB, 180.0f, SEC, ease_lin);

    test_non_blocking(&servoA);
    test_non_blocking(&servoB);

    // test_sequence(&servoA);
    // oscillate_to_center(&servoA);
    // oscillate_to_center_ease(&servoA);
    // test_sequence_wait(&servoA);

    // Cool non-blocking demo
    servo_set_deg_ease(&servoA, 180.0f, 10 * SEC, ease_lin);

    while (1) {
        pico_set_led(true);
        sleep_ms(200);
        pico_set_led(false);
        sleep_ms(200);
    }

}