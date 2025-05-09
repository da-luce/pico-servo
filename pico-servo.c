#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"

#include "servo.h"

#define CLK                         150000000u
#define SERVO_GPIO                  22u
#define START_ANGLE                 0.0f

// MG90S constants
#define MG90S_FRAME_PERIOD_USEC     10000u  // 10 ms (running at 2 x 50 Hz)
#define MG90S_PULSE_WIDTH_MIN_USEC  500u    // 0.5 ms
#define MG90S_PULSE_WIDTH_MAX_USEC  2500u   // 2.5 ms
#define MG90S_SEC_PER_60            0.1f

Servo servo = {
    .gpio               = SERVO_GPIO,
    .clock_freq         = CLK,
    .period_usec        = MG90S_FRAME_PERIOD_USEC,
    .duty_min_usec      = MG90S_PULSE_WIDTH_MIN_USEC,
    .duty_max_usec      = MG90S_PULSE_WIDTH_MAX_USEC,
    .start_angle_deg    = START_ANGLE,
    .sec_per_60         = MG90S_SEC_PER_60,
};

void test_sequence()
{
    // Simple stepping sequence
    bool forward = true;
    for (float step = 20.0f; step >= 5.0f; step -= 5.0f)
    {
        float angle;
        int sleep_time = 100 * (step / 5.0f);
        if (forward)
        {
            angle = 0.0f;
            while (angle <= 180.0f)
            {
                servo_set_deg(&servo, angle);
                angle += step;
                sleep_ms(sleep_time);
            }
        } else
        {
            angle = 180.0f;
            while (angle >= 0.0f)
            {
                servo_set_deg(&servo, angle);
                angle -= step;
                sleep_ms(sleep_time);
            }

        }

        forward = !forward;
    }

    servo_set_deg(&servo, 180.0f);
    sleep_ms(500);
    servo_set_deg(&servo, 0.0f);
    sleep_ms(500);

    servo_set_deg_ease(&servo, 180.0f, 1000000, ease_out_bounce);
    servo_set_deg_ease(&servo, 0.0f, 1000000u, ease_out_bounce);

    servo_set_deg_ease(&servo, 90.0f, 1000000, ease_out_bounce);
    servo_set_deg_ease(&servo, 180.0f, 1000000u, ease_in_bounce);

    servo_set_deg_ease(&servo, 0.0f, 2000000u, ease_in_expo);
    servo_set_deg_ease(&servo, 180.0f, 2000000u, ease_out_expo);
    servo_set_deg_ease(&servo, 0.0f, 2000000u, ease_in_expo);

    // Because easing is applied inside an interrupt handler,
    // we need to keep the CPU active long enough to observe the motion
    sleep_ms(10000);
}

void test_sequence_wait()
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
                servo_set_deg_wait(&servo, angle);
                angle += step;
            }
        } else
        {
            angle = 180.0f;
            while (angle >= 0.0f)
            {
                servo_set_deg_wait(&servo, angle);
                angle -= step;
            }

        }

        forward = !forward;
    }

    servo_set_deg_wait(&servo, 180.0f);
    servo_set_deg_wait(&servo, 0.0f);

    servo_set_deg_ease_wait(&servo, 180.0f, 1000000, ease_out_bounce);
    servo_set_deg_ease_wait(&servo, 0.0f, 1000000u, ease_out_bounce);

    servo_set_deg_ease_wait(&servo, 90.0f, 1000000, ease_out_bounce);
    servo_set_deg_ease_wait(&servo, 180.0f, 1000000u, ease_in_bounce);

    servo_set_deg_ease_wait(&servo, 0.0f, 2000000u, ease_in_expo);
    servo_set_deg_ease_wait(&servo, 180.0f, 2000000u, ease_out_expo);
    servo_set_deg_ease_wait(&servo, 0.0f, 2000000u, ease_in_expo); // FIXME: doesn't finish completely
    // Need like 20ms to get this to finish
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

int main() {

    stdio_init_all();
    int rc = pico_led_init();

    servo_init(&servo);
    sleep_ms(2500);

    test_sequence();
    test_sequence_wait();
}