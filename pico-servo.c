#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"

#include "servo.h"

#define CLK                 150000000u

#define SERVO_GPIO          22u
#define SERVO_PERIOD_USEC   20000u  // 20 ms
#define DUTY_CYCLE_MIN_USEC 500u    // 0.5 ms
#define DUTY_CYCLE_MAX_USEC 2500u   // 2.5 ms
#define START_ANGLE         0.0f

Servo servo = {
    .gpio               = SERVO_GPIO,
    .clock_freq         = CLK,
    .period_usec        = SERVO_PERIOD_USEC,
    .duty_min_usec      = DUTY_CYCLE_MIN_USEC,
    .duty_max_usec      = DUTY_CYCLE_MAX_USEC,
    .start_angle_deg    = START_ANGLE,
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

    // Because easing is applied inside an interrupt handler,
    // we need to keep the CPU active long enough to observe the motion
    sleep_ms(5000);
}

int main() {
    stdio_init_all();

    servo_init(&servo);
    sleep_ms(200);

    test_sequence();
}