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

int main() {
    stdio_init_all();

    servo_init(&servo);

    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    float angle = 0.0f;           // Start at 0 degrees
    float step = 15.0f;           // Rotate 30 degrees each time

    while (true) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(250);
        printf("Servo angle: %.1f\n", angle);

        servo_set_deg(&servo, angle);

        // Increment angle and wrap around at 180
        angle += step;
        if (angle > 180.0f) {
            angle = 0.0f;
            servo_set_deg_ease(&servo, 0.0f, 1000000u, ease_in_expo);
        }

        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(100);
    }
}