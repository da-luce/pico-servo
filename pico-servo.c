#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"

#include "servo.h"

#define CLK                         150000000u
#define SERVO_GPIO                  22u
#define START_ANGLE                 0.0f
#define SEC                         1000000u // One second in us, since we do everything in us

// MG90S constants
#define MG90S_FRAME_PERIOD_USEC     10000u  // 10 ms (running at 2 x 50 Hz)
#define MG90S_PULSE_WIDTH_MIN_USEC  500u    // 0.5 ms
#define MG90S_PULSE_WIDTH_MAX_USEC  2500u   // 2.5 ms
#define MG90S_SEC_PER_60            0.5f    // Setting this slower as I don't have a legit MG90S
#define MG90S_MAX_ANGLE             180.0f  // 180 degrees

Servo servo = {
    .gpio               = SERVO_GPIO,
    .clock_freq         = CLK,
    .period_usec        = MG90S_FRAME_PERIOD_USEC,
    .duty_min_usec      = MG90S_PULSE_WIDTH_MIN_USEC,
    .duty_max_usec      = MG90S_PULSE_WIDTH_MAX_USEC,
    .start_angle_deg    = START_ANGLE,
    .sec_per_60         = MG90S_SEC_PER_60,
    .max_degrees        = MG90S_MAX_ANGLE,
};

void oscillate_to_center()
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

        servo_set_deg(&servo, left);
        sleep_us(delay_us);

        servo_set_deg(&servo, right);
        sleep_us(delay_us);

        offset -= decay_step;
        delay_us -= decay_delay;
    }

    // Finish at center
    servo_set_deg(&servo, center);
}


void oscillate_to_center_ease()
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

        servo_set_deg_ease(&servo, left, delay_us, EASE_IN_OUT_SIGMOID(50.0f));
        servo_set_deg_ease(&servo, right, delay_us, EASE_IN_OUT_SIGMOID(50.0f));

        offset -= decay_step;
        delay_us -= decay_delay;
    }

    // Finish at center
    servo_set_deg(&servo, center);
}


void test_sequence()
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

    servo_set_deg_ease(&servo, 180.0f, SEC, ease_out_bounce);
    servo_set_deg_ease(&servo, 0.0f, SEC, ease_out_bounce);

    servo_set_deg_ease(&servo, 0.0f, SEC, ease_in_expo);
    servo_set_deg_ease(&servo, 180.0f, SEC, ease_out_expo);
    servo_set_deg_ease(&servo, 0.0f, SEC, ease_in_expo);

    // Because easing is applied inside an interrupt handler,
    // we need to keep the CPU active long enough to observe the motion
    sleep_ms(5000); // FIXME: why is this sleep value so off?
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

    servo_set_deg_ease_wait(&servo, 180.0f, SEC, ease_out_bounce);
    servo_set_deg_ease_wait(&servo, 0.0f, SEC, ease_out_bounce);

    servo_set_deg_ease_wait(&servo, 0.0f, SEC, ease_in_expo);
    servo_set_deg_ease_wait(&servo, 180.0f, SEC, ease_out_expo);
    servo_set_deg_ease_wait(&servo, 0.0f, SEC, ease_in_expo); 
}

int main() {

    stdio_init_all();

    servo_init(&servo);
    sleep_ms(500);

    test_sequence();
    oscillate_to_center();
    oscillate_to_center_ease();
    test_sequence_wait();
}