#include "servo.h"

#define SERVO_GPIO                  22u
#define START_ANGLE                 0.0f
#define SEC                         1000000u // One second in us, since we do everything in us

// MG90S constants
#define MG90S_FRAME_PERIOD_USEC     20000u  // 10 ms (running at 2 x 50 Hz)
#define MG90S_PULSE_WIDTH_MIN_USEC  500u    // 0.5 ms
#define MG90S_PULSE_WIDTH_MAX_USEC  2500u   // 2.5 ms
#define MG90S_MAX_ANGLE             180.0f  // 180 degrees

Servo servo = {
    .gpio               = SERVO_GPIO,
    .period_usec        = MG90S_FRAME_PERIOD_USEC,
    .duty_min_usec      = MG90S_PULSE_WIDTH_MIN_USEC,
    .duty_max_usec      = MG90S_PULSE_WIDTH_MAX_USEC,
    .start_deg    = START_ANGLE,
    .max_degrees        = MG90S_MAX_ANGLE,
};


void oscillate_to_center(Servo* servo, bool ease)
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

        if (ease)
        {
            servo_time_to_deg(servo, left, delay_us, ease_sin);
            servo_time_to_deg(servo, right, delay_us, ease_sin);
        } else {
            servo_set_deg(servo, left);
            sleep_us(delay_us);
            servo_set_deg(servo, right);
            sleep_us(delay_us);
        }

        offset -= decay_step;
        delay_us -= decay_delay;
    }

    // Finish at center
    servo_set_deg(servo, center);
    sleep_ms(250);
}


int main() {

    servo_init(&servo);
    sleep_ms(500);

    oscillate_to_center(&servo, false);
    sleep_ms(500);
    oscillate_to_center(&servo, true);

    while(true)
    {
        tight_loop_contents();
    }
}