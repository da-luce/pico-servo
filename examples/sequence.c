#include "servo.h"

#define SERVO_GPIO                  22u
#define START_ANGLE                 0.0f
#define SEC                         1000000u // One second in us, since we do everything in us

// MG90S constants
#define MG90S_FRAME_PERIOD_USEC     20000u  // 10 ms (running at 2 x 50 Hz)
#define MG90S_PULSE_WIDTH_MIN_USEC  500u    // 0.5 ms
#define MG90S_PULSE_WIDTH_MAX_USEC  2500u   // 2.5 ms
#define MG90S_SEC_PER_60            0.5f    // Setting this slower as I don't have a legit MG90S
#define MG90S_MAX_ANGLE             180.0f  // 180 degrees

Servo servo = {
    .gpio               = SERVO_GPIO,
    .period_usec        = MG90S_FRAME_PERIOD_USEC,
    .duty_min_usec      = MG90S_PULSE_WIDTH_MIN_USEC,
    .duty_max_usec      = MG90S_PULSE_WIDTH_MAX_USEC,
    .start_deg    = START_ANGLE,
    .sec_per_60         = MG90S_SEC_PER_60,
    .max_degrees        = MG90S_MAX_ANGLE,
};

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

    servo_time_to_deg(servo, 180.0f, SEC, ease_out_bounce);
    servo_time_to_deg(servo, 0.0f, SEC, ease_out_bounce);

    servo_time_to_deg(servo, 180.0f, SEC, ease_in_expo);
    servo_time_to_deg(servo, 0.0f, SEC, ease_inverse_smoothstep);
    servo_time_to_deg(servo, 180.0f, SEC, ease_inverse_smoothstep);
    servo_time_to_deg(servo, 0.0f, SEC, ease_out_expo);

    // Keep the core active for the last movement
    // TODO: is this a bug on the Pico, should it not be in interrupt wait mode?
    sleep_ms(1000);
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

    servo_time_to_deg_wait(servo, 180.0f, SEC, ease_out_bounce);
    servo_time_to_deg_wait(servo, 0.0f, SEC, ease_out_bounce);

    servo_time_to_deg_wait(servo, 180.0f, SEC, ease_in_expo);
    servo_time_to_deg_wait(servo, 0.0f, SEC, ease_inverse_smoothstep);
    servo_time_to_deg_wait(servo, 180.0f, SEC, ease_inverse_smoothstep);
    servo_time_to_deg_wait(servo, 0.0f, SEC, ease_out_expo);
}

int main() {

    servo_init(&servo);
    sleep_ms(500);

    test_sequence(&servo);
    test_sequence_wait(&servo);

    while (true)
    {
        tight_loop_contents();
    }

}