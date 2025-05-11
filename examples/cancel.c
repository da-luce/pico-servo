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
    .start_deg          = START_ANGLE,
    .sec_per_60         = MG90S_SEC_PER_60,
    .max_degrees        = MG90S_MAX_ANGLE,
};

int main() {

    servo_init(&servo);

    bool cancel = false;

    while (true) {
        servo_time_to_deg(&servo, 180.0f, 2 * SEC, ease_lin, &cancel);
        sleep_ms(1000);
        cancel = true;
        servo_set_deg(&servo, 0.0f);
        sleep_ms(250);
        cancel = false;
    }

}