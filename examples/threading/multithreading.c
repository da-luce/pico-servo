#include "pt-1.4/pt.h"

#include "servo.h"

#define SERVOA_GPIO                 22u
#define SERVOB_GPIO                 9u
#define SEC                         1000000u // One second in us, since we do everything in us

// MG90S constants
#define MG90S_FRAME_PERIOD_USEC     20000u  // 10 ms (running at 2 x 50 Hz)
#define MG90S_PULSE_WIDTH_MIN_USEC  500u    // 0.5 ms
#define MG90S_PULSE_WIDTH_MAX_USEC  2500u   // 2.5 ms
#define MG90S_SEC_PER_60            0.5f    // Setting this slower as I don't have a legit MG90S
#define MG90S_MAX_ANGLE             180.0f  // 180 degrees

#define MG90S_SERVO(gpio_pin, start_angle) \
    (Servo){                               \
        .gpio = (gpio_pin),                \
        .period_usec = MG90S_FRAME_PERIOD_USEC,     \
        .duty_min_usec = MG90S_PULSE_WIDTH_MIN_USEC,\
        .duty_max_usec = MG90S_PULSE_WIDTH_MAX_USEC,\
        .start_deg = (start_angle),        \
        .sec_per_60 = MG90S_SEC_PER_60,    \
        .max_degrees = MG90S_MAX_ANGLE     \
    }

Servo servoA = MG90S_SERVO(SERVOA_GPIO, 0.0f);
Servo servoB = MG90S_SERVO(SERVOB_GPIO, 180.0f);

static struct pt ptA, ptB;
PT_THREAD(servo_task(struct pt *pt, Servo *s, bool forwards)) {
    PT_BEGIN(pt);
    while (1) {
        servo_time_to_deg(s, forwards ? 180.0f : 0.0f, SEC, ease_lin, NULL);
        PT_YIELD(pt);

        servo_time_to_deg(s, forwards ? 0.0f : 180.0f, SEC, ease_lin, NULL);
        PT_YIELD(pt);
    }
    PT_END(pt);
}

int main() {
    PT_INIT(&ptA);
    PT_INIT(&ptB);

    servo_init(&servoA);
    servo_init(&servoB);
    sleep_ms(500);

    while (1) {
        servo_task(&ptA, &servoA, true);
        servo_task(&ptB, &servoB, false);
    }
}
