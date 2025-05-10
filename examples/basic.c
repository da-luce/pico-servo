#include "servo.h"

Servo servo = {
    .gpio               = 22,
    .period_usec        = 20000u,
    .duty_min_usec      = 500u,
    .duty_max_usec      = 2500u,
};

int main()
{
    servo_init(&servo);
    servo_set_deg(&servo, 90.0f);

    while(true)
    {
        tight_loop_contents();
    }
}