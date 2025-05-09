# Pico Servo Driver

This project was tested on [MG90S](https://towerpro.com.tw/product/mg90s-3/) servos and a [Pico 2 W](https://datasheets.raspberrypi.com/picow/pico-2-w-datasheet.pdf). It should work on all other Pico variants, and most standard 180 deg servos.

## Examples

Here's a minimal example showing how to configure and control a servo using the Servo struct:

```c
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
}
```

## TODO

- [ ] Add speed control

## Easing Functions

This servo library supports easing functions to control motion curves between angles, allowing more natural and expressive movement.

Available Easing Types

- `Linear` – Constant-speed transition.
- `Ease In` – Starts slow, speeds up (e.g. quadratic, exponential).
- `Ease Out` – Starts fast, slows to target.
- `Ease In-Out` – Combines both for smooth transitions.
- `Sine` – Smooth, sinusoidal motion.
- `Bounce` – Simulates bounce-back at the end of motion.
- `Wobble-Pop` – Overshoots slightly, then settles with a wobble.

## Waiting Functions

Warning: the speed of a servo is dependent not only it's formal specification, but the requested angle delta and torque on the motor. Thus, the waiting functions provide a general estimate on blocking time, but not exact.

## Important Notes

- Each servo must use a GPIO mapped to a unique [PWM slice](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf#%5B%7B%22num%22%3A1077%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C115%2C165.63628%2Cnull%5D)
- Servos must be configured to use GPIOs on separate 
- While the Pico exposes 24 PWM channels, only 12 independent slices are available—this library supports up to 12 servos.
- It's your responsibility to ensure that servos are not commanded beyond their mechanical speed or range limits

## Miscellanous

- 
