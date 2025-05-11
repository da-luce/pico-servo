#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <math.h>

#include "pico/sync.h"

struct Servo; // Forward declaration
typedef struct Servo Servo; // Typedef for ease of use
typedef float (*ease_fn_t)(float);
typedef void (*servo_callback_t)(Servo *);

// Everything needed to track a motion between two angles
typedef struct {
    unsigned int duration_us;
    unsigned int current_time_us;
    float start_deg;
    float end_deg;
    ease_fn_t ease_fn;
    volatile bool *cancel_flag;     // If true, the motion should be canceled
} Motion;

typedef struct Servo {
    unsigned int gpio;              // GPIO that servo is attached to
    unsigned int period_usec;       // Period of control signal in usec
    unsigned int duty_min_usec;     // Duty cycle of minumum angle in usec
    unsigned int duty_max_usec;     // Duty cycle of maximum angle in usec
    float start_deg;                // Angle to start at
    float sec_per_60;               // Speed to move 60 degrees (Defaults to 0.5)
    float max_degrees;              // Maximum allowable rotation angle of the servo in degrees  (Defaults to 180)
    servo_callback_t callback;      // Callback on motion complete

    /* Private fields */
    float _max_rad;
    unsigned int _pwm_slice;
    unsigned int _pwm_channel;
    volatile float _current_deg;    // Track current angle
    volatile Motion _motion;
    mutex_t _mutex;                 // Ensure we don't schedule multiple movements at once
} Servo;

/* Initialize hardware and structures required to drive a servo.
 * WARNING: this may not work for very high frequency clocks with low frequency
 * servos.
 */
void servo_init(Servo* servo);

/* Set the servo position in radians/degrees.
 */
void servo_set_rad(Servo* servo, float target_rad);
void servo_set_deg(Servo* servo, float target_deg);

/* Sets the servo position in radians/degrees and blocks until the motion is complete.
 * The wait duration is _estimated_ based on the servo's speed parameter. For 
 * longer movements, this is an over estimate, and slight under estimate for short
 * movements.
 */
void servo_set_rad_wait(Servo* servo, float target_rad);
void servo_set_deg_wait(Servo* servo, float target_deg);

/* Schedules the servo movement in radians/degrees over the specified duration using the provided
 * easing function. This function is non-blocking from the caller's perspective:
 * it returns immediately after scheduling. However, it prevents other servo
 * movement operations from running concurrently on the same servo until the
 * current motion is complete. Other unrelated code continues to run during the motion.
 * Defaults to linear motion if no easing specified.
 */
void servo_time_to_rad(Servo* servo, float target_rad, unsigned int duration_us, ease_fn_t ease_fn, volatile bool *cancel_flag);
void servo_time_to_deg(Servo* servo, float target_deg, unsigned int duration_us, ease_fn_t ease_fn, volatile bool *cancel_flag);

/* Schedules the servo movement in radians/degrees over the specified duration using the provided
 * easing function. This function is blocking from the caller's perspective.
 */
void servo_time_to_rad_wait(Servo* servo, float target_rad, unsigned int duration_us, ease_fn_t ease_fn);
void servo_time_to_deg_wait(Servo* servo, float target_deg, unsigned int duration_us, ease_fn_t ease_fn);

void servo_speed_to_rad(Servo* servo, float target_rad, float deg_per_sec, volatile bool *cancel_flag);
void servo_speed_to_deg(Servo* servo, float target_deg, float deg_per_sec, volatile bool *cancel_flag);

/* Easing functions that map input progress from [0.0, 1.0] to an output in [0.0, 1.0]
 */
float ease_lin(float x);
float ease_sin(float x);
float ease_in_quad(float x);
float ease_out_quad(float x);
float ease_in_expo(float x);
float ease_out_expo(float x);
float ease_in_bounce(float x);
float ease_out_bounce(float x);
float ease_inverse_smoothstep(float x);

#endif