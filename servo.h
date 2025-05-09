#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <math.h>

#include "pico/sync.h"


// Everthing needed to track a motion between two angles
typedef struct {
    unsigned int duration_us;
    unsigned int current_time_us;
    float start_deg;
    float end_deg;
    float (*ease_fn)(float);
} Motion;

typedef struct {
    unsigned int gpio;              // GPIO that servo is attached to
    unsigned int clock_freq;        // Each servo stores the clock freq that is driving it
    unsigned int period_usec;       // Period of control signal in usec
    unsigned int duty_min_usec;     // Duty cycle of minumum angle in usec
    unsigned int duty_max_usec;     // Duty cycle of maximum angle in usec
    float start_angle_deg;          // Angle to start at
    float sec_per_60;               // Speed to move 60 degrees (Defaults to 0.5)
    float max_degrees;              // Maximum allowable rotation angle of the servo in degrees  (Defaults to 180)
    // These are filled in automatically
    unsigned int slice_num;
    unsigned int channel_num;
    volatile float current_angle;   // Track current angle
    volatile Motion motion;
    mutex_t mutex;  // Lock the servo
} Servo;

/* Initialize hardware and structures required to drive a servo.
 * WARNING: this may not work for very high frequency clocks with low frequency
 * servos.
 */
void servo_init(Servo* servo);

/* Set the servo position in radians.
 */
void servo_set_rad(Servo* servo, float angle_rad);

/* Set the servo position in degrees.
 */
void servo_set_deg(Servo* servo, float angle_deg);

/* Sets the servo position in radians and blocks until the motion is complete.
 * The wait duration is _estimated_ based on the servo's speed parameter. For 
 * longer movements, this is an over estimate, and slight under estimate for short
 * movements.
 */
void servo_set_rad_wait(Servo* servo, float angle_rad);

/* Sets the servo position in degrees and blocks until the motion is complete.
 * The wait duration is _estimated_ based on the servo's speed parameter. For 
 * longer movements, this is an over estimate, and slight under estimate for short
 * movements.
 */
void servo_set_deg_wait(Servo* servo, float angle_deg);

/* Schedules the servo movement in degrees over the specified duration using the provided
 * easing function. This function is non-blocking from the caller's perspective:
 * it returns immediately after scheduling. However, it prevents other servo
 * movement operations from running concurrently on the same servo until the
 * current motion is complete. Other unrelated code continues to run during the motion.
 */
void servo_set_deg_ease(Servo* servo, float angle_deg, unsigned int duration_us, float (*ease_fn)(float));

/* Schedules the servo movement in degrees over the specified duration using the provided
 * easing function. This function is blocking from the caller's perspective.
 */
void servo_set_deg_ease_wait(Servo* servo, float angle_deg, unsigned int duration_us, float (*ease_fn)(float));

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

#define EASE_IN_OUT_SIGMOID(steepness) \
    ({ \
        float sigmoid(float x) { \
            float s = 1.0f / (1.0f + expf(-(steepness) * (x - 0.5f))); \
            float min = 1.0f / (1.0f + expf((steepness) / 2.0f)); \
            float max = 1.0f / (1.0f + expf(-(steepness) / 2.0f)); \
            return (s - min) / (max - min); \
        } \
        sigmoid; \
    })

#endif