#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>

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
    float sec_per_60;               // Speed to move 60 degrees, often used during 
    // These are filled in automatically
    unsigned int slice_num;
    unsigned int channel_num;
    volatile float current_angle;            // Track current angle
    volatile Motion motion;
    mutex_t mutex;  // Lock the servo
} Servo;

void servo_init(Servo* servo);
void servo_set_rad(Servo* servo, float angle_rad);
void servo_set_deg(Servo* servo, float angle_deg);
void servo_set_rad_wait(Servo* servo, float angle_rad);
void servo_set_deg_wait(Servo* servo, float angle_deg);
void servo_set_deg_ease(Servo* servo, float angle_deg, unsigned int duration_us, float (*ease_fn)(float));
void servo_set_deg_ease_wait(Servo* servo, float angle_deg, unsigned int duration_us, float (*ease_fn)(float));

// Easing functions
float ease_lin(float x);
float ease_sin(float x);
float ease_in_quad(float x);
float ease_out_quad(float x);
float ease_out_expo(float x);
float ease_in_expo(float x);
float ease_out_bounce(float x);
float ease_in_bounce(float x);
float ease_out_wobble_pop(float x);
float ease_in_out_sigmoid(float x);

#endif