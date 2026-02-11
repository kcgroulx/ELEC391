#include <stdint.h>

#include "motor_control.h"

typedef struct {
    float kp, ki, kd;
    float integrator;
    float prev_error;
    float out_min, out_max;   // e.g. -1..+1
} PID_t;

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float pid_update(PID_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    // P
    float p = pid->kp * error;

    // I
    pid->integrator += error * dt;
    float i = pid->ki * pid->integrator;

    // D (derivative on error)
    float d = pid->kd * ((error - pid->prev_error) / dt);

    float out = p + i + d;

    // Clamp output
    float out_clamped = clampf(out, pid->out_min, pid->out_max);

    // Simple anti-windup: only integrate when not saturated
    if (out != out_clamped) {
        pid->integrator -= error * dt; // undo integration this step
    }

    pid->prev_error = error;
    return out_clamped;
}

static PID_t pid_angle = {
    .kp = 0.001f,     // start here, tune
    .ki = 0.00f,     // add later if needed
    .kd = 0.001f,    // small, optional
    .integrator = 0.0f,
    .prev_error = 0.0f,
    .out_min = -1.0f,
    .out_max =  1.0f
};

// Deadband helps stop “buzzing” near the target
#define ANGLE_DEADBAND_DEG  1.0f

float angle_pid_step(float target_angle_deg, float dt)
{
    float angle_deg = encoder_get_angle_deg();   // your function

    float error = target_angle_deg - angle_deg;
    if (error < 0) error = -error;

    if (error < ANGLE_DEADBAND_DEG) {
        pid_angle.integrator = 0.0f;  // optional
        return 0.0f;                  // stop motor
    }

    // PID output in -1..+1 (direction + duty)
    float cmd = pid_update(&pid_angle, target_angle_deg, angle_deg, dt);
    return cmd;
}

