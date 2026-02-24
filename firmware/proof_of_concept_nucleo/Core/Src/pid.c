/**
 * @file pid.c
 * @brief Fresh position PID controller for motor angle control.
 */

#include "motor_control.h"

#define DT 0.001f

/* Position PID gains (normalized motor command output) */
static float kp = 0.0300f;
static float ki = 0.0015f;
static float kd = 0.0008f;

/* Controller limits and shaping */
static float output_limit = 1.0f;
static float deadband_deg = 1.5f;
static float i_zone_deg = 40.0f;
static float integrator_limit = 200.0f;
static float max_command_step = 0.02f; /* per 1 ms tick */
static float min_effective_command = 0.08f; /* overcome static friction */

/* Derivative filtering on measurement */
static float d_lpf_alpha = 0.15f;

/* Internal state */
static float integrator = 0.0f;
static float prev_angle = 0.0f;
static float d_filtered = 0.0f;
static float prev_command = 0.0f;

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/*
 * Kept function name for compatibility with existing call sites.
 * This is a single-loop position PID, not a cascaded controller.
 */
float cascaded_control_step(float target_angle_deg)
{
    float angle = motor_controller_encoderGetAngleDeg();
    float error = target_angle_deg - angle;
    float abs_error = (error < 0.0f) ? -error : error;

    /* Derivative on measurement to reduce derivative kick */
    float angle_rate = (angle - prev_angle) / DT;
    prev_angle = angle;
    d_filtered += d_lpf_alpha * (angle_rate - d_filtered);

    if (abs_error <= deadband_deg)
    {
        integrator = 0.0f;
        prev_command = 0.0f;
        return 0.0f;
    }

    /* Integrate only near target to avoid windup during large moves */
    if (abs_error <= i_zone_deg)
    {
        integrator += error * DT;
        integrator = clampf(integrator, -integrator_limit, integrator_limit);
    }
    else
    {
        integrator = 0.0f;
    }

    float p_term = kp * error;
    float i_term = ki * integrator;
    float d_term = -kd * d_filtered;

    float u = p_term + i_term + d_term;
    u = clampf(u, -output_limit, output_limit);

    /* Slew limit to prevent rapid sign flips on light loads */
    float du = clampf(u - prev_command, -max_command_step, max_command_step);
    float command = prev_command + du;

    /* If command is too small to move the motor, lift it to breakaway level. */
    if ((command > 0.0f) && (command < min_effective_command))
    {
        command = min_effective_command;
    }
    else if ((command < 0.0f) && (command > -min_effective_command))
    {
        command = -min_effective_command;
    }

    prev_command = command;

    return command;
}
