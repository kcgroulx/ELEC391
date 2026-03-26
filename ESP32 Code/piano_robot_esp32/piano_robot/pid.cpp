/*
 * pid.cpp
 * ===========================================================================
 * PID controller — logic identical to STM32 version.
 * Only change: include motor_control.h directly (no tim.h needed).
 * ===========================================================================
 */

#include "pid.h"
#include "motor_control.h"

#define DT 0.001f   /* 1 ms — matches the 1 kHz timer in piano_robot.ino */

/* Position PID gains */
static float kp = 0.0310f;
static float ki = 0.0018f;
static float kd = 0.0008f;

/* Controller limits */
static float output_limit          = 1.0f;
static float deadband_deg          = 1.0f;
static float breakaway_error_deg   = 4.0f;
static float i_zone_deg            = 40.0f;
static float integrator_limit      = 200.0f;
static float max_command_step      = 0.02f;
static float min_effective_command = 0.08f;

/* Derivative filter */
static float d_lpf_alpha = 0.15f;

/* Internal state */
static float integrator   = 0.0f;
static float prev_angle   = 0.0f;
static float d_filtered   = 0.0f;
static float prev_command = 0.0f;

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float cascaded_control_step(float target_angle_deg)
{
    float angle     = motor_controller_encoderGetAngleDeg();
    float error     = target_angle_deg - angle;
    float abs_error = (error < 0.0f) ? -error : error;

    /* Derivative on measurement */
    float angle_rate = (angle - prev_angle) / DT;
    prev_angle = angle;
    d_filtered += d_lpf_alpha * (angle_rate - d_filtered);

    if (abs_error <= deadband_deg) {
        integrator   = 0.0f;
        prev_command = 0.0f;
        return 0.0f;
    }

    if (abs_error <= i_zone_deg) {
        integrator += error * DT;
        integrator  = clampf(integrator, -integrator_limit, integrator_limit);
    } else {
        integrator = 0.0f;
    }

    float u = kp * error + ki * integrator + (-kd * d_filtered);
    u = clampf(u, -output_limit, output_limit);

    float du      = clampf(u - prev_command, -max_command_step, max_command_step);
    float command = prev_command + du;

    float abs_command = (command < 0.0f) ? -command : command;
    if (abs_error > breakaway_error_deg) {
        if ((command * error) > 0.0f) {
            if ((command > 0.0f) && (abs_command < min_effective_command))
                command = min_effective_command;
            else if ((command < 0.0f) && (abs_command < min_effective_command))
                command = -min_effective_command;
        } else if ((command * error) < 0.0f) {
            command = 0.0f;
        }
    }

    prev_command = command;
    return command;
}
