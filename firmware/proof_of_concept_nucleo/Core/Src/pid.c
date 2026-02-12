/**
 * @file pid.c
 * @brief PID control utilities for closed-loop motor angle control.
 *
 * Provides PID state update and a helper that generates a motor command from
 * encoder angle feedback and a target angle.
 */

/* Includes */
#include <stdint.h>
#include "pid.h"
#include "motor_control.h"

static pid_config_t pid_config;
static const float PID_DT_MIN_S = 1.0e-6f;
static const float PID_DEADBAND_INTEGRATOR_LEAK = 0.999f;

/* Private Functions */
static inline float pid_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float pid_wrapAngleErrorDeg(float error_deg)
{
    while (error_deg > 180.0f)
    {
        error_deg -= 360.0f;
    }
    while (error_deg < -180.0f)
    {
        error_deg += 360.0f;
    }
    return error_deg;
}

/**
 * @brief Compute one PID update step from an error value.
 * @param error Signed control error.
 * @param dt Loop period in seconds.
 * @return PID command clamped to configured output limits.
 */
static float pid_update(float error, float dt)
{
    // Guard against invalid dt to avoid derivative blow-up and NaN/Inf.
    if (dt <= 0.0f)
    {
        float out = (pid_config.kp * error) + (pid_config.ki * pid_config.integrator);
        float out_clamped = pid_clampf(out, pid_config.out_min, pid_config.out_max);
        pid_config.prev_error = error;
        return out_clamped;
    }

    // P
    float p = pid_config.kp * error;

    // I
    pid_config.integrator += error * dt;
    float i = pid_config.ki * pid_config.integrator;

    // D (derivative on error)
    float safe_dt = (dt < PID_DT_MIN_S) ? PID_DT_MIN_S : dt;
    float d = pid_config.kd * ((error - pid_config.prev_error) / safe_dt);

    float out = p + i + d;

    // Clamp output
    float out_clamped = pid_clampf(out, pid_config.out_min, pid_config.out_max);

    // Simple anti-windup: only integrate when not saturated
    if (out != out_clamped)
    {
        pid_config.integrator -= error * dt; // undo integration this step
    }

    pid_config.prev_error = error;
    return out_clamped;
}


/* Public Functions */
/**
 * @brief Initialize PID gains, limits, and internal state from a config struct.
 * @param config PID configuration and state seed values.
 */
void pid_init(pid_config_t* config)
{
    pid_config = *config;
}

/**
 * @brief Run PID for angle control and return the commanded motor input.
 * @param target_angle_deg Desired output angle in degrees.
 * @param dt Loop period in seconds.
 * @return Signed motor command (typically in configured output range).
 */
float pid_stepAndGetCommand(float target_angle_deg, float dt)
{
    float angle_deg = motor_controller_encoderGetAngleDeg();
    float signed_error = pid_wrapAngleErrorDeg(target_angle_deg - angle_deg);
    float abs_error = signed_error;
    if (abs_error < 0)
    {
        abs_error = -abs_error;
    }

    if (abs_error < pid_config.deadband_angle)
    {
        // Keep derivative state fresh and bleed integrator instead of hard reset.
        pid_config.prev_error = signed_error;
        pid_config.integrator *= PID_DEADBAND_INTEGRATOR_LEAK;
        return 0.0f;
    }

    // PID output in -1..+1 (direction + duty)
    float cmd = pid_update(signed_error, dt);
    return cmd;
}

