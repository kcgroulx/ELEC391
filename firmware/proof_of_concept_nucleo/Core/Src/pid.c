/**
 * @file cascaded_pid.c
 * @brief Cascaded position–velocity PID motor control (MATLAB-equivalent)
 */

#include <stdbool.h>
#include <stdint.h>
#include "motor_control.h"

/* ===================== Parameters ===================== */

#define DT 0.001f   // 1 kHz

/* -------- Position loop (outer) -------- */
static float Kp_pos = 5.0f;              // angle -> velocity
static float max_velocity = 180.0f;      // deg/s (example)
static float position_deadband_deg = 1.5f;

/* -------- Velocity loop (inner PID) -------- */
static float Kp_vel = 20.0f; //50.0f; for when doing linear motion
static float Ki_vel = 0.0f;
static float Kd_vel = 0.0f;

static float vel_integrator = 0.0f;
static float vel_integrator_limit = 1.5f;
static float output_limit = 0.9f;

static float prev_vel_error = 0.0f;
static float prev_velocity = 0.0f;

/* -------- Homing -------- */
static bool homed = true;
static float homing_speed = -0.25f;
static float limit_switch_angle = 0.0f;   // deg

/* -------- State -------- */
static float prev_angle = 0.0f;

/* ===================== Utility ===================== */

static inline float clamp(float x, float min, float max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

/* ===================== Hardware hooks ===================== */

/* ===================== Control Loop ===================== */
/* Call this from a 1 kHz timer ISR */

/**
 * @brief Execute one step of cascaded position–velocity control.
 * @param target_angle_deg Desired target angle in degrees.
 *
 * Call this from a fixed-rate (e.g. 1 kHz) timer ISR.
 */
float cascaded_control_step(float target_angle_deg)
{
    /* -------- Encoder measurement -------- */
    float measured_angle = motor_controller_encoderGetAngleDeg();
    float measured_velocity =
        (measured_angle - prev_angle) / DT;

    prev_angle = measured_angle;

    float control = 0.0f;
    float desired_velocity = 0.0f;

    /* -------- Homing logic -------- */
    if (!homed)
    {
        control = homing_speed;

        if (measured_angle <= limit_switch_angle)
        {
            homed = true;

            vel_integrator = 0.0f;
            prev_vel_error = 0.0f;
            prev_velocity = 0.0f;
        }
    }
    else
    {
        /* -------- Position loop -------- */
        float pos_error = target_angle_deg - measured_angle;
        float abs_pos_error = (pos_error < 0.0f) ? -pos_error : pos_error;

        if (abs_pos_error <= position_deadband_deg)
        {
            // Stop driving near the setpoint and clear inner-loop memory.
            vel_integrator = 0.0f;
            prev_vel_error = 0.0f;
            prev_velocity = measured_velocity;
            control = 0.0f;
            motor_control_setMotorSpeed(control);
            return;
        }

        desired_velocity = Kp_pos * pos_error;

        desired_velocity =
            clamp(desired_velocity,
                  -max_velocity,
                   max_velocity);

        /* -------- Velocity loop -------- */
        float vel_error =
            desired_velocity - measured_velocity;

        /* Trapezoidal integrator */
        vel_integrator +=
            0.5f * (vel_error + prev_vel_error) * DT;

        vel_integrator =
            clamp(vel_integrator,
                  -vel_integrator_limit,
                   vel_integrator_limit);

        /* Derivative on measurement */
        float d_vel =
            (measured_velocity - prev_velocity) / DT;

        float u =
            (Kp_vel * vel_error) +
            (Ki_vel * vel_integrator) -
            (Kd_vel * d_vel);

        u = clamp(u, -output_limit, output_limit);

        // if (u>0)
        //     u = 0.5;
        // else
        //     u = -0.5;
        
        control = u;

        prev_vel_error = vel_error;
        prev_velocity = measured_velocity;
    }
    return control;
}
