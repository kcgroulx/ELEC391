/**
 * @file cascaded_pid.c
 * @brief Cascaded position–velocity PID motor control (MATLAB-equivalent)
 */

#include <stdbool.h>
#include <stdint.h>

/* ===================== Parameters ===================== */

#define DT 0.001f   // 1 kHz

/* -------- Position loop (outer) -------- */
static float Kp_pos = 5.0f;              // angle -> velocity
static float max_velocity = 180.0f;      // deg/s (example)

/* -------- Velocity loop (inner PID) -------- */
static float Kp_vel = 20.0f; //50.0f; for when doing linear motion
static float Ki_vel = 4.0f;
static float Kd_vel = 1.0f;

static float vel_integrator = 0.0f;
static float vel_integrator_limit = 1.5f;
static float output_limit = 0.9f;

static float prev_vel_error = 0.0f;
static float prev_velocity = 0.0f;

/* -------- Homing -------- */
static bool homed = true;
static float homing_speed = -0.25f;
static float limit_switch_angle = 0.0f;   // deg
static float target_angle = 30.0f;         // deg

/* -------- State -------- */
static float prev_angle = 0.0f;

/* ===================== Utility ===================== */

static inline float clamp(float x, float min, float max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

static float wrap_angle_error(float error) //if error 250 deg, wrap to -110 deg //this makes it so the controller takes the short way around the circle
{
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

/* ===================== Hardware hooks ===================== */
/* YOU implement these */

float encoder_getAngleDeg(void);
void motor_setCommand(float u);

/* ===================== Control Loop ===================== */
/* Call this from a 1 kHz timer ISR */

void cascaded_control_step(void)
{
    /* -------- Encoder measurement -------- */
    float measured_angle = encoder_getAngleDeg();
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
        float pos_error =
            wrap_angle_error(target_angle - measured_angle); // signed error in -180..+180 deg

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

        control = u;

        prev_vel_error = vel_error;
        prev_velocity = measured_velocity;
    }

    /* -------- Actuate motor -------- */
    motor_setCommand(control);
}
