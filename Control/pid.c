/************************************************************
 * Cascaded Position–Velocity PID Control (1 kHz)
 * Mirrors provided MATLAB simulation
 *
 * Units:
 *  - Position: meters
 *  - Velocity: m/s
 *  - Control output: normalized motor command [-1, 1]
 ************************************************************/

#include <stdbool.h>
#include <stdint.h>

/* ===================== Constants ===================== */

#define DT 0.001f    // 1 kHz control loop

/* -------- Position loop -------- */
#define KP_POS        5.0f
#define MAX_VELOCITY  0.4f   // m/s

/* -------- Velocity PID -------- */
#define KP_VEL  50.0f
#define KI_VEL  4.0f
#define KD_VEL  1.0f

#define INTEGRATOR_LIMIT  1.5f
#define OUTPUT_LIMIT      0.9f

/* -------- Homing -------- */
#define HOMING_SPEED        -0.25f
#define LIMIT_SWITCH_POS     0.0f   // meters

/* ===================== Types ===================== */

typedef struct {
    float kp;
    float ki;
    float kd;

    float integrator;
    float integrator_limit;

    float prev_error;
    float prev_measurement;

    float output_limit;
} PID_Controller;

/* ===================== Globals ===================== */

static PID_Controller vel_pid = {
    .kp = KP_VEL,
    .ki = KI_VEL,
    .kd = KD_VEL,
    .integrator = 0.0f,
    .integrator_limit = INTEGRATOR_LIMIT,
    .prev_error = 0.0f,
    .prev_measurement = 0.0f,
    .output_limit = OUTPUT_LIMIT
};

static bool homed = true;
static float target_position = 0.02f;   // meters

static float prev_position = 0.0f;

/* ===================== Utility ===================== */

static inline float clamp(float x, float min, float max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

/* ===================== Hardware Stubs ===================== */
/* YOU must implement these for your system */

float read_encoder_position_m(void)
{
    // Return measured position in meters
    return 0.0f;
}

void set_motor_output(float u)
{
    // u ∈ [-0.9, 0.9]
    // Map to PWM / DAC / current command
    (void)u;
}

/* ===================== Velocity PID ===================== */

static float velocity_pid_update(PID_Controller *pid,
                                 float desired_velocity,
                                 float measured_velocity)
{
    float error = desired_velocity - measured_velocity;

    // Trapezoidal integrator
    pid->integrator += 0.5f * (error + pid->prev_error) * DT;
    pid->integrator = clamp(pid->integrator,
                            -pid->integrator_limit,
                             pid->integrator_limit);

    // Derivative on measurement
    float d_meas = (measured_velocity - pid->prev_measurement) / DT;

    float u = pid->kp * error
            + pid->ki * pid->integrator
            - pid->kd * d_meas;

    u = clamp(u, -pid->output_limit, pid->output_limit);

    pid->prev_error = error;
    pid->prev_measurement = measured_velocity;

    return u;
}

/* ===================== 1 kHz Control Loop ===================== */
/* Call this from a timer ISR */

void control_loop_1khz(void)
{
    float measured_position = read_encoder_position_m();
    float measured_velocity;
    float desired_velocity = 0.0f;
    float motor_command = 0.0f;

    /* Velocity estimation */
    measured_velocity = (measured_position - prev_position) / DT;
    prev_position = measured_position;

    /* -------- Homing -------- */
    if (!homed) {
        motor_command = HOMING_SPEED;

        if (measured_position <= LIMIT_SWITCH_POS) {
            homed = true;

            vel_pid.integrator = 0.0f;
            vel_pid.prev_error = 0.0f;
            vel_pid.prev_measurement = 0.0f;
        }
    }
    else {
        /* -------- Position loop -------- */
        float pos_error = target_position - measured_position;
        desired_velocity = KP_POS * pos_error;

        desired_velocity = clamp(desired_velocity,
                                 -MAX_VELOCITY,
                                  MAX_VELOCITY);

        /* -------- Velocity loop -------- */
        motor_command = velocity_pid_update(&vel_pid,
                                             desired_velocity,
                                             measured_velocity);
    }

    /* -------- Actuate motor -------- */
    set_motor_output(motor_command);
}

/* ===================== Optional main ===================== */
/* Control loop should normally run from a timer ISR */

int main(void)
{
    // Init hardware here (timers, encoder, PWM, etc.)

    while (1) {
        // Main loop does nothing
        // control_loop_1khz() runs in timer interrupt
    }
}