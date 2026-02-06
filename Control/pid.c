#include "pid.h"

void pid_init(pid_t *pid, float kp, float ki, float kd, float dt)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;

    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;

    pid->integrator_limit = 1.0f;
    pid->output_limit = 1.0f;
}

void pid_reset(pid_t *pid)
{
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
}

float pid_update(pid_t *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    /* Proportional */
    float p = pid->kp * error;

    /* Integral */
    pid->integrator += error * pid->dt;
    if (pid->integrator > pid->integrator_limit)
        pid->integrator = pid->integrator_limit;
    else if (pid->integrator < -pid->integrator_limit)
        pid->integrator = -pid->integrator_limit;

    float i = pid->ki * pid->integrator;

    /* Derivative (on measurement) */
    float derivative = (measurement - pid->prev_measurement) / pid->dt;
    float d = -pid->kd * derivative;

    /* Output */
    float output = p + i + d;

    if (output > pid->output_limit)
        output = pid->output_limit;
    else if (output < -pid->output_limit)
        output = -pid->output_limit;

    pid->prev_error = error;
    pid->prev_measurement = measurement;

    return output;
}
