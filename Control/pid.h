#pragma once

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integrator;
    float prev_error;
    float prev_measurement;

    float integrator_limit;
    float output_limit;

    float dt;
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float dt);
void pid_reset(pid_t *pid);
float pid_update(pid_t *pid, float setpoint, float measurement);
