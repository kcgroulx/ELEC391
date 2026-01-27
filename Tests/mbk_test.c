#include <stdio.h>
#include <stdlib.h>

/* ============================= */
/* PID Controller (simplified)   */
/* ============================= */
typedef struct
{
    float kp, ki, kd;
    float integrator;
    float prev_measurement;
    float dt;
    float integrator_limit;
    float output_limit;
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float dt)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;
    pid->integrator = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->integrator_limit = 1.0f;
    pid->output_limit = 1.0f;
}

float pid_update(pid_t *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    /* Proportional */
    float p = pid->kp * error;

    /* Integral */
    pid->integrator += error * pid->dt;
    if (pid->integrator > pid->integrator_limit) pid->integrator = pid->integrator_limit;
    if (pid->integrator < -pid->integrator_limit) pid->integrator = -pid->integrator_limit;
    float i = pid->ki * pid->integrator;

    /* Derivative on measurement */
    float d = -pid->kd * (measurement - pid->prev_measurement) / pid->dt;

    pid->prev_measurement = measurement;

    float output = p + i + d;

    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;

    return output;
}

/* ============================= */
/* MBK Rail Simulation           */
/* ============================= */
float mass = 1.0f;      // kg
float damping = 2.0f;   // N·s/m
float stiffness = 0.0f; // N/m

float position = 0.0f;
float velocity = 0.0f;
float dt = 0.001f;      // 1 kHz loop

void mbk_step(float input)
{
    float acceleration = (input - damping * velocity - stiffness * position) / mass;
    velocity += acceleration * dt;
    position += velocity * dt;
}

float mbk_get_position(void) { return position; }

/* ============================= */
/* Main simulation               */
/* ============================= */
int main(void)
{
    FILE *f = fopen("log.csv", "w");
    if (!f)
    {
        printf("Error opening file!\n");
        return 1;
    }

    fprintf(f, "Time,Position,Control\n"); // CSV header

    pid_t rail_pid;
    pid_init(&rail_pid, 5.0f, 0.0f, 0.1f, dt);
    rail_pid.output_limit = 0.8f; // motor max duty cycle

    float target_position = 0.1f; // meters (10 cm)
    float control;

    int steps = 5000; // simulate 5 seconds at 1 kHz

    for (int i = 0; i < steps; i++)
    {
        float t = i * dt;
        control = pid_update(&rail_pid, target_position, mbk_get_position());
        mbk_step(control);

        fprintf(f, "%f,%f,%f\n", t, mbk_get_position(), control);
    }

    fclose(f);
    printf("Simulation complete! log.csv generated.\n");
    return 0;
}
