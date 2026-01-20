#include <stdio.h>

/* Adjustable parameters (matches slide) */
float mass = 1.0f;      // kg
float damping = 2.0f;   // N·s/m
float stiffness = 0.0f; // N/m

/* State variables */
static float position = 0.0f;
static float velocity = 0.0f;

/* Time step */
static float dt = 0.001f; // 1 kHz

/* Simulate one step */
void mbk_step(float input_force)
{
    float acceleration =
        (input_force - damping * velocity - stiffness * position) / mass;

    velocity += acceleration * dt;
    position += velocity * dt;
}

float mbk_get_position(void)
{
    return position;
}

float mbk_get_velocity(void)
{
    return velocity;
}
