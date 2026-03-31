#ifndef PIANOBOT_MOTOR_CONTROL_H
#define PIANOBOT_MOTOR_CONTROL_H

#include <stdint.h>

void motor_control_init(void);
void motor_control_set_motor_speed(float speed);
void motor_control_update_encoder(void);
int32_t motor_control_get_position_counts(void);
float motor_control_get_angle_deg(void);
float motor_control_get_linear_position(void);
void motor_control_zero_position(void);

#endif
