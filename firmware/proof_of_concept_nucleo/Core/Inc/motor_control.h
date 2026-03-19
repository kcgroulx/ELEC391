/**
 * @file motor_control.h
 * @brief Public interface for motor PWM control and encoder angle tracking.
 */

#ifndef SRC_MOTOR_CONTROL_H_
#define SRC_MOTOR_CONTROL_H_

/* Includes */
#include <stdint.h>
#include <math.h>
#include "tim.h"

/* Defines */
#define ENCODER_CPR_MOTOR 64.0f
#define GEAR_RATIO 30.0f
#define ENCODER_CPR_OUTPUT (ENCODER_CPR_MOTOR * GEAR_RATIO)
#define LINEAR_TRAVEL_PER_REV 1.0f /* TODO: replace with the actual linear travel per output revolution. */
#define POSITION_TO_ANGLE_DEG_SCALE (360.0f / LINEAR_TRAVEL_PER_REV)
#define ANGLE_DEG_TO_POSITION_SCALE (LINEAR_TRAVEL_PER_REV / 360.0f)

/* Public Function Declarations */
void motor_control_init(void);
void motor_control_setMotorSpeed(float speed);
void motor_controller_encoderUpdatePosition(void);
float motor_controller_encoderGetAngleDeg(void);
float motor_controller_encoderGetLinearPosition(void);
void motor_controller_encoderZeroPosition(void);

#endif /* SRC_MOTOR_CONTROL_H_ */
