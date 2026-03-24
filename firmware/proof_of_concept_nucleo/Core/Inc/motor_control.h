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
#define LINEAR_TRAVEL_PER_REV 37.7f // 2*pi*radius, radius = 6mm pulley  // CONFIRM THIS MEASUREMENT

/* Public Function Declarations */
void motor_control_init(void);
void motor_control_setMotorSpeed(float speed);
void motor_controller_encoderUpdatePosition(void);
float motor_controller_encoderGetAngleDeg(void);
/**
 * @brief Get the current linear position from the encoder angle.
 * @return Linear position in the same units as LINEAR_TRAVEL_PER_REV.
 */
float motor_controller_encoderGetLinearPosition(void);
/**
 * @brief Reset the accumulated encoder position count to zero.
 */
void motor_controller_encoderZeroPosition(void);

#endif /* SRC_MOTOR_CONTROL_H_ */
