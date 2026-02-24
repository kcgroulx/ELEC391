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
#define ENCODER_CPR_MOTOR   32.0f
#define GEAR_RATIO          30.0f
#define ENCODER_CPR_OUTPUT  (ENCODER_CPR_MOTOR * GEAR_RATIO)

/* Typedefs */
typedef struct HalfBridgePWM_t {
    TIM_HandleTypeDef *htim;
    uint32_t Channel;
} HalfBridgePWM_t;

typedef struct motor_control_config_t {
    TIM_HandleTypeDef *halfBridge1Htim;
    TIM_HandleTypeDef *halfBridge2Htim;
    TIM_HandleTypeDef *encoderHtim;
    uint32_t halfBridge1Channel;
    uint32_t halfBridge2Channel;
} motor_control_config_t;

/* Public Function Declarations */
void motor_control_init(motor_control_config_t* config);
void motor_control_setMotorSpeed(float speed);
void motor_controller_encoderUpdatePosition(void);
float motor_controller_encoderGetAngleDeg(void);
int32_t motor_controller_encoderGetPositionCounts(void);

#endif /* SRC_MOTOR_CONTROL_H_ */
