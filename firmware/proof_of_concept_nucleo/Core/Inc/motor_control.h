/*
 * motor_control.h
 *
 *  Created on: Feb 5, 2026
 *      Author: Kyle Groulx
 */

#ifndef SRC_MOTOR_CONTROL_H_
#define SRC_MOTOR_CONTROL_H_

/* Includes */
#include <stdint.h>
#include <math.h>
#include "tim.h"

/* Defines */
#define ENCODER_CPR_MOTOR   64.0f
#define GEAR_RATIO          30.0f
#define ENCODER_CPR_OUTPUT  (ENCODER_CPR_MOTOR * GEAR_RATIO)

/* Typedefs */
typedef struct HalfBridgePWM {
    TIM_HandleTypeDef *htim;
    uint32_t Channel;
} HalfBridgePWM;

typedef enum motorDirection {
    FORWARD = 0,
    REVERSE = 1,
    OFF = 2
} motorDirection;

/* Public Function Declarations */
void motor_control_init(TIM_HandleTypeDef *htim1, uint32_t Channel1, TIM_HandleTypeDef *htim2, uint32_t Channel2);
void motor_control_setMotorSpeed(float speed);
void encoder_update_position(TIM_HandleTypeDef *htim);


#endif /* SRC_MOTOR_CONTROL_H_ */
