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
#include "tim.h"

/* Typedefs */
typedef struct HalfBridgePWM {
    TIM_HandleTypeDef *htim;
    uint32_t Channel;
} HalfBridgePWM;

typedef enum motorDirection {
    OFF,
    FORWARD,
    REVERSE
} motorDirection;

/* Public Function Declarations */
void motor_control_init(TIM_HandleTypeDef *htim1, uint32_t Channel1, TIM_HandleTypeDef *htim2, uint32_t Channel2);
void motor_control_setMotorSpeed(motorDirection direction, uint8_t speed);

#endif /* SRC_MOTOR_CONTROL_H_ */
