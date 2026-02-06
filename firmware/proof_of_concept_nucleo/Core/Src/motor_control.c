/*
 * motor_control.c
 *
 *  Created on: Feb 5, 2026
 *      Author: Kyle Groulx
 */

/* Includes */
#include "motor_control.h"

/* Private Variables */
HalfBridgePWM halfBridge1;
HalfBridgePWM halfBridge2;

/* Private Function Declarations */
void motor_control_setPWMDutyCycle(HalfBridgePWM pwm, uint8_t dutyCycle);

/* Private Function Definitions */

/**
 * @brief Set the PWM duty cycle (0-100%) for a given half-bridge.
 * @param pwm Half-bridge timer + channel to update.
 * @param dutyCycle Percent duty cycle; values >100 are clamped to 100.
 */
void motor_control_setPWMDutyCycle(HalfBridgePWM pwm, uint8_t dutyCycle)
{
    // Ensure that dutyCycle is < 100
    dutyCycle = (dutyCycle > 100) ? 100 : dutyCycle;

    // Get ARR value
    uint32_t ARR = __HAL_TIM_GET_AUTORELOAD(pwm.htim);

    // Calculate CRR for the requested duty cycle
    uint32_t CRR = (dutyCycle / 100) * ARR;
    
    // Set CRR
    __HAL_TIM_SET_COMPARE(pwm.htim, pwm.Channel, CRR);
}

/* Public Function Definitions */

/**
 * @brief Initialize the two half-bridge PWM channels used by the motor driver.
 * @param htim1 Timer handle for half-bridge 1.
 * @param Channel1 TIM_CHANNEL_x for half-bridge 1.
 * @param htim2 Timer handle for half-bridge 2.
 * @param Channel2 TIM_CHANNEL_x for half-bridge 2.
 */
void motor_control_init(TIM_HandleTypeDef *htim1, uint32_t Channel1, TIM_HandleTypeDef *htim2, uint32_t Channel2)
{
    halfBridge1.htim = htim1;
    halfBridge1.Channel = Channel1;

    halfBridge2.htim = htim2;
    halfBridge2.Channel = Channel2;

    HAL_TIM_PWM_Start(halfBridge1.htim, halfBridge1.Channel);
    HAL_TIM_PWM_Start(halfBridge2.htim, halfBridge2.Channel);

    motor_control_setMotorSpeed(OFF, 0);
}

/**
 * @brief Drive the motor in the requested direction at the requested speed.
 * @param direction OFF, FORWARD, or REVERSE.
 * @param speed Percent duty cycle (0-100).
 */
void motor_control_setMotorSpeed(motorDirection direction, uint8_t speed)
{
    switch (direction)
    {
        case FORWARD: // HalfBridge1 = PMW : HalfBridge2 = 0
            motor_control_setPWMDutyCycle(halfBridge1, speed);
            motor_control_setPWMDutyCycle(halfBridge2, 0);
            break;

        case REVERSE: // HalfBridge1 = 0 : HalfBridge2 = PWM
            motor_control_setPWMDutyCycle(halfBridge1, 0);
            motor_control_setPWMDutyCycle(halfBridge2, speed);
            break;

        default: // Other or OFF
            motor_control_setPWMDutyCycle(halfBridge1, 0);
            motor_control_setPWMDutyCycle(halfBridge2, 0);
            break;
    }    
}

