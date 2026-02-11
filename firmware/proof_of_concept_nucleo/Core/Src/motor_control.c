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

static int16_t  enc_last_count = 0;
static int32_t  enc_position_counts = 0;

/* Private Function Declarations */
void motor_control_setPWMDutyCycle(HalfBridgePWM pwm, float dutyCycle);

/* Private Function Definitions */

/**
 * @brief Set the PWM duty cycle (0.0 - 1.0) for a given half-bridge.
 * @param pwm Half-bridge timer + channel to update.
 * @param dutyCycle Percent duty cycle; values >1.0 are clamped to 1.0.
 */
void motor_control_setPWMDutyCycle(HalfBridgePWM pwm, float dutyCycle)
{
    // Ensure that dutyCycle is < 100
    dutyCycle = (dutyCycle > 1.0) ? 1.0 : dutyCycle;
    dutyCycle = (dutyCycle < 0.0) ? 0.0 : dutyCycle;

    // Get ARR value
    uint32_t ARR = __HAL_TIM_GET_AUTORELOAD(pwm.htim);

    // Calculate CRR for the requested duty cycle
    uint32_t CRR = (uint32_t)((float)ARR * dutyCycle);

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

    motor_control_setMotorSpeed(0.0);
}

/**
 * @brief Drive the motor in the requested direction at the requested speed.
 * @param speed Percent duty cycle -1.0 - 1.0.
 */
void motor_control_setMotorSpeed(float speed)
{
    if(speed > 0)
    {
        motor_control_setPWMDutyCycle(halfBridge1, fabs(speed));
        motor_control_setPWMDutyCycle(halfBridge2, 0);
    }
    else if (speed < 0)
    {
        motor_control_setPWMDutyCycle(halfBridge1, 0);
        motor_control_setPWMDutyCycle(halfBridge2, fabs(speed));
    }
    else
    {
        motor_control_setPWMDutyCycle(halfBridge1, 0);
        motor_control_setPWMDutyCycle(halfBridge2, 0);
    }   
}


void encoder_update_position(TIM_HandleTypeDef *htim)
{
    int16_t now = (int16_t)__HAL_TIM_GET_COUNTER(htim);

    // Wrap-safe delta
    int16_t delta = (int16_t)(now - enc_last_count);

    enc_last_count = now;
    enc_position_counts += (int32_t)delta;
}


float encoder_get_angle_deg(void)
{

    return ((float)enc_position_counts / ENCODER_CPR_OUTPUT) * 360.0f;
}
