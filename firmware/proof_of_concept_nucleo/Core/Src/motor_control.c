/**
 * @file motor_control.c
 * @brief Motor drive and encoder positioning.
 *
 * Intended for use with STM32 platform
 */

/* Includes */
#include "motor_control.h"

/* Private Variables */
motor_control_config_t motor_control_config;

static int16_t enc_last_count = 0;
static int32_t enc_position_counts = 0;

/* Private Function Declarations */

void motor_control_setPWMDutyCycle(TIM_HandleTypeDef* htim, uint32_t Channel, float dutyCycle);

/* Private Function Definitions */

/**
 * @brief Set the PWM duty cycle (0.0 - 1.0) for a given half-bridge.
 * @param htim Timer handle for the PWM output.
 * @param Channel TIM channel to update.
 * @param dutyCycle Percent duty cycle; values >1.0 are clamped to 1.0.
 */
void motor_control_setPWMDutyCycle(TIM_HandleTypeDef* htim, uint32_t Channel, float dutyCycle)
{
    // Ensure that dutyCycle is < 100
    dutyCycle = (dutyCycle > 1.0) ? 1.0 : dutyCycle;
    dutyCycle = (dutyCycle < 0.0) ? 0.0 : dutyCycle;

    // Get ARR value
    uint32_t ARR = __HAL_TIM_GET_AUTORELOAD(htim);

    // Calculate CRR for the requested duty cycle
    uint32_t CRR = (uint32_t)((float)ARR * dutyCycle);

    // Set CRR
    __HAL_TIM_SET_COMPARE(htim, Channel, CRR);
}

/* Public Function Definitions */

/**
 * @brief Initialize the two half-bridge PWM channels used by the motor driver.
 * @param config Struct that contains the htims for the halfbridge PWMs and encoder
 */
void motor_control_init(motor_control_config_t* config)
{
    motor_control_config = *config;

    HAL_TIM_PWM_Start(motor_control_config.halfBridge1Htim, motor_control_config.halfBridge1Channel);
    HAL_TIM_PWM_Start(motor_control_config.halfBridge2Htim, motor_control_config.halfBridge2Channel);
    HAL_TIM_Encoder_Start(motor_control_config.encoderHtim, TIM_CHANNEL_ALL);

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
        motor_control_setPWMDutyCycle(motor_control_config.halfBridge1Htim, motor_control_config.halfBridge1Channel, fabs(speed));
        motor_control_setPWMDutyCycle(motor_control_config.halfBridge2Htim, motor_control_config.halfBridge2Channel, 0);
    }
    else if (speed < 0)
    {
        motor_control_setPWMDutyCycle(motor_control_config.halfBridge1Htim, motor_control_config.halfBridge1Channel, 0);
        motor_control_setPWMDutyCycle(motor_control_config.halfBridge2Htim, motor_control_config.halfBridge2Channel, fabs(speed));
    }
    else
    {
        motor_control_setPWMDutyCycle(motor_control_config.halfBridge1Htim, motor_control_config.halfBridge1Channel, 0);
        motor_control_setPWMDutyCycle(motor_control_config.halfBridge2Htim, motor_control_config.halfBridge2Channel, 0);
    }   
}


/**
 * @brief Update accumulated encoder position counts from the current timer value.
 */
void motor_controller_encoderUpdatePosition(void)
{
    int16_t enc_current_count = (int16_t)__HAL_TIM_GET_COUNTER(motor_control_config.encoderHtim);

    int16_t enc_delta_count = (int16_t)(enc_current_count - enc_last_count);

    enc_last_count = enc_current_count;
    enc_position_counts += (int32_t)enc_delta_count;
}


/**
 * @brief Get the shaft angle in degrees from accumulated encoder counts.
 * @return Mechanical angle in degrees.
 */
float motor_controller_encoderGetAngleDeg(void)
{
    return ((float)enc_position_counts / ENCODER_CPR_OUTPUT) * 360.0f;

}
