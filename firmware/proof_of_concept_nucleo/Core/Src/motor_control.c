/**
 * @file motor_control.c
 * @brief Motor drive and encoder positioning.
 *
 * Intended for use with STM32 platform
 */

/* Includes */
#include "motor_control.h"

/* Private Defines */
#define MOTOR_HALFBRIDGE1_HTIM (&htim3)
#define MOTOR_HALFBRIDGE2_HTIM (&htim3)
#define MOTOR_ENCODER_HTIM (&htim1)
#define MOTOR_HALFBRIDGE1_CHANNEL TIM_CHANNEL_2
#define MOTOR_HALFBRIDGE2_CHANNEL TIM_CHANNEL_3

/* Private Variables */
static int16_t enc_last_count = 0;
static int32_t enc_position_counts = 0;

/* Private Function Definitions */

/**
 * @brief Set the PWM duty cycle (0.0 - 1.0) for a given half-bridge.
 * @param htim Timer handle for the PWM output.
 * @param Channel TIM channel to update.
 * @param dutyCycle Percent duty cycle; values >1.0 are clamped to 1.0.
 */
static void motor_control_setPWMDutyCycle(TIM_HandleTypeDef* htim, uint32_t Channel, float dutyCycle)
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
 */
void motor_control_init(void)
{
    HAL_TIM_PWM_Start(MOTOR_HALFBRIDGE1_HTIM, MOTOR_HALFBRIDGE1_CHANNEL);
    HAL_TIM_PWM_Start(MOTOR_HALFBRIDGE2_HTIM, MOTOR_HALFBRIDGE2_CHANNEL);
    HAL_TIM_Encoder_Start(MOTOR_ENCODER_HTIM, TIM_CHANNEL_ALL);

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
        motor_control_setPWMDutyCycle(MOTOR_HALFBRIDGE1_HTIM, MOTOR_HALFBRIDGE1_CHANNEL, fabs(speed));
        motor_control_setPWMDutyCycle(MOTOR_HALFBRIDGE2_HTIM, MOTOR_HALFBRIDGE2_CHANNEL, 0);
    }
    else if (speed < 0)
    {
        motor_control_setPWMDutyCycle(MOTOR_HALFBRIDGE1_HTIM, MOTOR_HALFBRIDGE1_CHANNEL, 0);
        motor_control_setPWMDutyCycle(MOTOR_HALFBRIDGE2_HTIM, MOTOR_HALFBRIDGE2_CHANNEL, fabs(speed));
    }
    else
    {
        motor_control_setPWMDutyCycle(MOTOR_HALFBRIDGE1_HTIM, MOTOR_HALFBRIDGE1_CHANNEL, 0);
        motor_control_setPWMDutyCycle(MOTOR_HALFBRIDGE2_HTIM, MOTOR_HALFBRIDGE2_CHANNEL, 0);
    }   
}


/**
 * @brief Update accumulated encoder position counts from the current timer value.
 */
void motor_controller_encoderUpdatePosition(void)
{
    int16_t enc_current_count = (int16_t)__HAL_TIM_GET_COUNTER(MOTOR_ENCODER_HTIM);

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

/**
 * @brief Get the current linear position from the encoder angle.
 * @return Linear position in the same units as LINEAR_TRAVEL_PER_REV.
 */
float motor_controller_encoderGetLinearPosition(void)
{
    return motor_controller_encoderGetAngleDeg() * ANGLE_DEG_TO_POSITION_SCALE;
}

/**
 * @brief Reset the accumulated encoder position count to zero.
 */
void motor_controller_encoderZeroPosition(void)
{
    enc_position_counts = 0;
}
