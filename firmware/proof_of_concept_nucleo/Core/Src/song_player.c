/**
 * @file song_player.c
 * @brief Logic for song playing
 *
 * Intended for use with STM32 platform
 */

/* Includes */
#include "song_player.h"
#include "main.h"
#include "motor_control.h"

/* Private Defines */
#define SONG_PLAYER_HOMING_SPEED 0.2f
#define SONG_PLAYER_HOME_DIRECTION (-1.0f) /* TODO: replace with the direction that moves toward home. */
#define SONG_PLAYER_LIMIT_SWITCH_GPIO_Port GPIOA /* TODO: replace with the actual limit switch GPIO port. */
#define SONG_PLAYER_LIMIT_SWITCH_Pin GPIO_PIN_0 /* TODO: replace with the actual limit switch pin. */
#define SONG_PLAYER_LIMIT_SWITCH_ACTIVE_STATE GPIO_PIN_SET /* TODO: replace with the switch's active state. */

/* Private Function Declarations */
static bool song_player_isHomeSwitchActive(void);

/* Private Function Definitions */
/**
 * @brief Read the home limit switch state.
 * @return true when the home switch is active.
 */
static bool song_player_isHomeSwitchActive(void)
{
    return HAL_GPIO_ReadPin(SONG_PLAYER_LIMIT_SWITCH_GPIO_Port, SONG_PLAYER_LIMIT_SWITCH_Pin) == SONG_PLAYER_LIMIT_SWITCH_ACTIVE_STATE;
}


/* Public Function Definitions */
/**
 * @brief Run a blocking homing sequence until the home limit switch is reached.
 */
void song_player_homing(void)
{
    // Move hand towards home
    motor_control_setMotorSpeed(SONG_PLAYER_HOME_DIRECTION * SONG_PLAYER_HOMING_SPEED);

    // Wait until homing switch is pressed
    while (!song_player_isHomeSwitchActive())
    {
    }

    // Stop motor and 
    motor_control_setMotorSpeed(0.0f);
    motor_controller_encoderZeroPosition();
}
