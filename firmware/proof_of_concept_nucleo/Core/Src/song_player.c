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
#include "pid.h"

/* Private Defines */
#define SONG_PLAYER_HOMING_SPEED 0.2f
#define SONG_PLAYER_HOME_DIRECTION (-1.0f) /* TODO: Check homing direction. */
#define SONG_PLAYER_LIMIT_SWITCH_ACTIVE_STATE GPIO_PIN_SET /* TODO: Check if active low/high. */
#define SONG_PLAYER_USER_BUTTON_ACTIVE_STATE GPIO_PIN_SET /* TODO: Check if active low/high. */
#define SONG_PLAYER_FINGER_ACTIVE_STATE GPIO_PIN_SET /* TODO: Check if active low/high. */
#define SONG_PLAYER_REACH_TOLERANCE_DEG 3.0f
#define SONG_PLAYER_SETTLED_TICKS_REQUIRED 5U

typedef enum
{
    SONG_PLAYER_PLAY_MOVE_TO_NOTE = 0,
    SONG_PLAYER_PLAY_PRESS_NOTE,
    SONG_PLAYER_PLAY_TRAVEL_TO_NEXT
} song_player_play_phase_t;

/* Private Variables */
static piano_bot_state currentState = INIT;
static song_t* currentSong = 0;
static bool songPlayerIsHomed = false;
static float currentTargetPosition = 0.0f;
static uint16_t settledTicks = 0U;
static uint32_t noteStartTick = 0U;
static song_player_play_phase_t playPhase = SONG_PLAYER_PLAY_MOVE_TO_NOTE;

/* Private Function Declarations */
static bool song_player_isHomeSwitchActive(void);
static void song_player_setFingers(uint8_t fingerBitmask);
static float song_player_getPlayTargetPosition(const song_state_t* currentNote);
static void song_player_handleMoveToNote(song_state_t* currentNote);
static void song_player_handlePressNote(song_state_t* currentNote);
static void song_player_handleTravelToNext(song_state_t* currentNote);
static void song_player_advanceToNextNote(void);

/* Private Function Definitions */
/**
 * @brief Read the home limit switch state.
 * @return true when the home switch is active.
 */
static bool song_player_isHomeSwitchActive(void)
{
    return HAL_GPIO_ReadPin(LIMIT1_GPIO_Port, LIMIT1_Pin) == SONG_PLAYER_LIMIT_SWITCH_ACTIVE_STATE;
}

/**
 * @brief Set the finger solenoid outputs from a bitmask.
 * @param fingerBitmask Bitmask of fingers to actuate, where bit 0 maps to solenoid 1.
 */
static void song_player_setFingers(uint8_t fingerBitmask)
{
    HAL_GPIO_WritePin(SOLENOID_1_GPIO_Port, SOLENOID_1_Pin, (fingerBitmask & (1U << 0)) ? SONG_PLAYER_FINGER_ACTIVE_STATE : (GPIO_PinState)!SONG_PLAYER_FINGER_ACTIVE_STATE);
    HAL_GPIO_WritePin(SOLENOID_2_GPIO_Port, SOLENOID_2_Pin, (fingerBitmask & (1U << 1)) ? SONG_PLAYER_FINGER_ACTIVE_STATE : (GPIO_PinState)!SONG_PLAYER_FINGER_ACTIVE_STATE);
    HAL_GPIO_WritePin(SOLENOID_3_GPIO_Port, SOLENOID_3_Pin, (fingerBitmask & (1U << 2)) ? SONG_PLAYER_FINGER_ACTIVE_STATE : (GPIO_PinState)!SONG_PLAYER_FINGER_ACTIVE_STATE);
    HAL_GPIO_WritePin(SOLENOID_4_GPIO_Port, SOLENOID_4_Pin, (fingerBitmask & (1U << 3)) ? SONG_PLAYER_FINGER_ACTIVE_STATE : (GPIO_PinState)!SONG_PLAYER_FINGER_ACTIVE_STATE);
    HAL_GPIO_WritePin(SOLENOID_5_GPIO_Port, SOLENOID_5_Pin, (fingerBitmask & (1U << 4)) ? SONG_PLAYER_FINGER_ACTIVE_STATE : (GPIO_PinState)!SONG_PLAYER_FINGER_ACTIVE_STATE);
    HAL_GPIO_WritePin(SOLENOID_6_GPIO_Port, SOLENOID_6_Pin, (fingerBitmask & (1U << 5)) ? SONG_PLAYER_FINGER_ACTIVE_STATE : (GPIO_PinState)!SONG_PLAYER_FINGER_ACTIVE_STATE);
}

/**
 * @brief Get the active motion target for the current play phase.
 * @param currentNote Current note being played.
 * @return Linear target position for the current phase.
 */
static float song_player_getPlayTargetPosition(const song_state_t* currentNote)
{
    if ((playPhase == SONG_PLAYER_PLAY_TRAVEL_TO_NEXT) && ((currentSong->count + 1U) < currentSong->num_states))
    {
        return currentSong->states[currentSong->count + 1U].targetPosition;
    }

    return currentNote->targetPosition;
}

/**
 * @brief Move the hand to the current note position and wait until it settles.
 * @param currentNote Current note being played.
 */
static void song_player_handleMoveToNote(song_state_t* currentNote)
{
    uint32_t noteElapsedTime = HAL_GetTick() - noteStartTick;
    float angle_error;
    float abs_angle_error;
    float motorCommand;

    currentTargetPosition = currentNote->targetPosition;
    angle_error = (currentTargetPosition * POSITION_TO_ANGLE_DEG_SCALE) - motor_controller_encoderGetAngleDeg();
    abs_angle_error = (angle_error < 0.0f) ? -angle_error : angle_error;
    motorCommand = cascaded_control_step(currentTargetPosition);

    motor_control_setMotorSpeed(motorCommand);
    song_player_setFingers(0U);

    if (abs_angle_error <= SONG_PLAYER_REACH_TOLERANCE_DEG)
    {
        if (++settledTicks >= SONG_PLAYER_SETTLED_TICKS_REQUIRED)
        {
            settledTicks = 0U;
            motor_control_setMotorSpeed(0.0f);

            /*
             * If the carriage arrives after this note's scheduled duration,
             * restart the note clock so the note is played late instead of skipped.
             */
            if (noteElapsedTime >= currentNote->noteDuration)
            {
                noteStartTick = HAL_GetTick();
                noteElapsedTime = 0U;
            }

            if (noteElapsedTime < currentNote->pressDuration)
            {
                song_player_setFingers(currentNote->fingerBitmask);
                playPhase = SONG_PLAYER_PLAY_PRESS_NOTE;
            }
            else
            {
                song_player_setFingers(0U);
                playPhase = SONG_PLAYER_PLAY_TRAVEL_TO_NEXT;
            }
        }
    }
    else
    {
        settledTicks = 0U;
    }
}

/**
 * @brief Hold the note pressed for the configured press duration.
 * @param currentNote Current note being played.
 */
static void song_player_handlePressNote(song_state_t* currentNote)
{
    uint32_t noteElapsedTime = HAL_GetTick() - noteStartTick;

    motor_control_setMotorSpeed(0.0f);
    song_player_setFingers(currentNote->fingerBitmask);

    if (noteElapsedTime >= currentNote->noteDuration)
    {
        song_player_setFingers(0U);
        song_player_advanceToNextNote();
    }
    else if (noteElapsedTime >= currentNote->pressDuration)
    {
        song_player_setFingers(0U);
        playPhase = SONG_PLAYER_PLAY_TRAVEL_TO_NEXT;
    }
}

/**
 * @brief Use the remaining note time to travel toward the next note.
 * @param currentNote Current note being played.
 */
static void song_player_handleTravelToNext(song_state_t* currentNote)
{
    uint32_t noteElapsedTime = HAL_GetTick() - noteStartTick;
    float motorCommand;

    currentTargetPosition = song_player_getPlayTargetPosition(currentNote);
    song_player_setFingers(0U);

    if ((currentSong->count + 1U) < currentSong->num_states)
    {
        motorCommand = cascaded_control_step(currentTargetPosition);
        motor_control_setMotorSpeed(motorCommand);
    }
    else
    {
        motor_control_setMotorSpeed(0.0f);
    }

    if (noteElapsedTime >= currentNote->noteDuration)
    {
        song_player_advanceToNextNote();
    }
}

/**
 * @brief Advance song playback to the next note and restart the note clock.
 */
static void song_player_advanceToNextNote(void)
{
    settledTicks = 0U;
    noteStartTick = HAL_GetTick();
    currentSong->count++;

    if (currentSong->count < currentSong->num_states)
    {
        playPhase = SONG_PLAYER_PLAY_MOVE_TO_NOTE;
    }
    else
    {
        playPhase = SONG_PLAYER_PLAY_MOVE_TO_NOTE;
        currentState = INIT;
    }
}


/* Public Function Definitions */
/**
 * @brief Initialize the song player state machine.
 * @param song Song to be used by the player state machine.
 */
void song_player_init(song_t* song)
{
    currentSong = song;
    currentState = INIT;
    songPlayerIsHomed = false;
    currentTargetPosition = 0.0f;
    settledTicks = 0U;
    noteStartTick = 0U;
    playPhase = SONG_PLAYER_PLAY_MOVE_TO_NOTE;
    song_player_setFingers(0U);
}

/**
 * @brief Advance the song player state machine by one step.
 */
void song_player_step(void)
{
    (void)currentSong;

    switch (currentState)
    {
        case INIT:
            if ((currentSong == NULL) || (currentSong->states == NULL) || (currentSong->num_states == 0U))
            {
                currentState = UHOH;
                break;
            }

            motor_control_setMotorSpeed(0.0f);
            song_player_setFingers(0U);

            if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == SONG_PLAYER_USER_BUTTON_ACTIVE_STATE)
            {
                currentSong->count = 0U;
                songPlayerIsHomed = false;
                settledTicks = 0U;
                noteStartTick = 0U;
                playPhase = SONG_PLAYER_PLAY_MOVE_TO_NOTE;
                currentState = WAIT;
            }
            break;

        case WAIT:
            if ((currentSong == NULL) || (currentSong->states == NULL))
            {
                currentState = UHOH;
                break;
            }

            if (!songPlayerIsHomed)
            {
                songPlayerIsHomed = song_player_homing();
                settledTicks = 0U;
            }
            else
            {
                motor_control_setMotorSpeed(0.0f);
                settledTicks = 0U;
                noteStartTick = 0U;
                song_player_setFingers(0U);

                if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) == SONG_PLAYER_USER_BUTTON_ACTIVE_STATE)
                {
                    noteStartTick = HAL_GetTick();
                    playPhase = SONG_PLAYER_PLAY_MOVE_TO_NOTE;
                    currentState = PLAY;
                }
            }
            break;

        case PLAY:
            if ((currentSong == NULL) || (currentSong->states == NULL) || (currentSong->count >= currentSong->num_states))
            {
                currentState = UHOH;
                break;
            }

            {
                song_state_t* currentNote;

                currentNote = &currentSong->states[currentSong->count];

                if (playPhase == SONG_PLAYER_PLAY_MOVE_TO_NOTE)
                {
                    song_player_handleMoveToNote(currentNote);
                }
                else if (playPhase == SONG_PLAYER_PLAY_PRESS_NOTE)
                {
                    song_player_handlePressNote(currentNote);
                }
                else
                {
                    song_player_handleTravelToNext(currentNote);
                }
            }
            break;

        case UHOH:
            /* TODO: Handle fault state and recovery behavior. */
            motor_control_setMotorSpeed(0.0f);
            song_player_setFingers(0U);
            break;

        default:
            currentState = UHOH;
            break;
    }
}

/**
 * @brief Get the current song player state.
 * @return Current piano bot state.
 */
piano_bot_state song_player_getState(void)
{
    return currentState;
}

/**
 * @brief Advance the homing sequence by one step.
 * @return true when the home position has been reached.
 */
bool song_player_homing(void)
{
    if (song_player_isHomeSwitchActive())
    {
        motor_control_setMotorSpeed(0.0f);
        motor_controller_encoderZeroPosition();
        return true;
    }

    motor_control_setMotorSpeed(SONG_PLAYER_HOME_DIRECTION * SONG_PLAYER_HOMING_SPEED);
    return false;
}
