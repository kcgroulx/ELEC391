#include "player_state_machine.h"

#include <math.h>

#include "config.h"
#include "motor_control.h"
#include "pid.h"
#include "platform_io.h"

namespace
{
constexpr float homingSpeed = 0.2f;
constexpr float homeDirection = -1.0f;
constexpr float reachToleranceDeg = 3.0f;
constexpr uint16_t settledTicksRequired = 5U;

typedef enum
{
    PLAY_PHASE_MOVE_TO_NOTE = 0,
    PLAY_PHASE_PRESS_NOTE,
    PLAY_PHASE_TRAVEL_TO_NEXT
} play_phase_t;

player_state_t currentState = PLAYER_STATE_INIT;
song_t* currentSong = nullptr;
bool playerIsHomed = false;
float currentTargetPosition = 0.0f;
uint16_t settledTicks = 0U;
uint32_t noteStartTick = 0U;
play_phase_t playPhase = PLAY_PHASE_MOVE_TO_NOTE;

float get_play_target_position(const song_state_t* currentNote)
{
    if ((playPhase == PLAY_PHASE_TRAVEL_TO_NEXT) && ((currentSong->count + 1U) < currentSong->num_states))
    {
        return currentSong->states[currentSong->count + 1U].targetPosition;
    }

    return currentNote->targetPosition;
}

void advance_to_next_note(void)
{
    settledTicks = 0U;
    noteStartTick = platform_io_millis();
    currentSong->count++;

    if (currentSong->count < currentSong->num_states)
    {
        playPhase = PLAY_PHASE_MOVE_TO_NOTE;
    }
    else
    {
        playPhase = PLAY_PHASE_MOVE_TO_NOTE;
        currentState = PLAYER_STATE_INIT;
    }
}

void handle_move_to_note(song_state_t* currentNote)
{
    uint32_t noteElapsedTime = platform_io_millis() - noteStartTick;
    currentTargetPosition = currentNote->targetPosition;

    const float angleErrorDeg =
        (currentTargetPosition * app_config::position_to_angle_deg_scale) - motor_control_get_angle_deg();
    const float absAngleErrorDeg = fabsf(angleErrorDeg);
    const float motorCommand = pid_step(currentTargetPosition);

    motor_control_set_motor_speed(motorCommand);
    platform_io_set_fingers(0U);

    if (absAngleErrorDeg <= reachToleranceDeg)
    {
        if (++settledTicks >= settledTicksRequired)
        {
            settledTicks = 0U;
            motor_control_set_motor_speed(0.0f);

            if (noteElapsedTime >= currentNote->noteDuration)
            {
                noteStartTick = platform_io_millis();
                noteElapsedTime = 0U;
            }

            if (noteElapsedTime < currentNote->pressDuration)
            {
                platform_io_set_fingers(currentNote->fingerBitmask);
                playPhase = PLAY_PHASE_PRESS_NOTE;
            }
            else
            {
                platform_io_set_fingers(0U);
                playPhase = PLAY_PHASE_TRAVEL_TO_NEXT;
            }
        }
    }
    else
    {
        settledTicks = 0U;
    }
}

void handle_press_note(song_state_t* currentNote)
{
    const uint32_t noteElapsedTime = platform_io_millis() - noteStartTick;

    motor_control_set_motor_speed(0.0f);
    platform_io_set_fingers(currentNote->fingerBitmask);

    if (noteElapsedTime >= currentNote->noteDuration)
    {
        platform_io_set_fingers(0U);
        advance_to_next_note();
    }
    else if (noteElapsedTime >= currentNote->pressDuration)
    {
        platform_io_set_fingers(0U);
        playPhase = PLAY_PHASE_TRAVEL_TO_NEXT;
    }
}

void handle_travel_to_next(song_state_t* currentNote)
{
    const uint32_t noteElapsedTime = platform_io_millis() - noteStartTick;
    currentTargetPosition = get_play_target_position(currentNote);
    platform_io_set_fingers(0U);

    if ((currentSong->count + 1U) < currentSong->num_states)
    {
        const float motorCommand = pid_step(currentTargetPosition);
        motor_control_set_motor_speed(motorCommand);
    }
    else
    {
        motor_control_set_motor_speed(0.0f);
    }

    if (noteElapsedTime >= currentNote->noteDuration)
    {
        advance_to_next_note();
    }
}
}

void player_state_machine_init(song_t* song)
{
    currentSong = song;
    currentState = PLAYER_STATE_INIT;
    playerIsHomed = false;
    currentTargetPosition = 0.0f;
    settledTicks = 0U;
    noteStartTick = 0U;
    playPhase = PLAY_PHASE_MOVE_TO_NOTE;
    platform_io_set_fingers(0U);
    pid_reset();
}

void player_state_machine_step(void)
{
    switch (currentState)
    {
        case PLAYER_STATE_INIT:
            if ((currentSong == nullptr) || (currentSong->states == nullptr) || (currentSong->num_states == 0U))
            {
                currentState = PLAYER_STATE_FAULT;
                break;
            }

            motor_control_set_motor_speed(0.0f);
            platform_io_set_fingers(0U);

            if (platform_io_is_user_button_active())
            {
                currentSong->count = 0U;
                playerIsHomed = false;
                settledTicks = 0U;
                noteStartTick = 0U;
                playPhase = PLAY_PHASE_MOVE_TO_NOTE;
                pid_reset();
                currentState = PLAYER_STATE_WAIT;
            }
            break;

        case PLAYER_STATE_WAIT:
            if ((currentSong == nullptr) || (currentSong->states == nullptr))
            {
                currentState = PLAYER_STATE_FAULT;
                break;
            }

            if (!playerIsHomed)
            {
                playerIsHomed = player_state_machine_home();
                settledTicks = 0U;
            }
            else
            {
                motor_control_set_motor_speed(0.0f);
                settledTicks = 0U;
                noteStartTick = 0U;
                platform_io_set_fingers(0U);

                if (platform_io_is_user_button_active())
                {
                    noteStartTick = platform_io_millis();
                    playPhase = PLAY_PHASE_MOVE_TO_NOTE;
                    pid_reset();
                    currentState = PLAYER_STATE_PLAY;
                }
            }
            break;

        case PLAYER_STATE_PLAY:
            if ((currentSong == nullptr) || (currentSong->states == nullptr) || (currentSong->count >= currentSong->num_states))
            {
                currentState = PLAYER_STATE_FAULT;
                break;
            }

            if (playPhase == PLAY_PHASE_MOVE_TO_NOTE)
            {
                handle_move_to_note(&currentSong->states[currentSong->count]);
            }
            else if (playPhase == PLAY_PHASE_PRESS_NOTE)
            {
                handle_press_note(&currentSong->states[currentSong->count]);
            }
            else
            {
                handle_travel_to_next(&currentSong->states[currentSong->count]);
            }
            break;

        case PLAYER_STATE_FAULT:
            motor_control_set_motor_speed(0.0f);
            platform_io_set_fingers(0U);
            break;

        default:
            currentState = PLAYER_STATE_FAULT;
            break;
    }
}

player_state_t player_state_machine_get_state(void)
{
    return currentState;
}

bool player_state_machine_home(void)
{
    if (platform_io_is_home_switch_active())
    {
        motor_control_set_motor_speed(0.0f);
        motor_control_zero_position();
        pid_reset();
        return true;
    }

    motor_control_set_motor_speed(homeDirection * homingSpeed);
    return false;
}
