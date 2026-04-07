#include "planned_song_player.h"

#include "Arduino.h"
#include "config.h"
#include "debug_log.h"
#include "hal_interface.h"
#include "motor_control.h"
#include "platform_io.h"
#include "song_player.h"

namespace
{
#define PLANNED_SONG_UART_TIMEOUT_MS     5000U
#define PLANNED_SONG_BUTTON_DEBOUNCE_MS   200U
#define PLANNED_SONG_WIRE_MAGIC_0         'P'
#define PLANNED_SONG_WIRE_MAGIC_1         'L'
#define PLANNED_SONG_WIRE_MAGIC_2         'N'
#define PLANNED_SONG_WIRE_MAGIC_3         '1'

enum PlannedSongRunState
{
    PLANNED_SONG_WAITING_FOR_UPLOAD = 0,
    PLANNED_SONG_PREPOSITION,
    PLANNED_SONG_PRESS,
    PLANNED_SONG_TRAVEL,
    PLANNED_SONG_PAUSED,
    PLANNED_SONG_HOMING,
    PLANNED_SONG_DONE,
    PLANNED_SONG_FAULTED
};

enum PlannedSongReceiveResult
{
    PLANNED_SONG_RX_NONE = 0,
    PLANNED_SONG_RX_COMMAND_ONLY,
    PLANNED_SONG_RX_SONG_READY,
    PLANNED_SONG_RX_INVALID
};

static PlannedSongRunState s_runState = PLANNED_SONG_WAITING_FOR_UPLOAD;
static PlannedSongRunState s_previousState = PLANNED_SONG_WAITING_FOR_UPLOAD;
static PlannedSongRunState s_resumeState = PLANNED_SONG_PREPOSITION;
static PlannedSongStep s_uploadedSong[PLANNED_SONG_MAX_STEPS];
static uint16_t s_uploadedSongLen = 0U;
static uint16_t s_stepIndex = 0U;
static uint32_t s_deadlineTick = 0U;
static uint32_t s_pauseStartTick = 0U;
static uint32_t s_buttonDebounceUntilTick = 0U;
static uint32_t s_stateEntryTick = 0U;
static uint32_t s_uploadReceiveStartTick = 0U;
static uint32_t s_uploadReceiveDoneTick = 0U;
static uint32_t s_songBeginTick = 0U;
static uint32_t s_prepositionStartTick = 0U;
static bool s_buttonPrevState = false;
static bool s_resumeNeedsRepress = false;

const __FlashStringHelper* plannedSongStateName(PlannedSongRunState state)
{
    switch (state) {
        case PLANNED_SONG_WAITING_FOR_UPLOAD:
            return F("WAITING");
        case PLANNED_SONG_PREPOSITION:
            return F("PREPOSITION");
        case PLANNED_SONG_PRESS:
            return F("PRESS");
        case PLANNED_SONG_TRAVEL:
            return F("TRAVEL");
        case PLANNED_SONG_PAUSED:
            return F("PAUSED");
        case PLANNED_SONG_HOMING:
            return F("HOMING");
        case PLANNED_SONG_DONE:
            return F("DONE");
        case PLANNED_SONG_FAULTED:
            return F("FAULTED");
        default:
            return F("UNKNOWN");
    }
}

void logStateTransition(PlannedSongRunState previousState, PlannedSongRunState nextState, uint32_t now)
{
    DEBUG_PRINT(F("[PLANDBG] state="));
    DEBUG_PRINT(plannedSongStateName(nextState));
    DEBUG_PRINT(F(" tick="));
    DEBUG_PRINT(now);
    DEBUG_PRINT(F("ms prev="));
    DEBUG_PRINT(plannedSongStateName(previousState));
    DEBUG_PRINT(F(" prev_ms="));
    DEBUG_PRINT(now - s_stateEntryTick);
    DEBUG_PRINT(F(" step="));
    if (s_uploadedSongLen == 0U) {
        DEBUG_PRINT(0U);
    } else {
        DEBUG_PRINT((uint32_t)s_stepIndex + 1U);
    }
    DEBUG_PRINT(F("/"));
    DEBUG_PRINT(s_uploadedSongLen);
    DEBUG_PRINTLN(F(""));
}

bool plannedFaultActive(void)
{
    return hal_isEStopped()
        || (hal_isHomeSwitchFaultActive() != 0)
        || (hal_isFarLimitFaultActive() != 0);
}

void plannedAbortForFault(void)
{
    motor_control_set_motor_speed(0.0f);
    hal_fingerReleaseAll();
}

bool readByteWithTimeout(int* outByte, uint32_t* deadlineTick, uint32_t timeoutMs)
{
    while ((int32_t)(*deadlineTick - (uint32_t)millis()) > 0) {
        hal_runPendingPID();
        if (Serial.available() > 0) {
            *outByte = Serial.read();
            *deadlineTick = (uint32_t)millis() + timeoutMs;
            return true;
        }
    }
    *outByte = -1;
    return false;
}

void discardAvailableInput(void)
{
    while (Serial.available() > 0) {
        (void)Serial.read();
    }
}

bool parsePwmCommand(uint32_t* deadlineTick, uint32_t timeoutMs)
{
    int percent = 0;
    int haveDigits = 0;
    int ch = -1;

    while (readByteWithTimeout(&ch, deadlineTick, timeoutMs)) {
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            break;
        }
        if (ch >= '0' && ch <= '9') {
            percent = (percent * 10) + (ch - '0');
            haveDigits = 1;
            if (percent > 100) {
                percent = 100;
            }
        }
    }

    if (!haveDigits) {
        return false;
    }

    platform_io_set_finger_pressed_duty((float)percent / 100.0f);
    return true;
}

PlannedSongReceiveResult receiveUploadedSong(uint32_t timeoutMs)
{
    uint32_t deadlineTick;
    int b0;
    int b1;
    int b2;
    int b3;
    int countHi;
    int countLo;
    uint16_t stepCount;
    uint16_t index;

    if (Serial.available() <= 0) {
        return PLANNED_SONG_RX_NONE;
    }

    deadlineTick = (uint32_t)millis() + timeoutMs;
    if (!readByteWithTimeout(&b0, &deadlineTick, timeoutMs)
            || !readByteWithTimeout(&b1, &deadlineTick, timeoutMs)
            || !readByteWithTimeout(&b2, &deadlineTick, timeoutMs)
            || !readByteWithTimeout(&b3, &deadlineTick, timeoutMs)) {
        return PLANNED_SONG_RX_INVALID;
    }

    if (b0 == 'P' && b1 == 'W' && b2 == 'M' && b3 == ' ') {
        DEBUG_PRINT(F("[PLANDBG] rx_pwm tick="));
        DEBUG_PRINT((uint32_t)millis());
        DEBUG_PRINTLN(F("ms"));
        return parsePwmCommand(&deadlineTick, timeoutMs)
            ? PLANNED_SONG_RX_COMMAND_ONLY
            : PLANNED_SONG_RX_INVALID;
    }

    if (b0 != PLANNED_SONG_WIRE_MAGIC_0
            || b1 != PLANNED_SONG_WIRE_MAGIC_1
            || b2 != PLANNED_SONG_WIRE_MAGIC_2
            || b3 != PLANNED_SONG_WIRE_MAGIC_3) {
        DEBUG_PRINT(F("[PLANDBG] rx_invalid_header tick="));
        DEBUG_PRINT((uint32_t)millis());
        DEBUG_PRINTLN(F("ms"));
        discardAvailableInput();
        return PLANNED_SONG_RX_INVALID;
    }

    s_uploadReceiveStartTick = (uint32_t)millis();

    if (!readByteWithTimeout(&countHi, &deadlineTick, timeoutMs)
            || !readByteWithTimeout(&countLo, &deadlineTick, timeoutMs)) {
        return PLANNED_SONG_RX_INVALID;
    }

    stepCount = (uint16_t)(((uint16_t)countHi << 8) | (uint16_t)countLo);
    DEBUG_PRINT(F("[PLANDBG] rx_begin tick="));
    DEBUG_PRINT(s_uploadReceiveStartTick);
    DEBUG_PRINT(F("ms steps="));
    DEBUG_PRINT(stepCount);
    DEBUG_PRINTLN(F(""));
    if (stepCount == 0U || stepCount > PLANNED_SONG_MAX_STEPS) {
        discardAvailableInput();
        return PLANNED_SONG_RX_INVALID;
    }

    for (index = 0U; index < stepCount; ++index) {
        int posHi;
        int posLo;
        int mask;
        int pressHi;
        int pressLo;
        int travelHi;
        int travelLo;
        uint16_t positionX100;

        if (!readByteWithTimeout(&posHi, &deadlineTick, timeoutMs)
                || !readByteWithTimeout(&posLo, &deadlineTick, timeoutMs)
                || !readByteWithTimeout(&mask, &deadlineTick, timeoutMs)
                || !readByteWithTimeout(&pressHi, &deadlineTick, timeoutMs)
                || !readByteWithTimeout(&pressLo, &deadlineTick, timeoutMs)
                || !readByteWithTimeout(&travelHi, &deadlineTick, timeoutMs)
                || !readByteWithTimeout(&travelLo, &deadlineTick, timeoutMs)) {
            return PLANNED_SONG_RX_INVALID;
        }

        positionX100 = (uint16_t)(((uint16_t)posHi << 8) | (uint16_t)posLo);
        s_uploadedSong[index].position_mm = (float)positionX100 / 100.0f;
        s_uploadedSong[index].fingers_mask = (uint8_t)mask;
        s_uploadedSong[index].press_duration_ms =
            (uint16_t)(((uint16_t)pressHi << 8) | (uint16_t)pressLo);
        s_uploadedSong[index].travel_duration_ms =
            (uint16_t)(((uint16_t)travelHi << 8) | (uint16_t)travelLo);
    }

    s_uploadedSongLen = stepCount;
    s_uploadReceiveDoneTick = (uint32_t)millis();
    DEBUG_PRINT(F("[PLANDBG] rx_done tick="));
    DEBUG_PRINT(s_uploadReceiveDoneTick);
    DEBUG_PRINT(F("ms rx_ms="));
    DEBUG_PRINT(s_uploadReceiveDoneTick - s_uploadReceiveStartTick);
    DEBUG_PRINT(F(" steps="));
    DEBUG_PRINT(stepCount);
    DEBUG_PRINT(F(" bytes="));
    DEBUG_PRINT((uint32_t)stepCount * 7U + 6U);
    DEBUG_PRINT(F(" first_pos="));
    DEBUG_PRINT(s_uploadedSong[0].position_mm, 2);
    DEBUG_PRINTLN(F("mm"));
    return PLANNED_SONG_RX_SONG_READY;
}

bool playbackButtonPressed(void)
{
    const bool currentState = platform_io_is_user_button_active();
    const uint32_t now = hal_getTick();
    const bool pressed = currentState
        && !s_buttonPrevState
        && ((int32_t)(now - s_buttonDebounceUntilTick) >= 0);

    s_buttonPrevState = currentState;
    if (pressed) {
        s_buttonDebounceUntilTick = now + PLANNED_SONG_BUTTON_DEBOUNCE_MS;
    }
    return pressed;
}

void pressFingerMask(uint8_t fingersMask)
{
    uint8_t fingerIndex;
    for (fingerIndex = 0U; fingerIndex < (uint8_t)app_config::finger_count; ++fingerIndex) {
        if ((fingersMask & (uint8_t)(1U << fingerIndex)) != 0U) {
            hal_fingerPress(fingerIndex);
        }
    }
}

void beginUploadedSong(void)
{
    s_stepIndex = 0U;
    s_deadlineTick = 0U;
    s_pauseStartTick = 0U;
    s_resumeNeedsRepress = false;
    s_songBeginTick = hal_getTick();
    s_runState = PLANNED_SONG_PREPOSITION;

    DEBUG_PRINT(F("[PLANDBG] song_begin tick="));
    DEBUG_PRINT(s_songBeginTick);
    DEBUG_PRINT(F("ms from_rx_done="));
    DEBUG_PRINT(s_songBeginTick - s_uploadReceiveDoneTick);
    DEBUG_PRINT(F("ms steps="));
    DEBUG_PRINT(s_uploadedSongLen);
    DEBUG_PRINTLN(F(""));
}

void enterPausedState(void)
{
    s_resumeState = s_runState;
    s_runState = PLANNED_SONG_PAUSED;
}

void resumeFromPause(uint32_t now)
{
    const uint32_t pausedMs = now - s_pauseStartTick;

    if (s_deadlineTick != 0U) {
        s_deadlineTick += pausedMs;
    }
    hal_pidSetEnabled(true);

    if (s_resumeState == PLANNED_SONG_PREPOSITION) {
        hal_motorSetTarget(s_uploadedSong[0].position_mm);
    } else if (s_resumeState == PLANNED_SONG_TRAVEL &&
               (s_stepIndex + 1U) < s_uploadedSongLen) {
        hal_motorSetTarget(s_uploadedSong[s_stepIndex + 1U].position_mm);
    } else if (s_resumeState == PLANNED_SONG_PRESS) {
        hal_motorSetTarget(s_uploadedSong[s_stepIndex].position_mm);
        pressFingerMask(s_uploadedSong[s_stepIndex].fingers_mask);
        s_resumeNeedsRepress = false;
    }

    s_runState = s_resumeState;
}

void resetUploadedSongState(void)
{
    s_uploadedSongLen = 0U;
    s_stepIndex = 0U;
    s_deadlineTick = 0U;
    s_pauseStartTick = 0U;
    s_stateEntryTick = hal_getTick();
    s_resumeNeedsRepress = false;
}
}

void PlannedSongPlayer_run(void)
{
    uint32_t now = hal_getTick();

    if (s_runState != PLANNED_SONG_PAUSED &&
            s_runState != PLANNED_SONG_FAULTED &&
            s_runState != PLANNED_SONG_HOMING &&
            s_runState != PLANNED_SONG_WAITING_FOR_UPLOAD &&
            plannedFaultActive()) {
        plannedAbortForFault();
        s_runState = PLANNED_SONG_FAULTED;
    }

    if (s_runState == PLANNED_SONG_WAITING_FOR_UPLOAD || s_runState == PLANNED_SONG_DONE) {
        const PlannedSongReceiveResult receiveResult =
            receiveUploadedSong(PLANNED_SONG_UART_TIMEOUT_MS);

        if (receiveResult == PLANNED_SONG_RX_SONG_READY) {
            beginUploadedSong();
        } else if (receiveResult == PLANNED_SONG_RX_INVALID) {
            plannedAbortForFault();
            resetUploadedSongState();
            s_runState = PLANNED_SONG_WAITING_FOR_UPLOAD;
            return;
        } else {
            return;
        }
    }

    now = hal_getTick();

    if (s_runState != PLANNED_SONG_PAUSED &&
            s_runState != PLANNED_SONG_FAULTED &&
            s_runState != PLANNED_SONG_HOMING &&
            playbackButtonPressed()) {
        if (s_runState == PLANNED_SONG_PRESS) {
            s_resumeNeedsRepress = true;
        }
        enterPausedState();
    }

    if (s_runState != s_previousState) {
        logStateTransition(s_previousState, s_runState, now);
        switch (s_runState) {
            case PLANNED_SONG_PREPOSITION:
                s_prepositionStartTick = now;
                hal_motorSetTarget(s_uploadedSong[0].position_mm);
                break;

            case PLANNED_SONG_PRESS:
                if (s_stepIndex == 0U) {
                    DEBUG_PRINT(F("[PLANDBG] first_press tick="));
                    DEBUG_PRINT(now);
                    DEBUG_PRINT(F("ms after_song_begin="));
                    DEBUG_PRINT(now - s_songBeginTick);
                    DEBUG_PRINT(F("ms after_rx_done="));
                    DEBUG_PRINT(now - s_uploadReceiveDoneTick);
                    DEBUG_PRINT(F("ms preposition_ms="));
                    DEBUG_PRINT(now - s_prepositionStartTick);
                    DEBUG_PRINT(F(" pos="));
                    DEBUG_PRINT(hal_motorGetPosition(), 2);
                    DEBUG_PRINT(F("mm tgt="));
                    DEBUG_PRINT(s_uploadedSong[0].position_mm, 2);
                    DEBUG_PRINTLN(F("mm"));
                }
                if (s_resumeNeedsRepress) {
                    pressFingerMask(s_uploadedSong[s_stepIndex].fingers_mask);
                    s_resumeNeedsRepress = false;
                } else {
                    pressFingerMask(s_uploadedSong[s_stepIndex].fingers_mask);
                    s_deadlineTick = now + s_uploadedSong[s_stepIndex].press_duration_ms;
                }
                break;

            case PLANNED_SONG_TRAVEL:
                if ((s_stepIndex + 1U) < s_uploadedSongLen) {
                    hal_motorSetTarget(s_uploadedSong[s_stepIndex + 1U].position_mm);
                    s_deadlineTick = now + s_uploadedSong[s_stepIndex].travel_duration_ms;
                }
                break;

            case PLANNED_SONG_PAUSED:
                motor_control_set_motor_speed(0.0f);
                hal_fingerReleaseAll();
                hal_pidSetEnabled(false);
                s_pauseStartTick = now;
                break;

            case PLANNED_SONG_FAULTED:
                plannedAbortForFault();
                hal_pidSetEnabled(false);
                break;

            case PLANNED_SONG_HOMING:
                if (hal_isEStopped()) {
                    hal_clearEStop();
                }
                song_player_homing();
                resetUploadedSongState();
                s_buttonPrevState = platform_io_is_user_button_active();
                s_runState = PLANNED_SONG_WAITING_FOR_UPLOAD;
                break;

            case PLANNED_SONG_DONE:
                hal_fingerReleaseAll();
                s_runState = PLANNED_SONG_WAITING_FOR_UPLOAD;
                break;

            case PLANNED_SONG_WAITING_FOR_UPLOAD:
            default:
                break;
        }
        s_stateEntryTick = now;
        s_previousState = s_runState;
        return;
    }

    switch (s_runState) {
        case PLANNED_SONG_PREPOSITION:
            if (hal_motorHasArrived()) {
                s_runState = PLANNED_SONG_PRESS;
            }
            break;

        case PLANNED_SONG_PRESS:
            if ((int32_t)(now - s_deadlineTick) >= 0) {
                hal_fingerReleaseAll();
                if ((s_stepIndex + 1U) < s_uploadedSongLen) {
                    s_runState = PLANNED_SONG_TRAVEL;
                } else {
                    s_runState = PLANNED_SONG_DONE;
                }
            }
            break;

        case PLANNED_SONG_TRAVEL:
            if ((int32_t)(now - s_deadlineTick) >= 0 && hal_motorHasArrived()) {
                s_stepIndex++;
                s_runState = PLANNED_SONG_PRESS;
            }
            break;

        case PLANNED_SONG_PAUSED:
            if (playbackButtonPressed()) {
                resumeFromPause(now);
            }
            break;

        case PLANNED_SONG_FAULTED:
            if (playbackButtonPressed()) {
                s_runState = PLANNED_SONG_HOMING;
            }
            break;

        case PLANNED_SONG_HOMING:
        case PLANNED_SONG_DONE:
        case PLANNED_SONG_WAITING_FOR_UPLOAD:
        default:
            break;
    }
}
