#include "planned_song_player.h"

#include "Arduino.h"
#include "chord_showcase_planned_song.h"
#include "config.h"
#include "debug_log.h"
#include "hal_interface.h"
#include "happy_birthday_better_planned_song.h"
#include "happy_birthday_planned_song.h"
#include "married_life_planned_song.h"
#include "motor_control.h"
#include "o_canada_chords_planned_song.h"
#include "o_canada_planned_song.h"
#include "platform_io.h"
#include "robot_hand_pattern_test_planned_song.h"
#include "song_player.h"
#include "take_on_me_planned_song.h"
#include <string.h>

namespace
{
#define PLANNED_SONG_BUTTON_DEBOUNCE_MS  200U
#define PLANNED_SONG_LINE_MAX             64U

enum PlannedSongRunState
{
    PLANNED_SONG_WAITING_FOR_SELECTION = 0,
    PLANNED_SONG_PREPOSITION,
    PLANNED_SONG_PRESS,
    PLANNED_SONG_TRAVEL,
    PLANNED_SONG_PAUSED,
    PLANNED_SONG_HOMING,
    PLANNED_SONG_DONE,
    PLANNED_SONG_FAULTED
};

enum PlannedSongCommandResult
{
    PLANNED_SONG_CMD_NONE = 0,
    PLANNED_SONG_CMD_PLAY_SELECTED,
    PLANNED_SONG_CMD_INVALID
};

typedef struct {
    const char* command_name;
    const PlannedSongStep* steps;
    uint16_t length;
} PlannedSongCatalogEntry;

static const PlannedSongCatalogEntry kSongCatalog[] = {
    { "chord_showcase", g_chord_showcase_song, g_chord_showcase_song_len },
    { "happy_birthday", g_happy_birthday_song, g_happy_birthday_song_len },
    { "happy_birthday_better", g_happy_birthday_better_song, g_happy_birthday_better_song_len },
    { "married_life", g_married_life_song, g_married_life_song_len },
    { "o_canada", g_o_canada_song, g_o_canada_song_len },
    { "o_canada_chords", g_o_canada_chords_song, g_o_canada_chords_song_len },
    { "robot_hand_pattern_test", g_robot_hand_pattern_test_song, g_robot_hand_pattern_test_song_len },
    { "take_on_me", g_take_on_me_song, g_take_on_me_song_len },
};

static PlannedSongRunState s_runState = PLANNED_SONG_WAITING_FOR_SELECTION;
static PlannedSongRunState s_previousState = PLANNED_SONG_WAITING_FOR_SELECTION;
static PlannedSongRunState s_resumeState = PLANNED_SONG_PREPOSITION;
static const PlannedSongStep* s_activeSong = NULL;
static const char* s_activeSongName = NULL;
static uint16_t s_activeSongLen = 0U;
static uint16_t s_stepIndex = 0U;
static uint32_t s_deadlineTick = 0U;
static uint32_t s_pauseStartTick = 0U;
static uint32_t s_buttonDebounceUntilTick = 0U;
static uint32_t s_stateEntryTick = 0U;
static uint32_t s_songBeginTick = 0U;
static uint32_t s_prepositionStartTick = 0U;
static bool s_buttonPrevState = false;
static bool s_resumeNeedsRepress = false;
static size_t s_receiveLineLen = 0U;
static char s_receiveLine[PLANNED_SONG_LINE_MAX];

const __FlashStringHelper* plannedSongStateName(PlannedSongRunState state)
{
    switch (state) {
        case PLANNED_SONG_WAITING_FOR_SELECTION:
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

bool asciiIsSpace(char value)
{
    return value == ' ' || value == '\t';
}

char asciiToLower(char value)
{
    if (value >= 'A' && value <= 'Z') {
        return (char)(value - 'A' + 'a');
    }
    return value;
}

char* trimAsciiWhitespace(char* text)
{
    size_t length;

    while (*text != '\0' && asciiIsSpace(*text)) {
        text++;
    }

    length = strlen(text);
    while (length > 0U && asciiIsSpace(text[length - 1U])) {
        text[length - 1U] = '\0';
        length--;
    }

    return text;
}

bool asciiEqualsIgnoreCase(const char* left, const char* right)
{
    while (*left != '\0' && *right != '\0') {
        if (asciiToLower(*left) != asciiToLower(*right)) {
            return false;
        }
        left++;
        right++;
    }
    return *left == '\0' && *right == '\0';
}

void stripOptionalMidExtension(char* songName)
{
    size_t length = strlen(songName);

    if (length > 4U &&
            asciiToLower(songName[length - 4U]) == '.' &&
            asciiToLower(songName[length - 3U]) == 'm' &&
            asciiToLower(songName[length - 2U]) == 'i' &&
            asciiToLower(songName[length - 1U]) == 'd') {
        songName[length - 4U] = '\0';
    }
}

const PlannedSongCatalogEntry* findSongEntry(const char* songName)
{
    size_t index;

    for (index = 0U; index < (sizeof(kSongCatalog) / sizeof(kSongCatalog[0])); ++index) {
        if (asciiEqualsIgnoreCase(songName, kSongCatalog[index].command_name)) {
            return &kSongCatalog[index];
        }
    }

    return NULL;
}

void clearCommandLineState(void)
{
    s_receiveLineLen = 0U;
    s_receiveLine[0] = '\0';
}

void clearSelectedSongState(void)
{
    s_activeSong = NULL;
    s_activeSongName = NULL;
    s_activeSongLen = 0U;
    s_stepIndex = 0U;
    s_deadlineTick = 0U;
    s_pauseStartTick = 0U;
    s_songBeginTick = 0U;
    s_prepositionStartTick = 0U;
    s_resumeNeedsRepress = false;
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
    if (s_activeSongLen == 0U) {
        DEBUG_PRINT(0U);
    } else {
        DEBUG_PRINT((uint32_t)s_stepIndex + 1U);
    }
    DEBUG_PRINT(F("/"));
    DEBUG_PRINT(s_activeSongLen);
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

bool readLineFromSerial(char* outLine, size_t outLineSize)
{
    while (Serial.available() > 0) {
        const int byteValue = Serial.read();

        if (byteValue < 0) {
            break;
        }
        if (byteValue == '\r') {
            continue;
        }
        if (byteValue == '\n') {
            if (s_receiveLineLen == 0U) {
                continue;
            }
            if (s_receiveLineLen >= outLineSize) {
                clearCommandLineState();
                return false;
            }

            s_receiveLine[s_receiveLineLen] = '\0';
            memcpy(outLine, s_receiveLine, s_receiveLineLen + 1U);
            clearCommandLineState();
            return true;
        }

        if (s_receiveLineLen + 1U < sizeof(s_receiveLine)) {
            s_receiveLine[s_receiveLineLen++] = (char)byteValue;
        } else {
            clearCommandLineState();
        }
    }

    return false;
}

PlannedSongCommandResult receiveCommand(void)
{
    char line[PLANNED_SONG_LINE_MAX];
    char songName[PLANNED_SONG_LINE_MAX];
    char* trimmedLine;
    const char* songStart;
    size_t songLength;
    const PlannedSongCatalogEntry* entry;

    if (!readLineFromSerial(line, sizeof(line))) {
        return PLANNED_SONG_CMD_NONE;
    }

    trimmedLine = trimAsciiWhitespace(line);
    if (*trimmedLine == '\0') {
        return PLANNED_SONG_CMD_NONE;
    }

    if (!(asciiToLower(trimmedLine[0]) == 'p' &&
            asciiToLower(trimmedLine[1]) == 'l' &&
            asciiToLower(trimmedLine[2]) == 'a' &&
            asciiToLower(trimmedLine[3]) == 'y' &&
            (trimmedLine[4] == '\0' || asciiIsSpace(trimmedLine[4])))) {
        DEBUG_PRINT(F("[PLANDBG] cmd_ignore line="));
        DEBUG_PRINT(trimmedLine);
        DEBUG_PRINTLN(F(""));
        return PLANNED_SONG_CMD_NONE;
    }

    songStart = trimmedLine + 4;
    while (*songStart != '\0' && asciiIsSpace(*songStart)) {
        songStart++;
    }

    if (*songStart == '\0') {
        DEBUG_PRINTLN(F("[PLANDBG] cmd_missing_song"));
        return PLANNED_SONG_CMD_INVALID;
    }

    songLength = strlen(songStart);
    if (songLength >= sizeof(songName)) {
        DEBUG_PRINT(F("[PLANDBG] cmd_song_name_too_long name="));
        DEBUG_PRINT(songStart);
        DEBUG_PRINTLN(F(""));
        return PLANNED_SONG_CMD_INVALID;
    }

    memcpy(songName, songStart, songLength + 1U);
    trimAsciiWhitespace(songName);
    stripOptionalMidExtension(songName);

    entry = findSongEntry(songName);
    if (entry == NULL) {
        DEBUG_PRINT(F("[PLANDBG] cmd_unknown_song name="));
        DEBUG_PRINT(songName);
        DEBUG_PRINTLN(F(""));
        return PLANNED_SONG_CMD_INVALID;
    }

    s_activeSong = entry->steps;
    s_activeSongLen = entry->length;
    s_activeSongName = entry->command_name;

    DEBUG_PRINT(F("[PLANDBG] cmd_play name="));
    DEBUG_PRINT(s_activeSongName);
    DEBUG_PRINT(F(" steps="));
    DEBUG_PRINT(s_activeSongLen);
    DEBUG_PRINTLN(F(""));

    return PLANNED_SONG_CMD_PLAY_SELECTED;
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

void beginSelectedSong(void)
{
    if (s_activeSong == NULL || s_activeSongLen == 0U) {
        return;
    }

    s_stepIndex = 0U;
    s_deadlineTick = 0U;
    s_pauseStartTick = 0U;
    s_resumeNeedsRepress = false;
    s_songBeginTick = hal_getTick();
    hal_pidSetEnabled(true);
    s_runState = PLANNED_SONG_PREPOSITION;

    DEBUG_PRINT(F("[PLANDBG] song_begin tick="));
    DEBUG_PRINT(s_songBeginTick);
    DEBUG_PRINT(F("ms name="));
    DEBUG_PRINT(s_activeSongName);
    DEBUG_PRINT(F(" steps="));
    DEBUG_PRINT(s_activeSongLen);
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
        hal_motorSetTarget(s_activeSong[0].position_mm);
    } else if (s_resumeState == PLANNED_SONG_TRAVEL &&
               (s_stepIndex + 1U) < s_activeSongLen) {
        hal_motorSetTarget(s_activeSong[s_stepIndex + 1U].position_mm);
    } else if (s_resumeState == PLANNED_SONG_PRESS) {
        hal_motorSetTarget(s_activeSong[s_stepIndex].position_mm);
        pressFingerMask(s_activeSong[s_stepIndex].fingers_mask);
        s_resumeNeedsRepress = false;
    }

    s_runState = s_resumeState;
}
}

void PlannedSongPlayer_run(void)
{
    uint32_t now = hal_getTick();

    if (s_runState != PLANNED_SONG_PAUSED &&
            s_runState != PLANNED_SONG_FAULTED &&
            s_runState != PLANNED_SONG_HOMING &&
            s_runState != PLANNED_SONG_WAITING_FOR_SELECTION &&
            plannedFaultActive()) {
        plannedAbortForFault();
        s_runState = PLANNED_SONG_FAULTED;
    }

    if (s_runState == PLANNED_SONG_WAITING_FOR_SELECTION) {
        const PlannedSongCommandResult commandResult = receiveCommand();

        if (commandResult == PLANNED_SONG_CMD_PLAY_SELECTED) {
            beginSelectedSong();
        } else {
            return;
        }
    }

    now = hal_getTick();

    if (s_runState != PLANNED_SONG_PAUSED &&
            s_runState != PLANNED_SONG_FAULTED &&
            s_runState != PLANNED_SONG_HOMING &&
            s_runState != PLANNED_SONG_WAITING_FOR_SELECTION &&
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
                hal_motorSetTarget(s_activeSong[0].position_mm);
                break;

            case PLANNED_SONG_PRESS:
                if (s_stepIndex == 0U) {
                    DEBUG_PRINT(F("[PLANDBG] first_press tick="));
                    DEBUG_PRINT(now);
                    DEBUG_PRINT(F("ms after_song_begin="));
                    DEBUG_PRINT(now - s_songBeginTick);
                    DEBUG_PRINT(F("ms preposition_ms="));
                    DEBUG_PRINT(now - s_prepositionStartTick);
                    DEBUG_PRINT(F(" pos="));
                    DEBUG_PRINT(hal_motorGetPosition(), 2);
                    DEBUG_PRINT(F("mm tgt="));
                    DEBUG_PRINT(s_activeSong[0].position_mm, 2);
                    DEBUG_PRINTLN(F("mm"));
                }
                pressFingerMask(s_activeSong[s_stepIndex].fingers_mask);
                s_resumeNeedsRepress = false;
                s_deadlineTick = now + s_activeSong[s_stepIndex].press_duration_ms;
                break;

            case PLANNED_SONG_TRAVEL:
                if ((s_stepIndex + 1U) < s_activeSongLen) {
                    hal_motorSetTarget(s_activeSong[s_stepIndex + 1U].position_mm);
                    s_deadlineTick = now + s_activeSong[s_stepIndex].travel_duration_ms;
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
                clearSelectedSongState();
                clearCommandLineState();
                s_buttonPrevState = platform_io_is_user_button_active();
                s_runState = PLANNED_SONG_WAITING_FOR_SELECTION;
                break;

            case PLANNED_SONG_DONE:
                hal_fingerReleaseAll();
                clearSelectedSongState();
                s_runState = PLANNED_SONG_WAITING_FOR_SELECTION;
                break;

            case PLANNED_SONG_WAITING_FOR_SELECTION:
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
                if ((s_stepIndex + 1U) < s_activeSongLen) {
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
        case PLANNED_SONG_WAITING_FOR_SELECTION:
        default:
            break;
    }
}
