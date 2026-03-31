/*
 * song_player.cpp
 * ===========================================================================
 * Top-level song playing logic — ESP32/Arduino version.
 *
 * STM32 → ESP32 changes:
 *   HAL_UART_Transmit  → Serial.print
 *   HAL_GPIO_ReadPin   → digitalRead
 *   bool               → bool (Arduino defines this natively)
 * ===========================================================================
 */

#include "song_player.h"
#include "motor_control.h"
#include "note_player.h"
#include "piano_keymap.h"
#include "midi_parser.h"
#include "hal_interface.h"
#include "Arduino.h"
#include <stdio.h>
#include <string.h>

/* --------------------------------------------------------------------------
 * Buffers
 * -------------------------------------------------------------------------- */
static uint8_t   uartBuf[MIDI_UART_BUF_SIZE];
static NoteEvent events[MIDI_MAX_NOTES];

/* --------------------------------------------------------------------------
 * SongPlayer_run
 * Waits for a MIDI file over Serial, parses it, plays it.
 * -------------------------------------------------------------------------- */
void SongPlayer_run(void)
{
    MidiParseResult result;
    Serial.println("Waiting for MIDI file...");
    Midi_receiveAndParse(uartBuf, sizeof(uartBuf), events, MIDI_MAX_NOTES, &result);
    Midi_printResult(&result);
    if (result.status == MIDI_OK) {
        NotePlayer_playSequence(events, result.noteCount);
    }
}

/* --------------------------------------------------------------------------
 * Test: play C major scale
 * -------------------------------------------------------------------------- */
static NoteEvent s_scale[] = {
    {60, 400, 50},  /* C4 */
    {62, 400, 50},  /* D4 */
    {64, 400, 50},  /* E4 */
    {65, 400, 50},  /* F4 */
    {67, 400, 50},  /* G4 */
};

void test_playScale(void)
{
    NotePlayer_playSequence(s_scale, 5);
}

/* --------------------------------------------------------------------------
 * Test: print key map over Serial
 * -------------------------------------------------------------------------- */
void test_printKeyMap(void)
{
    char buf[80];
    uint8_t i, j;
    for (i = 0; i < KEY_MAP_SIZE; i++) {
        const PianoKey* k = &KEY_MAP[i];
        for (j = 0; j < MAX_FINGER_OPTIONS; j++) {
            const FingerOption* o = &k->options[j];
            if (!o->valid) continue;
            snprintf(buf, sizeof(buf), "%s  finger=%s  pos=%.1fmm\r\n",
                k->name, fingerName(o->finger), o->motorPositionMM);
            Serial.print(buf);
        }
    }
}

/* --------------------------------------------------------------------------
 * Homing
 *
 * EDIT THESE to match your hardware:
 *   HOMING_SWITCH_PIN       GPIO pin connected to your limit switch
 *   HOMING_SWITCH_ACTIVE    HIGH or LOW when switch is pressed
 *   HOMING_DIRECTION        +1.0 or -1.0 depending on which way home is
 * -------------------------------------------------------------------------- */
#define HOMING_SWITCH_PIN       15      /* TODO: set your limit switch pin   */
#define HOMING_SWITCH_ACTIVE    LOW     /* TODO: HIGH or LOW when pressed    */
#define HOMING_SPEED            0.2f
#define HOMING_DIRECTION        (-1.0f) /* TODO: set direction toward home   */

static bool isHomeSwitchActive(void)
{
    return digitalRead(HOMING_SWITCH_PIN) == HOMING_SWITCH_ACTIVE;
}

void song_player_homing(void)
{
    pinMode(HOMING_SWITCH_PIN, INPUT_PULLUP);

    /* Drive toward home */
    motor_control_setMotorSpeed(HOMING_DIRECTION * HOMING_SPEED);

    /* Wait for limit switch */
    while (!isHomeSwitchActive()) { /* spin — PID timer still running */ }

    /* Stop and zero encoder */
    motor_control_setMotorSpeed(0.0f);
    motor_controller_encoderZeroPosition();
    Serial.println("Homing complete.");
}
