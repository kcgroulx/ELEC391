/*
 * song_player.cpp
 * ===========================================================================
 * Top-level song playing logic.
 *
 * Fixed function names to match teammate's refactor:
 *   motor_control_setMotorSpeed()        → motor_control_set_motor_speed()
 *   motor_controller_encoderZeroPosition() → motor_control_zero_position()
 *   home switch pin                      → app_config::home_switch_pin
 * ===========================================================================
 */

#include "song_player.h"
#include "motor_control.h"
#include "platform_io.h"
#include "config.h"
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
 * SongPlayer_run — receive MIDI over Serial and play it
 * -------------------------------------------------------------------------- */
void SongPlayer_run(void)
{
    MidiParseResult result;

    /* Do NOT print anything to Serial here — printing and receiving share
     * the same UART. Any bytes we send out could interfere with the PC
     * detecting readiness, and the ESP32's own TX chars don't corrupt RX
     * but closing the Serial Monitor before sending is still recommended.
     *
     * Only call Midi_printResult when a real transfer actually occurred
     * (i.e. not a routine UART timeout while waiting for data). Printing
     * UART_FAIL on every loop() call would flood Serial and corrupt the
     * next receive attempt. */
    Midi_receiveAndParse(uartBuf, sizeof(uartBuf), events, MIDI_MAX_NOTES, &result);
    if (result.status != MIDI_ERR_UART) {
        Midi_printResult(&result);
    }
    if (result.status == MIDI_OK) {
        NotePlayer_playSequence(events, result.noteCount);
    }
}

/* --------------------------------------------------------------------------
 * Hardcoded C major scale (C4–C5, one octave)
 * -------------------------------------------------------------------------- */
static NoteEvent s_scale[] = {
    {38, 400, 50},  /* C4 */
    {50, 400, 50},  /* D4 */
    {42, 400, 50},  /* E4 */
    {58, 400, 50},  /* F4 */
    {46, 400, 50},  /* G4 */
    {58, 400, 50},  /* A4 */
    {38, 400, 50},  /* B4 */
    {50, 400,  0},  /* C5 */
};
static const uint16_t s_scaleLen = sizeof(s_scale) / sizeof(s_scale[0]);

void test_playScale(void)
{
    NotePlayer_playSequence(s_scale, s_scaleLen);
}

/* --------------------------------------------------------------------------
 * Test: fire each solenoid in order (no motor movement)
 * Verifies wiring: W1(GPIO21), W2(22), W3(23), B1(25), B2(26)
 * -------------------------------------------------------------------------- */
void test_fingersSolenoidOnly(void)
{
    const char* names[] = {"W1 (GPIO21)", "W2 (GPIO22)", "W3 (GPIO23)",
                           "B1 (GPIO25)", "B2 (GPIO26)"};
    Serial.println("\r\n=== SOLENOID TEST — no motor movement ===");
    for (uint8_t i = 0; i < 5; i++) {
        char buf[48];
        snprintf(buf, sizeof(buf), "  Firing finger %s ...\r\n", names[i]);
        Serial.print(buf);
        hal_fingerPress(i);
        hal_delay(1300);
        hal_fingerRelease(i);
        hal_delay(200);
    }
    Serial.println("=== SOLENOID TEST DONE ===\r\n");
}

/* --------------------------------------------------------------------------
 * Test: play notes that exercise all 5 fingers
 *
 * Sequence chosen so the greedy finger picker selects each finger:
 *   1. C2  (midi 36) — only option is W1 at 0mm         → tests W1
 *   2. C#2 (midi 37) — B1 at 0mm is only reachable opt  → tests B1
 *   3. E3  (midi 52) — motor at 0, W3 at 117.5mm closest→ tests W3
 *   4. D#3 (midi 51) — motor at ~117, B2 at 141mm closest→ tests B2
 *   5. D3  (midi 50) — motor at ~141, W2 at 141mm closest→ tests W2
 * -------------------------------------------------------------------------- */
static NoteEvent s_fingerTest[] = {
    {36, 500, 100},  /* C2  → W1 */
    {37, 500, 100},  /* C#2 → B1 */
    {52, 500, 100},  /* E3  → W3 */
    {51, 500, 100},  /* D#3 → B2 */
    {50, 500,   0},  /* D3  → W2 */
};
static const uint16_t s_fingerTestLen = sizeof(s_fingerTest) / sizeof(s_fingerTest[0]);

void test_allFingers(void)
{
    Serial.println("\r\n=== FINGER TEST — move + press all 5 fingers ===");
    NotePlayer_playSequence(s_fingerTest, s_fingerTestLen);
    Serial.println("=== FINGER TEST DONE ===\r\n");
}

/* --------------------------------------------------------------------------
 * Test: print full key map over Serial
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
 * Homing — uses app_config::home_switch_pin from config.h
 *
 * To configure:
 *   Set home_switch_pin in config.h (currently -1 = disabled).
 *   Set HOMING_DIRECTION to +1.0f or -1.0f depending on which way is home.
 * -------------------------------------------------------------------------- */
#define HOMING_SPEED        0.2f
#define HOMING_DIRECTION    (-1.0f)   /* TODO: set direction toward home     */

void song_player_homing(void)
{
    if (app_config::home_switch_pin < 0) {
        /* No home switch configured — just zero the encoder at current pos */
        Serial.println("No home switch configured — zeroing at current position.");
        motor_control_zero_position();
        return;
    }

    Serial.println("Homing...");

    /* Drive toward home */
    motor_control_set_motor_speed(HOMING_DIRECTION * HOMING_SPEED);

    /* Wait for limit switch via platform_io */
    while (!platform_io_is_home_switch_active()) { /* spin */ }

    /* Stop and zero */
    motor_control_set_motor_speed(0.0f);
    motor_control_zero_position();
    Serial.println("Homing complete.");
}
