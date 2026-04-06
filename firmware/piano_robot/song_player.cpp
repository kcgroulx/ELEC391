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

/* --------------------------------------------------------------------------
 * Demo: chromatic sweep C2→F#4, then back down — all 31 reachable keys
 *
 * Every key (MIDI 36–66) played twice: once up, once down.
 * Tempo accelerates going up; even faster coming back down.
 * -------------------------------------------------------------------------- */
static NoteEvent s_demo[] = {
    /* === Chromatic run UP — C2 to F#4, accelerating === */
    {36, 250, 40},  /* C2  */
    {37, 250, 40},  /* C#2 */
    {38, 250, 40},  /* D2  */
    {39, 250, 40},  /* D#2 */
    {40, 250, 40},  /* E2  */
    {41, 250, 40},  /* F2  */
    {42, 250, 40},  /* F#2 */
    {43, 230, 35},  /* G2  */
    {44, 230, 35},  /* G#2 */
    {45, 230, 35},  /* A2  */
    {46, 230, 35},  /* A#2 */
    {47, 230, 35},  /* B2  */
    {48, 200, 30},  /* C3  */
    {49, 200, 30},  /* C#3 */
    {50, 200, 30},  /* D3  */
    {51, 200, 30},  /* D#3 */
    {52, 200, 30},  /* E3  */
    {53, 200, 30},  /* F3  */
    {54, 180, 25},  /* F#3 */
    {55, 180, 25},  /* G3  */
    {56, 180, 25},  /* G#3 */
    {57, 180, 25},  /* A3  */
    {58, 180, 25},  /* A#3 */
    {59, 180, 25},  /* B3  */
    {60, 160, 20},  /* C4  */
    {61, 160, 20},  /* C#4 */
    {62, 160, 20},  /* D4  */
    {63, 160, 20},  /* D#4 */
    {64, 160, 20},  /* E4  */
    {65, 160, 20},  /* F4  */
    {66, 400, 200}, /* F#4 — hold at the top! */

    /* === Chromatic run DOWN — F#4 back to C2, fast === */
    {66, 130, 15},  /* F#4 */
    {65, 130, 15},  /* F4  */
    {64, 130, 15},  /* E4  */
    {63, 130, 15},  /* D#4 */
    {62, 130, 15},  /* D4  */
    {61, 130, 15},  /* C#4 */
    {60, 120, 12},  /* C4  */
    {59, 120, 12},  /* B3  */
    {58, 120, 12},  /* A#3 */
    {57, 120, 12},  /* A3  */
    {56, 120, 12},  /* G#3 */
    {55, 120, 12},  /* G3  */
    {54, 110, 10},  /* F#3 */
    {53, 110, 10},  /* F3  */
    {52, 110, 10},  /* E3  */
    {51, 110, 10},  /* D#3 */
    {50, 110, 10},  /* D3  */
    {49, 110, 10},  /* C#3 */
    {48, 100, 10},  /* C3  */
    {47, 100, 10},  /* B2  */
    {46, 100, 10},  /* A#2 */
    {45, 100, 10},  /* A2  */
    {44, 100, 10},  /* G#2 */
    {43, 100, 10},  /* G2  */
    {42, 100, 10},  /* F#2 */
    {41, 100, 10},  /* F2  */
    {40, 100, 10},  /* E2  */
    {39, 100, 10},  /* D#2 */
    {38, 100, 10},  /* D2  */
    {37, 100, 10},  /* C#2 */
    {36, 500,  0},  /* C2  — big finish */
};
static const uint16_t s_demoLen = sizeof(s_demo) / sizeof(s_demo[0]);

void test_playScale(void)
{
    NotePlayer_playSequence(s_scale, s_scaleLen);
}

/* --------------------------------------------------------------------------
 * C major scale: C2 D2 E2 F2 G2 A2 B2 C3
 * -------------------------------------------------------------------------- */
static NoteEvent s_cmajor[] = {
    {36, 400, 50},  /* C2 */
    {38, 400, 50},  /* D2 */
    {40, 400, 50},  /* E2 */
    {41, 400, 50},  /* F2 */
    {43, 400, 50},  /* G2 */
    {45, 400, 50},  /* A2 */
    {47, 400, 50},  /* B2 */
    {48, 400,  0},  /* C3 */
};
static const uint16_t s_cmajorLen = sizeof(s_cmajor) / sizeof(s_cmajor[0]);

void test_CmajorScale(void)
{
    Serial.println("\r\n=== C MAJOR SCALE (C2-C3) ===");
    NotePlayer_playSequence(s_cmajor, s_cmajorLen);
    Serial.println("=== C MAJOR SCALE DONE ===\r\n");
}

/* --------------------------------------------------------------------------
 * Happy Birthday in C major (C3 range)
 * MIDI: G3=55, A3=57, B3=59, C4=60, D4=62, E4=64, F4=65
 * -------------------------------------------------------------------------- */
static NoteEvent s_happyBirthday[] = {
    /* "Hap-py birth-day to you" */
    {55, 200, 30},   /* G3 - Hap   */
    {55, 200, 30},   /* G3 - py    */
    {57, 400, 30},   /* A3 - birth */
    {55, 400, 30},   /* G3 - day   */
    {60, 400, 30},   /* C4 - to    */
    {59, 800, 60},   /* B3 - you   */

    /* "Hap-py birth-day to you" */
    {55, 200, 30},   /* G3 - Hap   */
    {55, 200, 30},   /* G3 - py    */
    {57, 400, 30},   /* A3 - birth */
    {55, 400, 30},   /* G3 - day   */
    {62, 400, 30},   /* D4 - to    */
    {60, 800, 60},   /* C4 - you   */

    /* "Hap-py birth-day dear ___" */
    {55, 200, 30},   /* G3 - Hap   */
    {55, 200, 30},   /* G3 - py    */
    {55, 400, 30},   /* G3 - birth (G4 out of range, using G3) */
    {64, 400, 30},   /* E4 - day   */
    {60, 400, 30},   /* C4 - dear  */
    {59, 400, 30},   /* B3 - ___   */
    {57, 800, 60},   /* A3         */

    /* "Hap-py birth-day to you" */
    {65, 200, 30},   /* F4 - Hap   */
    {65, 200, 30},   /* F4 - py    */
    {64, 400, 30},   /* E4 - birth */
    {60, 400, 30},   /* C4 - day   */
    {62, 400, 30},   /* D4 - to    */
    {60, 800,  0},   /* C4 - you   */
};
static const uint16_t s_happyBirthdayLen = sizeof(s_happyBirthday) / sizeof(s_happyBirthday[0]);

void test_happyBirthday(void)
{
    Serial.println("\r\n=== HAPPY BIRTHDAY ===");
    NotePlayer_playSequence(s_happyBirthday, s_happyBirthdayLen);
    Serial.println("=== HAPPY BIRTHDAY DONE ===\r\n");
}

/* --------------------------------------------------------------------------
 * Test: chord playback
 *
 * Plays a sequence of single notes and chords to verify the chord planner.
 * Chords are encoded as multiple NoteEvents with delayAfterMs = 0 between
 * the chord tones (so they start within the CHORD_WINDOW_MS of each other).
 *
 * Chord feasibility depends on finger spacing:
 *   W1 is at carriage pos, W2 at +2 keys, W3 at +4 keys
 *   So from one carriage position we can hit keys 0, +2, +4 white keys apart.
 *
 * Example chords (from one carriage position):
 *   C-E-G: wki 0,2,4 → W1=C, W2=E, W3=G  (spacing 2,2 white keys) ✓
 *   C-E:   wki 0,2   → W1=C, W2=E  ✓
 *   D-F#:  wki 1 + lwki 3  → W1=D, B1=F# (B1 between wki 3 and 4)
 * -------------------------------------------------------------------------- */
static NoteEvent s_chordTest[] = {
    /* Single note */
    {48, 600, 50},   /* C3 alone */

    /* C3 + E3 chord (2 notes, delayAfter=0 between them = simultaneous) */
    {48, 600, 0},    /* C3  ← chord start */
    {52, 600, 50},   /* E3  ← same chord (0ms gap from previous) */

    /* Single note */
    {55, 600, 50},   /* G3 alone */

    /* C3 + E3 + G3 chord (3 notes — full triad!) */
    {48, 600, 0},    /* C3  ← chord start */
    {52, 600, 0},    /* E3  ← same chord */
    {55, 600, 50},   /* G3  ← same chord */

    /* D3 + F3 chord */
    {50, 600, 0},    /* D3  ← chord start */
    {53, 600, 50},   /* F3  ← same chord */

    /* E3 + G3 + B3 chord */
    {52, 600, 0},    /* E3  ← chord start */
    {55, 600, 0},    /* G3  ← same chord */
    {59, 600, 50},   /* B3  ← same chord */

    /* Big finish: C3 alone */
    {48, 1000, 0},   /* C3 held long */
};
static const uint16_t s_chordTestLen = sizeof(s_chordTest) / sizeof(s_chordTest[0]);

void test_chords(void)
{
    Serial.println("\r\n=== CHORD TEST ===");
    NotePlayer_playSequence(s_chordTest, s_chordTestLen);
    Serial.println("=== CHORD TEST DONE ===\r\n");
}

/* --------------------------------------------------------------------------
 * Clocks — Coldplay (transposed to C major)
 *
 * Original key: Eb major → transposed down 3 semitones to C major.
 * Chord progression:  C major → G minor → D minor  (repeat)
 *
 * The iconic riff is rolling triplet arpeggios:
 *   C: root-5th-3rd  =  C4 - G3 - E4
 *   Gm: root-5th-3rd =  G3 - D4 - A#3
 *   Dm: root-5th-3rd =  D3 - A3 - F3
 *
 * Each triplet group repeats 4× per chord at ~131 BPM.
 * Second pass adds chord hits for impact.
 * -------------------------------------------------------------------------- */
#define CK_NOTE   140     /* triplet eighth note duration (ms)     */
#define CK_GAP     10     /* tiny gap between arpeggio notes       */
#define CK_CHORD  500     /* chord hold duration                   */
#define CK_REST    80     /* rest between sections                 */

static NoteEvent s_clocks[] = {
    /* ============ PASS 1: Pure arpeggio ================================= */

    /* ── C major arpeggio: C4(60) - G3(55) - E4(64) × 4 ──────────────── */
    {60, CK_NOTE, CK_GAP},  {55, CK_NOTE, CK_GAP},  {64, CK_NOTE, CK_GAP},
    {60, CK_NOTE, CK_GAP},  {55, CK_NOTE, CK_GAP},  {64, CK_NOTE, CK_GAP},
    {60, CK_NOTE, CK_GAP},  {55, CK_NOTE, CK_GAP},  {64, CK_NOTE, CK_GAP},
    {60, CK_NOTE, CK_GAP},  {55, CK_NOTE, CK_GAP},  {64, CK_NOTE, CK_REST},

    /* ── G minor arpeggio: G3(55) - D4(62) - A#3(58) × 4 ─────────────── */
    {55, CK_NOTE, CK_GAP},  {62, CK_NOTE, CK_GAP},  {58, CK_NOTE, CK_GAP},
    {55, CK_NOTE, CK_GAP},  {62, CK_NOTE, CK_GAP},  {58, CK_NOTE, CK_GAP},
    {55, CK_NOTE, CK_GAP},  {62, CK_NOTE, CK_GAP},  {58, CK_NOTE, CK_GAP},
    {55, CK_NOTE, CK_GAP},  {62, CK_NOTE, CK_GAP},  {58, CK_NOTE, CK_REST},

    /* ── D minor arpeggio: D3(50) - A3(57) - F3(53) × 4 ──────────────── */
    {50, CK_NOTE, CK_GAP},  {57, CK_NOTE, CK_GAP},  {53, CK_NOTE, CK_GAP},
    {50, CK_NOTE, CK_GAP},  {57, CK_NOTE, CK_GAP},  {53, CK_NOTE, CK_GAP},
    {50, CK_NOTE, CK_GAP},  {57, CK_NOTE, CK_GAP},  {53, CK_NOTE, CK_GAP},
    {50, CK_NOTE, CK_GAP},  {57, CK_NOTE, CK_GAP},  {53, CK_NOTE, CK_REST},

    /* ============ PASS 2: Arpeggio + chord hits ========================= */

    /* ── C major: arpeggio × 3, then C+E+G chord ─────────────────────── */
    {60, CK_NOTE, CK_GAP},  {55, CK_NOTE, CK_GAP},  {64, CK_NOTE, CK_GAP},
    {60, CK_NOTE, CK_GAP},  {55, CK_NOTE, CK_GAP},  {64, CK_NOTE, CK_GAP},
    {60, CK_NOTE, CK_GAP},  {55, CK_NOTE, CK_GAP},  {64, CK_NOTE, CK_GAP},
    /* C major chord */
    {48, CK_CHORD, 0},      {52, CK_CHORD, 0},       {55, CK_CHORD, CK_REST},

    /* ── G minor: arpeggio × 3, then G+A#+D chord ────────────────────── */
    {55, CK_NOTE, CK_GAP},  {62, CK_NOTE, CK_GAP},  {58, CK_NOTE, CK_GAP},
    {55, CK_NOTE, CK_GAP},  {62, CK_NOTE, CK_GAP},  {58, CK_NOTE, CK_GAP},
    {55, CK_NOTE, CK_GAP},  {62, CK_NOTE, CK_GAP},  {58, CK_NOTE, CK_GAP},
    /* G minor chord (G3 + A#3 + D4) */
    {55, CK_CHORD, 0},      {58, CK_CHORD, 0},       {62, CK_CHORD, CK_REST},

    /* ── D minor: arpeggio × 3, then D+F+A chord ─────────────────────── */
    {50, CK_NOTE, CK_GAP},  {57, CK_NOTE, CK_GAP},  {53, CK_NOTE, CK_GAP},
    {50, CK_NOTE, CK_GAP},  {57, CK_NOTE, CK_GAP},  {53, CK_NOTE, CK_GAP},
    {50, CK_NOTE, CK_GAP},  {57, CK_NOTE, CK_GAP},  {53, CK_NOTE, CK_GAP},
    /* D minor chord (D3 + F3 + A3) */
    {50, CK_CHORD, 0},      {53, CK_CHORD, 0},       {57, CK_CHORD, CK_REST},

    /* ============ ENDING: Big C major chord ============================= */
    {48, 800, 0},  /* C3 */
    {52, 800, 0},  /* E3 */
    {55, 800, 0},  /* G3 — final chord held long */
};
static const uint16_t s_clocksLen = sizeof(s_clocks) / sizeof(s_clocks[0]);

void test_clocks(void)
{
    Serial.println("\r\n=== CLOCKS — Coldplay ===");
    NotePlayer_playSequence(s_clocks, s_clocksLen);
    Serial.println("=== CLOCKS DONE ===\r\n");
}


/* --------------------------------------------------------------------------
 * Test: random notes, directly controlling motor + fingers to guarantee
 * all 5 fingers get used equally.
 *
 * Strategy: cycle through fingers W1→B1→W2→B2→W3, and for each finger
 * pick a random reachable key that uses that finger type (white/black).
 * -------------------------------------------------------------------------- */
#define RANDOM_NOTE_COUNT  50
#define RANDOM_HOLD_MS    200
#define RANDOM_GAP_MS      30

void test_randomKeys(void)
{
    Serial.println("\r\n=== RANDOM KEYS TEST (all fingers) ===");

    /* Finger cycle order — hits all 5 equally */
    const Finger cycle[] = { FINGER_W1, FINGER_B1, FINGER_W2, FINGER_B2, FINGER_W3 };
    const uint8_t cycleLen = 5;
    uint8_t cycleIdx = 0;

    randomSeed(analogRead(0));   /* seed from floating ADC pin */

    for (int n = 0; n < RANDOM_NOTE_COUNT; n++) {
        Finger wantFinger = cycle[cycleIdx];
        cycleIdx = (cycleIdx + 1) % cycleLen;

        /* Collect all reachable options that use wantFinger */
        struct { uint8_t keyIdx; uint8_t optIdx; } candidates[KEY_MAP_SIZE];
        uint8_t candCount = 0;

        for (uint8_t k = 0; k < KEY_MAP_SIZE; k++) {
            for (uint8_t o = 0; o < MAX_FINGER_OPTIONS; o++) {
                const FingerOption* fo = &KEY_MAP[k].options[o];
                if (fo->valid && fo->finger == wantFinger
                    && FingerOption_isReachable(fo)) {
                    candidates[candCount].keyIdx = k;
                    candidates[candCount].optIdx = o;
                    candCount++;
                }
            }
        }

        if (candCount == 0) continue;  /* shouldn't happen */

        /* Pick a random candidate */
        uint8_t pick = random(candCount);
        const PianoKey* key = &KEY_MAP[candidates[pick].keyIdx];
        const FingerOption* opt = &key->options[candidates[pick].optIdx];

        /* Log it */
        char buf[80];
        snprintf(buf, sizeof(buf), "  [%2d] %s (midi %u) finger=%s pos=%.1fmm\r\n",
                 n, key->name, key->midiNote,
                 fingerName(opt->finger), (double)opt->motorPositionMM);
        Serial.print(buf);

        /* Move motor */
        hal_motorSetTarget(opt->motorPositionMM);
        hal_motorWaitUntilArrived();
        hal_delay(HAL_SETTLE_TIME_MS);

        /* Press and hold */
        hal_fingerPress((uint8_t)opt->finger);
        hal_delay(RANDOM_HOLD_MS);
        hal_fingerRelease((uint8_t)opt->finger);
        hal_delay(RANDOM_GAP_MS);
    }

    hal_fingerReleaseAll();
    Serial.println("=== RANDOM KEYS TEST DONE ===\r\n");
}

/* --------------------------------------------------------------------------
 * Test: fire each solenoid in order (no motor movement)
 * Verifies wiring: W1(GPIO21), W2(22), W3(23), B1(25), B2(26)
 * -------------------------------------------------------------------------- */
void test_fingersSolenoidOnly(void)
{
    const char* names[] = {"W1 (GPIO12)", "W2 (GPIO14)", "W3 (GPIO27)",
                           "B1 (GPIO26)", "B2 (GPIO25)"};
    Serial.println("\r\n=== SOLENOID TEST — no motor movement ===");
    for (uint8_t i = 0; i < 5; i++) {
        char buf[48];
        snprintf(buf, sizeof(buf), "  Firing finger %s ...\r\n", names[i]);
        Serial.print(buf);
        hal_fingerPress(i);
        hal_delay(400);
        hal_fingerRelease(i);
        hal_delay(200);
    }
    Serial.println("=== SOLENOID TEST DONE ===\r\n");
}

/* --------------------------------------------------------------------------
 * Test: fire every pair of solenoids simultaneously (no motor movement)
 * 5 fingers → 10 unique pairs: W1+W2, W1+W3, W1+B1, W1+B2,
 *                                W2+W3, W2+B1, W2+B2,
 *                                W3+B1, W3+B2,
 *                                B1+B2
 * -------------------------------------------------------------------------- */
void test_solenoidPairs(void)
{
    const char* names[] = {"W1", "W2", "W3", "B1", "B2"};

    Serial.println("\r\n=== SOLENOID PAIR TEST — no motor movement ===");

    for (uint8_t a = 0; a < 5; a++) {
        for (uint8_t b = a + 1; b < 5; b++) {
            char buf[48];
            snprintf(buf, sizeof(buf), "  Firing %s + %s ...\r\n", names[a], names[b]);
            Serial.print(buf);

            hal_fingerPress(a);
            hal_fingerPress(b);
            hal_delay(400);
            hal_fingerRelease(a);
            hal_fingerRelease(b);
            hal_delay(300);
        }
    }

    hal_fingerReleaseAll();
    Serial.println("=== SOLENOID PAIR TEST DONE ===\r\n");
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
 * Debug: print limit switch state for 10 seconds so we can diagnose wiring
 * -------------------------------------------------------------------------- */
void test_limitSwitchDebug(void)
{
    Serial.println("\r\n=== LIMIT SWITCH DEBUG (10s) ===");
    Serial.print("  home_switch_pin = ");
    Serial.println(app_config::home_switch_pin);
    Serial.print("  active_level = ");
    Serial.println(app_config::home_switch_active_level ? "HIGH" : "LOW");

    for (int i = 0; i < 20; i++) {
        int raw = digitalRead(app_config::home_switch_pin);
        bool active = platform_io_is_home_switch_active();
        char buf[64];
        snprintf(buf, sizeof(buf),
            "  [%2d] raw=%d  active=%s\r\n",
            i, raw, active ? "YES" : "no");
        Serial.print(buf);
        hal_delay(500);
    }
    Serial.println("=== LIMIT SWITCH DEBUG DONE ===\r\n");
}


/* --------------------------------------------------------------------------
 * Debounced limit switch wait — switch must read active for DEBOUNCE_MS
 * consecutive milliseconds before we accept it.
 * -------------------------------------------------------------------------- */
#define HOMING_DEBOUNCE_MS  1U

static void waitForLimitSwitch(void)
{
    uint32_t activeStart = 0;
    bool     timing      = false;

    while (true) {
        hal_runPendingPID();

        if (platform_io_is_home_switch_active()) {
            if (!timing) {
                activeStart = (uint32_t)millis();
                timing = true;
            } else if ((uint32_t)millis() - activeStart >= HOMING_DEBOUNCE_MS) {
                return;  /* debounce passed */
            }
        } else {
            timing = false;  /* switch released, restart */
        }
    }
}


/* --------------------------------------------------------------------------
 * Test: drive negative until limit switch is hit, then play scale
 * -------------------------------------------------------------------------- */
#define LIMIT_SWITCH_HOMING_SPEED  0.2f

void test_limitSwitchThenPlay(void)
{
    Serial.println("\r\n=== LIMIT SWITCH HOME + PLAY TEST ===");
    Serial.println("  Driving negative until limit switch...");

    /* Disable PID so open-loop drive isn't overridden */
    hal_pidSetEnabled(false);

    /* Drive motor in the negative direction */
    motor_control_set_motor_speed(-LIMIT_SWITCH_HOMING_SPEED);

    /* Wait for limit switch (debounced) */
    waitForLimitSwitch();

    /* Stop and zero */
    motor_control_set_motor_speed(0.0f);
    motor_control_zero_position();
    Serial.println("  Limit switch hit — position zeroed.");

    /* Re-enable PID (resets target to 0, clears state) */
    hal_pidSetEnabled(true);

    /* Small delay to let everything settle */
    hal_delay(200);

    /* Play the demo — chromatic sweep up then down */
    Serial.println("  Playing demo...");
    NotePlayer_playSequence(s_demo, s_demoLen);
    Serial.println("=== LIMIT SWITCH HOME + PLAY TEST DONE ===\r\n");
}


/* --------------------------------------------------------------------------
 * Homing — uses app_config::home_switch_pin from config.h
 *
 * To configure:
 *   Set home_switch_pin in config.h (currently -1 = disabled).
 *   Set HOMING_DIRECTION to +1.0f or -1.0f depending on which way is home.
 * -------------------------------------------------------------------------- */
#define HOMING_SPEED        0.3f
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

    /* Disable PID so open-loop drive isn't overridden */
    hal_pidSetEnabled(false);

    /* Drive toward home */
    motor_control_set_motor_speed(HOMING_DIRECTION * HOMING_SPEED);

    /* Wait for limit switch (debounced) */
    waitForLimitSwitch();

    /* Stop and zero */
    motor_control_set_motor_speed(0.0f);
    motor_control_zero_position();

    /* Re-enable PID (resets target to 0, clears state) */
    hal_pidSetEnabled(true);
    Serial.println("Homing complete.");
}
