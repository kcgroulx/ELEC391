/* note_player.c
 * ==========================================================================
 * Blocking single-note playback using the piano keymap.
 *
 * EXECUTION FLOW for NotePlayer_playNote(midiNote, durationMs):
 *
 *   1. Look up the key in KEY_MAP by MIDI number.
 *   2. Ask the key for its best FingerOption given current motor position.
 *      (greedy — picks least travel for THIS note only.)
 *   3. Command the motor to that position.
 *   4. Block until the PID has settled (hal_motorWaitUntilArrived).
 *   5. Wait an extra SETTLE_TIME_MS for vibration to die down.
 *   6. Press the finger.
 *   7. Hold for durationMs * PRESS_DURATION_SCALE.
 *   8. Release the finger.
 *
 * NOTE: bestOption() is greedy and only looks at the current note.
 * Step 4 will replace this with a lookahead planner.
 * ========================================================================== */

#include "note_player.h"
#include "piano_keymap.h"
#include "hal_interface.h"
#include "platform_io.h"
#include "motor_control.h"
#include "pid.h"
#include "Arduino.h"
#include <stdio.h>

/* --------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------- */

static void debugLog(const char* msg) {
    Serial.print(msg);
}

/* Print a single labelled float on one line, e.g. "  pos_mm : 47.30\r\n" */
static void logFloat(const char* label, float val) {
    char buf[64];
    snprintf(buf, sizeof(buf), "  %-18s: %.2f\r\n", label, (double)val);
    Serial.print(buf);
}

/* Print a single labelled int on one line */
static void logInt(const char* label, int32_t val) {
    char buf[64];
    snprintf(buf, sizeof(buf), "  %-18s: %ld\r\n", label, (long)val);
    Serial.print(buf);
}

/* Print a divider + note header before each move */
static void logNoteHeader(const char* noteName, uint8_t midiNote,
                          float targetMM, float currentMM)
{
    char buf[80];
    snprintf(buf, sizeof(buf),
        "\r\n--- NOTE %s (midi %u) ---\r\n", noteName, (unsigned)midiNote);
    Serial.print(buf);
    logFloat("target_mm",  targetMM);
    logFloat("current_mm", currentMM);
    logFloat("travel_mm",  targetMM - currentMM);
    logInt  ("encoder_cnt", platform_io_get_encoder_count());
}

/* Print state after the motor has settled */
static void logArrival(float targetMM)
{
    float actualMM  = hal_motorGetPosition();
    float errorMM   = targetMM - actualMM;
    Serial.print("  [arrived]\r\n");
    logFloat("actual_mm",       actualMM);
    logFloat("error_mm",        errorMM);
    logInt  ("encoder_cnt",     platform_io_get_encoder_count());
    logFloat("pid_err_deg",     pid_get_last_error_deg());
    logFloat("pid_cmd",         pid_get_last_command());
    logFloat("pid_target_deg",  pid_get_last_target_angle_deg());
    logFloat("pid_actual_deg",  pid_get_last_angle_deg());
}


/* --------------------------------------------------------------------------
 * NotePlayer_playNote
 * -------------------------------------------------------------------------- */

int NotePlayer_playNote(uint8_t midiNote, uint32_t durationMs)
{
    const PianoKey* key;
    FingerOption    choice;
    float           currentMM;
    uint32_t        holdMs;

    /* ── 1. Look up key ─────────────────────────────────────────────────── */
    key = keyByMidi(midiNote);
    if (!key) {
        debugLog("playNote: MIDI note not in key map\r\n");
        return 0;
    }

    /* ── 2. Pick best finger option ─────────────────────────────────────── */
    currentMM = hal_motorGetPosition();
    choice    = PianoKey_bestOption(key, currentMM);

    if (!choice.valid || !FingerOption_isReachable(&choice)) {
        debugLog("playNote: no reachable finger option\r\n");
        return 0;
    }

    /* ── 3. Move motor ───────────────────────────────────────────────────── */
    logNoteHeader(key->name, midiNote, choice.motorPositionMM, currentMM);
    {
        char buf[48];
        snprintf(buf, sizeof(buf), "  %-18s: %d\r\n", "finger", (int)choice.finger);
        Serial.print(buf);
    }
    hal_motorSetTarget(choice.motorPositionMM);

    /* ── 4. Wait for arrival ─────────────────────────────────────────────── */
    hal_motorWaitUntilArrived();

    /* ── 5. Settle delay ─────────────────────────────────────────────────── */
    logArrival(choice.motorPositionMM);
    if (HAL_SETTLE_TIME_MS > 0U) {
        hal_delay(HAL_SETTLE_TIME_MS);
    }

    /* ── 6. Press finger ─────────────────────────────────────────────────── */
    {
        char buf[56];
        snprintf(buf, sizeof(buf), "  [press finger %d  hold %lums]\r\n",
                 (int)choice.finger, (unsigned long)durationMs);
        Serial.print(buf);
    }
    hal_fingerPress((uint8_t)choice.finger);

    /* ── 7. Hold ─────────────────────────────────────────────────────────── */
    holdMs = (uint32_t)((float)durationMs * HAL_PRESS_DURATION_SCALE);
    if (holdMs < HAL_MIN_PRESS_MS) holdMs = HAL_MIN_PRESS_MS;
    hal_delay(holdMs);

    /* ── 8. Release ──────────────────────────────────────────────────────── */
    debugLog("  [release]\r\n");
    hal_fingerRelease((uint8_t)choice.finger);

    return 1;
}


/* --------------------------------------------------------------------------
 * NotePlayer_playNoteByName
 * -------------------------------------------------------------------------- */

int NotePlayer_playNoteByName(const char* name, uint32_t durationMs)
{
    const PianoKey* key = keyByName(name);
    if (!key) {
        debugLog("playNoteByName: note name not found\r\n");
        return 0;
    }
    return NotePlayer_playNote(key->midiNote, durationMs);
}


/* --------------------------------------------------------------------------
 * Finger picker with variety
 *
 * All finger options for a given key land on the same motor position, so
 * travel cost can't differentiate them. Instead we:
 *   1. Penalise reusing the same finger as last note (strong)
 *   2. Penalise reusing the same finger as next note's best option (mild)
 *   3. Among ties, prefer cycling through fingers in order
 *
 * This distributes usage across W1/W2/W3 for white keys and B1/B2 for black.
 * -------------------------------------------------------------------------- */

static FingerOption pickBestOption(const PianoKey* key,
                                   float currentMM,
                                   const PianoKey* nextKey,
                                   Finger lastFinger)
{
    FingerOption best;
    float bestCost = 1e9f;
    uint8_t i;

    best.finger          = FINGER_NONE;
    best.motorPositionMM = -1.0f;
    best.valid           = 0;

    for (i = 0; i < MAX_FINGER_OPTIONS; i++) {
        const FingerOption* opt = &key->options[i];
        float cost, travelHere;

        if (!FingerOption_isReachable(opt)) continue;

        /* Cost to reach this option from current position */
        travelHere = opt->motorPositionMM - currentMM;
        if (travelHere < 0.0f) travelHere = -travelHere;

        cost = travelHere;

        /* Strong penalty for reusing the finger we just used */
        if (opt->finger == lastFinger) {
            cost += 100.0f;
        }

        /* Mild penalty if next note would want this same finger */
        if (nextKey) {
            uint8_t j;
            for (j = 0; j < MAX_FINGER_OPTIONS; j++) {
                const FingerOption* nopt = &nextKey->options[j];
                if (!FingerOption_isReachable(nopt)) continue;
                /* If the next note's first reachable option uses this finger,
                 * add a small penalty so we leave it free for the next note */
                if (nopt->finger == opt->finger) {
                    cost += 10.0f;
                }
                break;  /* only check the first reachable next option */
            }
        }

        if (cost < bestCost) {
            bestCost = cost;
            best     = *opt;
        }
    }
    return best;
}


/* --------------------------------------------------------------------------
 * playNoteWithOption — like NotePlayer_playNote but with a pre-chosen option
 * -------------------------------------------------------------------------- */

static int playNoteWithOption(const PianoKey* key, const FingerOption* choice,
                              uint32_t durationMs)
{
    float currentMM = hal_motorGetPosition();
    uint32_t holdMs;

    if (!choice->valid || !FingerOption_isReachable(choice)) {
        debugLog("playNote: no reachable finger option\r\n");
        return 0;
    }

    /* Move motor */
    logNoteHeader(key->name, key->midiNote, choice->motorPositionMM, currentMM);
    {
        char buf[48];
        snprintf(buf, sizeof(buf), "  %-18s: %s\r\n", "finger",
                 fingerName(choice->finger));
        Serial.print(buf);
    }
    hal_motorSetTarget(choice->motorPositionMM);

    /* Wait for arrival */
    hal_motorWaitUntilArrived();
    logArrival(choice->motorPositionMM);
    if (HAL_SETTLE_TIME_MS > 0U) {
        hal_delay(HAL_SETTLE_TIME_MS);
    }

    /* Press */
    {
        char buf[56];
        snprintf(buf, sizeof(buf), "  [press finger %s  hold %lums]\r\n",
                 fingerName(choice->finger), (unsigned long)durationMs);
        Serial.print(buf);
    }
    hal_fingerPress((uint8_t)choice->finger);

    /* Hold */
    holdMs = (uint32_t)((float)durationMs * HAL_PRESS_DURATION_SCALE);
    if (holdMs < HAL_MIN_PRESS_MS) holdMs = HAL_MIN_PRESS_MS;
    hal_delay(holdMs);

    /* Release */
    debugLog("  [release]\r\n");
    hal_fingerRelease((uint8_t)choice->finger);

    return 1;
}


/* --------------------------------------------------------------------------
 * NotePlayer_playSequence — stable nearest-position playback
 * -------------------------------------------------------------------------- */

void NotePlayer_playSequence(const NoteEvent* events, uint16_t count)
{
    uint16_t i;
    float currentMM = hal_motorGetPosition();

    for (i = 0; i < count; i++) {
        const NoteEvent* e = &events[i];
        const PianoKey* key = keyByMidi(e->midiNote);
        FingerOption choice;
        int ok;

        if (!key) {
            hal_delay((uint32_t)((float)e->durationMs * HAL_PRESS_DURATION_SCALE));
            if (e->delayAfterMs > 0U) hal_delay(e->delayAfterMs);
            continue;
        }

        /* For uploaded MIDI, prefer the nearest reachable option.
         * This keeps carriage travel predictable on repeated-note melodies
         * and avoids large back-and-forth moves just to vary fingers. */
        choice = PianoKey_bestOption(key, currentMM);

        /* Play */
        ok = playNoteWithOption(key, &choice, e->durationMs);
        if (!ok) {
            hal_delay((uint32_t)((float)e->durationMs * HAL_PRESS_DURATION_SCALE));
        } else {
            currentMM = choice.motorPositionMM;
        }

        /* Gap between notes */
        if (e->delayAfterMs > 0U) {
            hal_delay(e->delayAfterMs);
        }
    }
}


/* --------------------------------------------------------------------------
 * State queries
 * -------------------------------------------------------------------------- */

float NotePlayer_getCurrentPositionMM(void)
{
    return hal_motorGetPosition();
}

uint8_t NotePlayer_getCurrentNote(void)
{
    float   posMM      = hal_motorGetPosition();
    uint8_t closest    = 255U;
    float   closestDist = HAL_ARRIVAL_TOLERANCE_MM * 2.0f;
    uint8_t i;
    uint8_t j;

    for (i = 0; i < KEY_MAP_SIZE; i++) {
        for (j = 0; j < MAX_FINGER_OPTIONS; j++) {
            const FingerOption* o = &KEY_MAP[i].options[j];
            float dist;
            if (!o->valid) continue;
            dist = o->motorPositionMM - posMM;
            if (dist < 0.0f) dist = -dist;
            if (dist < closestDist) {
                closestDist = dist;
                closest     = KEY_MAP[i].midiNote;
            }
        }
    }
    return closest;
}
