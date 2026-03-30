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


/* --------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------- */

/* Optional debug output — wire up to your UART if needed.
 * Replace the body with e.g.:
 *   HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
 */
static void debugLog(const char* msg) {
    (void)msg;
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
    hal_motorSetTarget(choice.motorPositionMM);

    /* ── 4. Wait for arrival ─────────────────────────────────────────────── */
    hal_motorWaitUntilArrived();

    /* ── 5. Settle delay ─────────────────────────────────────────────────── */
    if (HAL_SETTLE_TIME_MS > 0U) {
        hal_delay(HAL_SETTLE_TIME_MS);
    }

    /* ── 6. Press finger ─────────────────────────────────────────────────── */
    hal_fingerPress((uint8_t)choice.finger);

    /* ── 7. Hold ─────────────────────────────────────────────────────────── */
    holdMs = (uint32_t)((float)durationMs * HAL_PRESS_DURATION_SCALE);
    if (holdMs < HAL_MIN_PRESS_MS) holdMs = HAL_MIN_PRESS_MS;
    hal_delay(holdMs);

    /* ── 8. Release ──────────────────────────────────────────────────────── */
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
 * NotePlayer_playSequence
 * -------------------------------------------------------------------------- */

void NotePlayer_playSequence(const NoteEvent* events, uint16_t count)
{
    uint16_t i;
    for (i = 0; i < count; i++) {
        const NoteEvent* e = &events[i];
        int ok;

        /* Play the note — blocks until finger releases */
        ok = NotePlayer_playNote(e->midiNote, e->durationMs);

        /* If unreachable, still wait the note's duration so rhythm is kept.
         * The song keeps going even if a note is skipped. */
        if (!ok) {
            hal_delay((uint32_t)((float)e->durationMs * HAL_PRESS_DURATION_SCALE));
        }

        /* Gap / rest between notes */
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
