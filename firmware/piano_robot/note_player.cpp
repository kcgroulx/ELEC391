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
 *   5. Press the finger as soon as the carriage is strike-ready.
 *   6. Hold for durationMs * PRESS_DURATION_SCALE.
 *   7. Release the finger.
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
#include <string.h>

#define NOTE_PLAYER_ENABLE_DEBUG 0

/* --------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------- */

static void debugLog(const char* msg) {
#if NOTE_PLAYER_ENABLE_DEBUG
    Serial.print(msg);
#else
    (void)msg;
#endif
}

/* Print a single labelled float on one line, e.g. "  pos_mm : 47.30\r\n" */
static void logFloat(const char* label, float val) {
#if NOTE_PLAYER_ENABLE_DEBUG
    char buf[64];
    snprintf(buf, sizeof(buf), "  %-18s: %.2f\r\n", label, (double)val);
    Serial.print(buf);
#else
    (void)label;
    (void)val;
#endif
}

/* Print a single labelled int on one line */
static void logInt(const char* label, int32_t val) {
#if NOTE_PLAYER_ENABLE_DEBUG
    char buf[64];
    snprintf(buf, sizeof(buf), "  %-18s: %ld\r\n", label, (long)val);
    Serial.print(buf);
#else
    (void)label;
    (void)val;
#endif
}

/* Print a divider + note header before each move */
static void logNoteHeader(const char* noteName, uint8_t midiNote,
                          float targetMM, float currentMM)
{
#if NOTE_PLAYER_ENABLE_DEBUG
    char buf[80];
    snprintf(buf, sizeof(buf),
        "\r\n--- NOTE %s (midi %u) ---\r\n", noteName, (unsigned)midiNote);
    Serial.print(buf);
    logFloat("target_mm",  targetMM);
    logFloat("current_mm", currentMM);
    logFloat("travel_mm",  targetMM - currentMM);
    logInt  ("encoder_cnt", platform_io_get_encoder_count());
#else
    (void)noteName;
    (void)midiNote;
    (void)targetMM;
    (void)currentMM;
#endif
}

/* Print state after the motor has settled */
static void logArrival(float targetMM)
{
#if NOTE_PLAYER_ENABLE_DEBUG
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
#else
    (void)targetMM;
#endif
}

static uint32_t pressTickForNoteStart(uint32_t noteStartTick)
{
    return noteStartTick - HAL_SOLENOID_STRIKE_LEAD_MS;
}

static uint32_t scaledHoldMs(uint32_t durationMs)
{
    uint32_t holdMs = (uint32_t)((float)durationMs * HAL_PRESS_DURATION_SCALE);
    if (holdMs < HAL_MIN_PRESS_MS) {
        holdMs = HAL_MIN_PRESS_MS;
    }
    return holdMs;
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
        debugLog(buf);
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
        debugLog(buf);
    }
    hal_fingerPress((uint8_t)choice.finger);

    /* ── 7. Hold ─────────────────────────────────────────────────────────── */
    holdMs = scaledHoldMs(durationMs);
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
 * 5-note lookahead finger/position picker  (dynamic programming)
 *
 * For the next LOOKAHEAD_DEPTH notes, builds a table of reachable motor
 * positions, then uses forward DP to find the path (sequence of finger
 * options) that minimises total carriage travel.  Returns the optimal
 * FingerOption for the FIRST note in the window.
 *
 * Complexity:  O(depth * MAX_FINGER_OPTIONS^2)  ≈  5 * 9 = 45 iterations
 *              — negligible on ESP32 at 240 MHz.
 *
 * Falls back to nearest-option if lookup fails (note out of range, etc.).
 * -------------------------------------------------------------------------- */

#define LOOKAHEAD_DEPTH  5U

typedef struct {
    float   motorMM[MAX_FINGER_OPTIONS];
    Finger  finger[MAX_FINGER_OPTIONS];
    uint8_t optIdx[MAX_FINGER_OPTIONS];   /* index into PianoKey.options[] */
    uint8_t count;                        /* how many reachable options    */
} LookaheadSlot;

static FingerOption lookaheadPick(const NoteEvent* events,
                                  uint16_t startIdx,
                                  uint16_t totalCount,
                                  float currentMM)
{
    LookaheadSlot window[LOOKAHEAD_DEPTH];
    const PianoKey* keys[LOOKAHEAD_DEPTH];
    float    dp[LOOKAHEAD_DEPTH][MAX_FINGER_OPTIONS];
    uint8_t  parent[LOOKAHEAD_DEPTH][MAX_FINGER_OPTIONS];
    uint8_t  depth = 0;
    uint8_t  i, j, k;

    /* ── 1. Collect reachable options for each note in the window ──────── */
    for (i = 0; i < LOOKAHEAD_DEPTH && (startIdx + i) < totalCount; i++) {
        const PianoKey* key = keyByMidi(events[startIdx + i].midiNote);
        if (!key) break;

        keys[i] = key;
        window[i].count = 0;

        for (j = 0; j < MAX_FINGER_OPTIONS; j++) {
            const FingerOption* fo = &key->options[j];
            if (!FingerOption_isReachable(fo)) continue;
            uint8_t c = window[i].count;
            window[i].motorMM[c] = fo->motorPositionMM;
            window[i].finger[c]  = fo->finger;
            window[i].optIdx[c]  = j;
            window[i].count++;
        }
        if (window[i].count == 0) break;
        depth++;

        /* Stop at chord boundary — if next note has delayAfterMs == 0
         * it's part of a chord group that the chord planner handles. */
        if (events[startIdx + i].delayAfterMs == 0 && (startIdx + i + 1) < totalCount) {
            break;
        }
    }

    /* Fallback: no reachable notes in window */
    if (depth == 0) {
        FingerOption invalid = { FINGER_NONE, -1.0f, 0 };
        return invalid;
    }

    /* Single note — just pick nearest */
    if (depth == 1) {
        return PianoKey_bestOption(keys[0], currentMM);
    }

    /* ── 2. Forward DP — dp[i][j] = min total travel to reach option j
     *       of note i.  parent[i][j] = which option at note i-1 led here. */

    /* Base case: travel from current position to each option of note 0 */
    for (j = 0; j < window[0].count; j++) {
        float d = window[0].motorMM[j] - currentMM;
        if (d < 0.0f) d = -d;
        dp[0][j] = d;
        parent[0][j] = 0;  /* unused for level 0 */
    }

    /* Fill levels 1 .. depth-1 */
    for (i = 1; i < depth; i++) {
        for (j = 0; j < window[i].count; j++) {
            dp[i][j] = 1e9f;
            parent[i][j] = 0;
            for (k = 0; k < window[i - 1].count; k++) {
                float d = window[i].motorMM[j] - window[i - 1].motorMM[k];
                if (d < 0.0f) d = -d;
                float cost = dp[i - 1][k] + d;
                if (cost < dp[i][j]) {
                    dp[i][j] = cost;
                    parent[i][j] = k;
                }
            }
        }
    }

    /* ── 3. Trace back from the best option at the last level to level 0 ── */
    /* Find best at last level */
    uint8_t bestEnd = 0;
    float bestEndCost = dp[depth - 1][0];
    for (j = 1; j < window[depth - 1].count; j++) {
        if (dp[depth - 1][j] < bestEndCost) {
            bestEndCost = dp[depth - 1][j];
            bestEnd = j;
        }
    }

    /* Walk parent pointers back to level 0 */
    uint8_t trace = bestEnd;
    for (i = depth - 1; i > 0; i--) {
        trace = parent[i][trace];
    }
    /* trace is now the optimal option index at level 0 */

    /* Return the corresponding FingerOption from the PianoKey */
    return keys[0]->options[window[0].optIdx[trace]];
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
        debugLog(buf);
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
        debugLog(buf);
    }
    hal_fingerPress((uint8_t)choice->finger);

    /* Hold */
    holdMs = scaledHoldMs(durationMs);
    hal_delay(holdMs);

    /* Release */
    debugLog("  [release]\r\n");
    hal_fingerRelease((uint8_t)choice->finger);

    return 1;
}


/* --------------------------------------------------------------------------
 * Chord support
 *
 * A "chord group" is a set of consecutive NoteEvents whose start times
 * fall within CHORD_WINDOW_MS of each other.  The planner finds the
 * single carriage position that lets the most notes (up to MAX_CHORD)
 * be played simultaneously, then fires all assigned solenoids at once.
 *
 * Algorithm:
 *   1. Collect chord group (up to MAX_CHORD notes within the window).
 *   2. For every candidate motor position (each finger option of each
 *      note in the group), score how many OTHER notes in the group
 *      can be reached by a different finger at that same position.
 *      White keys are prioritised for assignment.
 *   3. Pick the position with the highest score.
 *   4. Move carriage once, press all assigned fingers, hold for the
 *      shortest note duration, release all.
 * -------------------------------------------------------------------------- */

#define CHORD_WINDOW_MS   30U    /* notes within this window are one chord   */
#define MAX_CHORD          3U    /* max simultaneous fingers                 */

/* One assignment: which finger plays which note at what position */
typedef struct {
    uint8_t  midiNote;
    Finger   finger;
    float    motorMM;       /* all entries share the same motor position     */
    uint8_t  assigned;      /* 1 = will be played                           */
} ChordSlot;

static uint8_t gatherChordGroup(const NoteEvent* events,
                                uint16_t startIdx,
                                uint16_t totalCount,
                                const PianoKey* chordNotes[MAX_CHORD],
                                const NoteEvent* chordEvents[MAX_CHORD])
{
    uint8_t chordSize = 1;
    uint16_t j;

    chordNotes[0]  = keyByMidi(events[startIdx].midiNote);
    chordEvents[0] = &events[startIdx];

    for (j = startIdx + 1; j < totalCount && chordSize < MAX_CHORD; j++) {
        uint32_t gap = 0;
        uint16_t k;

        for (k = startIdx; k < j; k++) {
            gap += events[k].delayAfterMs;
        }
        if (gap > CHORD_WINDOW_MS) break;

        chordNotes[chordSize]  = keyByMidi(events[j].midiNote);
        chordEvents[chordSize] = &events[j];
        chordSize++;
    }

    return chordSize;
}

static uint8_t firstAssignedChordPosition(const ChordSlot slots[MAX_CHORD],
                                          uint8_t chordSize,
                                          float* motorMM)
{
    uint8_t s;

    for (s = 0; s < chordSize; s++) {
        if (slots[s].assigned) {
            *motorMM = slots[s].motorMM;
            return 1;
        }
    }
    return 0;
}

/* Try to assign fingers for notes[] at a given candidate motor position.
 * Returns how many notes got a unique finger.  Fills slots[].
 * White-key notes are assigned first (prioritised). */
static uint8_t tryChordAtPosition(float candidateMM,
                                  const PianoKey* notes[], uint8_t noteCount,
                                  ChordSlot slots[MAX_CHORD])
{
    uint8_t usedFingers = 0;   /* bitmask of Finger enum values already taken */
    uint8_t assigned = 0;
    uint8_t pass, n, o;

    /* Clear slots */
    for (n = 0; n < MAX_CHORD; n++) {
        slots[n].assigned = 0;
    }

    /* Two passes: pass 0 = white keys first, pass 1 = black keys */
    for (pass = 0; pass < 2; pass++) {
        for (n = 0; n < noteCount && assigned < MAX_CHORD; n++) {
            if (slots[n].assigned) continue;
            if (!notes[n]) continue;

            /* Pass 0: white only.  Pass 1: black only. */
            if (pass == 0 && notes[n]->type != KEY_WHITE) continue;
            if (pass == 1 && notes[n]->type != KEY_BLACK) continue;

            /* Find a finger option at candidateMM that isn't already used */
            for (o = 0; o < MAX_FINGER_OPTIONS; o++) {
                const FingerOption* fo = &notes[n]->options[o];
                float diff;
                if (!FingerOption_isReachable(fo)) continue;

                diff = fo->motorPositionMM - candidateMM;
                if (diff < 0.0f) diff = -diff;
                if (diff > HAL_ARRIVAL_TOLERANCE_MM) continue;

                /* Check finger not already taken */
                if (usedFingers & (1U << (uint8_t)fo->finger)) continue;

                /* Assign it */
                slots[n].midiNote = notes[n]->midiNote;
                slots[n].finger   = fo->finger;
                slots[n].motorMM  = candidateMM;
                slots[n].assigned = 1;
                usedFingers |= (1U << (uint8_t)fo->finger);
                assigned++;
                break;
            }
        }
    }
    return assigned;
}

/* Find the best carriage position for a chord group.
 * Returns number of notes assigned; fills bestSlots[]. */
static uint8_t planChord(const PianoKey* notes[], uint8_t noteCount,
                         float currentMM,
                         ChordSlot bestSlots[MAX_CHORD])
{
    uint8_t bestCount = 0;
    float   bestTravel = 1e9f;
    uint8_t n, o;
    ChordSlot trial[MAX_CHORD];

    /* Try every motor position that any note in the group can use */
    for (n = 0; n < noteCount; n++) {
        if (!notes[n]) continue;
        for (o = 0; o < MAX_FINGER_OPTIONS; o++) {
            const FingerOption* fo = &notes[n]->options[o];
            float candidateMM, travel;
            uint8_t count;

            if (!FingerOption_isReachable(fo)) continue;
            candidateMM = fo->motorPositionMM;

            count = tryChordAtPosition(candidateMM, notes, noteCount, trial);

            /* Pick position that covers the most notes; break ties by
             * least travel from current position */
            travel = candidateMM - currentMM;
            if (travel < 0.0f) travel = -travel;

            if (count > bestCount ||
                (count == bestCount && travel < bestTravel)) {
                bestCount  = count;
                bestTravel = travel;
                for (uint8_t s = 0; s < MAX_CHORD; s++) {
                    bestSlots[s] = trial[s];
                }
            }
        }
    }
    return bestCount;
}

typedef enum {
    PLAYBACK_STATE_IDLE = 0,
    PLAYBACK_STATE_PREPOSITION,
    PLAYBACK_STATE_MOVE,
    PLAYBACK_STATE_WAIT_STRIKE,
    PLAYBACK_STATE_STRIKE,
    PLAYBACK_STATE_RELEASE,
    PLAYBACK_STATE_DONE,
    PLAYBACK_STATE_FAULT
} PlaybackState;

typedef struct {
    uint8_t  midiNotes[MAX_CHORD];
    Finger   fingers[MAX_CHORD];
    uint8_t  playedCount;
    float    targetMM;
    uint32_t startOffsetMs;
    uint32_t intervalToNextStartMs;
    uint32_t strikeDurationMs;
} StrikeGroup;

#define NOTE_PLAYER_MAX_GROUPS 256U
static StrikeGroup s_strikeGroups[NOTE_PLAYER_MAX_GROUPS];

static const char* playbackStateName(PlaybackState state)
{
    switch (state) {
        case PLAYBACK_STATE_IDLE:         return "IDLE";
        case PLAYBACK_STATE_PREPOSITION:  return "PREPOSITION";
        case PLAYBACK_STATE_MOVE:         return "MOVE";
        case PLAYBACK_STATE_WAIT_STRIKE:  return "WAIT_STRIKE";
        case PLAYBACK_STATE_STRIKE:       return "STRIKE";
        case PLAYBACK_STATE_RELEASE:      return "RELEASE";
        case PLAYBACK_STATE_DONE:         return "DONE";
        case PLAYBACK_STATE_FAULT:        return "FAULT";
        default:                          return "UNKNOWN";
    }
}

static void emitPlaybackState(PlaybackState state,
                              uint16_t groupIdx,
                              uint16_t groupCount,
                              int32_t lateMs)
{
    char buf[96];
    snprintf(buf, sizeof(buf),
        "[PLAY] state=%s group=%u/%u late=%ldms\r\n",
        playbackStateName(state),
        (unsigned)(groupIdx + 1U),
        (unsigned)groupCount,
        (long)lateMs);
    Serial.print(buf);
}

static uint32_t staccatoStrikeDurationMs(uint32_t intervalToNextStartMs)
{
    uint32_t strikeMs = HAL_STACCATO_STRIKE_MS;
    uint32_t maxStrikeMs = intervalToNextStartMs;

    if (intervalToNextStartMs > HAL_SOLENOID_STRIKE_LEAD_MS) {
        maxStrikeMs = intervalToNextStartMs - HAL_SOLENOID_STRIKE_LEAD_MS;
    }

    if (maxStrikeMs > 0U && strikeMs > maxStrikeMs) {
        strikeMs = maxStrikeMs;
    }
    if (strikeMs < HAL_MIN_PRESS_MS) {
        strikeMs = HAL_MIN_PRESS_MS;
    }
    return strikeMs;
}

static uint16_t buildStrikeGroups(const NoteEvent* events,
                                  uint16_t count,
                                  float initialMM,
                                  StrikeGroup groups[NOTE_PLAYER_MAX_GROUPS])
{
    uint16_t eventIdx = 0U;
    uint16_t groupIdx = 0U;
    uint32_t startOffsetMs = 0U;
    float plannedMM = initialMM;

    while (eventIdx < count && groupIdx < NOTE_PLAYER_MAX_GROUPS) {
        const PianoKey* chordNotes[MAX_CHORD];
        const NoteEvent* chordEvents[MAX_CHORD];
        uint8_t chordSize = gatherChordGroup(
            events, eventIdx, count, chordNotes, chordEvents);
        StrikeGroup* group = &groups[groupIdx];
        uint32_t intervalToNextStartMs = 0U;
        uint8_t s;

        memset(group, 0, sizeof(*group));
        group->targetMM = plannedMM;
        group->startOffsetMs = startOffsetMs;

        for (s = 0; s < chordSize; s++) {
            uint32_t noteInterval =
                chordEvents[s]->durationMs + chordEvents[s]->delayAfterMs;
            if (noteInterval > intervalToNextStartMs) {
                intervalToNextStartMs = noteInterval;
            }
        }
        group->intervalToNextStartMs = intervalToNextStartMs;
        group->strikeDurationMs = staccatoStrikeDurationMs(intervalToNextStartMs);

        if (chordSize == 1U) {
            FingerOption choice = lookaheadPick(events, eventIdx, count, plannedMM);
            if (choice.valid && FingerOption_isReachable(&choice)) {
                group->midiNotes[0] = events[eventIdx].midiNote;
                group->fingers[0] = choice.finger;
                group->playedCount = 1U;
                group->targetMM = choice.motorPositionMM;
                plannedMM = group->targetMM;
            }
        } else {
            ChordSlot slots[MAX_CHORD];
            uint8_t assigned = planChord(chordNotes, chordSize, plannedMM, slots);
            float chordMM = plannedMM;

            if (assigned > 0U &&
                firstAssignedChordPosition(slots, chordSize, &chordMM)) {
                uint8_t playIdx = 0U;
                group->targetMM = chordMM;
                for (s = 0; s < chordSize; s++) {
                    if (!slots[s].assigned) continue;
                    group->midiNotes[playIdx] = slots[s].midiNote;
                    group->fingers[playIdx] = slots[s].finger;
                    playIdx++;
                }
                group->playedCount = playIdx;
                plannedMM = chordMM;
            }
        }

        startOffsetMs += intervalToNextStartMs;
        eventIdx += chordSize;
        groupIdx++;
    }

    return groupIdx;
}


/* --------------------------------------------------------------------------
 * NotePlayer_playSequence — beat-driven staccato group playback
 * -------------------------------------------------------------------------- */

void NotePlayer_playSequence(const NoteEvent* events, uint16_t count)
{
    float initialMM = hal_motorGetPosition();
    uint16_t groupCount = buildStrikeGroups(events, count, initialMM, s_strikeGroups);
    uint16_t groupIdx = 0U;
    PlaybackState state;
    bool stateEntered = true;
    uint32_t songStartTick;
    uint32_t strikeReleaseTick = 0U;
    uint32_t timingSlipMs = 0U;
    int32_t lateMs = 0;

    if (count == 0U || groupCount == 0U) {
        return;
    }

    emitPlaybackState(PLAYBACK_STATE_IDLE, 0U, groupCount, 0);

    if (s_strikeGroups[0].playedCount > 0U) {
        emitPlaybackState(PLAYBACK_STATE_PREPOSITION, 0U, groupCount, 0);
        if (fabsf(s_strikeGroups[0].targetMM - hal_motorGetPosition()) > HAL_ARRIVAL_TOLERANCE_MM) {
            hal_motorSetTarget(s_strikeGroups[0].targetMM);
            hal_motorWaitUntilArrived();
        }
    }

    songStartTick = hal_getTick() + HAL_SOLENOID_STRIKE_LEAD_MS;
    state = PLAYBACK_STATE_WAIT_STRIKE;

    while (state != PLAYBACK_STATE_DONE && state != PLAYBACK_STATE_FAULT) {
        StrikeGroup* group = &s_strikeGroups[groupIdx];
        uint32_t noteStartTick = songStartTick + group->startOffsetMs + timingSlipMs;
        uint32_t pressTick = pressTickForNoteStart(noteStartTick);
        uint32_t now = hal_getTick();
        uint8_t s;

        hal_runPendingPID();

        switch (state) {
            case PLAYBACK_STATE_MOVE:
                if (stateEntered) {
                    emitPlaybackState(state, groupIdx, groupCount, 0);
                    stateEntered = false;

                    if (fabsf(group->targetMM - hal_motorGetPosition()) > HAL_ARRIVAL_TOLERANCE_MM) {
                        hal_motorSetTarget(group->targetMM);
                    }
                }

                if (hal_motorHasArrived() ||
                    fabsf(group->targetMM - hal_motorGetPosition()) <= HAL_ARRIVAL_TOLERANCE_MM) {
                    if ((int32_t)(pressTick - now) > 0) {
                        state = PLAYBACK_STATE_WAIT_STRIKE;
                        stateEntered = true;
                    } else {
                        lateMs = (int32_t)(now - pressTick);
                        state = PLAYBACK_STATE_STRIKE;
                        stateEntered = true;
                    }
                }
                break;

            case PLAYBACK_STATE_WAIT_STRIKE:
                if (stateEntered) {
                    emitPlaybackState(state, groupIdx, groupCount, 0);
                    stateEntered = false;
                }

                if ((int32_t)(pressTick - now) <= 0) {
                    lateMs = (now > pressTick) ? (int32_t)(now - pressTick) : 0;
                    state = PLAYBACK_STATE_STRIKE;
                    stateEntered = true;
                }
                break;

            case PLAYBACK_STATE_STRIKE:
                if (stateEntered) {
                    emitPlaybackState(state, groupIdx, groupCount, lateMs);
                    if (lateMs > 0) {
                        timingSlipMs += (uint32_t)lateMs;
                    }
                    for (s = 0; s < group->playedCount; s++) {
                        hal_fingerPress((uint8_t)group->fingers[s]);
                    }
                    strikeReleaseTick = now + group->strikeDurationMs;
                    stateEntered = false;
                }

                if ((int32_t)(strikeReleaseTick - now) <= 0) {
                    state = PLAYBACK_STATE_RELEASE;
                    stateEntered = true;
                }
                break;

            case PLAYBACK_STATE_RELEASE:
                if (stateEntered) {
                    emitPlaybackState(state, groupIdx, groupCount, 0);
                    hal_fingerReleaseAll();
                    stateEntered = false;
                }

                if ((groupIdx + 1U) >= groupCount) {
                    state = PLAYBACK_STATE_DONE;
                    stateEntered = true;
                } else {
                    groupIdx++;
                    lateMs = 0;
                    state = PLAYBACK_STATE_MOVE;
                    stateEntered = true;
                }
                break;

            default:
                state = PLAYBACK_STATE_FAULT;
                stateEntered = true;
                break;
        }
    }

    if (state == PLAYBACK_STATE_DONE) {
        emitPlaybackState(PLAYBACK_STATE_DONE, groupCount - 1U, groupCount, 0);
        hal_fingerReleaseAll();
    } else {
        emitPlaybackState(PLAYBACK_STATE_FAULT, groupIdx, groupCount, lateMs);
        hal_fingerReleaseAll();
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
