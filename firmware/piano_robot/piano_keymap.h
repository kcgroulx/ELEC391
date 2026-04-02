#ifndef PIANO_KEYMAP_H
#define PIANO_KEYMAP_H
#include <stddef.h>
/* piano_keymap.h
 * ==========================================================================
 * Physical and musical layout of the robot piano system.
 *
 * KEY DESIGN PRINCIPLE:
 *   Every key stores ALL valid (motorPosition, finger) combinations.
 *   The position planner picks whichever option requires the least travel
 *   from the current motor position.
 *
 * TO CALIBRATE:
 *   1. Set WHITE_KEY_WIDTH_MM  — measure center-to-center of adjacent white
 *      keys on your piano (standard ≈ 23.5 mm).
 *   2. Set MOTOR_ORIGIN_MM    — motor position (mm) when W1 is over C2.
 *   3. Set MOTOR_MAX_MM       — furthest the carriage can safely travel.
 *   4. Set FINGER_SPACING_MM  — distance from a white finger to the adjacent
 *      black finger. Ideally equals WHITE_KEY_WIDTH_MM.
 *
 * COORDINATE SYSTEM:
 *   All positions are mm from MOTOR_ORIGIN_MM.
 *   Increasing mm = moving right across the keyboard.
 * ========================================================================== */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/* --------------------------------------------------------------------------
 * Hardware geometry — EDIT THESE
 * -------------------------------------------------------------------------- */

#define WHITE_KEY_WIDTH_MM   23.5f   /* white key center-to-center (mm)   looks to be around 23.5mm */ 
#define MOTOR_ORIGIN_MM       0.0f   /* encoder zero = W1 over C2            */
#define MOTOR_MIN_MM          0.0f   /* left travel limit                    */
#define MOTOR_MAX_MM        380.0f   /* physical carriage limit                */
#define FINGER_SPACING_MM   WHITE_KEY_WIDTH_MM  /* white-to-black finger gap */ //MAYBE UPDATE THIS ?!

#define KEY_MAP_SIZE        37U      /* C2 (midi 36) to C5 (midi 72)         */
#define MIDI_NOTE_MIN       36U      /* C2                                    */
#define MIDI_NOTE_MAX       72U      /* C5                                    */
#define MAX_FINGER_OPTIONS   3U      /* max (finger, position) pairs per key  */


/* --------------------------------------------------------------------------
 * Finger identifiers
 * -------------------------------------------------------------------------- */

typedef enum {
    FINGER_W1   = 0,   /* leftmost white-key finger                          */
    FINGER_W2   = 1,   /* middle white-key finger                            */
    FINGER_W3   = 2,   /* rightmost white-key finger                         */
    FINGER_B1   = 3,   /* black-key finger between W1 and W2                 */
    FINGER_B2   = 4,   /* black-key finger between W2 and W3                 */
    FINGER_NONE = 5
} Finger;

static inline const char* fingerName(Finger f) {
    switch (f) {
        case FINGER_W1:   return "W1";
        case FINGER_W2:   return "W2";
        case FINGER_W3:   return "W3";
        case FINGER_B1:   return "B1";
        case FINGER_B2:   return "B2";
        default:          return "NONE";
    }
}


/* --------------------------------------------------------------------------
 * Key type
 * -------------------------------------------------------------------------- */

typedef enum {
    KEY_WHITE = 0,
    KEY_BLACK = 1
} KeyType;


/* --------------------------------------------------------------------------
 * FingerOption — one valid way to play a key
 * -------------------------------------------------------------------------- */

typedef struct {
    Finger   finger;
    float    motorPositionMM;
    uint8_t  valid;          /* 1 = this slot is populated, 0 = unused        */
} FingerOption;

/* Returns 1 if this option is populated and within motor travel limits. */
static inline int FingerOption_isReachable(const FingerOption* o) {
    return o->valid
        && o->motorPositionMM >= MOTOR_MIN_MM
        && o->motorPositionMM <= MOTOR_MAX_MM;
}


/* --------------------------------------------------------------------------
 * PianoKey — one entry per key, up to MAX_FINGER_OPTIONS ways to play it
 * -------------------------------------------------------------------------- */

typedef struct {
    uint8_t      midiNote;
    const char*  name;
    KeyType      type;
    FingerOption options[MAX_FINGER_OPTIONS];
} PianoKey;

/* Returns the reachable option whose motor position is closest to
 * currentMotorMM. Returns an invalid option if nothing is reachable. */
static inline FingerOption PianoKey_bestOption(const PianoKey* key,
                                               float currentMotorMM)
{
    FingerOption best;
    float bestDist = 1e9f;
    uint8_t i;

    best.finger          = FINGER_NONE;
    best.motorPositionMM = -1.0f;
    best.valid           = 0;

    for (i = 0; i < MAX_FINGER_OPTIONS; i++) {
        const FingerOption* o = &key->options[i];
        float dist;
        if (!FingerOption_isReachable(o)) continue;
        dist = o->motorPositionMM - currentMotorMM;
        if (dist < 0.0f) dist = -dist;
        if (dist < bestDist) {
            bestDist = dist;
            best     = *o;
        }
    }
    return best;
}

static inline int PianoKey_isReachable(const PianoKey* key) {
    uint8_t i;
    for (i = 0; i < MAX_FINGER_OPTIONS; i++)
        if (FingerOption_isReachable(&key->options[i])) return 1;
    return 0;
}


/* --------------------------------------------------------------------------
 * Position helpers (used to build KEY_MAP below)
 *
 * White key index (wki): 0-based count of white keys from C2.
 *
 *   wki:  0  1  2  3  4  5  6 |  7  8  9 10 11 12 13 | 14 ...
 *   note: C2 D2 E2 F2 G2 A2 B2|  C3 D3 E3 F3 G3 A3 B3|  C4 ...
 *
 * White finger positions (motor mm to place finger over white key wki):
 *   W1 → ORIGIN + wki       * WHITE_KEY_WIDTH_MM
 *   W2 → ORIGIN + (wki - 2) * WHITE_KEY_WIDTH_MM
 *   W3 → ORIGIN + (wki - 4) * WHITE_KEY_WIDTH_MM
 *
 * Black finger positions (motor mm to place finger over black key gap
 * that sits between white keys leftWki and leftWki+1):
 *   B1 → ORIGIN + leftWki       * WHITE_KEY_WIDTH_MM
 *   B2 → ORIGIN + (leftWki - 2) * WHITE_KEY_WIDTH_MM
 *
 * Black key gaps per octave (leftWki offset within octave):
 *   0 → C#  ✓    1 → D#  ✓    2 → E-F  ✗ (no black key)
 *   3 → F#  ✓    4 → G#  ✓    5 → A#  ✓    6 → B-C  ✗
 * -------------------------------------------------------------------------- */

#define W1POS(wki)      (MOTOR_ORIGIN_MM + (wki)       * WHITE_KEY_WIDTH_MM)
#define W2POS(wki)      (MOTOR_ORIGIN_MM + ((wki) - 2) * WHITE_KEY_WIDTH_MM)
#define W3POS(wki)      (MOTOR_ORIGIN_MM + ((wki) - 4) * WHITE_KEY_WIDTH_MM)
#define B1POS(lwki)     (MOTOR_ORIGIN_MM + (lwki)       * WHITE_KEY_WIDTH_MM)
#define B2POS(lwki)     (MOTOR_ORIGIN_MM + ((lwki) - 2) * WHITE_KEY_WIDTH_MM)

/* Convenience macros for building the table */
#define OPT(f, mm)   { (f), (mm), 1 }
#define NO_OPT       { FINGER_NONE, -1.0f, 0 }


/* --------------------------------------------------------------------------
 * Key map — 49 keys, C2 (midi 36) to C6 (midi 84)
 * -------------------------------------------------------------------------- */

static const PianoKey KEY_MAP[KEY_MAP_SIZE] = {

    /* ── Octave 2  (wki 0–6) ──────────────────────────────────────────────
     * W2 needs wki >= 2, W3 needs wki >= 4.  B2 needs lwki >= 2.
     * Low keys only have 1–2 options because W2/W3/B2 would go negative.
     *
     * FIX: Each macro arg is now the KEY's own wki/lwki so each finger
     *      option produces a DIFFERENT motor position that correctly
     *      places that finger over the target key.                         */

    /* midi  name    type       option[0]                  option[1]                  option[2]   */
    { 36, "C2",  KEY_WHITE, { OPT(FINGER_W1,W1POS(0)),  NO_OPT,                    NO_OPT                    }},
    { 37, "C#2", KEY_BLACK, { OPT(FINGER_B1,B1POS(0)),  NO_OPT,                    NO_OPT                    }},
    { 38, "D2",  KEY_WHITE, { OPT(FINGER_W1,W1POS(1)),  NO_OPT,                    NO_OPT                    }},
    { 39, "D#2", KEY_BLACK, { OPT(FINGER_B1,B1POS(1)),  NO_OPT,                    NO_OPT                    }},
    { 40, "E2",  KEY_WHITE, { OPT(FINGER_W1,W1POS(2)),  OPT(FINGER_W2,W2POS(2)),  NO_OPT                    }},
    { 41, "F2",  KEY_WHITE, { OPT(FINGER_W1,W1POS(3)),  OPT(FINGER_W2,W2POS(3)),  NO_OPT                    }},
    { 42, "F#2", KEY_BLACK, { OPT(FINGER_B1,B1POS(3)),  OPT(FINGER_B2,B2POS(3)),  NO_OPT                    }},
    { 43, "G2",  KEY_WHITE, { OPT(FINGER_W1,W1POS(4)),  OPT(FINGER_W2,W2POS(4)),  OPT(FINGER_W3,W3POS(4))  }},
    { 44, "G#2", KEY_BLACK, { OPT(FINGER_B1,B1POS(4)),  OPT(FINGER_B2,B2POS(4)),  NO_OPT                    }},
    { 45, "A2",  KEY_WHITE, { OPT(FINGER_W1,W1POS(5)),  OPT(FINGER_W2,W2POS(5)),  OPT(FINGER_W3,W3POS(5))  }},
    { 46, "A#2", KEY_BLACK, { OPT(FINGER_B1,B1POS(5)),  OPT(FINGER_B2,B2POS(5)),  NO_OPT                    }},
    { 47, "B2",  KEY_WHITE, { OPT(FINGER_W1,W1POS(6)),  OPT(FINGER_W2,W2POS(6)),  OPT(FINGER_W3,W3POS(6))  }},

    /* ── Octave 3  (wki 7–13) ─────────────────────────────────────────────── */
    { 48, "C3",  KEY_WHITE, { OPT(FINGER_W1,W1POS(7)),  OPT(FINGER_W2,W2POS(7)),  OPT(FINGER_W3,W3POS(7))  }},
    { 49, "C#3", KEY_BLACK, { OPT(FINGER_B1,B1POS(7)),  OPT(FINGER_B2,B2POS(7)),  NO_OPT                    }},
    { 50, "D3",  KEY_WHITE, { OPT(FINGER_W1,W1POS(8)),  OPT(FINGER_W2,W2POS(8)),  OPT(FINGER_W3,W3POS(8))  }},
    { 51, "D#3", KEY_BLACK, { OPT(FINGER_B1,B1POS(8)),  OPT(FINGER_B2,B2POS(8)),  NO_OPT                    }},
    { 52, "E3",  KEY_WHITE, { OPT(FINGER_W1,W1POS(9)),  OPT(FINGER_W2,W2POS(9)),  OPT(FINGER_W3,W3POS(9))  }},
    { 53, "F3",  KEY_WHITE, { OPT(FINGER_W1,W1POS(10)), OPT(FINGER_W2,W2POS(10)), OPT(FINGER_W3,W3POS(10)) }},
    { 54, "F#3", KEY_BLACK, { OPT(FINGER_B1,B1POS(10)), OPT(FINGER_B2,B2POS(10)), NO_OPT                    }},
    { 55, "G3",  KEY_WHITE, { OPT(FINGER_W1,W1POS(11)), OPT(FINGER_W2,W2POS(11)), OPT(FINGER_W3,W3POS(11)) }},
    { 56, "G#3", KEY_BLACK, { OPT(FINGER_B1,B1POS(11)), OPT(FINGER_B2,B2POS(11)), NO_OPT                    }},
    { 57, "A3",  KEY_WHITE, { OPT(FINGER_W1,W1POS(12)), OPT(FINGER_W2,W2POS(12)), OPT(FINGER_W3,W3POS(12)) }},
    { 58, "A#3", KEY_BLACK, { OPT(FINGER_B1,B1POS(12)), OPT(FINGER_B2,B2POS(12)), NO_OPT                    }},
    { 59, "B3",  KEY_WHITE, { OPT(FINGER_W1,W1POS(13)), OPT(FINGER_W2,W2POS(13)), OPT(FINGER_W3,W3POS(13)) }},

    /* ── Octave 4  (wki 14–20) — middle octave ────────────────────────────── */
    { 60, "C4",  KEY_WHITE, { OPT(FINGER_W1,W1POS(14)), OPT(FINGER_W2,W2POS(14)), OPT(FINGER_W3,W3POS(14)) }},
    { 61, "C#4", KEY_BLACK, { OPT(FINGER_B1,B1POS(14)), OPT(FINGER_B2,B2POS(14)), NO_OPT                    }},
    { 62, "D4",  KEY_WHITE, { OPT(FINGER_W1,W1POS(15)), OPT(FINGER_W2,W2POS(15)), OPT(FINGER_W3,W3POS(15)) }},
    { 63, "D#4", KEY_BLACK, { OPT(FINGER_B1,B1POS(15)), OPT(FINGER_B2,B2POS(15)), NO_OPT                    }},
    { 64, "E4",  KEY_WHITE, { OPT(FINGER_W1,W1POS(16)), OPT(FINGER_W2,W2POS(16)), OPT(FINGER_W3,W3POS(16)) }},
    { 65, "F4",  KEY_WHITE, { OPT(FINGER_W1,W1POS(17)), OPT(FINGER_W2,W2POS(17)), OPT(FINGER_W3,W3POS(17)) }},
    { 66, "F#4", KEY_BLACK, { OPT(FINGER_B1,B1POS(17)), OPT(FINGER_B2,B2POS(17)), NO_OPT                    }},
    { 67, "G4",  KEY_WHITE, { OPT(FINGER_W1,W1POS(18)), OPT(FINGER_W2,W2POS(18)), OPT(FINGER_W3,W3POS(18)) }},
    { 68, "G#4", KEY_BLACK, { OPT(FINGER_B1,B1POS(18)), OPT(FINGER_B2,B2POS(18)), NO_OPT                    }},
    { 69, "A4",  KEY_WHITE, { OPT(FINGER_W1,W1POS(19)), OPT(FINGER_W2,W2POS(19)), OPT(FINGER_W3,W3POS(19)) }},
    { 70, "A#4", KEY_BLACK, { OPT(FINGER_B1,B1POS(19)), OPT(FINGER_B2,B2POS(19)), NO_OPT                    }},
    { 71, "B4",  KEY_WHITE, { OPT(FINGER_W1,W1POS(20)), OPT(FINGER_W2,W2POS(20)), OPT(FINGER_W3,W3POS(20)) }},

    /* ── Top C ──────────────────────────────────────────────────────────────── */
    { 72, "C5",  KEY_WHITE, { OPT(FINGER_W1,W1POS(21)), OPT(FINGER_W2,W2POS(21)), OPT(FINGER_W3,W3POS(21)) }},
};


/* --------------------------------------------------------------------------
 * Lookup helpers
 * -------------------------------------------------------------------------- */

/* Find key by MIDI note number. Returns NULL if not in range. */
static inline const PianoKey* keyByMidi(uint8_t midiNote) {
    uint8_t i;
    for (i = 0; i < KEY_MAP_SIZE; i++)
        if (KEY_MAP[i].midiNote == midiNote) return &KEY_MAP[i];
    return NULL;
}

/* Find key by name e.g. "C#4". Returns NULL if not found. */
static inline const PianoKey* keyByName(const char* name) {
    uint8_t i;
    uint8_t j;
    for (i = 0; i < KEY_MAP_SIZE; i++) {
        /* Simple string compare without pulling in string.h */
        const char* a = KEY_MAP[i].name;
        const char* b = name;
        for (j = 0; a[j] == b[j]; j++)
            if (a[j] == '\0') return &KEY_MAP[i];
    }
    return NULL;
}

/* Best option for a MIDI note given current motor position.
 * Returns an invalid option (valid=0) if note not found. */
static inline FingerOption bestOptionForNote(uint8_t midiNote,
                                             float currentMotorMM) {
    const PianoKey* key = keyByMidi(midiNote);
    FingerOption invalid = { FINGER_NONE, -1.0f, 0 };
    if (!key) return invalid;
    return PianoKey_bestOption(key, currentMotorMM);
}

/* Returns 1 if the note is in range and at least one option is reachable. */
static inline int noteIsReachable(uint8_t midiNote) {
    const PianoKey* key = keyByMidi(midiNote);
    return key && PianoKey_isReachable(key);
}


#ifdef __cplusplus
}
#endif

#endif /* PIANO_KEYMAP_H */
