#ifndef NOTE_PLAYER_H
#define NOTE_PLAYER_H

/* note_player.h
 * ==========================================================================
 * Single note and simple sequence playback.
 *
 * Step 2 — blocking playback. The position planner (Step 4) will replace
 * the greedy finger selection with a lookahead optimiser.
 *
 * USAGE:
 *   NotePlayer_playNote(60, 500);           // middle C for 500 ms
 *   NotePlayer_playNoteByName("C4", 500);   // same by name
 *
 *   NoteEvent seq[] = {
 *       { 60, 300, 50 },   // C4, 300 ms hold, 50 ms gap after
 *       { 62, 300, 50 },   // D4
 *       { 64, 500,  0 },   // E4
 *   };
 *   NotePlayer_playSequence(seq, 3);
 * ========================================================================== */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/* --------------------------------------------------------------------------
 * NoteEvent — one entry in a monophonic sequence
 * In Step 3 this will be filled by the MIDI parser.
 * -------------------------------------------------------------------------- */

typedef struct {
    uint8_t  midiNote;      /* 0–127, middle C = 60                          */
    uint32_t durationMs;    /* how long to hold the key down                 */
    uint32_t delayAfterMs;  /* silence between this note and the next        */
} NoteEvent;


/* --------------------------------------------------------------------------
 * Playback — all blocking
 * -------------------------------------------------------------------------- */

/* Play a single note by MIDI number.
 * Moves motor to best position, waits for arrival, presses finger,
 * holds for durationMs, then releases.
 * Returns 1 on success, 0 if note is out of range or unreachable. */
int NotePlayer_playNote(uint8_t midiNote, uint32_t durationMs);

/* Play a single note by name e.g. "C4", "F#3".
 * Returns 1 on success, 0 if name not found. */
int NotePlayer_playNoteByName(const char* name, uint32_t durationMs);

/* Play an array of NoteEvents in order (monophonic).
 * count = number of elements in events[]. */
void NotePlayer_playSequence(const NoteEvent* events, uint16_t count);


/* --------------------------------------------------------------------------
 * State queries
 * -------------------------------------------------------------------------- */

/* Current motor position in mm. */
float NotePlayer_getCurrentPositionMM(void);

/* MIDI note the robot is currently positioned to play.
 * Returns 255 if not within tolerance of any mapped key. */
uint8_t NotePlayer_getCurrentNote(void);


#ifdef __cplusplus
}
#endif

#endif /* NOTE_PLAYER_H */
