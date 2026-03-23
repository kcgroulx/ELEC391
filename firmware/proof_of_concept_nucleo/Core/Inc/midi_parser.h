#ifndef MIDI_PARSER_H
#define MIDI_PARSER_H

/* midi_parser.h
 * ==========================================================================
 * Parses a Standard MIDI File (SMF) byte buffer into a flat NoteEvent array
 * that NotePlayer_playSequence() can consume directly.
 *
 * SUPPORTED:
 *   - MIDI format 0 (single track) and format 1 (multi-track, track 1 only)
 *   - Note On / Note Off events
 *   - Variable-length delta times
 *   - Tempo changes (SET_TEMPO meta event)
 *   - Running status
 *
 * NOT SUPPORTED (silently skipped):
 *   - Format 2 (rare, pattern-based)
 *   - SysEx events
 *   - Notes outside MIDI_NOTE_MIN–MIDI_NOTE_MAX (robot can't reach them)
 *   - Polyphony (simultaneous notes — Step 5 will handle this)
 *
 * USAGE — hardcoded array:
 *   MidiParseResult result;
 *   NoteEvent events[MIDI_MAX_NOTES];
 *   Midi_parseBuffer(MY_SONG_BYTES, sizeof(MY_SONG_BYTES), events,
 *                    MIDI_MAX_NOTES, &result);
 *   if (result.status == MIDI_OK)
 *       NotePlayer_playSequence(events, result.noteCount);
 *
 * USAGE — received over UART:
 *   uint8_t uartBuf[MIDI_UART_BUF_SIZE];
 *   uint32_t len = Midi_receiveUART(uartBuf, sizeof(uartBuf), 5000);
 *   Midi_parseBuffer(uartBuf, len, events, MIDI_MAX_NOTES, &result);
 * ========================================================================== */

#include <stdint.h>
#include "note_player.h"   /* NoteEvent */

#ifdef __cplusplus
extern "C" {
#endif


/* --------------------------------------------------------------------------
 * Configuration
 * -------------------------------------------------------------------------- */

/* Maximum number of NoteEvents the parser will output.
 * Each NoteEvent is 9 bytes — 256 notes = 2304 bytes of RAM. */
#define MIDI_MAX_NOTES          256U

/* UART receive buffer size — must be large enough for your biggest .mid file.
 * Most simple melody MIDIs are under 4 KB. */
#define MIDI_UART_BUF_SIZE      8192U

/* UART timeout for receiving a full MIDI file (ms). */
#define MIDI_UART_TIMEOUT_MS    5000U

/* Which track to parse (0-based).
 * 0 = first track (tempo map in format 1, or only track in format 0).
 * 1 = second track (melody in most format 1 files).
 * Start with 1 for format 1 files, 0 for format 0. */
#define MIDI_TRACK_INDEX        1U


/* --------------------------------------------------------------------------
 * Result codes
 * -------------------------------------------------------------------------- */

typedef enum {
    MIDI_OK              = 0,  /* parse succeeded                            */
    MIDI_ERR_NOT_MIDI    = 1,  /* buffer doesn't start with MThd             */
    MIDI_ERR_FORMAT      = 2,  /* format 2 not supported                     */
    MIDI_ERR_NO_TRACK    = 3,  /* requested track index doesn't exist        */
    MIDI_ERR_CORRUPT     = 4,  /* unexpected end of buffer or bad data       */
    MIDI_ERR_BUF_FULL    = 5,  /* output buffer too small (notes truncated)  */
    MIDI_ERR_UART        = 6,  /* UART receive failed or timed out           */
} MidiStatus;

typedef struct {
    MidiStatus status;
    uint16_t   noteCount;       /* number of NoteEvents written              */
    uint16_t   notesSkipped;    /* notes outside robot range, skipped        */
    uint32_t   durationMs;      /* total song duration in ms                 */
} MidiParseResult;


/* --------------------------------------------------------------------------
 * Core parser — works on any byte buffer
 * --------------------------------------------------------------------------
 * buf        pointer to raw MIDI file bytes
 * len        number of bytes in buf
 * outEvents  caller-provided array to fill with NoteEvents
 * maxEvents  size of outEvents[]
 * result     filled with status and stats on return
 * -------------------------------------------------------------------------- */
void Midi_parseBuffer(const uint8_t*   buf,
                      uint32_t         len,
                      NoteEvent*       outEvents,
                      uint16_t         maxEvents,
                      MidiParseResult* result);


/* --------------------------------------------------------------------------
 * UART transport — receive a MIDI file from PC over UART
 * --------------------------------------------------------------------------
 * Blocks until either len bytes are received or timeoutMs elapses.
 * The PC-side Python script must send a 4-byte little-endian length header
 * followed by the raw MIDI bytes so the STM32 knows when to stop receiving.
 *
 * Protocol:
 *   PC sends:  [len_lo] [len_hi2] [len_hi1] [len_hi0] [midi_bytes...]
 *              (4-byte big-endian uint32 length, then raw bytes)
 *
 * Returns the number of bytes received (0 on timeout/error).
 * -------------------------------------------------------------------------- */
uint32_t Midi_receiveUART(uint8_t* buf, uint32_t bufSize, uint32_t timeoutMs);


/* --------------------------------------------------------------------------
 * Convenience: receive over UART then parse in one call
 * -------------------------------------------------------------------------- */
void Midi_receiveAndParse(uint8_t*         uartBuf,
                          uint32_t         uartBufSize,
                          NoteEvent*       outEvents,
                          uint16_t         maxEvents,
                          MidiParseResult* result);


/* --------------------------------------------------------------------------
 * Debug helper — print parse result over UART
 * Remove or stub out if you don't have UART logging set up.
 * -------------------------------------------------------------------------- */
void Midi_printResult(const MidiParseResult* result);


#ifdef __cplusplus
}
#endif

#endif /* MIDI_PARSER_H */
