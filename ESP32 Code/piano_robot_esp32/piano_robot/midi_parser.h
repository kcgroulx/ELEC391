#ifndef MIDI_PARSER_H
#define MIDI_PARSER_H

/*
 * midi_parser.h
 * ===========================================================================
 * Parses a Standard MIDI File byte buffer into a NoteEvent array.
 * Logic identical to STM32 version — only UART transport updated for Serial.
 * ===========================================================================
 */

#include <stdint.h>
#include "note_player.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Configuration
 * -------------------------------------------------------------------------- */
#define MIDI_MAX_NOTES       256U
#define MIDI_UART_BUF_SIZE   8192U
#define MIDI_UART_TIMEOUT_MS 5000U
#define MIDI_TRACK_INDEX     1U     /* 0 for format 0, 1 for format 1 files  */

/* --------------------------------------------------------------------------
 * Result
 * -------------------------------------------------------------------------- */
typedef enum {
    MIDI_OK              = 0,
    MIDI_ERR_NOT_MIDI    = 1,
    MIDI_ERR_FORMAT      = 2,
    MIDI_ERR_NO_TRACK    = 3,
    MIDI_ERR_CORRUPT     = 4,
    MIDI_ERR_BUF_FULL    = 5,
    MIDI_ERR_UART        = 6,
} MidiStatus;

typedef struct {
    MidiStatus status;
    uint16_t   noteCount;
    uint16_t   notesSkipped;
    uint32_t   durationMs;
} MidiParseResult;

/* --------------------------------------------------------------------------
 * API
 * -------------------------------------------------------------------------- */
void     Midi_parseBuffer(const uint8_t* buf, uint32_t len,
                          NoteEvent* outEvents, uint16_t maxEvents,
                          MidiParseResult* result);

uint32_t Midi_receiveUART(uint8_t* buf, uint32_t bufSize, uint32_t timeoutMs);

void     Midi_receiveAndParse(uint8_t* uartBuf, uint32_t uartBufSize,
                              NoteEvent* outEvents, uint16_t maxEvents,
                              MidiParseResult* result);

void     Midi_printResult(const MidiParseResult* result);

#ifdef __cplusplus
}
#endif

#endif /* MIDI_PARSER_H */
