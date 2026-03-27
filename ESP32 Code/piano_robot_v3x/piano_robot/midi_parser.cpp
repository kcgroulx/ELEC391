/* midi_parser.c
 * ==========================================================================
 * Parses a Standard MIDI File (SMF) byte buffer into a NoteEvent array.
 *
 * MIDI FILE STRUCTURE (quick reference):
 *
 *   MThd chunk (14 bytes):
 *     4D 54 68 64        "MThd" magic
 *     00 00 00 06        chunk length (always 6)
 *     00 00              format (0=single, 1=multi, 2=pattern)
 *     00 NN              number of tracks
 *     00 TT              ticks per quarter note (PPQ)
 *
 *   MTrk chunk (per track):
 *     4D 54 72 6B        "MTrk" magic
 *     LL LL LL LL        chunk length in bytes
 *     [events...]
 *
 *   Each event:
 *     <delta_time>       variable-length encoded ticks since last event
 *     <status_byte>      0x8n=NoteOff 0x9n=NoteOn 0xAn-0xEn=other 0xFF=meta
 *     <data bytes...>
 *
 *   Meta event 0xFF 0x51 0x03 tt tt tt = tempo (microseconds per beat)
 *
 * CONVERSION: ticks → milliseconds
 *   ms = (ticks * tempo_us_per_beat) / (ppq * 1000)
 * ========================================================================== */

#include "midi_parser.h"
#include "piano_keymap.h"
#include "hal_interface.h"
#include "Arduino.h"        /* Serial, millis()                           */
#include <string.h>
#include <stdio.h>

/* --------------------------------------------------------------------------
 * Internal reader — tracks position in the buffer safely
 * -------------------------------------------------------------------------- */

typedef struct {
    const uint8_t* data;
    uint32_t       len;
    uint32_t       pos;
    uint8_t        error;   /* set to 1 on overread */
} Reader;

static void     Reader_init(Reader* r, const uint8_t* data, uint32_t len) {
    r->data  = data;
    r->len   = len;
    r->pos   = 0;
    r->error = 0;
}

static uint8_t  Reader_u8(Reader* r) {
    if (r->pos >= r->len) { r->error = 1; return 0; }
    return r->data[r->pos++];
}

static uint16_t Reader_u16be(Reader* r) {
    uint16_t hi = Reader_u8(r);
    uint16_t lo = Reader_u8(r);
    return (uint16_t)((hi << 8) | lo);
}

static uint32_t Reader_u32be(Reader* r) {
    uint32_t a = Reader_u8(r);
    uint32_t b = Reader_u8(r);
    uint32_t c = Reader_u8(r);
    uint32_t d = Reader_u8(r);
    return (a << 24) | (b << 16) | (c << 8) | d;
}

/* Read a MIDI variable-length quantity (1–4 bytes). */
static uint32_t Reader_varlen(Reader* r) {
    uint32_t val = 0;
    uint8_t  b;
    uint8_t  i;
    for (i = 0; i < 4; i++) {
        b    = Reader_u8(r);
        val  = (val << 7) | (b & 0x7F);
        if (!(b & 0x80)) break;
    }
    return val;
}

/* Skip n bytes. */
static void Reader_skip(Reader* r, uint32_t n) {
    if (r->pos + n > r->len) { r->error = 1; r->pos = r->len; return; }
    r->pos += n;
}

/* Save / restore position (used to rewind to track start). */
static uint32_t Reader_tell(const Reader* r)        { return r->pos; }
static void     Reader_seek(Reader* r, uint32_t pos) { r->pos = pos;  }


/* --------------------------------------------------------------------------
 * Tick-to-ms conversion state
 * -------------------------------------------------------------------------- */

typedef struct {
    uint32_t ppq;              /* ticks per quarter note from MThd          */
    uint32_t tempo_us;         /* current tempo in microseconds per beat    */
} TempoState;

static void TempoState_init(TempoState* t, uint32_t ppq) {
    t->ppq      = ppq;
    t->tempo_us = 500000U;   /* default: 120 BPM = 500000 us/beat          */
}

/* Convert ticks to milliseconds using current tempo. */
static uint32_t ticks_to_ms(uint32_t ticks, const TempoState* t) {
    /* ms = ticks * tempo_us / (ppq * 1000)
     * Use 64-bit intermediate to avoid overflow on long notes. */
    uint64_t num = (uint64_t)ticks * (uint64_t)t->tempo_us;
    uint64_t den = (uint64_t)t->ppq * 1000ULL;
    return (uint32_t)(num / den);
}


/* --------------------------------------------------------------------------
 * Note-on tracking (needed to compute durations from note-off events)
 *
 * When we see a Note On, we store its tick time.
 * When we see the matching Note Off, we compute duration = off_tick - on_tick.
 * We track up to 16 simultaneous notes (more than enough for melody).
 * -------------------------------------------------------------------------- */

#define MAX_ACTIVE_NOTES 16U

typedef struct {
    uint8_t  note;
    uint32_t onTick;
    uint8_t  active;
} ActiveNote;

static ActiveNote s_active[MAX_ACTIVE_NOTES];

static void activeNotes_clear(void) {
    uint8_t i;
    for (i = 0; i < MAX_ACTIVE_NOTES; i++) s_active[i].active = 0;
}

static void activeNote_on(uint8_t note, uint32_t tick) {
    uint8_t i;
    for (i = 0; i < MAX_ACTIVE_NOTES; i++) {
        if (!s_active[i].active) {
            s_active[i].note   = note;
            s_active[i].onTick = tick;
            s_active[i].active = 1;
            return;
        }
    }
    /* Table full — overwrite oldest (slot 0) as fallback */
    s_active[0].note   = note;
    s_active[0].onTick = tick;
    s_active[0].active = 1;
}

/* Returns on-tick of the matching note, sets active=0. Returns 0 if not found. */
static uint32_t activeNote_off(uint8_t note) {
    uint8_t i;
    for (i = 0; i < MAX_ACTIVE_NOTES; i++) {
        if (s_active[i].active && s_active[i].note == note) {
            s_active[i].active = 0;
            return s_active[i].onTick;
        }
    }
    return 0;
}


/* --------------------------------------------------------------------------
 * Track parser
 * Reads one MTrk chunk from position r->pos and appends NoteEvents.
 * -------------------------------------------------------------------------- */

static void parseTrack(Reader*          r,
                       uint32_t         trackLen,
                       TempoState*      tempo,
                       NoteEvent*       outEvents,
                       uint16_t         maxEvents,
                       MidiParseResult* result)
{
    uint32_t trackEnd   = Reader_tell(r) + trackLen;
    uint32_t currentTick = 0;
    uint8_t  runningStatus = 0;

    activeNotes_clear();

    while (Reader_tell(r) < trackEnd && !r->error) {

        /* ── Delta time ──────────────────────────────────────────────────── */
        uint32_t delta = Reader_varlen(r);
        currentTick += delta;

        /* ── Status byte ─────────────────────────────────────────────────── */
        uint8_t statusByte = r->data[r->pos];   /* peek */

        if (statusByte & 0x80) {
            runningStatus = statusByte;
            r->pos++;                            /* consume it */
        }
        /* else: running status — reuse last status, don't advance */

        uint8_t type    = runningStatus & 0xF0;
        /* uint8_t channel = runningStatus & 0x0F; (unused for now) */

        /* ── Note Off (0x8n) ─────────────────────────────────────────────── */
        if (type == 0x80) {
            uint8_t note     = Reader_u8(r);
            uint8_t velocity = Reader_u8(r);
            (void)velocity;

            uint32_t onTick  = activeNote_off(note);
            if (onTick > 0 && result->noteCount < maxEvents) {
                uint32_t durMs = ticks_to_ms(currentTick - onTick, tempo);
                if (durMs < 10U) durMs = 10U;   /* clamp very short notes    */

                /* delayAfter filled in post-process pass below */
                outEvents[result->noteCount].midiNote     = note;
                outEvents[result->noteCount].durationMs   = durMs;
                outEvents[result->noteCount].delayAfterMs = 0;
                result->noteCount++;
            }
        }

        /* ── Note On (0x9n) ──────────────────────────────────────────────── */
        else if (type == 0x90) {
            uint8_t note     = Reader_u8(r);
            uint8_t velocity = Reader_u8(r);

            if (velocity == 0) {
                /* Note On with velocity 0 is equivalent to Note Off */
                uint32_t onTick = activeNote_off(note);
                if (onTick > 0 && result->noteCount < maxEvents) {
                    uint32_t durMs = ticks_to_ms(currentTick - onTick, tempo);
                    if (durMs < 10U) durMs = 10U;

                    outEvents[result->noteCount].midiNote     = note;
                    outEvents[result->noteCount].durationMs   = durMs;
                    outEvents[result->noteCount].delayAfterMs = 0;
                    result->noteCount++;
                }
            } else {
                /* Check if note is within robot's range */
                if (note >= MIDI_NOTE_MIN && note <= MIDI_NOTE_MAX) {
                    activeNote_on(note, currentTick);
                } else {
                    result->notesSkipped++;
                }
            }
        }

        /* ── Polyphonic key pressure (0xAn) — 2 data bytes, skip ─────────── */
        else if (type == 0xA0) { Reader_skip(r, 2); }

        /* ── Control change (0xBn) — 2 data bytes, skip ─────────────────── */
        else if (type == 0xB0) { Reader_skip(r, 2); }

        /* ── Program change (0xCn) — 1 data byte, skip ──────────────────── */
        else if (type == 0xC0) { Reader_skip(r, 1); }

        /* ── Channel pressure (0xDn) — 1 data byte, skip ────────────────── */
        else if (type == 0xD0) { Reader_skip(r, 1); }

        /* ── Pitch bend (0xEn) — 2 data bytes, skip ─────────────────────── */
        else if (type == 0xE0) { Reader_skip(r, 2); }

        /* ── Meta event (0xFF) ───────────────────────────────────────────── */
        else if (runningStatus == 0xFF) {
            uint8_t  metaType = Reader_u8(r);
            uint32_t metaLen  = Reader_varlen(r);

            /* Tempo change: 0xFF 0x51 0x03 tt tt tt */
            if (metaType == 0x51 && metaLen == 3) {
                uint32_t t0 = Reader_u8(r);
                uint32_t t1 = Reader_u8(r);
                uint32_t t2 = Reader_u8(r);
                tempo->tempo_us = (t0 << 16) | (t1 << 8) | t2;
            } else {
                Reader_skip(r, metaLen);
            }

            /* End of track meta event */
            if (metaType == 0x2F) break;
        }

        /* ── SysEx (0xF0 / 0xF7) — skip ─────────────────────────────────── */
        else if (runningStatus == 0xF0 || runningStatus == 0xF7) {
            uint32_t sysexLen = Reader_varlen(r);
            Reader_skip(r, sysexLen);
        }

        if (r->error) {
            result->status = MIDI_ERR_CORRUPT;
            return;
        }
    }

    /* Seek to exact track end in case we stopped early (end-of-track meta) */
    Reader_seek(r, trackEnd);

    /* ── Post-process: fill in delayAfterMs ──────────────────────────────────
     * delayAfterMs = gap between end of this note and start of next note.
     * We stored events in order of Note Off, so we need the Note On times.
     * Simple approach: delayAfter[i] = max(0, onTime[i+1] - offTime[i])
     * Since we don't store onTick in NoteEvent we use 0 for now — the
     * position planner in Step 4 will handle inter-note gaps properly.
     * For Step 2/3 testing, leaving delayAfterMs = 0 is fine. */
}


/* --------------------------------------------------------------------------
 * Midi_parseBuffer — main entry point
 * -------------------------------------------------------------------------- */

void Midi_parseBuffer(const uint8_t*   buf,
                      uint32_t         len,
                      NoteEvent*       outEvents,
                      uint16_t         maxEvents,
                      MidiParseResult* result)
{
    Reader      r;
    TempoState  tempo;
    uint16_t    numTracks;
    uint16_t    ppq;
    uint16_t    format;
    uint16_t    trackIndex;
    uint32_t    chunkLen;
    uint8_t     magic[4];
    uint8_t     i;

    /* Initialise result */
    result->status       = MIDI_OK;
    result->noteCount    = 0;
    result->notesSkipped = 0;
    result->durationMs   = 0;

    Reader_init(&r, buf, len);

    /* ── Read MThd header ─────────────────────────────────────────────────── */
    magic[0] = Reader_u8(&r); magic[1] = Reader_u8(&r);
    magic[2] = Reader_u8(&r); magic[3] = Reader_u8(&r);

    if (magic[0] != 'M' || magic[1] != 'T' ||
        magic[2] != 'h' || magic[3] != 'd') {
        result->status = MIDI_ERR_NOT_MIDI;
        return;
    }

    Reader_u32be(&r);                   /* header length — always 6, skip   */
    format    = Reader_u16be(&r);
    numTracks = Reader_u16be(&r);
    ppq       = Reader_u16be(&r);

    if (format == 2) {
        result->status = MIDI_ERR_FORMAT;
        return;
    }

    TempoState_init(&tempo, (uint32_t)ppq);

    /* ── Determine which track to parse ──────────────────────────────────── */
    /* Format 0: single track, always index 0.
     * Format 1: track 0 is tempo map, track 1 is first instrument (melody).
     * MIDI_TRACK_INDEX is set in the header — default 1 for format 1. */
    trackIndex = (format == 0) ? 0 : MIDI_TRACK_INDEX;

    if (trackIndex >= numTracks) {
        /* Requested track doesn't exist — fall back to track 0 */
        trackIndex = 0;
    }

    /* ── For format 1: parse track 0 first to pick up tempo changes ──────── */
    if (format == 1 && trackIndex > 0) {
        /* Find and parse track 0 (tempo map only, discard notes) */
        for (i = 0; i < numTracks; i++) {
            magic[0] = Reader_u8(&r); magic[1] = Reader_u8(&r);
            magic[2] = Reader_u8(&r); magic[3] = Reader_u8(&r);
            chunkLen  = Reader_u32be(&r);

            if (magic[0] == 'M' && magic[1] == 'T' &&
                magic[2] == 'r' && magic[3] == 'k') {
                if (i == 0) {
                    /* Parse tempo track but throw away note output */
                    NoteEvent  tempEvents[8];
                    MidiParseResult tempResult;
                    tempResult.status       = MIDI_OK;
                    tempResult.noteCount    = 0;
                    tempResult.notesSkipped = 0;
                    tempResult.durationMs   = 0;
                    parseTrack(&r, chunkLen, &tempo,
                               tempEvents, 8, &tempResult);
                } else {
                    Reader_skip(&r, chunkLen);
                }
                if (i + 1 == trackIndex) break;
            } else {
                /* Unknown chunk — skip it */
                Reader_skip(&r, chunkLen);
            }
            if (r.error) { result->status = MIDI_ERR_CORRUPT; return; }
        }
    }

    /* ── Seek to target track ─────────────────────────────────────────────── */
    /* Re-scan from after MThd to find the Nth MTrk chunk. */
    r.pos  = 14U;   /* skip MThd (4 magic + 4 len + 6 data) */
    r.error = 0;

    for (i = 0; i <= trackIndex; i++) {
        magic[0] = Reader_u8(&r); magic[1] = Reader_u8(&r);
        magic[2] = Reader_u8(&r); magic[3] = Reader_u8(&r);
        chunkLen  = Reader_u32be(&r);

        if (r.error) { result->status = MIDI_ERR_CORRUPT; return; }

        if (magic[0] != 'M' || magic[1] != 'T' ||
            magic[2] != 'r' || magic[3] != 'k') {
            /* Unknown chunk — skip and retry */
            Reader_skip(&r, chunkLen);
            i--;   /* don't count this as a track */
            continue;
        }

        if (i < trackIndex) {
            Reader_skip(&r, chunkLen);   /* skip unwanted tracks */
        }
    }

    if (r.error) { result->status = MIDI_ERR_CORRUPT; return; }

    /* ── Parse the target track ───────────────────────────────────────────── */
    parseTrack(&r, chunkLen, &tempo, outEvents, maxEvents, result);

    if (result->noteCount >= maxEvents) {
        result->status = MIDI_ERR_BUF_FULL;
    }

    /* Estimate total duration from last note */
    if (result->noteCount > 0) {
        uint32_t total = 0;
        uint16_t n;
        for (n = 0; n < result->noteCount; n++)
            total += outEvents[n].durationMs + outEvents[n].delayAfterMs;
        result->durationMs = total;
    }
}


/* --------------------------------------------------------------------------
 * Midi_receiveUART
 * --------------------------------------------------------------------------
 * Waits for a 4-byte big-endian length header then receives that many bytes.
 * Times out after timeoutMs ms.
 * -------------------------------------------------------------------------- */

uint32_t Midi_receiveUART(uint8_t* buf, uint32_t bufSize, uint32_t timeoutMs)
{
    uint8_t  lenBytes[4];
    uint32_t fileLen;
    uint32_t received;

    /* Wait for 4-byte length header */
    Serial.setTimeout((long)timeoutMs);
    if (Serial.readBytes((char*)lenBytes, 4) != 4) return 0;

    fileLen = ((uint32_t)lenBytes[0] << 24)
            | ((uint32_t)lenBytes[1] << 16)
            | ((uint32_t)lenBytes[2] <<  8)
            | ((uint32_t)lenBytes[3]);

    if (fileLen == 0 || fileLen > bufSize) return 0;

    /* Receive file bytes */
    received = (uint32_t)Serial.readBytes((char*)buf, fileLen);
    if (received != fileLen) return 0;

    return fileLen;
}


/* --------------------------------------------------------------------------
 * Midi_receiveAndParse
 * -------------------------------------------------------------------------- */

void Midi_receiveAndParse(uint8_t*         uartBuf,
                          uint32_t         uartBufSize,
                          NoteEvent*       outEvents,
                          uint16_t         maxEvents,
                          MidiParseResult* result)
{
    uint32_t len = Midi_receiveUART(uartBuf, uartBufSize, MIDI_UART_TIMEOUT_MS);
    if (len == 0) {
        result->status    = MIDI_ERR_UART;
        result->noteCount = 0;
        return;
    }
    Midi_parseBuffer(uartBuf, len, outEvents, maxEvents, result);
}


/* --------------------------------------------------------------------------
 * Midi_printResult — debug helper
 * -------------------------------------------------------------------------- */

void Midi_printResult(const MidiParseResult* result)
{
    char buf[80];
    const char* statusStr;

    switch (result->status) {
        case MIDI_OK:           statusStr = "OK";           break;
        case MIDI_ERR_NOT_MIDI: statusStr = "NOT_MIDI";     break;
        case MIDI_ERR_FORMAT:   statusStr = "BAD_FORMAT";   break;
        case MIDI_ERR_NO_TRACK: statusStr = "NO_TRACK";     break;
        case MIDI_ERR_CORRUPT:  statusStr = "CORRUPT";      break;
        case MIDI_ERR_BUF_FULL: statusStr = "BUF_FULL";     break;
        case MIDI_ERR_UART:     statusStr = "UART_FAIL";    break;
        default:                statusStr = "UNKNOWN";       break;
    }

    snprintf(buf, sizeof(buf),
        "MIDI: status=%s  notes=%u  skipped=%u  dur=%lums\r\n",
        statusStr,
        (unsigned)result->noteCount,
        (unsigned)result->notesSkipped,
        (unsigned long)result->durationMs);

    Serial.print(buf);
}
