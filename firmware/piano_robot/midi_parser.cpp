/* midi_parser.c
 * ==========================================================================
 * Parses a Standard MIDI File (SMF) byte buffer into a NoteEvent array.
 */

#include "midi_parser.h"
#include "piano_keymap.h"
#include "hal_interface.h"
#include "platform_io.h"
#include "Arduino.h"
#include <string.h>
#include <stdio.h>

typedef struct {
    const uint8_t* data;
    uint32_t       len;
    uint32_t       pos;
    uint8_t        error;
} Reader;

static void Reader_init(Reader* r, const uint8_t* data, uint32_t len) {
    r->data = data;
    r->len = len;
    r->pos = 0U;
    r->error = 0U;
}

static uint8_t Reader_u8(Reader* r) {
    if (r->pos >= r->len) {
        r->error = 1U;
        return 0U;
    }
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

static uint32_t Reader_varlen(Reader* r) {
    uint32_t val = 0U;
    uint8_t i;
    for (i = 0U; i < 4U; i++) {
        uint8_t b = Reader_u8(r);
        val = (val << 7) | (uint32_t)(b & 0x7FU);
        if (!(b & 0x80U)) {
            break;
        }
    }
    return val;
}

static void Reader_skip(Reader* r, uint32_t n) {
    if (r->pos + n > r->len) {
        r->error = 1U;
        r->pos = r->len;
        return;
    }
    r->pos += n;
}

static uint32_t Reader_tell(const Reader* r) { return r->pos; }
static void Reader_seek(Reader* r, uint32_t pos) { r->pos = pos; }

#define MAX_TEMPO_CHANGES 64U
#define MAX_ACTIVE_NOTES 16U

typedef struct {
    uint32_t tick;
    uint32_t tempo_us;
} TempoChange;

typedef struct {
    uint32_t    ppq;
    TempoChange changes[MAX_TEMPO_CHANGES];
    uint16_t    count;
    uint8_t     overflow;
} TempoMap;

typedef struct {
    uint8_t  note;
    uint32_t onTick;
    uint16_t outIndex;
    uint8_t  active;
} ActiveNote;

static ActiveNote s_active[MAX_ACTIVE_NOTES];

static void TempoMap_init(TempoMap* t, uint32_t ppq) {
    t->ppq = ppq;
    t->count = 1U;
    t->overflow = 0U;
    t->changes[0].tick = 0U;
    t->changes[0].tempo_us = 500000U;
}

static void TempoMap_add(TempoMap* t, uint32_t tick, uint32_t tempo_us) {
    if (t->count == 0U) {
        TempoMap_init(t, t->ppq);
    }

    if (t->changes[t->count - 1U].tick == tick) {
        t->changes[t->count - 1U].tempo_us = tempo_us;
        return;
    }

    if (t->changes[t->count - 1U].tempo_us == tempo_us) {
        return;
    }

    if (t->count >= MAX_TEMPO_CHANGES) {
        t->overflow = 1U;
        return;
    }

    t->changes[t->count].tick = tick;
    t->changes[t->count].tempo_us = tempo_us;
    t->count++;
}

static uint32_t TempoMap_tickToMs(const TempoMap* t, uint32_t tick) {
    uint64_t totalUs = 0ULL;
    uint32_t prevTick = 0U;
    uint32_t currentTempoUs = 500000U;
    uint16_t i;

    if (t->count > 0U) {
        currentTempoUs = t->changes[0].tempo_us;
    }

    for (i = 1U; i < t->count; i++) {
        const uint32_t changeTick = t->changes[i].tick;
        if (changeTick > tick) {
            break;
        }

        totalUs += (uint64_t)(changeTick - prevTick) * (uint64_t)currentTempoUs;
        prevTick = changeTick;
        currentTempoUs = t->changes[i].tempo_us;
    }

    totalUs += (uint64_t)(tick - prevTick) * (uint64_t)currentTempoUs;
    return (uint32_t)(totalUs / ((uint64_t)t->ppq * 1000ULL));
}

static uint32_t TempoMap_deltaToMs(const TempoMap* t, uint32_t startTick, uint32_t endTick) {
    if (endTick <= startTick) {
        return 0U;
    }
    return TempoMap_tickToMs(t, endTick) - TempoMap_tickToMs(t, startTick);
}

static void activeNotes_clear(void) {
    uint8_t i;
    for (i = 0U; i < MAX_ACTIVE_NOTES; i++) {
        s_active[i].active = 0U;
    }
}

static void activeNote_on(uint8_t note, uint32_t tick, uint16_t outIndex) {
    uint8_t i;
    for (i = 0U; i < MAX_ACTIVE_NOTES; i++) {
        if (!s_active[i].active) {
            s_active[i].note = note;
            s_active[i].onTick = tick;
            s_active[i].outIndex = outIndex;
            s_active[i].active = 1U;
            return;
        }
    }

    s_active[0].note = note;
    s_active[0].onTick = tick;
    s_active[0].outIndex = outIndex;
    s_active[0].active = 1U;
}

static uint8_t activeNote_off(uint8_t note, uint32_t* onTickOut, uint16_t* outIndexOut) {
    uint8_t i;
    for (i = 0U; i < MAX_ACTIVE_NOTES; i++) {
        if (s_active[i].active && s_active[i].note == note) {
            s_active[i].active = 0U;
            if (onTickOut) *onTickOut = s_active[i].onTick;
            if (outIndexOut) *outIndexOut = s_active[i].outIndex;
            return 1U;
        }
    }
    return 0U;
}

static uint8_t parseTrack(Reader*          r,
                          uint32_t         trackLen,
                          TempoMap*        tempoMap,
                          NoteEvent*       outEvents,
                          uint16_t         maxEvents,
                          MidiParseResult* result,
                          uint8_t          captureNotes,
                          uint8_t          captureTempo)
{
    uint32_t trackEnd = Reader_tell(r) + trackLen;
    uint32_t currentTick = 0U;
    uint8_t runningStatus = 0U;
    uint32_t noteOnTicks[MIDI_MAX_NOTES];
    uint32_t noteOffTicks[MIDI_MAX_NOTES];
    uint8_t noteComplete[MIDI_MAX_NOTES];
    uint16_t provisionalCount = 0U;
    uint16_t finalCount = 0U;
    uint16_t trackMaxEvents = maxEvents;
    uint8_t bufferFull = 0U;
    uint16_t i;

    if (trackEnd < Reader_tell(r) || trackEnd > r->len) {
        result->status = MIDI_ERR_CORRUPT;
        return 0U;
    }

    if (trackMaxEvents > MIDI_MAX_NOTES) {
        trackMaxEvents = MIDI_MAX_NOTES;
    }

    activeNotes_clear();
    memset(noteComplete, 0, sizeof(noteComplete));

    while (Reader_tell(r) < trackEnd && !r->error) {
        uint8_t type;

        currentTick += Reader_varlen(r);
        if (r->error) {
            result->status = MIDI_ERR_CORRUPT;
            return 0U;
        }

        if (Reader_tell(r) >= trackEnd || Reader_tell(r) >= r->len) {
            result->status = MIDI_ERR_CORRUPT;
            return 0U;
        }

        if (r->data[r->pos] & 0x80U) {
            runningStatus = r->data[r->pos];
            r->pos++;
        }

        if (runningStatus == 0U) {
            result->status = MIDI_ERR_CORRUPT;
            return 0U;
        }

        type = runningStatus & 0xF0U;

        if (type == 0x80U || type == 0x90U) {
            uint8_t note = Reader_u8(r);
            uint8_t velocity = Reader_u8(r);

            if (r->error) {
                result->status = MIDI_ERR_CORRUPT;
                return 0U;
            }

            if (type == 0x90U && velocity > 0U) {
                if (note >= MIDI_NOTE_MIN && note <= MIDI_NOTE_MAX) {
                    if (captureNotes) {
                        if (provisionalCount < trackMaxEvents) {
                            outEvents[provisionalCount].midiNote = note;
                            outEvents[provisionalCount].durationMs = 0U;
                            outEvents[provisionalCount].delayAfterMs = 0U;
                            noteOnTicks[provisionalCount] = currentTick;
                            noteOffTicks[provisionalCount] = currentTick;
                            noteComplete[provisionalCount] = 0U;
                            activeNote_on(note, currentTick, provisionalCount);
                            provisionalCount++;
                        } else {
                            bufferFull = 1U;
                            activeNote_on(note, currentTick, UINT16_MAX);
                        }
                    }
                } else {
                    result->notesSkipped++;
                }
            } else if (captureNotes) {
                uint32_t onTick;
                uint16_t outIndex;
                if (activeNote_off(note, &onTick, &outIndex) && outIndex < trackMaxEvents) {
                    noteOffTicks[outIndex] = currentTick;
                    noteComplete[outIndex] = 1U;
                }
            }
        }
        else if (type == 0xA0U || type == 0xB0U || type == 0xE0U) {
            Reader_skip(r, 2U);
        }
        else if (type == 0xC0U || type == 0xD0U) {
            Reader_skip(r, 1U);
        }
        else if (runningStatus == 0xFFU) {
            uint8_t metaType = Reader_u8(r);
            uint32_t metaLen = Reader_varlen(r);

            if (metaType == 0x51U && metaLen == 3U) {
                uint32_t t0 = Reader_u8(r);
                uint32_t t1 = Reader_u8(r);
                uint32_t t2 = Reader_u8(r);
                if (captureTempo) {
                    TempoMap_add(tempoMap, currentTick, (t0 << 16) | (t1 << 8) | t2);
                }
            } else {
                Reader_skip(r, metaLen);
            }

            if (metaType == 0x2FU) {
                break;
            }
        }
        else if (runningStatus == 0xF0U || runningStatus == 0xF7U) {
            uint32_t sysexLen = Reader_varlen(r);
            Reader_skip(r, sysexLen);
        }

        if (r->error) {
            result->status = MIDI_ERR_CORRUPT;
            return 0U;
        }
    }

    Reader_seek(r, trackEnd);

    if (!captureNotes) {
        if (tempoMap->overflow && result->status == MIDI_OK) {
            result->status = MIDI_ERR_FORMAT;
        }
        return 1U;
    }

    for (i = 0U; i < provisionalCount; i++) {
        if (!noteComplete[i]) {
            continue;
        }

        if (finalCount != i) {
            outEvents[finalCount] = outEvents[i];
            noteOnTicks[finalCount] = noteOnTicks[i];
            noteOffTicks[finalCount] = noteOffTicks[i];
        }
        finalCount++;
    }

    result->noteCount = finalCount;
    for (i = 0U; i < finalCount; i++) {
        uint32_t durationMs = TempoMap_deltaToMs(tempoMap, noteOnTicks[i], noteOffTicks[i]);
        if (durationMs < 10U) {
            durationMs = 10U;
        }
        outEvents[i].durationMs = durationMs;
        outEvents[i].delayAfterMs = 0U;
        if ((i + 1U) < finalCount) {
            outEvents[i].delayAfterMs = TempoMap_deltaToMs(
                tempoMap, noteOffTicks[i], noteOnTicks[i + 1U]);
        }
    }

    if (tempoMap->overflow && result->status == MIDI_OK) {
        result->status = MIDI_ERR_FORMAT;
    }

    if (bufferFull && result->status == MIDI_OK) {
        result->status = MIDI_ERR_BUF_FULL;
    }

    return 1U;
}

void Midi_parseBuffer(const uint8_t*   buf,
                      uint32_t         len,
                      NoteEvent*       outEvents,
                      uint16_t         maxEvents,
                      MidiParseResult* result)
{
    Reader     r;
    TempoMap   tempoMap;
    uint16_t   numTracks;
    uint16_t   ppq;
    uint16_t   format;
    uint16_t   trackIndex;
    uint8_t    magic[4];
    uint16_t   seenTracks;
    uint8_t    foundTarget = 0U;

    result->status = MIDI_OK;
    result->noteCount = 0U;
    result->notesSkipped = 0U;
    result->durationMs = 0U;

    Reader_init(&r, buf, len);

    magic[0] = Reader_u8(&r);
    magic[1] = Reader_u8(&r);
    magic[2] = Reader_u8(&r);
    magic[3] = Reader_u8(&r);
    if (magic[0] != 'M' || magic[1] != 'T' || magic[2] != 'h' || magic[3] != 'd') {
        result->status = MIDI_ERR_NOT_MIDI;
        return;
    }

    (void)Reader_u32be(&r);
    format = Reader_u16be(&r);
    numTracks = Reader_u16be(&r);
    ppq = Reader_u16be(&r);

    if (r.error || ppq == 0U) {
        result->status = MIDI_ERR_CORRUPT;
        return;
    }

    if (format == 2U) {
        result->status = MIDI_ERR_FORMAT;
        return;
    }

    TempoMap_init(&tempoMap, (uint32_t)ppq);

    trackIndex = (format == 0U) ? 0U : MIDI_TRACK_INDEX;
    if (trackIndex >= numTracks) {
        trackIndex = 0U;
    }

    if (format == 1U && trackIndex > 0U) {
        Reader_seek(&r, 14U);
        r.error = 0U;
        seenTracks = 0U;
        while (!r.error && Reader_tell(&r) < len && seenTracks < numTracks) {
            uint32_t chunkLen;
            uint32_t chunkStart;

            magic[0] = Reader_u8(&r);
            magic[1] = Reader_u8(&r);
            magic[2] = Reader_u8(&r);
            magic[3] = Reader_u8(&r);
            chunkLen = Reader_u32be(&r);
            if (r.error) {
                break;
            }

            chunkStart = Reader_tell(&r);
            if (chunkStart + chunkLen < chunkStart || chunkStart + chunkLen > len) {
                result->status = MIDI_ERR_CORRUPT;
                return;
            }

            if (magic[0] == 'M' && magic[1] == 'T' && magic[2] == 'r' && magic[3] == 'k') {
                if (seenTracks == 0U) {
                    if (!parseTrack(&r, chunkLen, &tempoMap, outEvents, 0U, result, 0U, 1U)) {
                        return;
                    }
                } else {
                    Reader_skip(&r, chunkLen);
                }
                seenTracks++;
            } else {
                Reader_skip(&r, chunkLen);
            }
        }

        if (r.error) {
            result->status = MIDI_ERR_CORRUPT;
            return;
        }
    }

    Reader_seek(&r, 14U);
    r.error = 0U;
    seenTracks = 0U;
    while (!r.error && Reader_tell(&r) < len && seenTracks < numTracks) {
        uint32_t chunkLen;
        uint32_t chunkStart;

        magic[0] = Reader_u8(&r);
        magic[1] = Reader_u8(&r);
        magic[2] = Reader_u8(&r);
        magic[3] = Reader_u8(&r);
        chunkLen = Reader_u32be(&r);
        if (r.error) {
            break;
        }

        chunkStart = Reader_tell(&r);
        if (chunkStart + chunkLen < chunkStart || chunkStart + chunkLen > len) {
            result->status = MIDI_ERR_CORRUPT;
            return;
        }

        if (magic[0] != 'M' || magic[1] != 'T' || magic[2] != 'r' || magic[3] != 'k') {
            Reader_skip(&r, chunkLen);
            continue;
        }

        if (seenTracks == trackIndex) {
            foundTarget = 1U;
            if (!parseTrack(&r, chunkLen, &tempoMap, outEvents, maxEvents, result, 1U, 1U)) {
                return;
            }
            break;
        }

        Reader_skip(&r, chunkLen);
        seenTracks++;
    }

    if (r.error) {
        result->status = MIDI_ERR_CORRUPT;
        return;
    }

    if (!foundTarget) {
        result->status = MIDI_ERR_NO_TRACK;
        return;
    }

    if (result->noteCount > 0U) {
        uint32_t total = 0U;
        uint16_t n;
        for (n = 0U; n < result->noteCount; n++) {
            total += outEvents[n].durationMs + outEvents[n].delayAfterMs;
        }
        result->durationMs = total;
    }
}

uint32_t Midi_receiveUART(uint8_t* buf, uint32_t bufSize, uint32_t timeoutMs)
{
    uint32_t deadline = (uint32_t)millis() + timeoutMs;
    uint32_t fileLen;
    uint32_t received;
    int b0, b1, b2, b3;

    #define READ_BYTE(var) \
        do { \
            (var) = -1; \
            while ((int32_t)(deadline - (uint32_t)millis()) > 0) { \
                hal_runPendingPID(); \
                if (Serial.available()) { \
                    (var) = Serial.read(); \
                    deadline = (uint32_t)millis() + timeoutMs; \
                    break; \
                } \
            } \
            if ((var) < 0) return 0; \
        } while(0)

    READ_BYTE(b0);
    READ_BYTE(b1);
    READ_BYTE(b2);
    READ_BYTE(b3);

    if (b0 == 'P' && b1 == 'W' && b2 == 'M' && b3 == ' ') {
        int percent = 0;
        int haveDigits = 0;
        int ch = -1;

        while ((int32_t)(deadline - (uint32_t)millis()) > 0) {
            hal_runPendingPID();
            if (!Serial.available()) continue;

            ch = Serial.read();
            deadline = (uint32_t)millis() + timeoutMs;
            if (ch == '\r') continue;
            if (ch == '\n') break;

            if (ch >= '0' && ch <= '9') {
                percent = (percent * 10) + (ch - '0');
                haveDigits = 1;
                if (percent > 100) percent = 100;
            }
        }

        if (haveDigits) {
            char msg[64];
            platform_io_set_finger_pressed_duty((float)percent / 100.0f);
            snprintf(msg, sizeof(msg),
                "[CFG] solenoid press PWM=%d%%\r\n",
                percent);
            Serial.print(msg);
        }
        return 0U;
    }

    fileLen = ((uint32_t)b0 << 24)
            | ((uint32_t)b1 << 16)
            | ((uint32_t)b2 <<  8)
            | ((uint32_t)b3);

    if (fileLen == 0U || fileLen > bufSize) return 0U;

    received = 0U;
    while (received < fileLen) {
        int available = Serial.available();
        if (available > 0) {
            uint32_t remaining = fileLen - received;
            size_t toRead = (size_t)available;
            size_t n;

            if (toRead > (size_t)remaining) toRead = (size_t)remaining;
            n = Serial.readBytes((char*)&buf[received], toRead);
            if (n > 0U) {
                received += (uint32_t)n;
                deadline = (uint32_t)millis() + timeoutMs;
                continue;
            }
        }

        if ((int32_t)(deadline - (uint32_t)millis()) <= 0) return 0U;
        hal_runPendingPID();
    }

    #undef READ_BYTE
    return fileLen;
}

void Midi_receiveAndParse(uint8_t*         uartBuf,
                          uint32_t         uartBufSize,
                          NoteEvent*       outEvents,
                          uint16_t         maxEvents,
                          MidiParseResult* result)
{
    uint32_t len = Midi_receiveUART(uartBuf, uartBufSize, MIDI_UART_TIMEOUT_MS);
    if (len == 0U) {
        result->status = MIDI_ERR_UART;
        result->noteCount = 0U;
        result->notesSkipped = 0U;
        result->durationMs = 0U;
        return;
    }

    Midi_parseBuffer(uartBuf, len, outEvents, maxEvents, result);
}

void Midi_printResult(const MidiParseResult* result)
{
    char buf[80];
    const char* statusStr;

    switch (result->status) {
        case MIDI_OK:           statusStr = "OK";        break;
        case MIDI_ERR_NOT_MIDI: statusStr = "NOT_MIDI";  break;
        case MIDI_ERR_FORMAT:   statusStr = "BAD_FORMAT"; break;
        case MIDI_ERR_NO_TRACK: statusStr = "NO_TRACK";  break;
        case MIDI_ERR_CORRUPT:  statusStr = "CORRUPT";   break;
        case MIDI_ERR_BUF_FULL: statusStr = "BUF_FULL";  break;
        case MIDI_ERR_UART:     statusStr = "UART_FAIL"; break;
        default:                statusStr = "UNKNOWN";   break;
    }

    snprintf(buf, sizeof(buf),
        "MIDI: status=%s  notes=%u  skipped=%u  dur=%lums\r\n",
        statusStr,
        (unsigned)result->noteCount,
        (unsigned)result->notesSkipped,
        (unsigned long)result->durationMs);

    Serial.print(buf);
}
