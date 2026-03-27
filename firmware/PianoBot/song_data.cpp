#include "song_data.h"

namespace
{
constexpr uint8_t songDefaultFingerBitmask = 0x01U;
constexpr uint16_t songPressDurationMs = 250U;
constexpr uint16_t songNoteDurationShortMs = 500U;
constexpr uint16_t songNoteDurationLongMs = 1000U;

#define SONG_NOTE_EVENT(note_position, duration_ms) \
    { \
        (note_position), \
        songDefaultFingerBitmask, \
        songPressDurationMs, \
        (duration_ms) \
    }

song_state_t happyBirthdayStates[] = {
    SONG_NOTE_EVENT(SONG_NOTE_POS_G4, songNoteDurationShortMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_G4, songNoteDurationShortMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_A4, songNoteDurationLongMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_G4, songNoteDurationLongMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_C5, songNoteDurationLongMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_B4, songNoteDurationLongMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_G4, songNoteDurationShortMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_G4, songNoteDurationShortMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_A4, songNoteDurationLongMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_G4, songNoteDurationLongMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_D5, songNoteDurationLongMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_C5, songNoteDurationLongMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_G4, songNoteDurationShortMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_G4, songNoteDurationShortMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_G4, songNoteDurationLongMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_E4, songNoteDurationLongMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_C5, songNoteDurationLongMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_B4, songNoteDurationLongMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_A4, songNoteDurationLongMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_F4, songNoteDurationShortMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_F4, songNoteDurationShortMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_E4, songNoteDurationLongMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_C5, songNoteDurationLongMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_D5, songNoteDurationLongMs),
    SONG_NOTE_EVENT(SONG_NOTE_POS_C5, songNoteDurationLongMs)
};
}

song_t happy_birthday_c_major_song = {
    happyBirthdayStates,
    sizeof(happyBirthdayStates) / sizeof(happyBirthdayStates[0]),
    0U
};
