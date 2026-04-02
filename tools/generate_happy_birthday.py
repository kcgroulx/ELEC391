#!/usr/bin/env python3
"""
generate_happy_birthday.py
==========================
Generates a Standard MIDI File (format 0) for "Happy Birthday" in C major.
All notes are in the range C3–C5 (MIDI 48–72) to stay within the piano robot's
reachable keys.

Output: happy_birthday.mid
"""

import struct


def vlq(val):
    """Encode an integer as a MIDI variable-length quantity."""
    out = []
    out.append(val & 0x7F)
    val >>= 7
    while val:
        out.append((val & 0x7F) | 0x80)
        val >>= 7
    out.reverse()
    return bytes(out)


def note_on(delta, note, vel=100):
    return vlq(delta) + bytes([0x90, note, vel])


def note_off(delta, note):
    return vlq(delta) + bytes([0x80, note, 0])


def tempo_event(us_per_beat):
    return (vlq(0) + bytes([0xFF, 0x51, 0x03])
            + us_per_beat.to_bytes(3, "big"))


def end_of_track():
    return vlq(0) + bytes([0xFF, 0x2F, 0x00])


def make_happy_birthday():
    ppq = 480  # ticks per quarter note

    # Note durations in ticks
    quarter = ppq
    half = ppq * 2
    dotted_half = ppq * 3
    eighth = ppq // 2
    dotted_quarter = quarter + eighth

    # Happy Birthday melody in C major (key of C, starting on G3)
    # Format: (midi_note, duration_ticks)
    #
    # "Hap-py birth-day to you" (x2)
    # "Hap-py birth-day dear ___"
    # "Hap-py birth-day to you"

    melody = [
        # "Hap-py birth-day to you"
        (55, eighth),          # G3 - "Hap"
        (55, eighth),          # G3 - "py"
        (57, quarter),         # A3 - "birth"
        (55, quarter),         # G3 - "day"
        (60, quarter),         # C4 - "to"
        (59, half),            # B3 - "you"

        # "Hap-py birth-day to you"
        (55, eighth),          # G3 - "Hap"
        (55, eighth),          # G3 - "py"
        (57, quarter),         # A3 - "birth"
        (55, quarter),         # G3 - "day"
        (62, quarter),         # D4 - "to"
        (60, half),            # C4 - "you"

        # "Hap-py birth-day dear ___"
        (55, eighth),          # G3 - "Hap"
        (55, eighth),          # G3 - "py"
        (55, quarter),         # G3 - "birth"  (G4 out of range, using G3)
        (64, quarter),         # E4 - "day"
        (60, quarter),         # C4 - "dear"
        (59, quarter),         # B3 - "___"
        (57, half),            # A3 - (held)

        # "Hap-py birth-day to you"
        (65, eighth),          # F4 - "Hap"
        (65, eighth),          # F4 - "py"
        (64, quarter),         # E4 - "birth"
        (60, quarter),         # C4 - "day"
        (62, quarter),         # D4 - "to"
        (60, dotted_half),     # C4 - "you"
    ]

    # Check range
    for note, _ in melody:
        if note > 66:
            print(f"WARNING: MIDI {note} is above reachable limit (66/F#4)")

    # Build track data
    track_data = bytearray()

    # Set tempo: 120 BPM = 500000 us/beat
    track_data += tempo_event(500000)

    # Write notes
    for midi_note, duration in melody:
        track_data += note_on(0, midi_note)
        track_data += note_off(duration, midi_note)
        track_data += vlq(eighth)  # small gap encoded as a rest before next note_on
        # Remove the trailing rest — we'll handle it differently

    # Oops, the gap approach above leaves a dangling vlq. Let me redo this properly.
    track_data = bytearray()
    track_data += tempo_event(500000)

    gap = eighth // 2  # small articulation gap between notes

    for midi_note, duration in melody:
        hold = duration - gap
        if hold < 1:
            hold = duration
            g = 0
        else:
            g = gap

        track_data += note_on(0, midi_note)
        track_data += note_off(hold, midi_note)
        # The gap before the next note is the delta of the next note_on
        # We need to prepend it to the next event, so we track it
        # Actually, let's just add a rest by making the next note_on's delta = g

    # Third time's the charm — clean approach
    track_data = bytearray()
    track_data += tempo_event(500000)

    gap = eighth // 4  # small articulation gap

    for i, (midi_note, duration) in enumerate(melody):
        hold = max(duration - gap, 1)

        # Delta before note_on: 0 for first note, gap for subsequent
        delta_on = gap if i > 0 else 0
        track_data += note_on(delta_on, midi_note)
        track_data += note_off(hold, midi_note)

    track_data += end_of_track()

    # Build the file
    # MThd header
    header = b"MThd"
    header += struct.pack(">I", 6)        # header length
    header += struct.pack(">HHH", 0, 1, ppq)  # format 0, 1 track, ppq

    # MTrk chunk
    track_chunk = b"MTrk" + struct.pack(">I", len(track_data)) + bytes(track_data)

    midi_file = header + track_chunk

    with open("happy_birthday.mid", "wb") as f:
        f.write(midi_file)

    print(f"Generated happy_birthday.mid ({len(midi_file)} bytes)")
    print(f"  Notes: {len(melody)}")
    print(f"  PPQ: {ppq}, Tempo: 120 BPM")


if __name__ == "__main__":
    make_happy_birthday()
