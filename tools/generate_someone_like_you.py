#!/usr/bin/env python3
"""
generate_someone_like_you.py
============================
Generates a MIDI file for "Someone Like You" by Adele.
The iconic piano intro + verse melody in A major.
Range kept within MIDI 45-67 (A2-G4) for the robot.

Output: someone_like_you.mid
"""

import struct


def vlq(val):
    out = []
    out.append(val & 0x7F)
    val >>= 7
    while val:
        out.append((val & 0x7F) | 0x80)
        val >>= 7
    out.reverse()
    return bytes(out)


def note_on(delta, note, vel=80):
    return vlq(delta) + bytes([0x90, note, vel])


def note_off(delta, note):
    return vlq(delta) + bytes([0x80, note, 0])


def tempo_event(us_per_beat):
    return vlq(0) + bytes([0xFF, 0x51, 0x03]) + us_per_beat.to_bytes(3, "big")


def end_of_track():
    return vlq(0) + bytes([0xFF, 0x2F, 0x00])


def make_someone_like_you():
    ppq = 480
    quarter = ppq
    half = ppq * 2
    eighth = ppq // 2
    sixteenth = ppq // 4
    dotted_quarter = quarter + eighth
    dotted_half = ppq * 3
    whole = ppq * 4

    # 67 BPM — slow ballad
    bpm = 67
    tempo_us = 60_000_000 // bpm

    # Key of A major
    # A2=45, B2=47, C#3=49, D3=50, E3=52, F#3=54, G#3=56, A3=57
    # B3=59, C#4=61, D4=62, E4=64, F#4=66, G#4=68... too high
    # Keep below G4(67)

    # The piano intro is a rolling arpeggio pattern over 4 chords:
    # A - E/G# - F#m - D
    #
    # A:    A2-E3-A3-C#4    (45-52-57-61)
    # E/G#: G#2-E3-B3-E4   ... G#2=44 too low for nice sound
    #       Use E3-B3-E4    (52-59-64)
    # F#m:  F#2-C#3-F#3-A3  ... F#2=42
    #       Use F#3-A3-C#4  (54-57-61)
    # D:    D3-A3-D4-F#4   (50-57-62-66)

    # Simplified arpeggio pattern per chord: broken up as eighth notes
    # Pattern: root - 3rd - 5th - octave - 5th - 3rd (rolling)

    intro_a = [
        (45, eighth),    # A2
        (52, eighth),    # E3
        (57, eighth),    # A3
        (61, eighth),    # C#4
        (57, eighth),    # A3
        (52, eighth),    # E3
        (57, eighth),    # A3
        (61, eighth),    # C#4
    ]

    intro_e = [
        (52, eighth),    # E3
        (56, eighth),    # G#3
        (59, eighth),    # B3
        (64, eighth),    # E4
        (59, eighth),    # B3
        (56, eighth),    # G#3
        (59, eighth),    # B3
        (64, eighth),    # E4
    ]

    intro_fm = [
        (54, eighth),    # F#3
        (57, eighth),    # A3
        (61, eighth),    # C#4
        (57, eighth),    # A3
        (54, eighth),    # F#3
        (57, eighth),    # A3
        (61, eighth),    # C#4
        (57, eighth),    # A3
    ]

    intro_d = [
        (50, eighth),    # D3
        (57, eighth),    # A3
        (62, eighth),    # D4
        (66, eighth),    # F#4
        (62, eighth),    # D4
        (57, eighth),    # A3
        (62, eighth),    # D4
        (66, eighth),    # F#4
    ]

    # Full intro: 2 bars of each chord
    intro = intro_a + intro_e + intro_fm + intro_d

    # Verse melody: "I heard that you're settled down"
    verse = [
        # "I heard that you're settled down"
        (61, quarter),         # C#4 - I
        (61, eighth),          # C#4 - heard
        (59, eighth),          # B3  - that
        (57, quarter),         # A3  - you're
        (57, eighth),          # A3  - set-
        (59, eighth),          # B3  - tled
        (57, half),            # A3  - down

        # "That you found a girl"
        (61, quarter),         # C#4 - That
        (61, eighth),          # C#4 - you
        (59, eighth),          # B3  - found
        (57, quarter),         # A3  - a
        (59, dotted_quarter),  # B3  - girl
        (57, eighth),          # A3

        # "and you're married now"
        (61, quarter),         # C#4 - and
        (61, eighth),          # C#4 - you're
        (62, eighth),          # D4  - mar-
        (61, quarter),         # C#4 - ried
        (59, quarter),         # B3  - now
        (57, half),            # A3

        # Rest
        (0, half),
    ]

    # Chorus: "Never mind I'll find someone like you"
    chorus = [
        # "Never mind, I'll find"
        (64, dotted_quarter),  # E4  - Ne-
        (62, eighth),          # D4  - ver
        (61, quarter),         # C#4 - mind
        (59, quarter),         # B3  - I'll
        (61, quarter),         # C#4 - find

        # "someone like you"
        (62, dotted_quarter),  # D4  - some-
        (61, eighth),          # C#4 - one
        (59, quarter),         # B3  - like
        (57, dotted_half),     # A3  - you

        # "I wish nothing but the best"
        (64, dotted_quarter),  # E4  - I
        (62, eighth),          # D4  - wish
        (61, quarter),         # C#4 - no-
        (59, eighth),          # B3  - thing
        (57, eighth),          # A3  - but
        (59, quarter),         # B3  - the
        (57, quarter),         # A3  - best

        # "for you, too"
        (54, quarter),         # F#3 - for
        (57, quarter),         # A3  - you
        (52, dotted_half),     # E3  - too

        (0, quarter),          # rest
    ]

    melody = intro + verse + chorus

    # Build track
    track_data = bytearray()
    track_data += tempo_event(tempo_us)

    staccato = 0.80
    rest_ticks = 0

    for i, (midi_note, duration) in enumerate(melody):
        if midi_note == 0:
            rest_ticks += duration
            continue

        hold = max(int(duration * staccato), 1)
        gap = duration - hold

        delta_on = gap + rest_ticks if i > 0 else rest_ticks
        rest_ticks = 0

        track_data += note_on(delta_on, midi_note)
        track_data += note_off(hold, midi_note)

    track_data += end_of_track()

    # MThd
    header = b"MThd"
    header += struct.pack(">I", 6)
    header += struct.pack(">HHH", 0, 1, ppq)

    # MTrk
    track_chunk = b"MTrk" + struct.pack(">I", len(track_data)) + bytes(track_data)

    midi_file = header + track_chunk

    with open("someone_like_you.mid", "wb") as f:
        f.write(midi_file)

    total_ticks = sum(d for _, d in melody)
    duration_sec = total_ticks / ppq * (tempo_us / 1_000_000)

    note_names = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
    notes_used = sorted(set(n for n, _ in melody if n > 0))

    print(f"Generated someone_like_you.mid ({len(midi_file)} bytes)")
    print(f"  Notes: {sum(1 for n, _ in melody if n > 0)}")
    print(f"  PPQ: {ppq}, Tempo: {bpm} BPM")
    print(f"  Duration: ~{duration_sec:.1f}s")
    print(f"  Notes used: {', '.join(f'{note_names[n%12]}{n//12-1}({n})' for n in notes_used)}")


if __name__ == "__main__":
    make_someone_like_you()
