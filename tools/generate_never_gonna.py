#!/usr/bin/env python3
"""
generate_never_gonna.py
=======================
Generates a MIDI file for "Never Gonna Give You Up" by Rick Astley.
The iconic intro riff + verse melody, transposed to fit robot range.

Original key: Ab major. Transposed to C major for white keys.
Range kept within MIDI 48-67 (C3-G4) to stay safe within 380mm travel.

Output: never_gonna_give_you_up.mid
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


def note_on(delta, note, vel=90):
    return vlq(delta) + bytes([0x90, note, vel])


def note_off(delta, note):
    return vlq(delta) + bytes([0x80, note, 0])


def tempo_event(us_per_beat):
    return vlq(0) + bytes([0xFF, 0x51, 0x03]) + us_per_beat.to_bytes(3, "big")


def end_of_track():
    return vlq(0) + bytes([0xFF, 0x2F, 0x00])


def make_never_gonna():
    ppq = 480
    quarter = ppq
    half = ppq * 2
    eighth = ppq // 2
    sixteenth = ppq // 4
    dotted_quarter = quarter + eighth
    dotted_half = ppq * 3
    whole = ppq * 4

    # 113 BPM
    bpm = 113
    tempo_us = 60_000_000 // bpm

    # Notes in C major (transposed from Ab):
    # C3=48, D3=50, E3=52, F3=53, G3=55, A3=57, B3=59
    # C4=60, D4=62, E4=64, F4=65, G4=67

    # The intro bass riff (iconic synth line)
    intro_riff = [
        # "dun dun dun dun-dun dun-dun-dun"
        (55, eighth),          # G3
        (57, eighth),          # A3
        (60, quarter),         # C4
        (57, eighth),          # A3

        (60, eighth),          # C4
        (62, eighth),          # D4
        (59, eighth),          # B3
        (55, quarter),         # G3
        (55, eighth),          # G3

        (62, quarter),         # D4
        (60, eighth),          # C4
        (57, quarter),         # A3
        (55, eighth),          # G3

        (55, eighth),          # G3
        (62, quarter),         # D4
        (60, eighth),          # C4
        (57, eighth),          # A3
        (60, eighth),          # C4
        (55, quarter),         # G3
    ]

    # Verse melody: "Never gonna give you up"
    verse1 = [
        # "We're no strangers to love"
        (60, quarter),         # C4 - We're
        (62, quarter),         # D4 - no
        (57, quarter),         # A3 - stran-
        (57, eighth),          # A3 - gers
        (64, quarter),         # E4 - to
        (64, eighth),          # E4
        (62, dotted_quarter),  # D4 - love

        # "You know the rules and so do I"
        (60, quarter),         # C4 - You
        (62, quarter),         # D4 - know
        (55, quarter),         # G3 - the
        (55, eighth),          # G3
        (57, quarter),         # A3 - rules
        (60, eighth),          # C4 - and
        (57, quarter),         # A3 - so
        (55, eighth),          # G3 - do
        (55, quarter),         # G3 - I
    ]

    # Chorus: "Never gonna give you up"
    chorus = [
        # "Never gonna give you up"
        (55, eighth),          # G3 - Ne-
        (57, eighth),          # A3 - ver
        (60, eighth),          # C4 - gon-
        (57, quarter),         # A3 - na
        (64, quarter),         # E4 - give
        (64, quarter),         # E4 - you
        (62, dotted_quarter),  # D4 - up

        # "Never gonna let you down"
        (55, eighth),          # G3 - Ne-
        (57, eighth),          # A3 - ver
        (60, eighth),          # C4 - gon-
        (57, quarter),         # A3 - na
        (67, quarter),         # G4 - let
        (62, quarter),         # D4 - you
        (60, dotted_quarter),  # C4 - down

        # "Never gonna run around and desert you"
        (55, eighth),          # G3 - Ne-
        (57, eighth),          # A3 - ver
        (60, eighth),          # C4 - gon-
        (57, quarter),         # A3 - na
        (62, quarter),         # D4 - run
        (64, eighth),          # E4 - a-
        (62, eighth),          # D4 - round
        (60, quarter),         # C4 - and
        (57, eighth),          # A3 - de-
        (55, eighth),          # G3 - sert
        (55, quarter),         # G3 - you
        (0, quarter),          # rest
    ]

    melody = intro_riff + verse1 + chorus

    # Build track
    track_data = bytearray()
    track_data += tempo_event(tempo_us)

    # Slightly staccato for clarity
    staccato = 0.75

    prev_was_rest = False
    rest_ticks = 0

    for i, (midi_note, duration) in enumerate(melody):
        if midi_note == 0:
            # Rest — accumulate and add to next note's delta
            rest_ticks += duration
            prev_was_rest = True
            continue

        hold = max(int(duration * staccato), 1)
        gap = duration - hold

        delta_on = gap + rest_ticks if i > 0 else rest_ticks
        rest_ticks = 0
        prev_was_rest = False

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

    with open("never_gonna_give_you_up.mid", "wb") as f:
        f.write(midi_file)

    total_ticks = sum(d for _, d in melody)
    duration_sec = total_ticks / ppq * (tempo_us / 1_000_000)

    note_names = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
    notes_used = sorted(set(n for n, _ in melody if n > 0))

    print(f"Generated never_gonna_give_you_up.mid ({len(midi_file)} bytes)")
    print(f"  Notes: {sum(1 for n, _ in melody if n > 0)}")
    print(f"  PPQ: {ppq}, Tempo: {bpm} BPM")
    print(f"  Duration: ~{duration_sec:.1f}s")
    print(f"  Notes used: {', '.join(f'{note_names[n%12]}{n//12-1}({n})' for n in notes_used)}")


if __name__ == "__main__":
    make_never_gonna()
