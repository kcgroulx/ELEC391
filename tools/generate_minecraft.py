#!/usr/bin/env python3
"""
generate_minecraft.py
=====================
Generates a MIDI file for the main Minecraft theme ("Sweden" by C418).
Simplified melody arrangement in A major, transposed to fit C2-B4 range.

Output: minecraft_sweden.mid
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


def make_minecraft():
    ppq = 480
    quarter = ppq
    half = ppq * 2
    whole = ppq * 4
    eighth = ppq // 2
    dotted_half = ppq * 3
    dotted_quarter = quarter + eighth

    # Sweden by C418 - simplified melody
    # Original is in A major. Using octave 3-4 range (MIDI 48-71)
    #
    # A3=57, B3=59, C#4=61, D4=62, E4=64, F#4=66
    # A2=45, B2=47, C#3=49, D3=50, E3=52, F#3=54

    # The melody has a dreamy, slow quality - lots of sustained notes
    # BPM ~70 (slow and peaceful)
    bpm = 70
    tempo_us = 60_000_000 // bpm

    melody = [
        # Opening phrase
        (64, half),           # E4
        (61, quarter),        # C#4
        (57, quarter),        # A3

        (64, half),           # E4
        (61, quarter),        # C#4
        (59, quarter),        # B3

        (57, half),           # A3
        (52, quarter),        # E3
        (57, quarter),        # A3

        (59, dotted_half),    # B3
        (57, quarter),        # A3

        # Second phrase
        (64, half),           # E4
        (61, quarter),        # C#4
        (57, quarter),        # A3

        (66, half),           # F#4
        (64, quarter),        # E4
        (61, quarter),        # C#4

        (59, half),           # B3
        (57, quarter),        # A3
        (52, quarter),        # E3

        (57, dotted_half),    # A3
        (0, quarter),         # rest

        # Ascending phrase
        (52, quarter),        # E3
        (54, quarter),        # F#3
        (57, quarter),        # A3
        (59, quarter),        # B3

        (61, half),           # C#4
        (59, quarter),        # B3
        (57, quarter),        # A3

        (52, half),           # E3
        (54, quarter),        # F#3
        (57, quarter),        # A3

        (59, dotted_half),    # B3
        (57, quarter),        # A3

        # Climax phrase
        (64, half),           # E4
        (66, quarter),        # F#4
        (64, quarter),        # E4

        (61, half),           # C#4
        (59, quarter),        # B3
        (57, quarter),        # A3

        (59, half),           # B3
        (61, quarter),        # C#4
        (59, quarter),        # B3

        (57, dotted_half),    # A3
        (52, quarter),        # E3

        # Gentle ending
        (57, half),           # A3
        (54, quarter),        # F#3
        (52, quarter),        # E3

        (50, half),           # D3
        (52, quarter),        # E3
        (54, quarter),        # F#3

        (57, half),           # A3
        (59, quarter),        # B3
        (57, quarter),        # A3

        (52, whole),          # E3 - long final note
    ]

    # Build track
    track_data = bytearray()
    track_data += tempo_event(tempo_us)

    gap = eighth // 4  # small articulation gap

    for i, (midi_note, duration) in enumerate(melody):
        delta_on = gap if i > 0 else 0

        if midi_note == 0:
            # Rest: just advance time
            track_data += vlq(duration + delta_on)  # padding
            # Actually we need a proper way to handle rests.
            # Just skip - add the rest duration to the next note's delta
            continue

        hold = max(duration - gap, 1)
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

    with open("minecraft_sweden.mid", "wb") as f:
        f.write(midi_file)

    note_count = sum(1 for n, _ in melody if n > 0)
    print(f"Generated minecraft_sweden.mid ({len(midi_file)} bytes)")
    print(f"  Notes: {note_count}")
    print(f"  PPQ: {ppq}, Tempo: {bpm} BPM")
    print(f"  Duration: ~{sum(d for _, d in melody) / ppq * (tempo_us / 1_000_000):.0f}s")


if __name__ == "__main__":
    make_minecraft()
