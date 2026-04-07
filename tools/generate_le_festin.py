#!/usr/bin/env python3
"""
generate_le_festin.py
=====================
Generates a MIDI file for "Le Festin" from Ratatouille.
Sung by Camille, composed by Michael Giacchino.
The charming French waltz melody.

Original key: F major. Kept in F major — fits robot range.
Range: C3(48) to D4(62).
3/4 time (waltz).

Output: le_festin.mid
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


def note_on(delta, note, vel=85):
    return vlq(delta) + bytes([0x90, note, vel])


def note_off(delta, note):
    return vlq(delta) + bytes([0x80, note, 0])


def tempo_event(us_per_beat):
    return vlq(0) + bytes([0xFF, 0x51, 0x03]) + us_per_beat.to_bytes(3, "big")


def end_of_track():
    return vlq(0) + bytes([0xFF, 0x2F, 0x00])


def make_le_festin():
    ppq = 480
    quarter = ppq
    half = ppq * 2
    eighth = ppq // 2
    dotted_half = ppq * 3
    dotted_quarter = quarter + eighth
    whole = ppq * 4

    # Waltz tempo ~138 BPM
    bpm = 138
    tempo_us = 60_000_000 // bpm

    # Key of F major
    # C3=48, D3=50, E3=52, F3=53, G3=55, A3=57, Bb3=58
    # C4=60, D4=62, E4=64, F4=65

    melody = [
        # ── Intro: gentle pickup ─────────────────────────────────
        (60, quarter),         # C4

        # ── "Les reves des amoureux sont comme le bon vin" ───────
        # Bar 1
        (60, half),            # C4 - Les
        (57, quarter),         # A3 - re-

        # Bar 2
        (58, half),            # Bb3 - ves
        (57, quarter),         # A3 - des

        # Bar 3
        (55, half),            # G3 - a-
        (53, quarter),         # F3 - mou-

        # Bar 4
        (55, dotted_half),     # G3 - reux

        # Bar 5
        (57, half),            # A3 - sont
        (58, quarter),         # Bb3 - com-

        # Bar 6
        (60, half),            # C4 - me
        (57, quarter),         # A3 - le

        # Bar 7
        (55, quarter),         # G3 - bon
        (53, quarter),         # F3
        (55, quarter),         # G3

        # Bar 8
        (53, dotted_half),     # F3 - vin

        # ── "Ils donnent de la joie ou bien du chagrin" ──────────
        # Bar 9
        (60, half),            # C4 - Ils
        (57, quarter),         # A3 - don-

        # Bar 10
        (58, half),            # Bb3 - nent
        (57, quarter),         # A3 - de

        # Bar 11
        (55, half),            # G3 - la
        (53, quarter),         # F3 - joi-

        # Bar 12
        (55, dotted_half),     # G3 - e

        # Bar 13
        (57, half),            # A3 - ou
        (58, quarter),         # Bb3 - bien

        # Bar 14
        (60, half),            # C4 - du
        (62, quarter),         # D4 - cha-

        # Bar 15
        (60, quarter),         # C4 - grin
        (58, quarter),         # Bb3
        (57, quarter),         # A3

        # Bar 16
        (53, dotted_half),     # F3

        # ── Bridge / ascending phrase ────────────────────────────
        # Bar 17
        (48, quarter),         # C3
        (50, quarter),         # D3
        (52, quarter),         # E3

        # Bar 18
        (53, half),            # F3
        (55, quarter),         # G3

        # Bar 19
        (57, half),            # A3
        (58, quarter),         # Bb3

        # Bar 20
        (60, dotted_half),     # C4

        # Bar 21
        (62, half),            # D4
        (60, quarter),         # C4

        # Bar 22
        (58, half),            # Bb3
        (57, quarter),         # A3

        # Bar 23
        (55, half),            # G3
        (53, quarter),         # F3

        # Bar 24
        (55, dotted_half),     # G3

        # ── Return to main melody ────────────────────────────────
        # Bar 25
        (60, half),            # C4
        (57, quarter),         # A3

        # Bar 26
        (58, half),            # Bb3
        (57, quarter),         # A3

        # Bar 27
        (55, half),            # G3
        (53, quarter),         # F3

        # Bar 28
        (55, dotted_half),     # G3

        # Bar 29
        (57, half),            # A3
        (58, quarter),         # Bb3

        # Bar 30
        (60, half),            # C4
        (57, quarter),         # A3

        # ── Ending ───────────────────────────────────────────────
        # Bar 31
        (55, quarter),         # G3
        (57, quarter),         # A3
        (55, quarter),         # G3

        # Bar 32
        (53, dotted_half),     # F3

        # Bar 33 - final F held
        (53, dotted_half),     # F3
    ]

    # Build track
    track_data = bytearray()
    track_data += tempo_event(tempo_us)

    staccato = 0.82
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

    with open("le_festin.mid", "wb") as f:
        f.write(midi_file)

    total_ticks = sum(d for _, d in melody)
    duration_sec = total_ticks / ppq * (tempo_us / 1_000_000)

    note_names = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
    notes_used = sorted(set(n for n, _ in melody if n > 0))

    print(f"Generated le_festin.mid ({len(midi_file)} bytes)")
    print(f"  Notes: {sum(1 for n, _ in melody if n > 0)}")
    print(f"  PPQ: {ppq}, Tempo: {bpm} BPM (waltz 3/4)")
    print(f"  Duration: ~{duration_sec:.1f}s")
    print(f"  Notes used: {', '.join(f'{note_names[n%12]}{n//12-1}({n})' for n in notes_used)}")


if __name__ == "__main__":
    make_le_festin()
