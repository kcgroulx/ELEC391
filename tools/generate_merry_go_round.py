#!/usr/bin/env python3
"""
generate_merry_go_round.py
==========================
Generates a MIDI file for "Merry-Go-Round of Life" from Howl's Moving Castle.
Composed by Joe Hisaishi. The iconic waltz theme.

Original key: D minor. Kept in D minor — fits robot range well.
Range: A2(45) to F4(65).
3/4 time (waltz).

Output: merry_go_round_of_life.mid
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


def make_merry_go_round():
    ppq = 480
    quarter = ppq
    half = ppq * 2
    eighth = ppq // 2
    dotted_half = ppq * 3      # one full bar in 3/4
    dotted_quarter = quarter + eighth
    whole = ppq * 4

    # Waltz tempo ~168 BPM (per quarter note)
    bpm = 168
    tempo_us = 60_000_000 // bpm

    # Key of D minor
    # D3=50, E3=52, F3=53, G3=55, A3=57, Bb3=58, C4=60
    # D4=62, E4=64, F4=65, G4=67, A4=69

    # The main waltz theme — the haunting melody
    # 3/4 time: each bar = 3 quarter notes

    melody = [
        # ── Intro: gentle waltz bass pattern (2 bars) ────────────
        (50, quarter),         # D3
        (57, quarter),         # A3
        (57, quarter),         # A3

        (50, quarter),         # D3
        (57, quarter),         # A3
        (57, quarter),         # A3

        # ── Main theme A ─────────────────────────────────────────
        # Bar 1: D4 held
        (62, dotted_half),     # D4 (full bar)

        # Bar 2: E4 - D4
        (64, half),            # E4
        (62, quarter),         # D4

        # Bar 3: F4 held
        (65, dotted_half),     # F4 (full bar)

        # Bar 4: E4 - D4
        (64, half),            # E4
        (62, quarter),         # D4

        # Bar 5: C4 held
        (60, dotted_half),     # C4 (full bar)

        # Bar 6: D4 - C4
        (62, half),            # D4
        (60, quarter),         # C4

        # Bar 7-8: Bb3 held, resolve to A3
        (58, dotted_half),     # Bb3 (full bar)
        (57, dotted_half),     # A3  (full bar)

        # ── Main theme B (repeat with variation) ─────────────────
        # Bar 9: D4
        (62, dotted_half),     # D4

        # Bar 10: E4 - F4
        (64, half),            # E4
        (65, quarter),         # F4

        # Bar 11: A4... too high. Use A3 octave lower approach
        # Bar 11: G3 held
        (55, dotted_half),     # G3

        # Bar 12: A3 - Bb3
        (57, half),            # A3
        (58, quarter),         # Bb3

        # Bar 13: A3
        (57, dotted_half),     # A3

        # Bar 14: G3 - F3
        (55, half),            # G3
        (53, quarter),         # F3

        # Bar 15-16: E3 held, resolve to D3
        (52, dotted_half),     # E3
        (50, dotted_half),     # D3

        # ── Ascending phrase (emotional climax) ──────────────────
        # Bar 17: D3 - E3 - F3
        (50, quarter),         # D3
        (52, quarter),         # E3
        (53, quarter),         # F3

        # Bar 18: G3 - A3 - Bb3
        (55, quarter),         # G3
        (57, quarter),         # A3
        (58, quarter),         # Bb3

        # Bar 19: A3 held
        (57, dotted_half),     # A3

        # Bar 20: G3 - F3 - E3
        (55, quarter),         # G3
        (53, quarter),         # F3
        (52, quarter),         # E3

        # ── Return to main theme ─────────────────────────────────
        # Bar 21: D4
        (62, dotted_half),     # D4

        # Bar 22: E4 - D4
        (64, half),            # E4
        (62, quarter),         # D4

        # Bar 23: F4
        (65, dotted_half),     # F4

        # Bar 24: E4 - D4
        (64, half),            # E4
        (62, quarter),         # D4

        # Bar 25: C4
        (60, dotted_half),     # C4

        # Bar 26: Bb3 - A3
        (58, half),            # Bb3
        (57, quarter),         # A3

        # ── Ending ───────────────────────────────────────────────
        # Bar 27: Bb3
        (58, dotted_half),     # Bb3

        # Bar 28: A3
        (57, dotted_half),     # A3

        # Bar 29-30: D3 final (held long)
        (50, dotted_half),     # D3
        (50, dotted_half),     # D3 (held another bar)
    ]

    # Build track
    track_data = bytearray()
    track_data += tempo_event(tempo_us)

    staccato = 0.85  # slightly detached for waltz feel
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

    with open("merry_go_round_of_life.mid", "wb") as f:
        f.write(midi_file)

    total_ticks = sum(d for _, d in melody)
    duration_sec = total_ticks / ppq * (tempo_us / 1_000_000)

    note_names = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
    notes_used = sorted(set(n for n, _ in melody if n > 0))

    print(f"Generated merry_go_round_of_life.mid ({len(midi_file)} bytes)")
    print(f"  Notes: {sum(1 for n, _ in melody if n > 0)}")
    print(f"  PPQ: {ppq}, Tempo: {bpm} BPM (waltz 3/4)")
    print(f"  Duration: ~{duration_sec:.1f}s")
    print(f"  Notes used: {', '.join(f'{note_names[n%12]}{n//12-1}({n})' for n in notes_used)}")


if __name__ == "__main__":
    make_merry_go_round()
