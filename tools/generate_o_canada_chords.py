#!/usr/bin/env python3
"""
generate_o_canada_chords.py
===========================
O Canada with chords for the piano robot.

Chords are encoded as simultaneous note-ons (delta=0 between chord tones).
The robot's chord planner detects notes starting at the same time and
assigns fingers automatically.

Feasible triads with W1/W2/W3 spacing (0, +2, +4 white keys):
  C-E-G, D-F-A, E-G-B, F-A-C, G-B-D

Output: o_canada_chords.mid
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


def make_o_canada_chords():
    ppq = 480
    quarter = ppq
    half = ppq * 2
    eighth = ppq // 2
    dotted_half = ppq * 3
    dotted_quarter = quarter + eighth
    whole = ppq * 4

    bpm = 100
    tempo_us = 60_000_000 // bpm

    # Build as a list of events: each is either
    #   ("note", midi, duration_ticks)        — single note
    #   ("chord", [midi1,midi2,midi3], dur)   — simultaneous notes
    #   ("rest", duration_ticks)              — silence

    events = [
        # ══════ "O Canada!" ══════════════════════════════════════
        # F major chord (F3-A3-C4)
        ("chord", [53, 57, 60], half),

        # "Our home and native land"
        ("note", 60, dotted_quarter),    # C4
        ("note", 60, eighth),            # C4

        # F major chord
        ("chord", [53, 57, 60], dotted_half),

        # "True patriot love"
        ("note", 55, quarter),           # G3
        ("note", 57, quarter),           # A3
        ("note", 58, quarter),           # Bb3
        ("note", 60, quarter),           # C4
        ("note", 62, quarter),           # D4

        # C major chord (C3-E3-G3)
        ("chord", [48, 52, 55], dotted_half),

        # ══════ "in all thy sons command" ════════════════════════
        ("note", 57, half),              # A3
        ("note", 59, dotted_quarter),    # B3
        ("note", 59, eighth),            # B3

        # C major chord on "command"
        ("chord", [48, 52, 60], dotted_half),

        # "With glowing hearts we see thee rise"
        ("note", 62, quarter),           # D4
        ("note", 64, quarter),           # E4
        ("note", 64, quarter),           # E4
        ("note", 62, quarter),           # D4
        ("note", 62, quarter),           # D4

        # F chord
        ("chord", [53, 57, 60], dotted_half),

        # ══════ "The true north strong and free" ═════════════════
        ("note", 55, quarter),           # G3
        ("note", 58, dotted_quarter),    # Bb3
        ("note", 57, eighth),            # A3
        ("note", 55, quarter),           # G3

        ("note", 57, quarter),           # A3
        ("note", 60, dotted_quarter),    # C4
        ("note", 58, eighth),            # Bb3
        ("note", 57, quarter),           # A3

        # D minor chord (D3-F3-A3)
        ("chord", [50, 53, 57], quarter),

        ("note", 62, quarter),           # D4
        ("note", 60, quarter),           # C4
        ("note", 58, quarter),           # Bb3
        ("note", 57, quarter),           # A3

        # C chord resolve
        ("chord", [48, 52, 55], dotted_half),

        # ══════ Repeat ═══════════════════════════════════════════
        ("note", 55, quarter),           # G3
        ("note", 58, dotted_quarter),    # Bb3
        ("note", 57, eighth),            # A3
        ("note", 55, quarter),           # G3

        ("note", 57, quarter),           # A3
        ("note", 60, dotted_quarter),    # C4
        ("note", 58, eighth),            # Bb3
        ("note", 57, quarter),           # A3

        ("note", 57, quarter),           # A3
        ("note", 55, quarter),           # G3
        ("note", 60, quarter),           # C4
        ("note", 60, eighth),            # C4
        ("note", 59, eighth),            # B3
        ("note", 57, eighth),            # A3
        ("note", 59, eighth),            # B3

        # F chord
        ("chord", [53, 57, 60], dotted_half),

        # ══════ "God keep our land" ══════════════════════════════
        # F chord
        ("chord", [53, 57, 60], half),

        ("note", 60, dotted_quarter),    # C4
        ("note", 60, eighth),            # C4

        # F chord
        ("chord", [53, 57, 60], dotted_half),

        # "Glorious and free"
        ("note", 58, half),              # Bb3
        ("note", 62, dotted_quarter),    # D4
        ("note", 62, eighth),            # D4

        # C chord
        ("chord", [48, 52, 55], dotted_half),

        # ══════ "O Canada we stand on guard for thee" ════════════
        ("note", 60, half),              # C4
        ("note", 61, dotted_quarter),    # C#4
        ("note", 61, eighth),            # C#4

        ("note", 62, quarter),           # D4
        ("note", 58, quarter),           # Bb3
        ("note", 57, quarter),           # A3
        ("note", 55, quarter),           # G3

        # F chord
        ("chord", [53, 57, 60], half),

        ("note", 55, half),              # G3

        ("note", 57, dotted_half),       # A3

        # Big C chord
        ("chord", [48, 52, 60], whole),

        # ══════ Final F major chord held long ════════════════════
        ("chord", [53, 57, 60], whole),

        ("rest", whole),
    ]

    # Encode to MIDI
    track_data = bytearray()
    track_data += tempo_event(tempo_us)

    staccato = 0.85
    gap_ticks = eighth // 2  # small gap between events

    for i, ev in enumerate(events):
        typ = ev[0]

        # Delta before this event (gap from previous)
        delta = gap_ticks if i > 0 else 0

        if typ == "rest":
            # Just skip time — add to next event's delta
            # We'll handle this by writing nothing and adding to gap
            # Actually, track the accumulated rest
            pass  # handled below

        elif typ == "note":
            midi = ev[1]
            dur = ev[2]
            hold = max(int(dur * staccato), 1)

            track_data += note_on(delta, midi)
            track_data += note_off(hold, midi)

        elif typ == "chord":
            notes = ev[1]
            dur = ev[2]
            hold = max(int(dur * staccato), 1)

            # First note gets the delta, rest get delta=0 (simultaneous)
            for j, midi in enumerate(notes):
                d = delta if j == 0 else 0
                track_data += note_on(d, midi)

            # Release all at the same time
            for j, midi in enumerate(notes):
                d = hold if j == 0 else 0
                track_data += note_off(d, midi)

    track_data += end_of_track()

    # MThd
    header = b"MThd"
    header += struct.pack(">I", 6)
    header += struct.pack(">HHH", 0, 1, ppq)

    # MTrk
    track_chunk = b"MTrk" + struct.pack(">I", len(track_data)) + bytes(track_data)

    midi_file = header + track_chunk

    with open("o_canada_chords.mid", "wb") as f:
        f.write(midi_file)

    # Count
    total_notes = sum(
        len(ev[1]) if ev[0] == "chord" else (1 if ev[0] == "note" else 0)
        for ev in events
    )
    all_midi = set()
    for ev in events:
        if ev[0] == "note":
            all_midi.add(ev[1])
        elif ev[0] == "chord":
            all_midi.update(ev[1])

    note_names = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
    chord_count = sum(1 for ev in events if ev[0] == "chord")

    print(f"Generated o_canada_chords.mid ({len(midi_file)} bytes)")
    print(f"  Total note events: {total_notes}")
    print(f"  Chords: {chord_count}")
    print(f"  PPQ: {ppq}, Tempo: {bpm} BPM")
    print(f"  Notes used: {', '.join(f'{note_names[n%12]}{n//12-1}({n})' for n in sorted(all_midi))}")


if __name__ == "__main__":
    make_o_canada_chords()
