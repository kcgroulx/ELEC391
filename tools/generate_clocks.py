#!/usr/bin/env python3
"""
generate_clocks.py
==================
Generates a MIDI file for the Clocks piano riff by Coldplay.
Original key: Eb major. Single track, just the iconic arpeggio pattern.
All notes within robot range (MIDI 36-71).

The riff is a rolling triplet arpeggio over 3 chords:
  Eb major: Eb-G-Bb  (MIDI: 51-55-58 in octave 3)
  Bbm:      Bb-Db-F  (MIDI: 58-61-65 in octave 3-4)
  Fm:       F-Ab-C   (MIDI: 53-56-60 in octave 3-4)

Output: clocks_piano.mid
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


def make_clocks():
    ppq = 480
    # Original tempo ~131 BPM, triplet eighths
    bpm = 131
    tempo_us = 60_000_000 // bpm

    # Triplet eighth = 1/3 of a quarter note
    triplet_eighth = ppq // 3

    # The Clocks riff pattern per chord:
    # Three notes arpeggiated as triplets, repeated 4 times per bar
    # Pattern: low - high - mid (repeating)

    # Eb major: Bb3(58) - Eb4(63) - G3(55)
    eb_chord = [
        (58, triplet_eighth),   # Bb3
        (63, triplet_eighth),   # Eb4
        (55, triplet_eighth),   # G3
    ]

    # Bbm: F3(53) - Bb3(58) - Db4(61)
    bbm_chord = [
        (53, triplet_eighth),   # F3
        (58, triplet_eighth),   # Bb3
        (61, triplet_eighth),   # Db4/C#4
    ]

    # Fm: C4(60) - F3(53) - Ab3(56)
    fm_chord = [
        (60, triplet_eighth),   # C4
        (53, triplet_eighth),   # F3
        (56, triplet_eighth),   # Ab3/G#3
    ]

    # Each chord repeats 4x per bar (4 bars of triplets = 12 notes per chord)
    def repeat_chord(chord, times=4):
        return chord * times

    # Full riff: Eb x4 | Bbm x4 | Fm x4
    riff = repeat_chord(eb_chord) + repeat_chord(bbm_chord) + repeat_chord(fm_chord)

    # Play the riff 4 times for a full section
    melody = riff * 4

    # Build track
    track_data = bytearray()
    track_data += tempo_event(tempo_us)

    # Staccato-ish: hold for 70% of note duration
    staccato = 0.70

    for i, (midi_note, duration) in enumerate(melody):
        hold = max(int(duration * staccato), 1)
        gap = duration - hold

        delta_on = gap if i > 0 else 0
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

    with open("clocks_piano.mid", "wb") as f:
        f.write(midi_file)

    total_ticks = sum(d for _, d in melody)
    duration_sec = total_ticks / ppq * (tempo_us / 1_000_000)

    note_names = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
    notes_used = sorted(set(n for n, _ in melody))

    print(f"Generated clocks_piano.mid ({len(midi_file)} bytes)")
    print(f"  Notes: {len(melody)}")
    print(f"  PPQ: {ppq}, Tempo: {bpm} BPM")
    print(f"  Duration: ~{duration_sec:.1f}s")
    print(f"  Notes used: {', '.join(f'{note_names[n%12]}{n//12-1}({n})' for n in notes_used)}")


if __name__ == "__main__":
    make_clocks()
