#!/usr/bin/env python3
"""
generate_take_on_me.py
======================
Generates a MIDI file for the iconic synth riff from "Take On Me" by A-ha.
Transposed to fit within C2-B4 (MIDI 36-71).

The original riff is in A major. We keep it in A major since
F#, C#, G# are all reachable black keys on the robot.

Output: take_on_me.mid
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


def note_on(delta, note, vel=100):
    return vlq(delta) + bytes([0x90, note, vel])


def note_off(delta, note):
    return vlq(delta) + bytes([0x80, note, 0])


def tempo_event(us_per_beat):
    return vlq(0) + bytes([0xFF, 0x51, 0x03]) + us_per_beat.to_bytes(3, "big")


def end_of_track():
    return vlq(0) + bytes([0xFF, 0x2F, 0x00])


def make_take_on_me():
    ppq = 480
    quarter = ppq
    half = ppq * 2
    eighth = ppq // 2
    sixteenth = ppq // 4
    dotted_quarter = quarter + eighth

    # Take On Me synth riff - 169 BPM
    bpm = 169
    tempo_us = 60_000_000 // bpm

    # The iconic riff uses these notes (original key):
    # F#4-F#4-D4-B3-B3-E4-E4-E4-G#4-G#4-A4-B4
    #
    # In MIDI (octave 3-4 to stay in range):
    # F#4=66, D4=62, B3=59, E4=64, G#4=68... G#4 is MIDI 68
    #
    # G#4 (MIDI 68): wki for G#=18, B1POS(18)=423mm - OUT OF RANGE via B1
    #                 B2POS(18)=(18-2)*23.5=376mm - OK via B2
    # A4 (MIDI 69): wki=19, W1POS(19)=446.5 - out of range
    #               W2POS(19)=(19-2)*23.5=399.5 - out of range with 380mm limit
    #               W3POS(19)=(19-4)*23.5=352.5 - OK via W3
    # B4 (MIDI 71): wki=20, W3POS(20)=(20-4)*23.5=376mm - OK via W3
    #
    # So all notes ARE reachable. Let's use octave 3-4.

    # The riff pattern (each note with its rhythm):
    # Format: (midi_note, duration_ticks)
    riff = [
        # Bar 1: F#-F#-D-B---B-E-E-E
        (66, eighth),          # F#4
        (66, eighth),          # F#4
        (62, eighth),          # D4
        (59, quarter),         # B3
        (59, eighth),          # B3
        (64, eighth),          # E4
        (64, eighth),          # E4
        (64, eighth),          # E4

        # Bar 2: G#-G#-A-B---A-A-A-E
        (68, eighth),          # G#4
        (68, eighth),          # G#4
        (69, eighth),          # A4
        (71, quarter),         # B4
        (69, eighth),          # A4
        (69, eighth),          # A4
        (69, eighth),          # A4
        (64, eighth),          # E4

        # Bar 3: D---D-E-F#-F#---F#-E-E-F#-E
        (62, quarter),         # D4
        (62, eighth),          # D4
        (64, eighth),          # E4
        (66, eighth),          # F#4
        (66, quarter),         # F#4
        (66, eighth),          # F#4
        (64, eighth),          # E4
        (64, eighth),          # E4
        (66, eighth),          # F#4
        (64, eighth),          # E4
    ]

    # Play the riff 3 times
    melody = riff + riff + riff

    # Build track
    track_data = bytearray()
    track_data += tempo_event(tempo_us)

    # Staccato: hold each note for only 30% of its duration,
    # rest for the remaining 70%. Gives a punchy, percussive feel.
    staccato = 0.30

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

    with open("take_on_me.mid", "wb") as f:
        f.write(midi_file)

    total_ticks = sum(d for _, d in melody)
    duration_sec = total_ticks / ppq * (tempo_us / 1_000_000)

    print(f"Generated take_on_me.mid ({len(midi_file)} bytes)")
    print(f"  Notes: {len(melody)}")
    print(f"  PPQ: {ppq}, Tempo: {bpm} BPM")
    print(f"  Duration: ~{duration_sec:.1f}s")

    # Verify range
    notes_used = sorted(set(n for n, _ in melody))
    note_names = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
    print(f"  Notes used: {', '.join(f'{note_names[n%12]}{n//12-1}({n})' for n in notes_used)}")


if __name__ == "__main__":
    make_take_on_me()
