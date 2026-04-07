#!/usr/bin/env python3
"""Generate a simple melody-plus-chords showcase MIDI for the piano robot.

Pattern:
    note-note-note-note
    chord

Repeated with easy C-major-family material so the robot can show both
single-note and chord playback without stressing the planner.
"""

from __future__ import annotations

from pathlib import Path
import struct


PPQ = 480
TEMPO_US_PER_BEAT = 625_000  # 96 BPM
VELOCITY = 96
MELODY_NOTE_TICKS = PPQ      # quarter note
CHORD_TICKS = MELODY_NOTE_TICKS
MELODY_GAP_TICKS = PPQ // 8
CHORD_GAP_TICKS = MELODY_GAP_TICKS
OUTPUT_NAME = "chord_showcase.mid"


SECTIONS = [
    ((48, 50, 52, 55), (48, 52, 55)),  # C major
    ((57, 55, 53, 52), (41, 45, 48)),  # F major
    ((43, 45, 47, 50), (43, 47, 50)),  # G major
    ((52, 50, 48, 43), (48, 52, 55)),  # C major
]


def vlq(value: int) -> bytes:
    parts = [value & 0x7F]
    value >>= 7
    while value:
        parts.append((value & 0x7F) | 0x80)
        value >>= 7
    return bytes(reversed(parts))


def tempo_event(us_per_beat: int) -> bytes:
    return vlq(0) + bytes((0xFF, 0x51, 0x03)) + us_per_beat.to_bytes(3, "big")


def track_name_event(name: str) -> bytes:
    payload = name.encode("ascii")
    return vlq(0) + bytes((0xFF, 0x03)) + vlq(len(payload)) + payload


def note_on(delta: int, note: int, velocity: int = VELOCITY) -> bytes:
    return vlq(delta) + bytes((0x90, note, velocity))


def note_off(delta: int, note: int) -> bytes:
    return vlq(delta) + bytes((0x80, note, 0))


def end_of_track() -> bytes:
    return vlq(0) + bytes((0xFF, 0x2F, 0x00))


def add_single_note(track: bytearray, pending_delta: int, note: int) -> int:
    hold_ticks = MELODY_NOTE_TICKS - MELODY_GAP_TICKS
    track += note_on(pending_delta, note)
    track += note_off(hold_ticks, note)
    return MELODY_GAP_TICKS


def add_chord(track: bytearray, pending_delta: int, chord: tuple[int, int, int]) -> int:
    hold_ticks = CHORD_TICKS - CHORD_GAP_TICKS
    for index, note in enumerate(chord):
        track += note_on(pending_delta if index == 0 else 0, note)
    for index, note in enumerate(chord):
        track += note_off(hold_ticks if index == 0 else 0, note)
    return CHORD_GAP_TICKS


def build_track() -> bytes:
    track = bytearray()
    track += track_name_event("Chord Showcase")
    track += tempo_event(TEMPO_US_PER_BEAT)

    pending_delta = 0
    for melody, chord in SECTIONS:
        for note in melody:
            pending_delta = add_single_note(track, pending_delta, note)
        pending_delta = add_chord(track, pending_delta, chord)

    track += end_of_track()
    return bytes(track)


def build_midi(track_data: bytes) -> bytes:
    header = b"MThd" + struct.pack(">IHHH", 6, 0, 1, PPQ)
    track = b"MTrk" + struct.pack(">I", len(track_data)) + track_data
    return header + track


def main() -> None:
    output_path = Path(__file__).with_name("midi") / OUTPUT_NAME
    output_path.write_bytes(build_midi(build_track()))
    print(f"Wrote {output_path} ({output_path.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
