#!/usr/bin/env python3
"""Generate a hand-pattern sweep MIDI for the piano robot.

The sequence starts with the hand at the lowest position in the current editor
range (C2 through B4). At each hand position it plays the notes that sit under
the fixed finger layout in this order:

    W1 -> B1 -> W2 -> B2 -> W3

Black-finger steps are skipped when that finger lands over an E-F or B-C gap,
because there is no black key there. After each position, the whole hand moves
up by one white-key step and repeats until W3 reaches B4.
"""

from __future__ import annotations

from pathlib import Path
import struct


PPQ = 480
TEMPO_US_PER_BEAT = 500_000  # 120 BPM
NOTE_ON_VELOCITY = 96
NOTE_HOLD_TICKS = 300
NOTE_GAP_TICKS = 90
POSITION_GAP_TICKS = 180
OUTPUT_NAME = "robot_hand_pattern_test.mid"

NOTE_NAMES = ("C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B")
WHITE_NOTES = (
    36, 38, 40, 41, 43, 45, 47,
    48, 50, 52, 53, 55, 57, 59,
    60, 62, 64, 65, 67, 69, 71,
)
WHITE_WITH_BLACK_AFTER = {0, 2, 5, 7, 9}
FINGER_ORDER = ("W1", "B1", "W2", "B2", "W3")


def vlq(value: int) -> bytes:
    parts = [value & 0x7F]
    value >>= 7
    while value:
        parts.append((value & 0x7F) | 0x80)
        value >>= 7
    parts.reverse()
    return bytes(parts)


def note_on(delta: int, note: int, velocity: int = NOTE_ON_VELOCITY) -> bytes:
    return vlq(delta) + bytes((0x90, note, velocity))


def note_off(delta: int, note: int) -> bytes:
    return vlq(delta) + bytes((0x80, note, 0))


def tempo_event(us_per_beat: int) -> bytes:
    return vlq(0) + bytes((0xFF, 0x51, 0x03)) + us_per_beat.to_bytes(3, "big")


def track_name_event(name: str) -> bytes:
    payload = name.encode("ascii")
    return vlq(0) + bytes((0xFF, 0x03)) + vlq(len(payload)) + payload


def end_of_track() -> bytes:
    return vlq(0) + bytes((0xFF, 0x2F, 0x00))


def note_name(note: int) -> str:
    return f"{NOTE_NAMES[note % 12]}{(note // 12) - 1}"


def black_after_white(white_note: int) -> int | None:
    if (white_note % 12) not in WHITE_WITH_BLACK_AFTER:
        return None
    return white_note + 1


def anchor_pattern(anchor_index: int) -> list[tuple[str, int]]:
    w1 = WHITE_NOTES[anchor_index]
    w2 = WHITE_NOTES[anchor_index + 2]
    w3 = WHITE_NOTES[anchor_index + 4]
    notes_by_finger = {
        "W1": w1,
        "B1": black_after_white(w1),
        "W2": w2,
        "B2": black_after_white(w2),
        "W3": w3,
    }
    pattern: list[tuple[str, int]] = []
    for finger in FINGER_ORDER:
        note = notes_by_finger[finger]
        if note is not None:
            pattern.append((finger, note))
    return pattern


def build_track(patterns: list[list[tuple[str, int]]]) -> bytes:
    track = bytearray()
    track += track_name_event("Robot Hand Pattern Test")
    track += tempo_event(TEMPO_US_PER_BEAT)

    delta_before_note = 0
    for pattern in patterns:
        for index, (_, note) in enumerate(pattern):
            track += note_on(delta_before_note, note)
            track += note_off(NOTE_HOLD_TICKS, note)
            delta_before_note = (
                POSITION_GAP_TICKS if index == len(pattern) - 1 else NOTE_GAP_TICKS
            )

    track += end_of_track()
    return bytes(track)


def build_midi_file(track_data: bytes) -> bytes:
    header = b"MThd" + struct.pack(">IHHH", 6, 0, 1, PPQ)
    track = b"MTrk" + struct.pack(">I", len(track_data)) + track_data
    return header + track


def main() -> None:
    max_anchor = len(WHITE_NOTES) - 5
    patterns = [anchor_pattern(anchor) for anchor in range(max_anchor + 1)]
    midi_file = build_midi_file(build_track(patterns))

    output_path = Path(__file__).with_name(OUTPUT_NAME)
    output_path.write_bytes(midi_file)

    total_notes = sum(len(pattern) for pattern in patterns)
    print(f"Wrote {output_path.name} ({len(midi_file)} bytes)")
    print(f"Hand positions: {len(patterns)}")
    print(f"Total notes: {total_notes}")
    print("Pattern:")
    for anchor, pattern in enumerate(patterns):
        anchor_name = note_name(WHITE_NOTES[anchor])
        note_list = ", ".join(f"{finger}={note_name(note)}" for finger, note in pattern)
        print(f"  {anchor_name}: {note_list}")


if __name__ == "__main__":
    main()
