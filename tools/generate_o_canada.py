#!/usr/bin/env python3
"""Generate a robot-playable MIDI for O Canada.

The melody tokens below are transcribed from the public-domain LilyPond score on
Wikimedia Commons, which in turn cites the Government of Canada's official
sheet music. The robot can currently reach MIDI 36-66, so the melody is
transposed down one octave from the published treble-clef score.
"""

from __future__ import annotations

from pathlib import Path
import re
import struct


PPQ = 480
TEMPO_US_PER_BEAT = 600_000  # 100 BPM
VELOCITY = 96
ARTICULATION_GAP = PPQ // 16
OUTPUT_NAME = "o_canada.mid"
MIN_NOTE = 36
MAX_NOTE = 66
TRANSPOSE_SEMITONES = -12
RELATIVE_START_MIDI = 72  # LilyPond \relative c'' anchor

LILYPOND_MELODY = """
    a2 c4. c8 | f,2. g4 | a bes c d | g,2. r4 |
    a2 b4. b8 | c2. d4 | e e d d | c2. g8. a16 |
    bes4. a8 g4 a8. bes16 | c4. bes8 a4 bes8. c16 |
    d4 c bes a | g2. g8. a16 | bes4. a8 g4 a8. bes16 |
    c4. bes8 a4 a | g c c8 b a b | c2. r4 |
    a2 c4. c8 | f,2. r4 | bes2 d4. d8 | g,2. r4 |
    c2 cis4. cis8 | d4 bes a g | f2 g2 | a2. r4 |
    c2 f4. f8 | d4 bes a g | c2 e, | f2. r4
"""

NOTE_TO_PITCH_CLASS = {
    "c": 0,
    "cis": 1,
    "d": 2,
    "e": 4,
    "f": 5,
    "g": 7,
    "a": 9,
    "b": 11,
    "bes": 10,
}


def vlq(value: int) -> bytes:
    parts = [value & 0x7F]
    value >>= 7
    while value:
        parts.append((value & 0x7F) | 0x80)
        value >>= 7
    parts.reverse()
    return bytes(parts)


def note_on(delta: int, note: int, velocity: int = VELOCITY) -> bytes:
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


def duration_to_ticks(duration_text: str, dotted: bool) -> int:
    ticks = (PPQ * 4) // int(duration_text)
    if dotted:
        ticks += ticks // 2
    return ticks


def closest_relative_pitch(prev_midi: int, pitch_class: int, octave_marks: str) -> int:
    candidates = [pitch_class + 12 * octave for octave in range(0, 11)]
    note_midi = min(candidates, key=lambda midi: abs(midi - prev_midi))
    note_midi += 12 * octave_marks.count("'")
    note_midi -= 12 * octave_marks.count(",")
    return note_midi


def tokenize_lilypond(source: str) -> list[str]:
    cleaned = source.replace("|", " ")
    return [token for token in cleaned.split() if token]


def build_note_events() -> list[tuple[int | None, int]]:
    events: list[tuple[int | None, int]] = []
    token_re = re.compile(r"^(?P<name>cis|bes|[a-gr])(?P<marks>[',]*)(?P<dur>\d+)?(?P<dot>\.)?$")
    prev_duration = "4"
    prev_midi = RELATIVE_START_MIDI

    for token in tokenize_lilypond(LILYPOND_MELODY):
        match = token_re.match(token)
        if not match:
            raise ValueError(f"Unsupported token: {token}")

        name = match.group("name")
        marks = match.group("marks") or ""
        duration_text = match.group("dur") or prev_duration
        dotted = match.group("dot") == "."
        duration_ticks = duration_to_ticks(duration_text, dotted)
        prev_duration = duration_text

        if name == "r":
            events.append((None, duration_ticks))
            continue

        source_midi = closest_relative_pitch(prev_midi, NOTE_TO_PITCH_CLASS[name], marks)
        robot_midi = source_midi + TRANSPOSE_SEMITONES
        if robot_midi < MIN_NOTE or robot_midi > MAX_NOTE:
            raise ValueError(f"Transposed note {robot_midi} is outside robot range")

        events.append((robot_midi, duration_ticks))
        prev_midi = source_midi

    return events


def build_track(events: list[tuple[int | None, int]]) -> bytes:
    track = bytearray()
    track += track_name_event("O Canada")
    track += tempo_event(TEMPO_US_PER_BEAT)

    pending_delta = 0
    for note, duration_ticks in events:
        if note is None:
            pending_delta += duration_ticks
            continue

        if duration_ticks > ARTICULATION_GAP:
            hold_ticks = duration_ticks - ARTICULATION_GAP
            pending_after = ARTICULATION_GAP
        else:
            hold_ticks = duration_ticks
            pending_after = 0

        track += note_on(pending_delta, note)
        track += note_off(hold_ticks, note)
        pending_delta = pending_after

    track += end_of_track()
    return bytes(track)


def build_midi(track_data: bytes) -> bytes:
    header = b"MThd" + struct.pack(">IHHH", 6, 0, 1, PPQ)
    track = b"MTrk" + struct.pack(">I", len(track_data)) + track_data
    return header + track


def main() -> None:
    events = build_note_events()
    output_path = Path(__file__).with_name(OUTPUT_NAME)
    midi_bytes = build_midi(build_track(events))
    output_path.write_bytes(midi_bytes)

    note_values = [note for note, _ in events if note is not None]
    total_notes = len(note_values)
    total_rests = sum(1 for note, _ in events if note is None)

    print(f"Wrote {output_path.name} ({len(midi_bytes)} bytes)")
    print(f"Notes: {total_notes}, rests: {total_rests}")
    print(f"Tempo: {60_000_000 // TEMPO_US_PER_BEAT} BPM")
    print(f"Range: {min(note_values)}-{max(note_values)}")


if __name__ == "__main__":
    main()
