#!/usr/bin/env python3
"""Generate planned-song C/C++ source files from a MIDI file.

The generated output matches the firmware's `PlannedSongStep` shape:
    - `position_mm`
    - `fingers_mask`
    - `press_duration_ms`
    - `travel_duration_ms`

Example:
    py -3 tools/generate_planned_song.py tools/c_major_scale.mid

To replace the current hard-coded Twinkle data in firmware:
    py -3 tools/generate_planned_song.py tools/c_major_scale.mid ^
        --output-dir firmware/piano_robot ^
        --basename planned_song_test_data ^
        --array-name twinkle_song ^
        --source-extension cpp
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import re
import struct
import sys


WHITE_KEY_WIDTH_MM = 23.3125
MOTOR_MIN_MM = 0.0
MOTOR_MAX_MM = 380.0
ROBOT_NOTE_MIN = 36
ROBOT_NOTE_MAX = 71
NOTE_NAMES = ("C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B")
WHITE_OFFSETS = {0: 0, 2: 1, 4: 2, 5: 3, 7: 4, 9: 5, 11: 6}
FINGER_BITS = {"W1": 0, "W2": 1, "W3": 2, "B1": 3, "B2": 4}
MAX_CHORD_NOTES = 5
DEFAULT_TEMPO_US_PER_BEAT = 500_000
DEFAULT_PRESS_RATIO = 0.30
DEFAULT_MIN_PRESS_MS = 60
DEFAULT_MAX_PRESS_MS = 180
DEFAULT_TRAVEL_RESERVE_MS = 80
DEFAULT_CHORD_WINDOW_MS = 30
WIRE_MAGIC = b"PLN1"
WIRE_STEP_STRUCT = struct.Struct(">HBHH")


@dataclass(frozen=True)
class MidiNote:
    start_ms: int
    end_ms: int
    midi_note: int
    velocity: int

    @property
    def duration_ms(self) -> int:
        return max(1, self.end_ms - self.start_ms)


@dataclass(frozen=True)
class RobotChordPlan:
    target_mm: float
    assignments: list[tuple[str, int]]


@dataclass(frozen=True)
class MidiGroup:
    start_ms: int
    midi_notes: tuple[int, ...]
    note_names: tuple[str, ...]
    min_duration_ms: int
    max_end_ms: int


@dataclass(frozen=True)
class PlannedStep:
    position_mm: float
    fingers_mask: int
    press_duration_ms: int
    travel_duration_ms: int
    comment: str


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))


def note_name(midi_note: int) -> str:
    return f"{NOTE_NAMES[midi_note % 12]}{(midi_note // 12) - 1}"


def note_is_black(midi_note: int) -> bool:
    return (midi_note % 12) in {1, 3, 6, 8, 10}


def white_key_index(midi_note: int) -> int:
    note_in_octave = midi_note % 12
    octave = (midi_note // 12) - 3
    return octave * 7 + WHITE_OFFSETS[note_in_octave]


def parse_vlq(data: bytes, pos: int) -> tuple[int, int]:
    value = 0
    while True:
        if pos >= len(data):
            raise ValueError("Unexpected end of file while reading VLQ.")
        byte = data[pos]
        pos += 1
        value = (value << 7) | (byte & 0x7F)
        if not (byte & 0x80):
            return value, pos


def ticks_to_ms(tick: int, tempo_map: list[tuple[int, int]], division: int) -> int:
    elapsed_us = 0
    prev_tick = 0
    current_tempo = tempo_map[0][1]

    for change_tick, tempo_us_per_beat in tempo_map[1:]:
        if change_tick >= tick:
            break
        elapsed_us += (change_tick - prev_tick) * current_tempo // division
        prev_tick = change_tick
        current_tempo = tempo_us_per_beat

    elapsed_us += (tick - prev_tick) * current_tempo // division
    return int(round(elapsed_us / 1000.0))


def load_notes_from_midi(path: Path) -> list[MidiNote]:
    data = path.read_bytes()
    if data[:4] != b"MThd":
        raise ValueError("Not a valid MIDI file.")

    _, midi_format, num_tracks, division = struct.unpack_from(">IHHH", data, 4)
    if division & 0x8000:
        raise ValueError("SMPTE time-division MIDI files are not supported.")
    if midi_format not in (0, 1):
        raise ValueError(f"Unsupported MIDI format: {midi_format}")

    tempo_events = [(0, DEFAULT_TEMPO_US_PER_BEAT)]
    note_spans: list[tuple[int, int, int, int]] = []
    pos = 14

    for _track_index in range(num_tracks):
        if pos + 8 > len(data):
            raise ValueError("Corrupt MIDI file.")
        chunk_id = data[pos:pos + 4]
        chunk_len = struct.unpack_from(">I", data, pos + 4)[0]
        pos += 8
        track_end = pos + chunk_len
        if track_end > len(data):
            raise ValueError("Corrupt MIDI track length.")
        if chunk_id != b"MTrk":
            pos = track_end
            continue

        tick = 0
        running_status: int | None = None
        active_notes: dict[tuple[int, int], list[tuple[int, int]]] = {}

        while pos < track_end:
            delta, pos = parse_vlq(data, pos)
            tick += delta
            if pos >= track_end:
                break

            status = data[pos]
            if status & 0x80:
                running_status = status
                pos += 1
            elif running_status is None:
                raise ValueError("Invalid running status in MIDI track.")

            assert running_status is not None

            if running_status == 0xFF:
                if pos >= track_end:
                    break
                meta_type = data[pos]
                pos += 1
                meta_len, pos = parse_vlq(data, pos)
                meta_data = data[pos:pos + meta_len]
                pos += meta_len
                if meta_type == 0x51 and meta_len == 3:
                    tempo_events.append((tick, int.from_bytes(meta_data, "big")))
                continue

            if running_status in (0xF0, 0xF7):
                sysex_len, pos = parse_vlq(data, pos)
                pos += sysex_len
                continue

            message_type = running_status & 0xF0
            channel = running_status & 0x0F

            if message_type == 0x90:
                note = data[pos]
                velocity = data[pos + 1]
                pos += 2
                key = (channel, note)
                if velocity:
                    active_notes.setdefault(key, []).append((tick, velocity))
                elif active_notes.get(key):
                    start_tick, start_velocity = active_notes[key].pop()
                    note_spans.append((start_tick, tick, note, start_velocity))
            elif message_type == 0x80:
                note = data[pos]
                pos += 2
                key = (channel, note)
                if active_notes.get(key):
                    start_tick, start_velocity = active_notes[key].pop()
                    note_spans.append((start_tick, tick, note, start_velocity))
            elif message_type in (0xA0, 0xB0, 0xE0):
                pos += 2
            elif message_type in (0xC0, 0xD0):
                pos += 1
            else:
                raise ValueError("Unsupported MIDI event encountered.")

        pos = track_end

    tempo_events.sort(key=lambda item: (item[0], item[1]))
    compressed_tempo = [tempo_events[0]]
    for tick, tempo_us in tempo_events[1:]:
        if tick == compressed_tempo[-1][0]:
            compressed_tempo[-1] = (tick, tempo_us)
        elif tempo_us != compressed_tempo[-1][1]:
            compressed_tempo.append((tick, tempo_us))

    notes = []
    for start_tick, end_tick, midi_note, velocity in sorted(
        note_spans, key=lambda item: (item[0], item[2], item[1])
    ):
        start_ms = ticks_to_ms(start_tick, compressed_tempo, division)
        end_ms = ticks_to_ms(end_tick, compressed_tempo, division)
        if end_ms <= start_ms:
            end_ms = start_ms + 1
        notes.append(MidiNote(start_ms, end_ms, midi_note, velocity))

    return notes


def note_options_for_robot(midi_note: int) -> list[tuple[str, float]]:
    if not (ROBOT_NOTE_MIN <= midi_note <= ROBOT_NOTE_MAX):
        return []

    options: list[tuple[str, float]] = []
    if note_is_black(midi_note):
        left_white_index = white_key_index(midi_note - 1)
        candidates = (
            ("B1", left_white_index * WHITE_KEY_WIDTH_MM),
            ("B2", (left_white_index - 2) * WHITE_KEY_WIDTH_MM),
        )
    else:
        white_index = white_key_index(midi_note)
        candidates = (
            ("W1", white_index * WHITE_KEY_WIDTH_MM),
            ("W2", (white_index - 2) * WHITE_KEY_WIDTH_MM),
            ("W3", (white_index - 4) * WHITE_KEY_WIDTH_MM),
        )

    for finger_name, position_mm in candidates:
        if MOTOR_MIN_MM <= position_mm <= MOTOR_MAX_MM:
            options.append((finger_name, position_mm))
    return options


def canonicalize_midi_notes(midi_notes: list[int]) -> list[int]:
    unique_notes = sorted(
        {int(note) for note in midi_notes if ROBOT_NOTE_MIN <= int(note) <= ROBOT_NOTE_MAX}
    )
    if not unique_notes:
        raise ValueError("At least one note must be in range C2-B4.")
    if len(unique_notes) > MAX_CHORD_NOTES:
        raise ValueError(f"At most {MAX_CHORD_NOTES} simultaneous notes are supported.")
    return unique_notes


def plan_robot_chord(midi_notes: list[int], current_pos_mm: float) -> RobotChordPlan | None:
    notes = canonicalize_midi_notes(midi_notes)
    per_note_options = {note: note_options_for_robot(note) for note in notes}
    if any(not options for options in per_note_options.values()):
        return None

    ordered_notes = sorted(notes, key=lambda note: len(per_note_options[note]))
    best_plan: RobotChordPlan | None = None
    best_cost = float("inf")

    def search(
        index: int,
        target_mm: float | None,
        used_fingers: set[str],
        assignments: list[tuple[str, int]],
    ) -> None:
        nonlocal best_plan, best_cost

        if index == len(ordered_notes):
            assert target_mm is not None
            ordered_assignments = sorted(assignments, key=lambda item: item[1])
            cost = abs(target_mm - current_pos_mm)
            if cost < best_cost:
                best_cost = cost
                best_plan = RobotChordPlan(target_mm=target_mm, assignments=ordered_assignments)
            return

        midi_note = ordered_notes[index]
        for finger_name, position_mm in per_note_options[midi_note]:
            if finger_name in used_fingers:
                continue
            if target_mm is not None and abs(position_mm - target_mm) > 0.01:
                continue

            used_fingers.add(finger_name)
            assignments.append((finger_name, midi_note))
            search(
                index + 1,
                position_mm if target_mm is None else target_mm,
                used_fingers,
                assignments,
            )
            assignments.pop()
            used_fingers.remove(finger_name)

    search(0, None, set(), [])
    return best_plan


def group_notes(notes: list[MidiNote], chord_window_ms: int) -> list[MidiGroup]:
    if not notes:
        return []

    ordered = sorted(notes, key=lambda note: (note.start_ms, note.midi_note, note.end_ms))
    groups: list[MidiGroup] = []
    index = 0

    while index < len(ordered):
        group_start = ordered[index].start_ms
        grouped_notes = [ordered[index]]
        index += 1
        while index < len(ordered) and ordered[index].start_ms - group_start <= chord_window_ms:
            grouped_notes.append(ordered[index])
            index += 1

        unique_midi_notes = sorted({note.midi_note for note in grouped_notes})
        groups.append(
            MidiGroup(
                start_ms=group_start,
                midi_notes=tuple(unique_midi_notes),
                note_names=tuple(note_name(note) for note in unique_midi_notes),
                min_duration_ms=min(note.duration_ms for note in grouped_notes),
                max_end_ms=max(note.end_ms for note in grouped_notes),
            )
        )

    return groups


def slugify(text: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9]+", "_", text).strip("_").lower()
    if not slug:
        slug = "planned_song"
    if slug[0].isdigit():
        slug = f"song_{slug}"
    return slug


def build_steps(
    groups: list[MidiGroup],
    *,
    press_ratio: float,
    min_press_ms: int,
    max_press_ms: int,
    travel_reserve_ms: int,
    skip_unplayable: bool,
) -> tuple[list[PlannedStep], list[str]]:
    current_pos_mm = 0.0
    playable_groups: list[tuple[MidiGroup, RobotChordPlan]] = []
    warnings: list[str] = []

    for group in groups:
        try:
            plan = plan_robot_chord(list(group.midi_notes), current_pos_mm)
        except ValueError as exc:
            plan = None
            error_text = str(exc)
        else:
            error_text = "No valid hand geometry for this note/chord."

        if plan is None:
            description = " + ".join(group.note_names)
            warning = (
                f"Unplayable group at {group.start_ms}ms: {description}. {error_text}"
            )
            if skip_unplayable:
                warnings.append(f"Skipped: {warning}")
                continue
            raise ValueError(warning)

        playable_groups.append((group, plan))
        current_pos_mm = plan.target_mm

    steps: list[PlannedStep] = []
    for index, (group, plan) in enumerate(playable_groups):
        if index + 1 < len(playable_groups):
            total_duration_ms = max(
                1, playable_groups[index + 1][0].start_ms - group.start_ms
            )
            last_step = False
        else:
            total_duration_ms = max(1, group.max_end_ms - group.start_ms)
            last_step = True

        finger_mask = 0
        for finger_name, _midi_note in plan.assignments:
            finger_mask |= 1 << FINGER_BITS[finger_name]

        if last_step:
            press_duration_ms = total_duration_ms
            travel_duration_ms = 0
        else:
            press_ceiling_ms = max(1, min(group.min_duration_ms, total_duration_ms))
            press_duration_ms = int(round(total_duration_ms * press_ratio))
            press_duration_ms = int(clamp(press_duration_ms, min_press_ms, max_press_ms))
            press_duration_ms = min(press_duration_ms, press_ceiling_ms)
            if total_duration_ms - press_duration_ms < travel_reserve_ms:
                press_duration_ms = max(1, total_duration_ms - travel_reserve_ms)
                press_duration_ms = min(press_duration_ms, press_ceiling_ms)
            press_duration_ms = max(1, min(press_duration_ms, total_duration_ms))
            travel_duration_ms = total_duration_ms - press_duration_ms

        comment = (
            f"{' + '.join(group.note_names)} @ {group.start_ms}ms "
            f"dur={total_duration_ms}ms"
        )
        steps.append(
            PlannedStep(
                position_mm=plan.target_mm,
                fingers_mask=finger_mask,
                press_duration_ms=press_duration_ms,
                travel_duration_ms=travel_duration_ms,
                comment=comment,
            )
        )

    return steps, warnings


def plan_midi_to_steps(
    midi_path: Path,
    *,
    press_ratio: float = DEFAULT_PRESS_RATIO,
    min_press_ms: int = DEFAULT_MIN_PRESS_MS,
    max_press_ms: int = DEFAULT_MAX_PRESS_MS,
    travel_reserve_ms: int = DEFAULT_TRAVEL_RESERVE_MS,
    chord_window_ms: int = DEFAULT_CHORD_WINDOW_MS,
    skip_unplayable: bool = False,
) -> tuple[list[MidiNote], list[MidiGroup], list[PlannedStep], list[str]]:
    notes = load_notes_from_midi(midi_path)
    if not notes:
        raise ValueError("The MIDI file contained no note events.")

    groups = group_notes(notes, max(0, chord_window_ms))
    steps, warnings = build_steps(
        groups,
        press_ratio=press_ratio,
        min_press_ms=max(1, min_press_ms),
        max_press_ms=max(1, max_press_ms),
        travel_reserve_ms=max(0, travel_reserve_ms),
        skip_unplayable=skip_unplayable,
    )
    if not steps:
        raise ValueError("No playable steps were generated.")

    return notes, groups, steps, warnings


def total_step_duration_ms(steps: list[PlannedStep]) -> int:
    return sum(step.press_duration_ms + step.travel_duration_ms for step in steps)


def render_header(header_name: str, array_name: str) -> str:
    guard = f"{slugify(header_name).upper()}_H"
    return (
        f"#ifndef {guard}\n"
        f"#define {guard}\n\n"
        "#include <stdint.h>\n\n"
        '#include "planned_song_player.h"\n\n'
        "#ifdef __cplusplus\n"
        'extern "C" {\n'
        "#endif\n\n"
        f"extern const PlannedSongStep g_{array_name}[];\n"
        f"extern const uint16_t g_{array_name}_len;\n\n"
        "#ifdef __cplusplus\n"
        "}\n"
        "#endif\n\n"
        f"#endif /* {guard} */\n"
    )


def render_source(
    header_filename: str,
    midi_path: Path,
    array_name: str,
    steps: list[PlannedStep],
) -> str:
    lines = [
        f'#include "{header_filename}"',
        "",
        f"/* Generated from {midi_path.name} by tools/generate_planned_song.py */",
        "",
        f"const PlannedSongStep g_{array_name}[] = {{",
    ]
    for step in steps:
        lines.append(
            "    { "
            f"{step.position_mm:.2f}f, "
            f"0x{step.fingers_mask:02X}, "
            f"{step.press_duration_ms}U, "
            f"{step.travel_duration_ms}U "
            f"}}, /* {step.comment} */"
        )
    lines.extend(
        [
            "};",
            "",
            f"const uint16_t g_{array_name}_len =",
            f"    (uint16_t)(sizeof(g_{array_name}) / sizeof(g_{array_name}[0]));",
            "",
        ]
    )
    return "\n".join(lines)


def write_generated_song_files(
    midi_path: Path,
    *,
    output_dir: Path,
    basename: str,
    array_name: str,
    steps: list[PlannedStep],
    source_extension: str = "cpp",
) -> tuple[Path, Path]:
    basename_slug = slugify(basename)
    array_name_slug = slugify(array_name)
    header_path = output_dir / f"{basename_slug}.h"
    source_path = output_dir / f"{basename_slug}.{source_extension}"

    output_dir.mkdir(parents=True, exist_ok=True)
    header_path.write_text(
        render_header(header_path.stem, array_name_slug),
        encoding="ascii",
    )
    source_path.write_text(
        render_source(header_path.name, midi_path, array_name_slug, steps),
        encoding="ascii",
    )
    return header_path, source_path


def serialize_steps_wire(steps: list[PlannedStep]) -> bytes:
    if not steps:
        raise ValueError("At least one planned step is required.")
    if len(steps) > 0xFFFF:
        raise ValueError("Too many planned steps to send over the wire.")

    payload = bytearray()
    payload += WIRE_MAGIC
    payload += struct.pack(">H", len(steps))
    for step in steps:
        position_x100 = int(round(step.position_mm * 100.0))
        if not (0 <= position_x100 <= 0xFFFF):
            raise ValueError(
                f"Step position {step.position_mm:.2f}mm is out of upload range."
            )
        payload += WIRE_STEP_STRUCT.pack(
            position_x100,
            step.fingers_mask & 0xFF,
            step.press_duration_ms,
            step.travel_duration_ms,
        )
    return bytes(payload)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("midi_file", type=Path, help="Path to the source MIDI file.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("firmware/piano_robot"),
        help="Directory where the generated files should be written.",
    )
    parser.add_argument(
        "--basename",
        default="generated_planned_song",
        help="Base filename for the generated .h/.c or .cpp files.",
    )
    parser.add_argument(
        "--array-name",
        default="planned_song",
        help="C symbol stem. Output defines g_<array-name> and g_<array-name>_len.",
    )
    parser.add_argument(
        "--source-extension",
        choices=("c", "cpp"),
        default="cpp",
        help="Source file extension to emit. Use cpp for the current Arduino build.",
    )
    parser.add_argument(
        "--press-ratio",
        type=float,
        default=DEFAULT_PRESS_RATIO,
        help="Fraction of each step budget allocated to press time before travel.",
    )
    parser.add_argument(
        "--min-press-ms",
        type=int,
        default=DEFAULT_MIN_PRESS_MS,
        help="Minimum generated press duration in milliseconds.",
    )
    parser.add_argument(
        "--max-press-ms",
        type=int,
        default=DEFAULT_MAX_PRESS_MS,
        help="Maximum generated press duration in milliseconds.",
    )
    parser.add_argument(
        "--travel-reserve-ms",
        type=int,
        default=DEFAULT_TRAVEL_RESERVE_MS,
        help="Minimum travel budget to keep for non-final notes.",
    )
    parser.add_argument(
        "--chord-window-ms",
        type=int,
        default=DEFAULT_CHORD_WINDOW_MS,
        help="Group notes starting within this many milliseconds as one chord.",
    )
    parser.add_argument(
        "--skip-unplayable",
        action="store_true",
        help="Skip notes/chords the robot cannot play instead of failing.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    notes, groups, steps, warnings = plan_midi_to_steps(
        args.midi_file,
        press_ratio=args.press_ratio,
        min_press_ms=max(1, args.min_press_ms),
        max_press_ms=max(1, args.max_press_ms),
        travel_reserve_ms=max(0, args.travel_reserve_ms),
        chord_window_ms=max(0, args.chord_window_ms),
        skip_unplayable=args.skip_unplayable,
    )
    header_path, source_path = write_generated_song_files(
        args.midi_file,
        output_dir=args.output_dir,
        basename=args.basename,
        array_name=args.array_name,
        steps=steps,
        source_extension=args.source_extension,
    )

    print(f"Parsed {len(notes)} MIDI notes into {len(groups)} timed groups.")
    print(f"Generated {len(steps)} planned steps.")
    print(f"Wrote {header_path}")
    print(f"Wrote {source_path}")
    for warning in warnings:
        print(f"Warning: {warning}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1)
