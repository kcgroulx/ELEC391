#!/usr/bin/env python3
"""Robot Piano Studio.

Create, preview, import, and export MIDI files for the piano robot.
The editor uses a drag-based piano roll and warns when overlapping notes
cannot fit the robot's fixed finger geometry in the range C2 through B4.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from pathlib import Path
import struct
import tempfile
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import wave

try:
    import winsound
except ImportError:  # pragma: no cover - only unavailable off Windows
    winsound = None


EDITOR_NOTE_MIN = 0
EDITOR_NOTE_MAX = 127
ROBOT_NOTE_MIN = 36
ROBOT_NOTE_MAX = 71
PPQ = 480
DEFAULT_BPM = 120
MIN_BEAT_VALUE = 0.125
DEFAULT_SNAP = 0.25
MAX_CHORD_NOTES = 5
NOTE_NAMES = (
    "C",
    "C#",
    "D",
    "D#",
    "E",
    "F",
    "F#",
    "G",
    "G#",
    "A",
    "A#",
    "B",
)
WHITE_OFFSETS = {0: 0, 2: 1, 4: 2, 5: 3, 7: 4, 9: 5, 11: 6}
SAMPLE_RATE = 22050
BASE_BEAT_WIDTH = 64.0
BASE_ROW_HEIGHT = 18.0
MIN_TIME_ZOOM = 0.5
MAX_TIME_ZOOM = 6.0
MIN_PITCH_ZOOM = 0.75
MAX_PITCH_ZOOM = 4.0

COLORS = {
    "bg": "#f3efe4",
    "panel": "#fffaf2",
    "panel_alt": "#ebe4d4",
    "accent": "#1e608f",
    "accent_dark": "#133955",
    "preview": "#d97b2d",
    "danger": "#b84c39",
    "warning_bg": "#f8e4db",
    "success": "#446548",
    "text": "#1f2f3c",
    "text_muted": "#5f6f7c",
    "timeline_bg": "#10202d",
    "timeline_grid": "#264155",
    "timeline_minor": "#183244",
    "timeline_note": "#5ea7cf",
    "timeline_note_dark": "#2c7aa6",
    "timeline_selected": "#f3d250",
    "timeline_preview": "#f08a24",
    "timeline_cursor": "#7cc6f2",
    "timeline_invalid": "#d16060",
    "timeline_invalid_band": "#5c1f24",
    "timeline_draft": "#7bc2ea",
    "white_key": "#fffdf9",
    "black_key": "#24333d",
    "white_key_border": "#b8c0c7",
    "keyboard_selected": "#5ea7cf",
    "keyboard_preview": "#f08a24",
}

WHITE_KEY_WIDTH_MM = 23.3125
MOTOR_MIN_MM = 0.0
MOTOR_MAX_MM = 380.0


@dataclass
class NoteBlock:
    note_id: int
    midi_note: int
    start_beats: float
    duration_beats: float
    velocity: int = 96

    @property
    def end_beats(self) -> float:
        return round(self.start_beats + self.duration_beats, 6)


@dataclass
class ImportResult:
    notes: list[NoteBlock]
    bpm: int
    outside_robot_range: int
    ignored_tempo_changes: int


@dataclass
class RobotChordPlan:
    target_mm: float
    assignments: list[tuple[str, int]]


@dataclass
class ConflictWindow:
    start_beats: float
    end_beats: float
    midi_notes: list[int]
    note_ids: tuple[int, ...]
    duplicate_pitch: bool = False


@dataclass
class DragState:
    mode: str
    note_id: int | None
    anchor_beat: float
    press_beat: float
    press_midi: int
    original_note: NoteBlock | None
    moved: bool = False
    selected_note_ids: tuple[int, ...] = ()


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


def note_frequency(midi_note: int) -> float:
    return 440.0 * (2.0 ** ((midi_note - 69) / 12.0))


def canonicalize_midi_notes(midi_notes: list[int]) -> list[int]:
    unique_notes = sorted(
        {int(note) for note in midi_notes if ROBOT_NOTE_MIN <= int(note) <= ROBOT_NOTE_MAX}
    )
    if not unique_notes:
        raise ValueError("At least one note must be in range C2-B4.")
    if len(unique_notes) > MAX_CHORD_NOTES:
        raise ValueError(f"At most {MAX_CHORD_NOTES} simultaneous notes are supported.")
    return unique_notes


def normalize_beat_value(value: float, minimum: float = 0.0) -> float:
    return round(max(minimum, value), 3)


def sanitize_note(note: NoteBlock) -> NoteBlock:
    return NoteBlock(
        note_id=note.note_id,
        midi_note=int(clamp(int(note.midi_note), EDITOR_NOTE_MIN, EDITOR_NOTE_MAX)),
        start_beats=normalize_beat_value(note.start_beats, 0.0),
        duration_beats=normalize_beat_value(note.duration_beats, MIN_BEAT_VALUE),
        velocity=int(clamp(int(note.velocity), 30, 127)),
    )


def sort_notes(notes: list[NoteBlock]) -> list[NoteBlock]:
    return sorted(
        (sanitize_note(note) for note in notes),
        key=lambda note: (note.start_beats, note.midi_note, note.end_beats, note.note_id),
    )


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


def plan_robot_chord(
    midi_notes: list[int], current_pos_mm: float = 0.0
) -> RobotChordPlan | None:
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
                best_plan = RobotChordPlan(
                    target_mm=target_mm, assignments=ordered_assignments
                )
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


def describe_note_options(midi_note: int) -> str:
    options = note_options_for_robot(midi_note)
    if not options:
        return "This pitch is outside the robot hand geometry."
    return ", ".join(f"{finger}@{position_mm:.1f} mm" for finger, position_mm in options)


def describe_conflict(window: ConflictWindow) -> str:
    note_text = " + ".join(note_name(note) for note in window.midi_notes)
    if window.duplicate_pitch:
        return (
            f"{window.start_beats:.2f}-{window.end_beats:.2f} beats: "
            f"{note_text} (duplicate pitch overlap)"
        )
    return f"{window.start_beats:.2f}-{window.end_beats:.2f} beats: {note_text}"


def analyze_robot_conflicts(
    notes: list[NoteBlock],
) -> tuple[set[int], list[ConflictWindow]]:
    ordered_notes = sort_notes(notes)
    if len(ordered_notes) < 2:
        return set(), []

    time_points = sorted(
        {round(note.start_beats, 6) for note in ordered_notes}
        | {round(note.end_beats, 6) for note in ordered_notes}
    )

    invalid_ids: set[int] = set()
    windows: list[ConflictWindow] = []

    for start, end in zip(time_points, time_points[1:]):
        if end - start <= 1e-6:
            continue

        active = [
            note
            for note in ordered_notes
            if note.start_beats < end - 1e-6 and note.end_beats > start + 1e-6
        ]
        if len(active) < 2:
            continue

        active_pitches = sorted(note.midi_note for note in active)
        unique_pitches = sorted(set(active_pitches))
        duplicate_pitch = len(active_pitches) != len(unique_pitches)
        invalid = duplicate_pitch or len(unique_pitches) > MAX_CHORD_NOTES
        if not invalid:
            invalid = plan_robot_chord(unique_pitches) is None
        if not invalid:
            continue

        note_ids = tuple(sorted(note.note_id for note in active))
        invalid_ids.update(note_ids)
        if (
            windows
            and abs(windows[-1].end_beats - start) < 1e-6
            and windows[-1].midi_notes == unique_pitches
            and windows[-1].note_ids == note_ids
            and windows[-1].duplicate_pitch == duplicate_pitch
        ):
            windows[-1].end_beats = end
        else:
            windows.append(
                ConflictWindow(start, end, unique_pitches, note_ids, duplicate_pitch)
            )

    return invalid_ids, windows


def encode_vlq(value: int) -> bytes:
    buffer = [value & 0x7F]
    value >>= 7
    while value:
        buffer.append(0x80 | (value & 0x7F))
        value >>= 7
    buffer.reverse()
    return bytes(buffer)


def parse_vlq(data: bytes, pos: int) -> tuple[int, int]:
    value = 0
    for _ in range(4):
        if pos >= len(data):
            raise ValueError("Unexpected end of file while parsing VLQ")
        byte = data[pos]
        pos += 1
        value = (value << 7) | (byte & 0x7F)
        if not (byte & 0x80):
            return value, pos
    return value, pos


def beats_to_ticks(beats: float) -> int:
    return max(0, int(round(beats * PPQ)))


def notes_to_midi_bytes(notes: list[NoteBlock], bpm: int) -> bytes:
    ordered_notes = sort_notes(notes)
    if not ordered_notes:
        raise ValueError("Cannot export an empty song.")
    if bpm <= 0:
        raise ValueError("BPM must be positive.")

    microseconds_per_beat = int(round(60_000_000 / bpm))
    track = bytearray()
    track += encode_vlq(0)
    track += bytes([0xFF, 0x51, 0x03])
    track += microseconds_per_beat.to_bytes(3, "big")

    events: list[tuple[int, int, int, int, int]] = []
    for note in ordered_notes:
        start_tick = beats_to_ticks(note.start_beats)
        end_tick = start_tick + max(1, beats_to_ticks(note.duration_beats))
        velocity = int(clamp(note.velocity, 1, 127))
        events.append((start_tick, 1, note.midi_note, velocity, 0x90))
        events.append((end_tick, 0, note.midi_note, 0, 0x80))

    events.sort(key=lambda item: (item[0], item[1], item[2], item[4]))
    previous_tick = 0
    for tick, _priority, midi_note, velocity, status in events:
        track += encode_vlq(tick - previous_tick)
        track += bytes([status, midi_note, velocity])
        previous_tick = tick

    track += encode_vlq(0)
    track += bytes([0xFF, 0x2F, 0x00])

    header = b"MThd" + struct.pack(">IHHH", 6, 0, 1, PPQ)
    track_chunk = b"MTrk" + struct.pack(">I", len(track)) + bytes(track)
    return header + track_chunk


def save_notes_as_midi(path: Path, notes: list[NoteBlock], bpm: int) -> None:
    path.write_bytes(notes_to_midi_bytes(notes, bpm))


def load_notes_from_midi(path: Path) -> ImportResult:
    data = path.read_bytes()
    if data[:4] != b"MThd":
        raise ValueError("Not a valid MIDI file.")

    _, midi_format, num_tracks, division = struct.unpack_from(">IHHH", data, 4)
    if division & 0x8000:
        raise ValueError("SMPTE time-division MIDI files are not supported.")
    if midi_format not in (0, 1):
        raise ValueError(f"Unsupported MIDI format: {midi_format}")

    tempo_events = [(0, 500_000)]
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

    bpm = max(1, int(round(60_000_000 / compressed_tempo[0][1])))
    outside_robot_range = 0
    notes: list[NoteBlock] = []

    for start_tick, end_tick, midi_note, velocity in sorted(
        note_spans, key=lambda item: (item[0], item[2], item[1])
    ):
        if not (ROBOT_NOTE_MIN <= midi_note <= ROBOT_NOTE_MAX):
            outside_robot_range += 1
        duration_ticks = max(1, end_tick - start_tick)
        notes.append(
            NoteBlock(
                note_id=0,
                midi_note=midi_note,
                start_beats=round(start_tick / division, 3),
                duration_beats=round(max(MIN_BEAT_VALUE, duration_ticks / division), 3),
                velocity=int(clamp(velocity, 30, 127)),
            )
        )

    return ImportResult(
        notes=notes,
        bpm=bpm,
        outside_robot_range=outside_robot_range,
        ignored_tempo_changes=max(0, len(compressed_tempo) - 1),
    )


def build_preview_schedule(
    notes: list[NoteBlock], bpm: int
) -> tuple[list[tuple[int, float, float]], float]:
    beat_seconds = 60.0 / bpm
    schedule = [
        (
            note.note_id,
            note.start_beats * beat_seconds,
            note.end_beats * beat_seconds,
        )
        for note in sort_notes(notes)
    ]
    total_seconds = max((end for _note_id, _start, end in schedule), default=0.0)
    return schedule, total_seconds


def _render_tone_samples(
    midi_note: int, duration_seconds: float, velocity: int, sample_rate: int
) -> list[int]:
    total_samples = max(1, int(round(duration_seconds * sample_rate)))
    frequency = note_frequency(midi_note)
    velocity_gain = clamp(velocity / 127.0, 0.15, 1.0)
    attack = max(1, int(sample_rate * 0.01))
    release = max(1, int(sample_rate * 0.04))

    samples: list[int] = []
    for index in range(total_samples):
        t = index / sample_rate
        envelope = 1.0
        if index < attack:
            envelope = index / attack
        elif index > total_samples - release:
            envelope = max(0.0, (total_samples - index) / release)

        harmonic = (
            math.sin(2.0 * math.pi * frequency * t)
            + 0.35 * math.sin(2.0 * math.pi * frequency * 2.0 * t)
            + 0.12 * math.sin(2.0 * math.pi * frequency * 3.0 * t)
        )
        decay = math.exp(-3.0 * t)
        sample = int(
            clamp(harmonic * decay * envelope * velocity_gain * 11500, -32767, 32767)
        )
        samples.append(sample)

    return samples


def render_preview_wav(path: Path, notes: list[NoteBlock], bpm: int) -> float:
    ordered_notes = sort_notes(notes)
    if not ordered_notes:
        raise ValueError("Cannot preview an empty song.")

    beat_seconds = 60.0 / bpm
    total_seconds = max(note.end_beats for note in ordered_notes) * beat_seconds
    total_samples = max(1, int(round((total_seconds + 0.05) * SAMPLE_RATE)))
    mix = [0.0] * total_samples

    for note in ordered_notes:
        duration_seconds = max(MIN_BEAT_VALUE, note.duration_beats) * beat_seconds
        tone_samples = _render_tone_samples(
            note.midi_note, duration_seconds, note.velocity, SAMPLE_RATE
        )
        start_sample = int(round(note.start_beats * beat_seconds * SAMPLE_RATE))
        for index, sample in enumerate(tone_samples):
            mix_index = start_sample + index
            if mix_index >= total_samples:
                break
            mix[mix_index] += sample

    frames = bytearray()
    for sample in mix:
        frames += struct.pack("<h", int(clamp(sample, -32767, 32767)))

    with wave.open(str(path), "wb") as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(SAMPLE_RATE)
        wav_file.writeframes(frames)

    return total_seconds


def build_preview_notes_from_cursor(
    notes: list[NoteBlock], start_beat: float
) -> list[NoteBlock]:
    clipped_notes: list[NoteBlock] = []
    for note in sort_notes(notes):
        if note.end_beats <= start_beat:
            continue
        clipped_start = max(0.0, note.start_beats - start_beat)
        clipped_duration = max(0.01, note.end_beats - max(note.start_beats, start_beat))
        clipped_notes.append(
            NoteBlock(
                note_id=note.note_id,
                midi_note=note.midi_note,
                start_beats=round(clipped_start, 3),
                duration_beats=round(clipped_duration, 3),
                velocity=note.velocity,
            )
        )
    return clipped_notes


class RobotPianoStudio:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Robot Piano Studio")
        self.root.geometry("1520x920")
        self.root.minsize(1260, 780)
        self.root.configure(bg=COLORS["bg"])

        self.notes: list[NoteBlock] = []
        self.next_note_id = 1
        self.selected_note_id: int | None = None
        self.selected_note_ids: set[int] = set()
        self.current_path: Path | None = None
        self.dirty = False
        self.note_regions: list[tuple[float, float, float, float, int]] = []
        self.keyboard_regions: list[tuple[int, float, float, float, float, bool]] = []
        self.invalid_note_ids: set[int] = set()
        self.invalid_windows: list[ConflictWindow] = []
        self.out_of_range_note_ids: set[int] = set()

        self.drag_state: DragState | None = None
        self.drag_preview: NoteBlock | None = None
        self.cursor_drag_active = False
        self.time_zoom = 1.0
        self.pitch_zoom = 1.0
        self.playback_cursor_beats = 0.0

        self.preview_after_id: str | None = None
        self.preview_token = 0
        self.preview_schedule: list[tuple[int, float, float]] = []
        self.preview_total_seconds = 0.0
        self.preview_start_time = 0.0
        self.preview_active_ids: set[int] = set()
        self.preview_origin_beats = 0.0
        self.preview_path = Path(tempfile.gettempdir()) / "robot_piano_preview.wav"

        self.bpm_var = tk.IntVar(value=DEFAULT_BPM)
        self.default_duration_var = tk.DoubleVar(value=1.0)
        self.snap_var = tk.DoubleVar(value=DEFAULT_SNAP)
        self.default_velocity_var = tk.IntVar(value=96)

        self.pitch_var = tk.StringVar(value="C4")
        self.start_var = tk.DoubleVar(value=0.0)
        self.duration_var = tk.DoubleVar(value=1.0)
        self.velocity_var = tk.IntVar(value=96)
        self.selected_note_var = tk.StringVar(value="No note selected")
        self.robot_plan_var = tk.StringVar(
            value="Select a note or draw one on the piano roll."
        )
        self.warning_var = tk.StringVar(
            value="All overlapping notes currently fit the robot hand."
        )
        self.time_zoom_var = tk.StringVar(value="Time 100%")
        self.pitch_zoom_var = tk.StringVar(value="Pitch 100%")
        self.file_var = tk.StringVar(value="Unsaved song")
        self.song_info_var = tk.StringVar(value="0 notes | 0.0 s")
        self.status_var = tk.StringVar(
            value=(
                "Drag on the piano roll to draw notes. Drag a note to move it, or drag "
                "its right edge to resize it."
            )
        )

        self._build_style()
        self._build_ui()
        self._bind_shortcuts()
        self._update_zoom_labels()
        self._refresh_views()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # BUILD METHODS
    def _build_style(self) -> None:
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(
            "Studio.Treeview",
            background=COLORS["panel"],
            fieldbackground=COLORS["panel"],
            foreground=COLORS["text"],
            rowheight=26,
            borderwidth=0,
        )
        style.map(
            "Studio.Treeview",
            background=[("selected", COLORS["accent"])],
            foreground=[("selected", "#ffffff")],
        )
        style.configure(
            "Studio.Treeview.Heading",
            background=COLORS["panel_alt"],
            foreground=COLORS["text"],
            relief="flat",
        )
        style.configure("Studio.TCombobox", fieldbackground="#ffffff")

    def _build_ui(self) -> None:
        toolbar = tk.Frame(self.root, bg=COLORS["accent_dark"], padx=18, pady=14)
        toolbar.pack(fill="x")

        title_wrap = tk.Frame(toolbar, bg=COLORS["accent_dark"])
        title_wrap.pack(side="left")
        tk.Label(
            title_wrap,
            text="ROBOT PIANO STUDIO",
            bg=COLORS["accent_dark"],
            fg="#ffffff",
            font=("Segoe UI", 18, "bold"),
        ).pack(anchor="w")
        tk.Label(
            title_wrap,
            text="Drag notes directly onto the piano roll and validate them against the robot hand.",
            bg=COLORS["accent_dark"],
            fg="#d5e1ea",
            font=("Segoe UI", 9),
        ).pack(anchor="w")

        action_wrap = tk.Frame(toolbar, bg=COLORS["accent_dark"])
        action_wrap.pack(side="right")

        self._make_toolbar_button(action_wrap, "New", self._new_song).pack(
            side="left", padx=4
        )
        self._make_toolbar_button(action_wrap, "Open MIDI", self._open_midi).pack(
            side="left", padx=4
        )
        self._make_toolbar_button(action_wrap, "Save MIDI", self._save_midi_as).pack(
            side="left", padx=4
        )
        self._make_toolbar_button(
            action_wrap, "Insert Note", self._insert_note_from_picker
        ).pack(side="left", padx=(20, 4))
        self.play_cursor_button = self._make_toolbar_button(
            action_wrap, "Play From Cursor", self._play_from_cursor, bg=COLORS["preview"]
        )
        self.play_cursor_button.pack(side="left", padx=4)
        self.play_start_button = self._make_toolbar_button(
            action_wrap, "Play From Start", self._play_from_start
        )
        self.play_start_button.pack(side="left", padx=4)
        self.stop_button = self._make_toolbar_button(
            action_wrap, "Stop", self._stop_preview, bg=COLORS["danger"], fg="#ffffff"
        )
        self.stop_button.pack(side="left", padx=4)

        info_bar = tk.Frame(self.root, bg=COLORS["bg"], padx=18, pady=10)
        info_bar.pack(fill="x")
        tk.Label(
            info_bar,
            textvariable=self.file_var,
            bg=COLORS["bg"],
            fg=COLORS["text"],
            font=("Segoe UI", 10, "bold"),
        ).pack(side="left")
        tk.Label(
            info_bar,
            textvariable=self.song_info_var,
            bg=COLORS["bg"],
            fg=COLORS["text_muted"],
            font=("Segoe UI", 10),
        ).pack(side="right")

        body = tk.Frame(self.root, bg=COLORS["bg"], padx=18, pady=6)
        body.pack(fill="both", expand=True)
        body.grid_columnconfigure(0, weight=0, minsize=320)
        body.grid_columnconfigure(1, weight=1)
        body.grid_columnconfigure(2, weight=0, minsize=440)
        body.grid_rowconfigure(0, weight=1)

        left = tk.Frame(body, bg=COLORS["bg"])
        left.grid(row=0, column=0, sticky="nsw", padx=(0, 12))

        center = tk.Frame(body, bg=COLORS["bg"])
        center.grid(row=0, column=1, sticky="nsew", padx=(0, 12))
        center.grid_rowconfigure(0, weight=1)
        center.grid_rowconfigure(1, weight=0)
        center.grid_columnconfigure(0, weight=1)

        right = tk.Frame(body, bg=COLORS["bg"])
        right.grid(row=0, column=2, sticky="nse")

        self._build_song_settings(left)
        self._build_selected_note_panel(left)
        self._build_timeline_panel(center)
        self._build_keyboard_panel(center)
        self._build_sequence_panel(right)

        status_bar = tk.Frame(self.root, bg=COLORS["panel_alt"], padx=18, pady=8)
        status_bar.pack(fill="x")
        tk.Label(
            status_bar,
            textvariable=self.status_var,
            bg=COLORS["panel_alt"],
            fg=COLORS["text"],
            font=("Segoe UI", 9),
            anchor="w",
        ).pack(fill="x")

    def _build_song_settings(self, parent: tk.Frame) -> None:
        card = self._make_card(parent, "Song Settings")
        card.pack(fill="x", pady=(0, 12))

        self._labeled_spinbox(card, "Tempo (BPM)", self.bpm_var, 30, 300, 1).pack(
            fill="x", pady=4
        )
        self._labeled_spinbox(
            card,
            "Default Note Length (beats)",
            self.default_duration_var,
            MIN_BEAT_VALUE,
            16.0,
            MIN_BEAT_VALUE,
        ).pack(fill="x", pady=4)
        self._labeled_spinbox(
            card,
            "Grid Snap (beats)",
            self.snap_var,
            MIN_BEAT_VALUE,
            4.0,
            MIN_BEAT_VALUE,
        ).pack(fill="x", pady=4)

        velocity_wrap = tk.Frame(card, bg=COLORS["panel"])
        velocity_wrap.pack(fill="x", pady=(8, 2))
        tk.Label(
            velocity_wrap,
            text="Default Velocity",
            bg=COLORS["panel"],
            fg=COLORS["text"],
            font=("Segoe UI", 9),
        ).pack(anchor="w")
        self.default_velocity_scale = tk.Scale(
            velocity_wrap,
            from_=30,
            to=127,
            resolution=1,
            orient="horizontal",
            variable=self.default_velocity_var,
            bg=COLORS["panel"],
            fg=COLORS["text"],
            troughcolor=COLORS["panel_alt"],
            highlightthickness=0,
            activebackground=COLORS["accent"],
        )
        self.default_velocity_scale.pack(fill="x")

        octave_row = tk.Frame(card, bg=COLORS["panel"])
        octave_row.pack(fill="x", pady=(10, 0))
        self._make_panel_button(
            octave_row, "All Notes -12", lambda: self._transpose_all_notes(-12)
        ).pack(side="left", fill="x", expand=True, padx=(0, 4))
        self._make_panel_button(
            octave_row, "All Notes +12", lambda: self._transpose_all_notes(12)
        ).pack(side="left", fill="x", expand=True, padx=(4, 0))

        tips = tk.Label(
            card,
            text=(
                "Robot rules:\n"
                "Range is C2 to B4.\n"
                "You can still place notes outside that range; they will show in red.\n"
                "The hand keeps one fixed finger pattern.\n"
                "White fingers land on W1/W2/W3, black fingers on B1/B2.\n"
                "Red notes or bands mean the robot cannot play them.\n"
                "Ctrl-click notes or tree rows to multi-select, then drag to move them together."
            ),
            justify="left",
            bg=COLORS["panel"],
            fg=COLORS["text_muted"],
            font=("Segoe UI", 9),
        )
        tips.pack(fill="x", pady=(10, 0))

        warning_wrap = tk.Frame(card, bg=COLORS["warning_bg"], padx=10, pady=8)
        warning_wrap.pack(fill="x", pady=(12, 0))
        tk.Label(
            warning_wrap,
            text="Robot Overlap Check",
            bg=COLORS["warning_bg"],
            fg=COLORS["text"],
            font=("Segoe UI", 8, "bold"),
        ).pack(anchor="w")
        self.warning_label = tk.Label(
            warning_wrap,
            textvariable=self.warning_var,
            justify="left",
            wraplength=270,
            bg=COLORS["warning_bg"],
            fg=COLORS["success"],
            font=("Segoe UI", 9),
        )
        self.warning_label.pack(anchor="w", fill="x")

    def _build_selected_note_panel(self, parent: tk.Frame) -> None:
        card = self._make_card(parent, "Selected Note")
        card.pack(fill="x")

        row = tk.Frame(card, bg=COLORS["panel"])
        row.pack(fill="x", pady=4)
        tk.Label(
            row,
            text="Pitch",
            bg=COLORS["panel"],
            fg=COLORS["text"],
            font=("Segoe UI", 9),
            width=16,
            anchor="w",
        ).pack(side="left")
        self.pitch_combo = ttk.Combobox(
            row,
            textvariable=self.pitch_var,
            values=[note_name(note) for note in range(EDITOR_NOTE_MIN, EDITOR_NOTE_MAX + 1)],
            state="readonly",
            style="Studio.TCombobox",
        )
        self.pitch_combo.pack(side="left", fill="x", expand=True)

        tk.Label(
            card,
            text="Summary",
            bg=COLORS["panel"],
            fg=COLORS["text_muted"],
            font=("Segoe UI", 8, "bold"),
        ).pack(anchor="w", pady=(8, 0))
        tk.Label(
            card,
            textvariable=self.selected_note_var,
            bg=COLORS["panel"],
            fg=COLORS["text"],
            justify="left",
            wraplength=260,
            font=("Segoe UI", 9),
        ).pack(anchor="w", fill="x")

        tk.Label(
            card,
            text="Robot Fit",
            bg=COLORS["panel"],
            fg=COLORS["text_muted"],
            font=("Segoe UI", 8, "bold"),
        ).pack(anchor="w", pady=(8, 0))
        tk.Label(
            card,
            textvariable=self.robot_plan_var,
            bg=COLORS["panel"],
            fg=COLORS["text"],
            justify="left",
            wraplength=260,
            font=("Segoe UI", 9),
        ).pack(anchor="w", fill="x")

        self._labeled_spinbox(
            card, "Start Beat", self.start_var, 0.0, 512.0, MIN_BEAT_VALUE
        ).pack(fill="x", pady=4)
        self._labeled_spinbox(
            card,
            "Length (beats)",
            self.duration_var,
            MIN_BEAT_VALUE,
            64.0,
            MIN_BEAT_VALUE,
        ).pack(fill="x", pady=4)

        velocity_wrap = tk.Frame(card, bg=COLORS["panel"])
        velocity_wrap.pack(fill="x", pady=(8, 2))
        tk.Label(
            velocity_wrap,
            text="Velocity",
            bg=COLORS["panel"],
            fg=COLORS["text"],
            font=("Segoe UI", 9),
        ).pack(anchor="w")
        self.velocity_scale = tk.Scale(
            velocity_wrap,
            from_=30,
            to=127,
            resolution=1,
            orient="horizontal",
            variable=self.velocity_var,
            bg=COLORS["panel"],
            fg=COLORS["text"],
            troughcolor=COLORS["panel_alt"],
            highlightthickness=0,
            activebackground=COLORS["accent"],
        )
        self.velocity_scale.pack(fill="x")

        button_row = tk.Frame(card, bg=COLORS["panel"])
        button_row.pack(fill="x", pady=(10, 0))
        self.insert_note_button = self._make_panel_button(
            button_row, "Insert From Picker", self._insert_note_from_picker
        )
        self.insert_note_button.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.apply_button = self._make_panel_button(
            button_row, "Apply", self._apply_selected_edits
        )
        self.apply_button.pack(side="left", fill="x", expand=True, padx=4)
        self.duplicate_button = self._make_panel_button(
            button_row, "Duplicate", self._duplicate_selected
        )
        self.duplicate_button.pack(side="left", fill="x", expand=True, padx=(4, 0))

        lower_row = tk.Frame(card, bg=COLORS["panel"])
        lower_row.pack(fill="x", pady=(8, 0))
        self.delete_button = self._make_panel_button(
            lower_row, "Delete", self._delete_selected, bg=COLORS["danger"], fg="#ffffff"
        )
        self.delete_button.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.snap_button = self._make_panel_button(
            lower_row, "Snap To Grid", self._snap_selected_to_grid
        )
        self.snap_button.pack(side="left", fill="x", expand=True, padx=(4, 0))

    def _build_timeline_panel(self, parent: tk.Frame) -> None:
        card = self._make_card(parent, "Piano Roll")
        card.grid(row=0, column=0, sticky="nsew")

        tk.Label(
            card,
            text=(
                "Drag on empty space to place a note. Drag a note body to move it. "
                "Drag its right edge to resize it. Right-click a note to delete it. "
                "Use the top ruler to place the playback cursor. Mouse wheel scrolls "
                "vertically, Shift scrolls horizontally, Ctrl zooms."
            ),
            bg=COLORS["panel"],
            fg=COLORS["text_muted"],
            font=("Segoe UI", 9),
        ).pack(anchor="w", pady=(0, 10))

        zoom_row = tk.Frame(card, bg=COLORS["panel"])
        zoom_row.pack(fill="x", pady=(0, 10))
        tk.Label(
            zoom_row,
            textvariable=self.time_zoom_var,
            bg=COLORS["panel"],
            fg=COLORS["text"],
            font=("Segoe UI", 9, "bold"),
        ).pack(side="left")
        self._make_panel_button(
            zoom_row, "Time -", lambda: self._change_time_zoom(-1)
        ).pack(side="left", padx=(8, 4))
        self._make_panel_button(
            zoom_row, "Time +", lambda: self._change_time_zoom(1)
        ).pack(side="left", padx=4)
        tk.Label(
            zoom_row,
            textvariable=self.pitch_zoom_var,
            bg=COLORS["panel"],
            fg=COLORS["text"],
            font=("Segoe UI", 9, "bold"),
        ).pack(side="left", padx=(16, 0))
        self._make_panel_button(
            zoom_row, "Pitch -", lambda: self._change_pitch_zoom(-1)
        ).pack(side="left", padx=(8, 4))
        self._make_panel_button(
            zoom_row, "Pitch +", lambda: self._change_pitch_zoom(1)
        ).pack(side="left", padx=4)
        self._make_panel_button(
            zoom_row, "Reset View", self._reset_timeline_view
        ).pack(side="right")

        canvas_frame = tk.Frame(card, bg=COLORS["panel"])
        canvas_frame.pack(fill="both", expand=True)
        canvas_frame.grid_rowconfigure(0, weight=1)
        canvas_frame.grid_columnconfigure(0, weight=1)

        self.timeline_v_scroll = ttk.Scrollbar(
            canvas_frame, orient="vertical"
        )
        self.timeline_h_scroll = ttk.Scrollbar(
            canvas_frame, orient="horizontal"
        )
        self.timeline_canvas = tk.Canvas(
            canvas_frame,
            bg=COLORS["timeline_bg"],
            highlightthickness=0,
            relief="flat",
            xscrollcommand=self.timeline_h_scroll.set,
            yscrollcommand=self.timeline_v_scroll.set,
        )
        self.timeline_canvas.grid(row=0, column=0, sticky="nsew")
        self.timeline_v_scroll.configure(command=self._timeline_yview)
        self.timeline_v_scroll.grid(row=0, column=1, sticky="ns")
        self.timeline_h_scroll.configure(command=self._timeline_xview)
        self.timeline_h_scroll.grid(row=1, column=0, sticky="ew")
        self.timeline_canvas.bind("<Configure>", lambda _event: self._draw_timeline())
        self.timeline_canvas.bind("<ButtonPress-1>", self._on_timeline_press)
        self.timeline_canvas.bind("<B1-Motion>", self._on_timeline_drag)
        self.timeline_canvas.bind("<ButtonRelease-1>", self._on_timeline_release)
        self.timeline_canvas.bind("<Button-3>", self._on_timeline_right_click)
        self.timeline_canvas.bind("<MouseWheel>", self._on_timeline_mousewheel)

    def _build_keyboard_panel(self, parent: tk.Frame) -> None:
        card = self._make_card(parent, "Pitch Reference")
        card.grid(row=1, column=0, sticky="ew", pady=(12, 0))

        tk.Label(
            card,
            text="Click a key to load that pitch into the picker. Preview playback also highlights active notes.",
            bg=COLORS["panel"],
            fg=COLORS["text_muted"],
            font=("Segoe UI", 9),
        ).pack(anchor="w", pady=(0, 10))

        self.keyboard_canvas = tk.Canvas(
            card,
            height=180,
            bg=COLORS["panel_alt"],
            highlightthickness=0,
            relief="flat",
        )
        self.keyboard_canvas.pack(fill="x")
        self.keyboard_canvas.bind("<Configure>", lambda _event: self._draw_keyboard())
        self.keyboard_canvas.bind("<Button-1>", self._on_keyboard_click)

    def _build_sequence_panel(self, parent: tk.Frame) -> None:
        card = self._make_card(parent, "Notes")
        card.pack(fill="both", expand=True)

        tk.Label(
            card,
            text="Each row is one note block with independent start time and duration.",
            bg=COLORS["panel"],
            fg=COLORS["text_muted"],
            font=("Segoe UI", 9),
        ).pack(anchor="w", pady=(0, 10))

        tree_frame = tk.Frame(card, bg=COLORS["panel"])
        tree_frame.pack(fill="both", expand=True)
        tree_frame.grid_rowconfigure(0, weight=1)
        tree_frame.grid_columnconfigure(0, weight=1)

        columns = ("index", "start", "pitch", "duration", "velocity", "state")
        self.sequence_tree = ttk.Treeview(
            tree_frame,
            columns=columns,
            show="headings",
            style="Studio.Treeview",
            selectmode="extended",
        )
        self.sequence_tree.heading("index", text="#")
        self.sequence_tree.heading("start", text="Start")
        self.sequence_tree.heading("pitch", text="Pitch")
        self.sequence_tree.heading("duration", text="Dur")
        self.sequence_tree.heading("velocity", text="Vel")
        self.sequence_tree.heading("state", text="State")

        self.sequence_tree.column("index", width=48, anchor="center", stretch=False)
        self.sequence_tree.column("start", width=84, anchor="center", stretch=False)
        self.sequence_tree.column("pitch", width=88, anchor="center", stretch=False)
        self.sequence_tree.column("duration", width=78, anchor="center", stretch=False)
        self.sequence_tree.column("velocity", width=70, anchor="center", stretch=False)
        self.sequence_tree.column("state", width=88, anchor="center", stretch=False)
        self.sequence_tree.tag_configure("invalid", foreground=COLORS["danger"])

        scroll = ttk.Scrollbar(
            tree_frame, orient="vertical", command=self.sequence_tree.yview
        )
        self.sequence_tree.configure(yscrollcommand=scroll.set)
        self.sequence_tree.grid(row=0, column=0, sticky="nsew")
        scroll.grid(row=0, column=1, sticky="ns")
        self.sequence_tree.bind("<<TreeviewSelect>>", self._on_tree_select)

    def _make_card(self, parent: tk.Widget, title: str) -> tk.Frame:
        card = tk.Frame(parent, bg=COLORS["panel"], padx=14, pady=14, bd=0)
        tk.Label(
            card,
            text=title,
            bg=COLORS["panel"],
            fg=COLORS["text"],
            font=("Segoe UI", 12, "bold"),
        ).pack(anchor="w", pady=(0, 10))
        return card

    def _make_toolbar_button(
        self,
        parent: tk.Widget,
        text: str,
        command,
        bg: str = "#ffffff",
        fg: str = COLORS["text"],
    ) -> tk.Button:
        return tk.Button(
            parent,
            text=text,
            command=command,
            bg=bg,
            fg=fg,
            activebackground=bg,
            activeforeground=fg,
            relief="flat",
            bd=0,
            padx=12,
            pady=8,
            font=("Segoe UI", 9, "bold"),
            cursor="hand2",
        )

    def _make_panel_button(
        self,
        parent: tk.Widget,
        text: str,
        command,
        bg: str = COLORS["panel_alt"],
        fg: str = COLORS["text"],
    ) -> tk.Button:
        return tk.Button(
            parent,
            text=text,
            command=command,
            bg=bg,
            fg=fg,
            activebackground=bg,
            activeforeground=fg,
            relief="flat",
            bd=0,
            padx=10,
            pady=8,
            font=("Segoe UI", 9, "bold"),
            cursor="hand2",
        )

    def _labeled_spinbox(
        self,
        parent: tk.Widget,
        label: str,
        variable,
        minimum: float,
        maximum: float,
        increment: float,
    ) -> tk.Frame:
        wrap = tk.Frame(parent, bg=COLORS["panel"])
        tk.Label(
            wrap,
            text=label,
            bg=COLORS["panel"],
            fg=COLORS["text"],
            font=("Segoe UI", 9),
            width=24,
            anchor="w",
        ).pack(side="left")
        spinbox_kwargs = {}
        if any(
            abs(value - round(value)) > 1e-9
            for value in (minimum, maximum, increment)
        ):
            spinbox_kwargs["format"] = "%.3f"
        spin = tk.Spinbox(
            wrap,
            textvariable=variable,
            from_=minimum,
            to=maximum,
            increment=increment,
            bg="#ffffff",
            fg=COLORS["text"],
            relief="solid",
            bd=1,
            width=8,
            justify="center",
            font=("Segoe UI", 9),
            **spinbox_kwargs,
        )
        spin.pack(side="right")
        return wrap

    def _bind_shortcuts(self) -> None:
        self.root.bind("<Control-o>", lambda _event: self._open_midi())
        self.root.bind("<Control-s>", lambda _event: self._save_midi_as())
        self.root.bind("<Delete>", lambda _event: self._delete_selected())
        self.root.bind("<Control-Up>", lambda _event: self._transpose_all_notes(12))
        self.root.bind("<Control-Down>", lambda _event: self._transpose_all_notes(-12))

    # STATE METHODS
    def _coerce_bpm(self) -> int:
        try:
            bpm = int(self.bpm_var.get())
        except (tk.TclError, ValueError):
            bpm = DEFAULT_BPM
        bpm = int(clamp(bpm, 30, 300))
        self.bpm_var.set(bpm)
        return bpm

    def _coerce_float_var(
        self, variable, minimum: float, maximum: float, default: float
    ) -> float:
        try:
            value = float(variable.get())
        except (tk.TclError, ValueError):
            value = default
        value = round(clamp(value, minimum, maximum), 3)
        variable.set(value)
        return value

    def _coerce_int_var(
        self, variable, minimum: int, maximum: int, default: int
    ) -> int:
        try:
            value = int(variable.get())
        except (tk.TclError, ValueError):
            value = default
        value = int(clamp(value, minimum, maximum))
        variable.set(value)
        return value

    def _grid_snap(self) -> float:
        return self._coerce_float_var(
            self.snap_var, MIN_BEAT_VALUE, 4.0, DEFAULT_SNAP
        )

    def _snap_beat(self, beat: float) -> float:
        snap = self._grid_snap()
        return round(round(max(0.0, beat) / snap) * snap, 3)

    def _update_zoom_labels(self) -> None:
        self.time_zoom_var.set(f"Time {int(round(self.time_zoom * 100))}%")
        self.pitch_zoom_var.set(f"Pitch {int(round(self.pitch_zoom * 100))}%")

    def _scroll_timeline_to_focus(
        self,
        focus_x: float,
        focus_y: float,
        beat_focus: float,
        pitch_focus: float,
        metrics: dict[str, float],
    ) -> None:
        canvas = self.timeline_canvas
        max_left = max(0.0, metrics["content_width"] - metrics["width"])
        max_top = max(0.0, metrics["content_height"] - metrics["height"])
        new_canvas_x = metrics["left_margin"] + beat_focus * metrics["beat_width"]
        new_canvas_y = metrics["top_margin"] + pitch_focus * metrics["row_height"]

        if max_left > 0:
            canvas.xview_moveto(clamp((new_canvas_x - focus_x) / max_left, 0.0, 1.0))
        else:
            canvas.xview_moveto(0.0)
        if max_top > 0:
            canvas.yview_moveto(clamp((new_canvas_y - focus_y) / max_top, 0.0, 1.0))
        else:
            canvas.yview_moveto(0.0)
        self._draw_timeline()

    def _set_timeline_zoom(
        self,
        *,
        time_zoom: float | None = None,
        pitch_zoom: float | None = None,
        focus_x: float | None = None,
        focus_y: float | None = None,
    ) -> None:
        canvas = self.timeline_canvas
        old_metrics = self._timeline_metrics()
        focus_x = canvas.winfo_width() / 2 if focus_x is None else focus_x
        focus_y = canvas.winfo_height() / 2 if focus_y is None else focus_y
        old_canvas_x = canvas.canvasx(focus_x)
        old_canvas_y = canvas.canvasy(focus_y)
        beat_focus = max(
            0.0, (old_canvas_x - old_metrics["left_margin"]) / old_metrics["beat_width"]
        )
        pitch_focus = max(
            0.0, (old_canvas_y - old_metrics["top_margin"]) / old_metrics["row_height"]
        )

        if time_zoom is not None:
            self.time_zoom = clamp(time_zoom, MIN_TIME_ZOOM, MAX_TIME_ZOOM)
        if pitch_zoom is not None:
            self.pitch_zoom = clamp(pitch_zoom, MIN_PITCH_ZOOM, MAX_PITCH_ZOOM)
        self._update_zoom_labels()
        self._draw_timeline()
        self.timeline_canvas.update_idletasks()
        self._scroll_timeline_to_focus(
            focus_x, focus_y, beat_focus, pitch_focus, self._timeline_metrics()
        )

    def _change_time_zoom(self, direction: int, focus_x: float | None = None) -> None:
        factor = 1.25 if direction > 0 else 0.8
        self._set_timeline_zoom(
            time_zoom=self.time_zoom * factor,
            focus_x=focus_x,
        )

    def _change_pitch_zoom(self, direction: int, focus_y: float | None = None) -> None:
        factor = 1.2 if direction > 0 else (1 / 1.2)
        self._set_timeline_zoom(
            pitch_zoom=self.pitch_zoom * factor,
            focus_y=focus_y,
        )

    def _reset_timeline_view(self) -> None:
        self.time_zoom = 1.0
        self.pitch_zoom = 1.0
        self._update_zoom_labels()
        self._draw_timeline()
        self.timeline_canvas.xview_moveto(0.0)
        self.timeline_canvas.yview_moveto(0.0)
        self._draw_timeline()

    def _timeline_xview(self, *args) -> None:
        self.timeline_canvas.xview(*args)
        self._draw_timeline()

    def _timeline_yview(self, *args) -> None:
        self.timeline_canvas.yview(*args)
        self._draw_timeline()

    def _mark_dirty(self) -> None:
        self.dirty = True
        self._update_title()

    def _mark_clean(self) -> None:
        self.dirty = False
        self._update_title()

    def _update_title(self) -> None:
        name = self.current_path.name if self.current_path else "Unsaved song"
        suffix = " *" if self.dirty else ""
        self.root.title(f"Robot Piano Studio - {name}{suffix}")
        self.file_var.set(name + (" (unsaved edits)" if self.dirty else ""))

    def _song_end_beats(self) -> float:
        return max((note.end_beats for note in self.notes), default=0.0)

    def _song_duration_seconds(self) -> float:
        if not self.notes:
            return 0.0
        return self._song_end_beats() * (60.0 / self._coerce_bpm())

    def _set_playback_cursor(self, beat: float, announce: bool = False) -> None:
        self.playback_cursor_beats = round(
            clamp(beat, 0.0, self._song_end_beats()),
            3,
        )
        if announce:
            self.status_var.set(
                f"Playback cursor set to beat {self.playback_cursor_beats:.2f}."
            )
        self._draw_timeline()

    def _assign_note_id(self, note: NoteBlock) -> NoteBlock:
        note = sanitize_note(note)
        note.note_id = self.next_note_id
        self.next_note_id += 1
        return note

    def _selected_notes(self) -> list[NoteBlock]:
        selected = [note for note in self.notes if note.note_id in self.selected_note_ids]
        if not selected and self.selected_note_id is not None:
            note = self._note_by_id(self.selected_note_id)
            if note is not None:
                selected = [note]
        return selected

    def _set_selection(
        self, note_ids: set[int], primary_note_id: int | None = None, refresh: bool = False
    ) -> None:
        valid_ids = {note.note_id for note in self.notes}
        self.selected_note_ids = {note_id for note_id in note_ids if note_id in valid_ids}
        if primary_note_id is not None and primary_note_id in self.selected_note_ids:
            self.selected_note_id = primary_note_id
        elif self.selected_note_ids:
            self.selected_note_id = next(iter(sorted(self.selected_note_ids)))
        else:
            self.selected_note_id = None
        if refresh:
            self._refresh_views()

    def _note_by_id(self, note_id: int | None) -> NoteBlock | None:
        if note_id is None:
            return None
        for note in self.notes:
            if note.note_id == note_id:
                return note
        return None

    def _replace_note(self, updated_note: NoteBlock) -> None:
        for index, note in enumerate(self.notes):
            if note.note_id == updated_note.note_id:
                self.notes[index] = sanitize_note(updated_note)
                return

    def _remove_note(self, note_id: int) -> NoteBlock | None:
        for index, note in enumerate(self.notes):
            if note.note_id == note_id:
                return self.notes.pop(index)
        return None

    def _transpose_note_ids(self, note_ids: set[int], semitones: int) -> int:
        moved = 0
        for index, note in enumerate(self.notes):
            if note.note_id not in note_ids:
                continue
            self.notes[index] = sanitize_note(
                NoteBlock(
                    note_id=note.note_id,
                    midi_note=note.midi_note + semitones,
                    start_beats=note.start_beats,
                    duration_beats=note.duration_beats,
                    velocity=note.velocity,
                )
            )
            moved += 1
        return moved

    def _transpose_all_notes(self, semitones: int) -> None:
        if not self.notes:
            return
        moved = self._transpose_note_ids({note.note_id for note in self.notes}, semitones)
        if moved <= 0:
            return
        self._mark_dirty()
        direction = "up" if semitones > 0 else "down"
        self.status_var.set(f"Moved all {moved} notes {abs(semitones)} semitones {direction}.")
        self._refresh_views()

    def _notes_equal(self, first: NoteBlock, second: NoteBlock) -> bool:
        return sanitize_note(first) == sanitize_note(second)

    def _refresh_views(self) -> None:
        self.notes = sort_notes(self.notes)
        self._set_selection(self.selected_note_ids, self.selected_note_id, refresh=False)
        self.playback_cursor_beats = round(
            clamp(self.playback_cursor_beats, 0.0, self._song_end_beats()),
            3,
        )
        self.invalid_note_ids, self.invalid_windows = analyze_robot_conflicts(self.notes)
        self.out_of_range_note_ids = {
            note.note_id for note in self.notes if not note_options_for_robot(note.midi_note)
        }
        self._rebuild_tree()
        self._sync_editor_from_selection()
        self._update_warning_banner()
        self._update_song_info()
        self._update_button_states()
        self._draw_timeline()
        self._draw_keyboard()
        self._update_title()

    def _warning_summary(self, limit: int = 3) -> str:
        if not self.invalid_windows and not self.out_of_range_note_ids:
            return "All notes and overlaps currently fit the robot hand."
        lines: list[str] = []
        if self.out_of_range_note_ids:
            lines.append(
                f"{len(self.out_of_range_note_ids)} note(s) are outside the robot range C2-B4."
            )
        if self.invalid_windows:
            lines.append(
                f"{len(self.invalid_windows)} impossible overlap region(s) detected."
            )
        for window in self.invalid_windows[:limit]:
            lines.append(describe_conflict(window))
        remaining = len(self.invalid_windows) - limit
        if remaining > 0:
            lines.append(f"...and {remaining} more.")
        return "\n".join(lines)

    def _update_warning_banner(self) -> None:
        self.warning_var.set(self._warning_summary())
        self.warning_label.configure(
            fg=(
                COLORS["danger"]
                if self.invalid_windows or self.out_of_range_note_ids
                else COLORS["success"]
            )
        )

    def _update_song_info(self) -> None:
        total_seconds = self._song_duration_seconds()
        issue_count = len(self.invalid_windows) + len(self.out_of_range_note_ids)
        invalid_suffix = f" | {issue_count} flagged" if issue_count else ""
        self.song_info_var.set(
            f"{len(self.notes)} notes{invalid_suffix} | {total_seconds:.1f} s"
        )

    def _update_button_states(self) -> None:
        selection_count = len(self._selected_notes())
        has_selection = selection_count > 0
        single_selection = selection_count == 1
        self.apply_button.configure(state="normal" if single_selection else "disabled")
        self.duplicate_button.configure(state="normal" if single_selection else "disabled")
        for button in (self.delete_button, self.snap_button):
            button.configure(state="normal" if has_selection else "disabled")
        self.insert_note_button.configure(state="normal")

        preview_state = (
            "normal"
            if winsound is not None and self.notes and self.preview_after_id is None
            else "disabled"
        )
        self.play_cursor_button.configure(state=preview_state)
        self.play_start_button.configure(state=preview_state)
        self.stop_button.configure(
            state="normal" if self.preview_after_id is not None else "disabled"
        )

    def _rebuild_tree(self) -> None:
        selected_items = [str(note_id) for note_id in sorted(self.selected_note_ids)]
        for item in self.sequence_tree.get_children():
            self.sequence_tree.delete(item)

        for index, note in enumerate(self.notes):
            is_out_of_range = note.note_id in self.out_of_range_note_ids
            is_invalid = note.note_id in self.invalid_note_ids
            tags = ("invalid",) if (is_invalid or is_out_of_range) else ()
            state_text = ""
            if is_out_of_range and is_invalid:
                state_text = "range+warn"
            elif is_out_of_range:
                state_text = "range"
            elif is_invalid:
                state_text = "warn"
            self.sequence_tree.insert(
                "",
                "end",
                iid=str(note.note_id),
                values=(
                    index + 1,
                    f"{note.start_beats:.2f}",
                    note_name(note.midi_note),
                    f"{note.duration_beats:.2f}",
                    note.velocity,
                    state_text,
                ),
                tags=tags,
            )

        existing_items = [item for item in selected_items if self.sequence_tree.exists(item)]
        if existing_items:
            self.sequence_tree.selection_set(existing_items)
            focus_item = (
                str(self.selected_note_id)
                if self.selected_note_id is not None and self.sequence_tree.exists(str(self.selected_note_id))
                else existing_items[0]
            )
            self.sequence_tree.focus(focus_item)
            self.sequence_tree.see(focus_item)

    def _selected_note_conflicts(self, note_id: int) -> list[ConflictWindow]:
        return [window for window in self.invalid_windows if note_id in window.note_ids]

    def _sync_editor_from_selection(self) -> None:
        selected_notes = self._selected_notes()
        if len(selected_notes) > 1:
            primary_note = self._note_by_id(self.selected_note_id) or selected_notes[0]
            self.pitch_var.set(note_name(primary_note.midi_note))
            self.start_var.set(primary_note.start_beats)
            self.duration_var.set(primary_note.duration_beats)
            self.velocity_var.set(primary_note.velocity)
            self.selected_note_var.set(
                f"{len(selected_notes)} notes selected | primary {note_name(primary_note.midi_note)}"
            )
            out_of_range_count = sum(
                1 for note in selected_notes if note.note_id in self.out_of_range_note_ids
            )
            overlap_count = sum(
                1 for note in selected_notes if note.note_id in self.invalid_note_ids
            )
            plan_lines = ["Drag any selected note to move the full selection together."]
            if out_of_range_count:
                plan_lines.append(f"{out_of_range_count} selected note(s) are outside C2-B4.")
            if overlap_count:
                plan_lines.append(f"{overlap_count} selected note(s) are in impossible overlaps.")
            self.robot_plan_var.set("\n".join(plan_lines))
            return

        note = self._note_by_id(self.selected_note_id)
        if note is None:
            self.pitch_var.set("C4")
            self.start_var.set(0.0)
            self.duration_var.set(
                self._coerce_float_var(
                    self.default_duration_var, MIN_BEAT_VALUE, 16.0, 1.0
                )
            )
            self.velocity_var.set(
                self._coerce_int_var(self.default_velocity_var, 30, 127, 96)
            )
            self.selected_note_var.set("No note selected")
            self.robot_plan_var.set(
                "Pick a note from the keyboard or drag one onto the piano roll."
            )
            return

        self.pitch_var.set(note_name(note.midi_note))
        self.start_var.set(note.start_beats)
        self.duration_var.set(note.duration_beats)
        self.velocity_var.set(note.velocity)
        self.selected_note_var.set(
            f"{note_name(note.midi_note)} | start {note.start_beats:.2f} | "
            f"length {note.duration_beats:.2f} | velocity {note.velocity}"
        )

        conflicts = self._selected_note_conflicts(note.note_id)
        if conflicts:
            lines = ["This note is inside an impossible overlap:"]
            for window in conflicts[:3]:
                lines.append(describe_conflict(window))
            if len(conflicts) > 3:
                lines.append(f"...and {len(conflicts) - 3} more.")
            self.robot_plan_var.set("\n".join(lines))
        else:
            if note.note_id in self.out_of_range_note_ids:
                self.robot_plan_var.set("This note is outside the robot range C2-B4.")
            else:
                self.robot_plan_var.set(
                    f"Playable positions: {describe_note_options(note.midi_note)}"
                )

    def _select_note_id(self, note_id: int | None) -> None:
        self._set_selection(set() if note_id is None else {note_id}, note_id, refresh=True)

    def _on_tree_select(self, _event=None) -> None:
        selection = tuple(int(item) for item in self.sequence_tree.selection())
        primary_note_id = int(self.sequence_tree.focus()) if self.sequence_tree.focus() else None
        self._set_selection(set(selection), primary_note_id, refresh=False)
        self._sync_editor_from_selection()
        self._update_button_states()
        self._draw_timeline()
        self._draw_keyboard()

    def _note_name_to_midi(self, label: str) -> int:
        for midi_note in range(EDITOR_NOTE_MIN, EDITOR_NOTE_MAX + 1):
            if note_name(midi_note) == label:
                return midi_note
        raise ValueError(f"Unknown note name: {label}")

    def _apply_selected_edits(self) -> None:
        note = self._note_by_id(self.selected_note_id)
        if note is None:
            return

        updated = sanitize_note(
            NoteBlock(
                note_id=note.note_id,
                midi_note=self._note_name_to_midi(self.pitch_var.get()),
                start_beats=self._coerce_float_var(self.start_var, 0.0, 512.0, 0.0),
                duration_beats=self._coerce_float_var(
                    self.duration_var, MIN_BEAT_VALUE, 64.0, 1.0
                ),
                velocity=self._coerce_int_var(self.velocity_var, 30, 127, 96),
            )
        )
        self._replace_note(updated)
        self._mark_dirty()
        self.status_var.set(f"Updated {note_name(updated.midi_note)}.")
        self._refresh_views()

    def _duplicate_selected(self) -> None:
        note = self._note_by_id(self.selected_note_id)
        if note is None:
            return
        duplicate = self._assign_note_id(
            NoteBlock(
                note_id=0,
                midi_note=note.midi_note,
                start_beats=self._snap_beat(note.end_beats),
                duration_beats=note.duration_beats,
                velocity=note.velocity,
            )
        )
        self.notes.append(duplicate)
        self._set_selection({duplicate.note_id}, duplicate.note_id, refresh=False)
        self._mark_dirty()
        self.status_var.set(f"Duplicated {note_name(note.midi_note)}.")
        self._refresh_views()

    def _delete_selected(self) -> None:
        selected_notes = self._selected_notes()
        if not selected_notes:
            return
        for note in selected_notes:
            self._remove_note(note.note_id)
        deleted_count = len(selected_notes)
        self._set_selection(set(), None, refresh=False)
        self._mark_dirty()
        self.status_var.set(f"Deleted {deleted_count} selected note(s).")
        self._refresh_views()

    def _snap_selected_to_grid(self) -> None:
        selected_notes = self._selected_notes()
        if not selected_notes:
            return
        for note in selected_notes:
            snapped = sanitize_note(
                NoteBlock(
                    note_id=note.note_id,
                    midi_note=note.midi_note,
                    start_beats=self._snap_beat(note.start_beats),
                    duration_beats=max(self._grid_snap(), self._snap_beat(note.duration_beats)),
                    velocity=note.velocity,
                )
            )
            self._replace_note(snapped)
        self._mark_dirty()
        self.status_var.set(f"Snapped {len(selected_notes)} selected note(s) to the grid.")
        self._refresh_views()

    def _insert_note_from_picker(self) -> None:
        start_beat = self._song_end_beats()
        selected = self._note_by_id(self.selected_note_id)
        if selected is not None:
            start_beat = selected.end_beats
        note = self._assign_note_id(
            NoteBlock(
                note_id=0,
                midi_note=self._note_name_to_midi(self.pitch_var.get()),
                start_beats=self._snap_beat(start_beat),
                duration_beats=self._coerce_float_var(
                    self.default_duration_var, MIN_BEAT_VALUE, 16.0, 1.0
                ),
                velocity=self._coerce_int_var(self.default_velocity_var, 30, 127, 96),
            )
        )
        self.notes.append(note)
        self._set_selection({note.note_id}, note.note_id, refresh=False)
        self._mark_dirty()
        self.status_var.set(
            f"Inserted {note_name(note.midi_note)} at beat {note.start_beats:.2f}."
        )
        self._refresh_views()

    # FILE/PREVIEW METHODS
    def _maybe_discard_changes(self) -> bool:
        if not self.dirty:
            return True
        return messagebox.askyesno(
            "Unsaved Changes",
            "Discard unsaved changes to the current song?",
        )

    def _new_song(self) -> None:
        if not self._maybe_discard_changes():
            return
        self._stop_preview()
        self.notes.clear()
        self.next_note_id = 1
        self.selected_note_id = None
        self.selected_note_ids.clear()
        self.current_path = None
        self._mark_clean()
        self.status_var.set("Started a new empty song.")
        self._refresh_views()

    def _open_midi(self) -> None:
        if not self._maybe_discard_changes():
            return
        path_str = filedialog.askopenfilename(
            title="Open MIDI File",
            filetypes=[("MIDI files", "*.mid *.midi"), ("All files", "*.*")],
        )
        if not path_str:
            return

        try:
            result = load_notes_from_midi(Path(path_str))
        except Exception as exc:
            messagebox.showerror("Import Error", str(exc))
            return

        self._stop_preview()
        self.next_note_id = 1
        self.notes = [self._assign_note_id(note) for note in result.notes]
        self._set_selection(
            {self.notes[0].note_id} if self.notes else set(),
            self.notes[0].note_id if self.notes else None,
            refresh=False,
        )
        self.current_path = Path(path_str)
        self.bpm_var.set(result.bpm)
        self._mark_clean()
        self._refresh_views()

        warnings: list[str] = []
        if result.outside_robot_range:
            warnings.append(
                f"Imported {result.outside_robot_range} note(s) outside C2-B4; they are shown in red."
            )
        if result.ignored_tempo_changes:
            warnings.append(
                f"Ignored {result.ignored_tempo_changes} tempo change event(s); the first tempo was used."
            )
        if self.invalid_windows:
            warnings.append(
                f"Detected {len(self.invalid_windows)} overlap region(s) that do not fit the robot hand."
            )
        if warnings:
            messagebox.showinfo("Import Notes", "\n".join(warnings))

        self.status_var.set(
            f"Loaded {len(self.notes)} note(s) from {self.current_path.name} at {result.bpm} BPM."
        )

    def _warn_about_impossible_overlaps(self, action: str) -> bool:
        if not self.invalid_windows and not self.out_of_range_note_ids:
            return True
        warning_lines: list[str] = []
        if self.out_of_range_note_ids:
            warning_lines.append(
                f"This song has {len(self.out_of_range_note_ids)} note(s) outside the robot range C2-B4."
            )
        if self.invalid_windows:
            warning_lines.append(
                f"This song has {len(self.invalid_windows)} overlap region(s) that the robot cannot play with one hand pattern."
            )
        return messagebox.askyesno(
            "Robot Playability Warning",
            (
                f"{' '.join(warning_lines)}\n\n{self._warning_summary(3)}\n\n{action} anyway?"
            ),
        )

    def _save_midi_as(self) -> None:
        if not self.notes:
            messagebox.showinfo("No Notes", "Add at least one note before saving.")
            return
        if not self._warn_about_impossible_overlaps("Save the MIDI"):
            return

        initial_name = self.current_path.name if self.current_path else "robot_song.mid"
        path_str = filedialog.asksaveasfilename(
            title="Save MIDI File",
            defaultextension=".mid",
            initialfile=initial_name,
            filetypes=[("MIDI files", "*.mid"), ("All files", "*.*")],
        )
        if not path_str:
            return

        try:
            save_notes_as_midi(Path(path_str), self.notes, self._coerce_bpm())
        except Exception as exc:
            messagebox.showerror("Save Error", str(exc))
            return

        self.current_path = Path(path_str)
        self._mark_clean()
        self.status_var.set(f"Saved MIDI to {self.current_path.name}.")
        self._refresh_views()

    def _play_from_start(self) -> None:
        self._play_preview_from(0.0, "start")

    def _play_from_cursor(self) -> None:
        self._play_preview_from(self.playback_cursor_beats, "cursor")

    def _play_preview_from(self, start_beat: float, origin_label: str) -> None:
        if winsound is None:
            messagebox.showinfo(
                "Playback Unavailable",
                "Local preview playback currently requires Windows winsound.",
            )
            return
        if not self.notes:
            messagebox.showinfo("No Notes", "Add at least one note before previewing.")
            return
        if not self._warn_about_impossible_overlaps("Preview the song"):
            return

        self._stop_preview()
        bpm = self._coerce_bpm()
        notes_snapshot = build_preview_notes_from_cursor(self.notes, start_beat)
        if not notes_snapshot:
            messagebox.showinfo(
                "Nothing To Play",
                "There are no notes at or after the selected playback start point.",
            )
            return

        self.preview_token += 1
        token = self.preview_token
        self.play_cursor_button.configure(state="disabled")
        self.play_start_button.configure(state="disabled")
        self.preview_origin_beats = round(start_beat, 3)
        if origin_label == "cursor":
            self.status_var.set(
                f"Rendering preview audio from beat {self.preview_origin_beats:.2f}..."
            )
        else:
            self.status_var.set("Rendering preview audio from the beginning...")

        thread = threading.Thread(
            target=self._render_preview_worker,
            args=(token, notes_snapshot, bpm),
            daemon=True,
        )
        thread.start()

    def _render_preview_worker(
        self, token: int, notes_snapshot: list[NoteBlock], bpm: int
    ) -> None:
        error_message: str | None = None
        total_seconds = 0.0
        schedule: list[tuple[int, float, float]] = []
        try:
            total_seconds = render_preview_wav(self.preview_path, notes_snapshot, bpm)
            schedule, _ = build_preview_schedule(notes_snapshot, bpm)
        except Exception as exc:
            error_message = str(exc)

        self.root.after(
            0,
            lambda: self._finish_preview_render(
                token, error_message, total_seconds, schedule
            ),
        )

    def _finish_preview_render(
        self,
        token: int,
        error_message: str | None,
        total_seconds: float,
        schedule: list[tuple[int, float, float]],
    ) -> None:
        if token != self.preview_token:
            return

        if error_message:
            self.status_var.set(f"Preview render failed: {error_message}")
            self._update_button_states()
            messagebox.showerror("Preview Error", error_message)
            return

        self.preview_schedule = schedule
        self.preview_total_seconds = total_seconds
        self.preview_start_time = time.perf_counter()
        self.preview_active_ids.clear()
        winsound.PlaySound(
            str(self.preview_path), winsound.SND_FILENAME | winsound.SND_ASYNC
        )
        self.preview_after_id = self.root.after(20, self._poll_preview)
        if self.preview_origin_beats > 0:
            self.status_var.set(
                f"Playing preview from beat {self.preview_origin_beats:.2f}..."
            )
        else:
            self.status_var.set("Playing preview from the beginning...")
        self._update_button_states()

    def _poll_preview(self) -> None:
        self.preview_after_id = None
        elapsed = time.perf_counter() - self.preview_start_time
        active_ids = {
            note_id
            for note_id, start, end in self.preview_schedule
            if start <= elapsed <= end
        }

        if active_ids != self.preview_active_ids:
            self.preview_active_ids = active_ids
            self._draw_timeline()
            self._draw_keyboard()

        if elapsed <= self.preview_total_seconds + 0.1:
            self.preview_after_id = self.root.after(20, self._poll_preview)
            self._update_button_states()
            return

        self._stop_preview()
        self.status_var.set("Preview complete.")

    def _stop_preview(self) -> None:
        if winsound is not None:
            winsound.PlaySound(None, 0)
        if self.preview_after_id is not None:
            self.root.after_cancel(self.preview_after_id)
            self.preview_after_id = None
        self.preview_active_ids.clear()
        self.preview_schedule = []
        self.preview_total_seconds = 0.0
        self.preview_origin_beats = 0.0
        self._draw_timeline()
        self._draw_keyboard()
        self._update_button_states()

    # DRAW/INTERACTION METHODS
    def _timeline_metrics(self) -> dict[str, float]:
        canvas = self.timeline_canvas
        width = max(200, canvas.winfo_width())
        height = max(240, canvas.winfo_height())
        left_margin = 72
        right_margin = 22
        top_margin = 28
        bottom_margin = 24
        row_height = BASE_ROW_HEIGHT * self.pitch_zoom
        beat_width = BASE_BEAT_WIDTH * self.time_zoom
        content_end = self._song_end_beats()
        if self.drag_preview is not None:
            content_end = max(content_end, self.drag_preview.end_beats)
        total_beats = max(8.0, math.ceil(content_end + 4.0))
        content_width = max(width, left_margin + total_beats * beat_width + right_margin)
        content_height = max(
            height,
            top_margin
            + (EDITOR_NOTE_MAX - EDITOR_NOTE_MIN + 1) * row_height
            + bottom_margin,
        )
        return {
            "width": width,
            "height": height,
            "content_width": content_width,
            "content_height": content_height,
            "left_margin": left_margin,
            "right_margin": right_margin,
            "top_margin": top_margin,
            "bottom_margin": bottom_margin,
            "row_height": row_height,
            "beat_width": beat_width,
            "total_beats": total_beats,
        }

    def _beat_to_x(self, beat: float, metrics: dict[str, float]) -> float:
        return metrics["left_margin"] + beat * metrics["beat_width"]

    def _x_to_beat(self, x: float, metrics: dict[str, float]) -> float | None:
        if (
            x < metrics["left_margin"]
            or x > metrics["content_width"] - metrics["right_margin"]
        ):
            return None
        plot_x = clamp(
            x - metrics["left_margin"],
            0.0,
            metrics["total_beats"] * metrics["beat_width"],
        )
        return round(plot_x / metrics["beat_width"], 3)

    def _midi_to_y(self, midi_note: int, metrics: dict[str, float]) -> tuple[float, float]:
        y1 = (
            metrics["top_margin"]
            + (EDITOR_NOTE_MAX - midi_note) * metrics["row_height"]
            + metrics["row_height"] * 0.12
        )
        y2 = y1 + metrics["row_height"] * 0.76
        return y1, y2

    def _y_to_midi(self, y: float, metrics: dict[str, float]) -> int | None:
        if (
            y < metrics["top_margin"]
            or y > metrics["content_height"] - metrics["bottom_margin"]
        ):
            return None
        row = int((y - metrics["top_margin"]) / metrics["row_height"])
        midi_note = EDITOR_NOTE_MAX - row
        return int(clamp(midi_note, EDITOR_NOTE_MIN, EDITOR_NOTE_MAX))

    def _note_region_at_point(
        self, x: float, y: float
    ) -> tuple[float, float, float, float, int] | None:
        for region in reversed(self.note_regions):
            x1, y1, x2, y2, note_id = region
            if x1 <= x <= x2 and y1 <= y <= y2:
                return region
        return None

    def _point_hits_resize_handle(
        self, x: float, region: tuple[float, float, float, float, int]
    ) -> bool:
        x1, _y1, x2, _y2, _note_id = region
        handle_width = min(12.0, max(6.0, (x2 - x1) * 0.35))
        return x >= x2 - handle_width

    def _selected_pitch_set(self) -> set[int]:
        if self.drag_preview is not None:
            if self.drag_state is not None and self.drag_state.mode == "move":
                preview_notes = self._drag_preview_notes()
                if preview_notes:
                    return {note.midi_note for note in preview_notes}
            return {self.drag_preview.midi_note}
        return {note.midi_note for note in self._selected_notes()}

    def _drag_preview_notes(self) -> list[NoteBlock]:
        if (
            self.drag_state is None
            or self.drag_preview is None
            or self.drag_state.mode != "move"
            or self.drag_state.original_note is None
        ):
            return []

        beat_delta = (
            self.drag_preview.start_beats - self.drag_state.original_note.start_beats
        )
        pitch_delta = self.drag_preview.midi_note - self.drag_state.original_note.midi_note
        preview_notes: list[NoteBlock] = []
        for note_id in self.drag_state.selected_note_ids:
            note = self._note_by_id(note_id)
            if note is None:
                continue
            preview_notes.append(
                sanitize_note(
                    NoteBlock(
                        note_id=note.note_id,
                        midi_note=note.midi_note + pitch_delta,
                        start_beats=note.start_beats + beat_delta,
                        duration_beats=note.duration_beats,
                        velocity=note.velocity,
                    )
                )
            )
        return preview_notes

    def _preview_pitch_set(self) -> set[int]:
        return {
            note.midi_note
            for note in self.notes
            if note.note_id in self.preview_active_ids
        }

    def _draw_timeline(self) -> None:
        canvas = self.timeline_canvas
        canvas.delete("all")
        self.note_regions.clear()

        metrics = self._timeline_metrics()
        width = metrics["content_width"]
        height = metrics["content_height"]
        visible_left = canvas.canvasx(0)
        visible_top = canvas.canvasy(0)
        canvas.configure(scrollregion=(0, 0, width, height))

        for midi_note in range(EDITOR_NOTE_MIN, EDITOR_NOTE_MAX + 1):
            y1, y2 = self._midi_to_y(midi_note, metrics)
            row_fill = COLORS["timeline_bg"] if not note_is_black(midi_note) else "#142838"
            canvas.create_rectangle(
                metrics["left_margin"],
                y1 - metrics["row_height"] * 0.12,
                width - metrics["right_margin"],
                y2 + metrics["row_height"] * 0.12,
                fill=row_fill,
                outline="",
            )

        beat_line = 0
        while beat_line <= math.ceil(metrics["total_beats"]):
            x = self._beat_to_x(float(beat_line), metrics)
            color = (
                COLORS["timeline_grid"]
                if beat_line % 4 == 0
                else COLORS["timeline_minor"]
            )
            canvas.create_line(
                x,
                metrics["top_margin"],
                x,
                height - metrics["bottom_margin"],
                fill=color,
            )
            canvas.create_text(
                x,
                height - 10,
                text=str(beat_line),
                fill="#9eb1c1",
                font=("Consolas", 8),
            )
            beat_line += 1

        for window in self.invalid_windows:
            x1 = self._beat_to_x(window.start_beats, metrics)
            x2 = self._beat_to_x(window.end_beats, metrics)
            canvas.create_rectangle(
                x1,
                metrics["top_margin"],
                x2,
                height - metrics["bottom_margin"],
                fill=COLORS["timeline_invalid_band"],
                outline="",
                stipple="gray25",
            )

        for note in self.notes:
            x1 = self._beat_to_x(note.start_beats, metrics)
            x2 = self._beat_to_x(note.end_beats, metrics)
            y1, y2 = self._midi_to_y(note.midi_note, metrics)

            fill = COLORS["timeline_note"]
            outline = COLORS["timeline_note_dark"]
            width_px = 1
            if (
                note.note_id in self.invalid_note_ids
                or note.note_id in self.out_of_range_note_ids
            ):
                fill = COLORS["timeline_invalid"]
                outline = "#5c1f24"
            if note.note_id in self.selected_note_ids:
                if note.note_id not in self.invalid_note_ids and note.note_id not in self.out_of_range_note_ids:
                    fill = COLORS["timeline_selected"]
                outline = "#fff2b3"
                width_px = 2
            if note.note_id in self.preview_active_ids:
                fill = COLORS["timeline_preview"]
                outline = "#ffd8a8"
                width_px = 2

            canvas.create_rectangle(
                x1,
                y1,
                max(x1 + 4, x2),
                y2,
                fill=fill,
                outline=outline,
                width=width_px,
            )
            self.note_regions.append((x1, y1, max(x1 + 4, x2), y2, note.note_id))

            if (x2 - x1) > 42:
                canvas.create_text(
                    x1 + 6,
                    (y1 + y2) / 2,
                    text=note_name(note.midi_note),
                    anchor="w",
                    fill="#ffffff",
                    font=("Consolas", 8),
                )
            if note.note_id == self.selected_note_id:
                handle_x = max(x1 + 4, x2) - 5
                canvas.create_line(
                    handle_x,
                    y1 + 3,
                    handle_x,
                    y2 - 3,
                    fill="#fff7d6",
                    width=2,
                )

        preview_notes = self._drag_preview_notes()
        if preview_notes:
            for preview_note in preview_notes:
                x1 = self._beat_to_x(preview_note.start_beats, metrics)
                x2 = self._beat_to_x(preview_note.end_beats, metrics)
                y1, y2 = self._midi_to_y(preview_note.midi_note, metrics)
                canvas.create_rectangle(
                    x1,
                    y1,
                    max(x1 + 4, x2),
                    y2,
                    fill=COLORS["timeline_draft"],
                    outline="#d9f1ff",
                    width=2,
                    dash=(4, 2),
                )
        elif self.drag_preview is not None:
            x1 = self._beat_to_x(self.drag_preview.start_beats, metrics)
            x2 = self._beat_to_x(self.drag_preview.end_beats, metrics)
            y1, y2 = self._midi_to_y(self.drag_preview.midi_note, metrics)
            canvas.create_rectangle(
                x1,
                y1,
                max(x1 + 4, x2),
                y2,
                fill=COLORS["timeline_draft"],
                outline="#d9f1ff",
                width=2,
                dash=(4, 2),
            )

        label_size = max(7, min(9, int(metrics["row_height"] * 0.45)))
        gutter_right = visible_left + metrics["left_margin"]
        for midi_note in range(EDITOR_NOTE_MIN, EDITOR_NOTE_MAX + 1):
            y1, y2 = self._midi_to_y(midi_note, metrics)
            row_fill = COLORS["timeline_bg"] if not note_is_black(midi_note) else "#142838"
            canvas.create_rectangle(
                visible_left,
                y1 - metrics["row_height"] * 0.12,
                gutter_right,
                y2 + metrics["row_height"] * 0.12,
                fill=row_fill,
                outline="",
            )
            canvas.create_text(
                gutter_right - 6,
                (y1 + y2) / 2,
                text=note_name(midi_note),
                anchor="e",
                fill="#9eb1c1",
                font=("Consolas", label_size),
            )
        canvas.create_line(
            gutter_right,
            metrics["top_margin"],
            gutter_right,
            height - metrics["bottom_margin"],
            fill=COLORS["timeline_grid"],
        )

        visible_right = visible_left + metrics["width"]
        ruler_bottom = visible_top + metrics["top_margin"]
        canvas.create_rectangle(
            visible_left,
            visible_top,
            visible_right,
            ruler_bottom,
            fill=COLORS["timeline_bg"],
            outline="",
        )
        canvas.create_line(
            visible_left,
            ruler_bottom,
            visible_right,
            ruler_bottom,
            fill=COLORS["timeline_grid"],
        )

        cursor_x = self._beat_to_x(self.playback_cursor_beats, metrics)
        canvas.create_line(
            cursor_x,
            ruler_bottom,
            cursor_x,
            height - metrics["bottom_margin"],
            fill=COLORS["timeline_cursor"],
            width=2,
        )
        canvas.create_polygon(
            cursor_x - 7,
            visible_top + 4,
            cursor_x + 7,
            visible_top + 4,
            cursor_x,
            ruler_bottom - 4,
            fill=COLORS["timeline_cursor"],
            outline="",
        )
        canvas.create_text(
            cursor_x + 8,
            visible_top + 10,
            text=f"{self.playback_cursor_beats:.2f}",
            anchor="w",
            fill=COLORS["timeline_cursor"],
            font=("Consolas", 8),
        )

        if self.preview_total_seconds > 0:
            elapsed = time.perf_counter() - self.preview_start_time
            current_beat = min(
                self.preview_origin_beats + elapsed / (60.0 / self._coerce_bpm()),
                metrics["total_beats"],
            )
            timeline_x = self._beat_to_x(current_beat, metrics)
            canvas.create_line(
                timeline_x,
                ruler_bottom,
                timeline_x,
                height - metrics["bottom_margin"],
                fill="#ffffff",
                width=2,
            )

    def _draw_keyboard(self) -> None:
        canvas = self.keyboard_canvas
        canvas.delete("all")
        self.keyboard_regions.clear()

        width = max(200, canvas.winfo_width())
        height = max(120, canvas.winfo_height())
        margin = 18
        top = 18
        white_height = height - top - 16
        black_height = white_height * 0.63
        white_key_count = sum(
            1
            for midi_note in range(ROBOT_NOTE_MIN, ROBOT_NOTE_MAX + 1)
            if not note_is_black(midi_note)
        )
        white_width = (width - margin * 2) / white_key_count

        white_keys: list[tuple[int, float, float, float, float]] = []
        black_keys: list[tuple[int, float, float, float, float]] = []

        white_number = 0
        for midi_note in range(ROBOT_NOTE_MIN, ROBOT_NOTE_MAX + 1):
            if note_is_black(midi_note):
                continue
            x1 = margin + white_number * white_width
            x2 = x1 + white_width
            white_keys.append((midi_note, x1, top, x2, top + white_height))
            white_number += 1

        for midi_note in range(ROBOT_NOTE_MIN, ROBOT_NOTE_MAX + 1):
            if not note_is_black(midi_note):
                continue
            left_index = white_key_index(midi_note - 1) - white_key_index(ROBOT_NOTE_MIN)
            black_width = white_width * 0.65
            x1 = margin + (left_index + 1) * white_width - black_width / 2
            x2 = x1 + black_width
            black_keys.append((midi_note, x1, top, x2, top + black_height))

        selected_notes = self._selected_pitch_set()
        preview_notes = self._preview_pitch_set()

        for midi_note, x1, y1, x2, y2 in white_keys:
            fill = COLORS["white_key"]
            if midi_note in selected_notes:
                fill = COLORS["keyboard_selected"]
            if midi_note in preview_notes:
                fill = COLORS["keyboard_preview"]
            canvas.create_rectangle(
                x1,
                y1,
                x2,
                y2,
                fill=fill,
                outline=COLORS["white_key_border"],
                width=1,
            )
            if midi_note % 12 == 0:
                canvas.create_text(
                    (x1 + x2) / 2,
                    y2 - 10,
                    text=note_name(midi_note),
                    fill=COLORS["text_muted"],
                    font=("Consolas", 7),
                )
            self.keyboard_regions.append((midi_note, x1, y1, x2, y2, False))

        for midi_note, x1, y1, x2, y2 in black_keys:
            fill = COLORS["black_key"]
            if midi_note in selected_notes:
                fill = COLORS["keyboard_selected"]
            if midi_note in preview_notes:
                fill = COLORS["keyboard_preview"]
            canvas.create_rectangle(x1, y1, x2, y2, fill=fill, outline="#0a1217", width=1)
            self.keyboard_regions.append((midi_note, x1, y1, x2, y2, True))

    def _on_timeline_mousewheel(self, event) -> str:
        control_down = bool(event.state & 0x0004)
        shift_down = bool(event.state & 0x0001)
        direction = 1 if event.delta > 0 else -1

        if control_down and shift_down:
            self._change_pitch_zoom(direction, focus_y=event.y)
            return "break"
        if control_down:
            self._change_time_zoom(direction, focus_x=event.x)
            return "break"

        units = -1 if event.delta > 0 else 1
        if shift_down:
            self.timeline_canvas.xview_scroll(units * 3, "units")
        else:
            self.timeline_canvas.yview_scroll(units * 3, "units")
        self._draw_timeline()
        return "break"

    def _on_timeline_press(self, event) -> None:
        metrics = self._timeline_metrics()
        canvas_x = self.timeline_canvas.canvasx(event.x)
        canvas_y = self.timeline_canvas.canvasy(event.y)
        visible_top = self.timeline_canvas.canvasy(0)
        control_down = bool(event.state & 0x0004)
        beat = self._x_to_beat(canvas_x, metrics)
        midi_note = self._y_to_midi(canvas_y, metrics)
        if canvas_y <= visible_top + metrics["top_margin"]:
            if beat is not None:
                self.cursor_drag_active = True
                self._set_playback_cursor(self._snap_beat(beat), announce=True)
            return
        if beat is None or midi_note is None:
            return

        region = self._note_region_at_point(canvas_x, canvas_y)
        if region is not None:
            note_id = region[4]
            note = self._note_by_id(note_id)
            if note is None:
                return
            if control_down:
                next_selection = set(self.selected_note_ids)
                if note_id in next_selection:
                    next_selection.remove(note_id)
                    primary_note_id = self.selected_note_id
                    if primary_note_id == note_id:
                        primary_note_id = next(iter(sorted(next_selection)), None)
                else:
                    next_selection.add(note_id)
                    primary_note_id = note_id
                self._set_selection(next_selection, primary_note_id, refresh=False)
                self._sync_editor_from_selection()
                self._update_button_states()
                self._draw_timeline()
                self._draw_keyboard()
                return
            mode = "resize" if self._point_hits_resize_handle(canvas_x, region) else "move"
            if mode == "resize" or note_id not in self.selected_note_ids:
                selection_ids = {note_id}
            else:
                selection_ids = set(self.selected_note_ids)
            self._set_selection(selection_ids, note_id, refresh=False)
            self.drag_state = DragState(
                mode=mode,
                note_id=note_id,
                anchor_beat=note.start_beats,
                press_beat=beat,
                press_midi=midi_note,
                original_note=replace(note),
                selected_note_ids=tuple(sorted(self.selected_note_ids)),
            )
            self.drag_preview = replace(note)
            self._sync_editor_from_selection()
            self._update_button_states()
            self._draw_timeline()
            self._draw_keyboard()
            return

        duration = self._coerce_float_var(
            self.default_duration_var, MIN_BEAT_VALUE, 16.0, 1.0
        )
        velocity = self._coerce_int_var(self.default_velocity_var, 30, 127, 96)
        start_beat = self._snap_beat(beat)
        self._set_selection(set(), None, refresh=False)
        self.drag_state = DragState(
            mode="create",
            note_id=None,
            anchor_beat=start_beat,
            press_beat=start_beat,
            press_midi=midi_note,
            original_note=None,
        )
        self.drag_preview = NoteBlock(-1, midi_note, start_beat, duration, velocity)
        self._sync_editor_from_selection()
        self._update_button_states()
        self._draw_timeline()
        self._draw_keyboard()

    def _on_timeline_drag(self, event) -> None:
        if self.cursor_drag_active:
            metrics = self._timeline_metrics()
            canvas_x = self.timeline_canvas.canvasx(event.x)
            beat = self._x_to_beat(canvas_x, metrics)
            if beat is not None:
                self._set_playback_cursor(self._snap_beat(beat))
            return

        if self.drag_state is None or self.drag_preview is None:
            return

        metrics = self._timeline_metrics()
        canvas_x = self.timeline_canvas.canvasx(event.x)
        canvas_y = self.timeline_canvas.canvasy(event.y)
        raw_beat = self._x_to_beat(canvas_x, metrics)
        if raw_beat is None:
            raw_beat = self.drag_state.anchor_beat
        snapped_beat = self._snap_beat(raw_beat)
        current_midi = self._y_to_midi(canvas_y, metrics)
        if current_midi is None:
            current_midi = self.drag_state.press_midi
        self.drag_state.moved = True

        if self.drag_state.mode == "create":
            snap = self._grid_snap()
            start = min(self.drag_state.anchor_beat, snapped_beat)
            end = max(self.drag_state.anchor_beat + snap, snapped_beat + snap)
            self.drag_preview = sanitize_note(
                NoteBlock(
                    -1,
                    self.drag_state.press_midi,
                    start,
                    end - start,
                    self.drag_preview.velocity,
                )
            )
        elif self.drag_state.mode == "move" and self.drag_state.original_note is not None:
            offset = self.drag_state.press_beat - self.drag_state.original_note.start_beats
            new_start = self._snap_beat(raw_beat - offset)
            pitch_delta = current_midi - self.drag_state.press_midi
            self.drag_preview = sanitize_note(
                NoteBlock(
                    self.drag_state.original_note.note_id,
                    self.drag_state.original_note.midi_note + pitch_delta,
                    new_start,
                    self.drag_state.original_note.duration_beats,
                    self.drag_state.original_note.velocity,
                )
            )
        elif self.drag_state.mode == "resize" and self.drag_state.original_note is not None:
            snap = self._grid_snap()
            end_beat = max(self.drag_state.original_note.start_beats + snap, snapped_beat + snap)
            self.drag_preview = sanitize_note(
                NoteBlock(
                    self.drag_state.original_note.note_id,
                    self.drag_state.original_note.midi_note,
                    self.drag_state.original_note.start_beats,
                    end_beat - self.drag_state.original_note.start_beats,
                    self.drag_state.original_note.velocity,
                )
            )

        self._draw_timeline()
        self._draw_keyboard()

    def _on_timeline_release(self, _event) -> None:
        if self.cursor_drag_active:
            self.cursor_drag_active = False
            self.status_var.set(
                f"Playback cursor set to beat {self.playback_cursor_beats:.2f}."
            )
            return
        if self.drag_state is None or self.drag_preview is None:
            return

        changed = False
        preview_note = sanitize_note(self.drag_preview)
        if self.drag_state.mode == "create":
            if not self.drag_state.moved:
                preview_note = sanitize_note(
                    NoteBlock(
                        -1,
                        preview_note.midi_note,
                        preview_note.start_beats,
                        self._coerce_float_var(
                            self.default_duration_var, MIN_BEAT_VALUE, 16.0, 1.0
                        ),
                        preview_note.velocity,
                    )
                )
            new_note = self._assign_note_id(
                NoteBlock(
                    0,
                    preview_note.midi_note,
                    preview_note.start_beats,
                    preview_note.duration_beats,
                    preview_note.velocity,
                )
            )
            self.notes.append(new_note)
            self._set_selection({new_note.note_id}, new_note.note_id, refresh=False)
            self.status_var.set(
                f"Placed {note_name(new_note.midi_note)} at beat {new_note.start_beats:.2f}."
            )
            self._mark_dirty()
            changed = True
        elif self.drag_state.original_note is not None:
            if (
                self.drag_state.mode == "move"
                and len(self.drag_state.selected_note_ids) > 1
            ):
                preview_notes = self._drag_preview_notes()
                original_by_id = {
                    note.note_id: note
                    for note in self.notes
                    if note.note_id in self.drag_state.selected_note_ids
                }
                if any(
                    original_by_id.get(note.note_id) is None
                    or not self._notes_equal(note, original_by_id[note.note_id])
                    for note in preview_notes
                ):
                    for note in preview_notes:
                        self._replace_note(note)
                    self._set_selection(
                        set(self.drag_state.selected_note_ids),
                        self.drag_state.note_id,
                        refresh=False,
                    )
                    self.status_var.set(f"Moved {len(preview_notes)} selected notes together.")
                    self._mark_dirty()
                    changed = True
            elif not self._notes_equal(preview_note, self.drag_state.original_note):
                self._replace_note(preview_note)
                self._set_selection({preview_note.note_id}, preview_note.note_id, refresh=False)
                action = "Resized" if self.drag_state.mode == "resize" else "Moved"
                self.status_var.set(
                    f"{action} {note_name(preview_note.midi_note)} to beat {preview_note.start_beats:.2f}."
                )
                self._mark_dirty()
                changed = True
            else:
                self._set_selection(
                    set(self.drag_state.selected_note_ids) or {preview_note.note_id},
                    preview_note.note_id,
                    refresh=False,
                )

        self.drag_state = None
        self.drag_preview = None
        if not changed and self.selected_note_id is not None:
            self.status_var.set("Selected note.")
        self._refresh_views()

    def _on_timeline_right_click(self, event) -> None:
        canvas_x = self.timeline_canvas.canvasx(event.x)
        canvas_y = self.timeline_canvas.canvasy(event.y)
        region = self._note_region_at_point(canvas_x, canvas_y)
        if region is None:
            return
        note = self._remove_note(region[4])
        if note is None:
            return
        updated_selection = set(self.selected_note_ids)
        updated_selection.discard(note.note_id)
        next_primary = self.selected_note_id
        if next_primary == note.note_id:
            next_primary = next(iter(sorted(updated_selection)), None)
        self._set_selection(updated_selection, next_primary, refresh=False)
        self._mark_dirty()
        self.status_var.set(f"Deleted {note_name(note.midi_note)}.")
        self._refresh_views()

    def _on_keyboard_click(self, event) -> None:
        black_regions = [region for region in self.keyboard_regions if region[5]]
        white_regions = [region for region in self.keyboard_regions if not region[5]]
        for regions in (black_regions, white_regions):
            for midi_note, x1, y1, x2, y2, _is_black in regions:
                if x1 <= event.x <= x2 and y1 <= event.y <= y2:
                    self.pitch_var.set(note_name(midi_note))
                    self.status_var.set(f"Loaded {note_name(midi_note)} into the pitch picker.")
                    return

    def _on_close(self) -> None:
        if not self._maybe_discard_changes():
            return
        self._stop_preview()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()

    try:
        from ctypes import windll

        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

    RobotPianoStudio(root)
    root.mainloop()


if __name__ == "__main__":
    main()
