#!/usr/bin/env python3
"""
piano_gui.py — Piano Robot Control GUI
=======================================
Visual interface for the ESP32 piano robot.

Features:
  - MIDI file browser with drag-and-drop-style file list
  - Live piano keyboard visualization (C2–C5)
  - Robot carriage + finger position overlay
  - Note timeline / sheet view
  - Real-time serial monitor with ESP32 telemetry parsing
  - One-click upload & play
  - Homing control

Requirements:
  pip install pyserial
"""

import struct
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from pathlib import Path
import serial
import serial.tools.list_ports
import queue
import re
import math

# ─── MIDI Parser (pure Python, matches firmware logic) ────────────────────────

NOTE_NAMES = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']

def note_name(midi_note):
    return f"{NOTE_NAMES[midi_note % 12]}{(midi_note // 12) - 1}"

def is_black_key(midi_note):
    return (midi_note % 12) in [1, 3, 6, 8, 10]

def parse_vlq(data, pos):
    val = 0
    for _ in range(4):
        if pos >= len(data):
            break
        b = data[pos]; pos += 1
        val = (val << 7) | (b & 0x7F)
        if not (b & 0x80):
            break
    return val, pos

def parse_midi_file(path):
    """Parse a .mid file into a list of (time_ms, midi_note, duration_ms, velocity)."""
    with open(path, 'rb') as f:
        data = f.read()

    if data[:4] != b'MThd':
        raise ValueError("Not a valid MIDI file")

    fmt = struct.unpack_from('>H', data, 8)[0]
    num_tracks = struct.unpack_from('>H', data, 10)[0]
    ppq = struct.unpack_from('>H', data, 12)[0]

    # Collect all note events from all tracks
    all_events = []  # (tick, 'on'/'off', note, velocity, track)

    pos = 14
    for track_idx in range(num_tracks):
        if pos + 8 > len(data):
            break
        chunk_id = data[pos:pos+4]
        chunk_len = struct.unpack_from('>I', data, pos + 4)[0]
        pos += 8

        if chunk_id != b'MTrk':
            pos += chunk_len
            continue

        track_end = pos + chunk_len
        tick = 0
        running_status = 0
        tempo = 500000  # default 120 BPM

        while pos < track_end:
            delta, pos = parse_vlq(data, pos)
            tick += delta

            if pos >= len(data):
                break

            status = data[pos]
            if status & 0x80:
                running_status = status
                pos += 1

            msg_type = running_status & 0xF0

            if running_status == 0xFF:  # Meta event
                if pos + 1 > len(data):
                    break
                meta_type = data[pos]; pos += 1
                meta_len, pos = parse_vlq(data, pos)
                if meta_type == 0x51 and meta_len == 3:
                    tempo = (data[pos] << 16) | (data[pos+1] << 8) | data[pos+2]
                    all_events.append((tick, 'tempo', tempo, 0, track_idx))
                pos += meta_len
            elif msg_type == 0x90:  # Note On
                if pos + 1 >= len(data):
                    break
                note, vel = data[pos], data[pos+1]; pos += 2
                if vel > 0:
                    all_events.append((tick, 'on', note, vel, track_idx))
                else:
                    all_events.append((tick, 'off', note, 0, track_idx))
            elif msg_type == 0x80:  # Note Off
                if pos + 1 >= len(data):
                    break
                note = data[pos]; pos += 2
                all_events.append((tick, 'off', note, 0, track_idx))
            elif msg_type in (0xA0, 0xB0, 0xE0):
                pos += 2
            elif msg_type in (0xC0, 0xD0):
                pos += 1
            elif running_status in (0xF0, 0xF7):
                sysex_len, pos = parse_vlq(data, pos)
                pos += sysex_len

    # Sort by tick
    all_events.sort(key=lambda e: e[0])

    # Convert to timed note list
    tempo = 500000
    active = {}  # note -> (on_tick, velocity)
    notes = []   # (time_ms, note, duration_ms, velocity)

    # Build tempo map
    tempo_changes = [(0, 500000)]
    for ev in all_events:
        if ev[1] == 'tempo':
            tempo_changes.append((ev[0], ev[2]))

    def ticks_to_ms(tick):
        ms = 0.0
        prev_tick = 0
        cur_tempo = 500000
        for t, tmp in tempo_changes:
            if t > tick:
                break
            ms += (min(t, tick) - prev_tick) * cur_tempo / (ppq * 1000.0)
            prev_tick = t
            cur_tempo = tmp
        ms += (tick - prev_tick) * cur_tempo / (ppq * 1000.0)
        return ms

    for ev in all_events:
        tick, typ, val, vel, _ = ev
        if typ == 'on':
            active[val] = (tick, vel)
        elif typ == 'off' and val in active:
            on_tick, on_vel = active.pop(val)
            time_ms = ticks_to_ms(on_tick)
            dur_ms = ticks_to_ms(tick) - time_ms
            if dur_ms < 10:
                dur_ms = 10
            notes.append((time_ms, val, dur_ms, on_vel))

    notes.sort(key=lambda n: n[0])
    return notes


# ─── Robot geometry (matches firmware config) ────────────────────────────────

WHITE_KEY_WIDTH_MM = 23.5
MOTOR_MIN_MM = 0.0
MOTOR_MAX_MM = 380.0
MIDI_MIN = 36  # C2
MIDI_MAX = 72  # C5

FINGER_NAMES = ['W1', 'W2', 'W3', 'B1', 'B2']
FINGER_COLORS = {
    'W1': '#FF6B6B', 'W2': '#4ECDC4', 'W3': '#45B7D1',
    'B1': '#F7DC6F', 'B2': '#BB8FCE'
}

def white_key_index(midi_note):
    """Get 0-based white key index for a white key MIDI note."""
    octave = (midi_note // 12) - 2
    note_in_octave = midi_note % 12
    white_offsets = {0:0, 2:1, 4:2, 5:3, 7:4, 9:5, 11:6}
    if note_in_octave in white_offsets:
        return octave * 7 + white_offsets[note_in_octave]
    return -1

def motor_pos_for_note(midi_note, finger='W1'):
    """Calculate motor position for a given note and finger."""
    if is_black_key(midi_note):
        # Black key: find the white key to the left
        lwki = white_key_index(midi_note - 1)
        if finger == 'B1':
            return lwki * WHITE_KEY_WIDTH_MM
        elif finger == 'B2':
            return (lwki - 2) * WHITE_KEY_WIDTH_MM
    else:
        wki = white_key_index(midi_note)
        if wki < 0:
            return -1
        if finger == 'W1':
            return wki * WHITE_KEY_WIDTH_MM
        elif finger == 'W2':
            return (wki - 2) * WHITE_KEY_WIDTH_MM
        elif finger == 'W3':
            return (wki - 4) * WHITE_KEY_WIDTH_MM
    return -1


# ─── Color theme ──────────────────────────────────────────────────────────────

COLORS = {
    'bg':           '#1a1a2e',
    'bg_light':     '#16213e',
    'bg_card':      '#0f3460',
    'accent':       '#e94560',
    'accent2':      '#533483',
    'text':         '#eaeaea',
    'text_dim':     '#8892b0',
    'white_key':    '#f8f8f8',
    'black_key':    '#1a1a1a',
    'key_pressed':  '#e94560',
    'key_target':   '#533483',
    'robot_body':   '#e9456080',
    'grid':         '#ffffff15',
    'success':      '#2ecc71',
    'warning':      '#f39c12',
}


# ─── Main Application ────────────────────────────────────────────────────────

class PianoRobotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Piano Robot Controller")
        self.root.geometry("1400x900")
        self.root.configure(bg=COLORS['bg'])
        self.root.minsize(1100, 700)

        # State
        self.serial_port = None
        self.serial_thread = None
        self.serial_running = False
        self.serial_queue = queue.Queue()
        self.midi_files = []
        self.current_notes = []
        self.current_note_idx = -1
        self.robot_pos_mm = 0.0
        self.robot_target_mm = 0.0
        self.active_finger = None
        self.active_keys = set()
        self.is_playing = False
        self.playback_thread = None
        self.stop_playback = False
        self.connected = False
        self.firmware_ready = False
        self.playback_finish_after = None
        self.ready_probe_after = None
        self.saw_boot_banner = False
        self.serial_pause_reads = False
        self.solenoid_pwm_percent_var = tk.IntVar(value=100)

        # Telemetry
        self.tel_pos = 0.0
        self.tel_target = 0.0
        self.tel_state = "IDLE"
        self.tel_fingers = 0x00

        self._build_ui()
        self._start_serial_poll()
        self._refresh_ports()

    # ── UI Construction ───────────────────────────────────────────────────

    def _build_ui(self):
        style = ttk.Style()
        style.theme_use('clam')

        # Configure dark theme styles
        style.configure('Dark.TFrame', background=COLORS['bg'])
        style.configure('Card.TFrame', background=COLORS['bg_card'])
        style.configure('Dark.TLabel', background=COLORS['bg'], foreground=COLORS['text'],
                       font=('Segoe UI', 10))
        style.configure('Title.TLabel', background=COLORS['bg'], foreground=COLORS['text'],
                       font=('Segoe UI', 14, 'bold'))
        style.configure('Card.TLabel', background=COLORS['bg_card'], foreground=COLORS['text'],
                       font=('Segoe UI', 10))
        style.configure('CardTitle.TLabel', background=COLORS['bg_card'], foreground=COLORS['text'],
                       font=('Segoe UI', 11, 'bold'))
        style.configure('Status.TLabel', background=COLORS['bg'], foreground=COLORS['success'],
                       font=('Segoe UI', 10, 'bold'))
        style.configure('Accent.TButton', font=('Segoe UI', 10, 'bold'))

        # ── Top toolbar ──────────────────────────────────────────────────
        toolbar = tk.Frame(self.root, bg=COLORS['bg_light'], pady=8, padx=12)
        toolbar.pack(fill='x')

        # Logo / Title
        tk.Label(toolbar, text="PIANO ROBOT", font=('Segoe UI', 16, 'bold'),
                bg=COLORS['bg_light'], fg=COLORS['accent']).pack(side='left', padx=(0, 20))

        # Serial controls
        serial_frame = tk.Frame(toolbar, bg=COLORS['bg_light'])
        serial_frame.pack(side='left', padx=10)

        tk.Label(serial_frame, text="Port:", font=('Segoe UI', 10),
                bg=COLORS['bg_light'], fg=COLORS['text']).pack(side='left')

        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(serial_frame, textvariable=self.port_var,
                                       width=15, state='readonly')
        self.port_combo.pack(side='left', padx=4)

        self.refresh_btn = tk.Button(serial_frame, text="Refresh", command=self._refresh_ports,
                                     bg=COLORS['bg_card'], fg=COLORS['text'],
                                     font=('Segoe UI', 9), relief='flat', padx=8)
        self.refresh_btn.pack(side='left', padx=2)

        self.connect_btn = tk.Button(serial_frame, text="Connect", command=self._toggle_connection,
                                     bg=COLORS['accent'], fg='white',
                                     font=('Segoe UI', 10, 'bold'), relief='flat', padx=12)
        self.connect_btn.pack(side='left', padx=8)

        # Status indicator
        self.status_label = tk.Label(toolbar, text="Disconnected",
                                     font=('Segoe UI', 10, 'bold'),
                                     bg=COLORS['bg_light'], fg=COLORS['warning'])
        self.status_label.pack(side='left', padx=10)

        # Telemetry display
        self.tel_label = tk.Label(toolbar, text="",
                                  font=('Consolas', 9),
                                  bg=COLORS['bg_light'], fg=COLORS['text_dim'])
        self.tel_label.pack(side='right', padx=10)

        # ── Main content area ────────────────────────────────────────────
        main = tk.Frame(self.root, bg=COLORS['bg'])
        main.pack(fill='both', expand=True, padx=12, pady=(8, 12))

        # Left panel: file list + controls
        left = tk.Frame(main, bg=COLORS['bg'], width=300)
        left.pack(side='left', fill='y', padx=(0, 8))
        left.pack_propagate(False)

        self._build_file_panel(left)
        self._build_controls_panel(left)

        # Right panel: piano + timeline + log
        right = tk.Frame(main, bg=COLORS['bg'])
        right.pack(side='left', fill='both', expand=True)

        self._build_piano_panel(right)
        self._build_timeline_panel(right)
        self._build_log_panel(right)

    def _build_file_panel(self, parent):
        card = tk.Frame(parent, bg=COLORS['bg_card'], padx=12, pady=10)
        card.pack(fill='x', pady=(0, 8))

        header = tk.Frame(card, bg=COLORS['bg_card'])
        header.pack(fill='x')
        tk.Label(header, text="MIDI Files", font=('Segoe UI', 12, 'bold'),
                bg=COLORS['bg_card'], fg=COLORS['text']).pack(side='left')

        btn_frame = tk.Frame(header, bg=COLORS['bg_card'])
        btn_frame.pack(side='right')
        tk.Button(btn_frame, text="+ Add", command=self._add_midi_file,
                 bg=COLORS['accent'], fg='white', font=('Segoe UI', 9, 'bold'),
                 relief='flat', padx=8).pack(side='left', padx=2)

        # File listbox
        list_frame = tk.Frame(card, bg=COLORS['bg_card'])
        list_frame.pack(fill='both', expand=True, pady=(8, 0))

        self.file_listbox = tk.Listbox(list_frame, bg=COLORS['bg'],
                                        fg=COLORS['text'], selectbackground=COLORS['accent'],
                                        font=('Consolas', 10), relief='flat',
                                        highlightthickness=1, highlightcolor=COLORS['accent'],
                                        height=8)
        self.file_listbox.pack(fill='both', expand=True)
        self.file_listbox.bind('<<ListboxSelect>>', self._on_file_select)

        # Note info
        self.file_info = tk.Label(card, text="No file selected",
                                   font=('Segoe UI', 9), bg=COLORS['bg_card'],
                                   fg=COLORS['text_dim'], anchor='w')
        self.file_info.pack(fill='x', pady=(6, 0))

    def _build_controls_panel(self, parent):
        card = tk.Frame(parent, bg=COLORS['bg_card'], padx=12, pady=10)
        card.pack(fill='x', pady=(0, 8))

        tk.Label(card, text="Controls", font=('Segoe UI', 12, 'bold'),
                bg=COLORS['bg_card'], fg=COLORS['text']).pack(anchor='w')

        btn_grid = tk.Frame(card, bg=COLORS['bg_card'])
        btn_grid.pack(fill='x', pady=(8, 0))

        self.play_btn = tk.Button(btn_grid, text="Upload & Play",
                                  command=self._upload_and_play,
                                  bg='#2ecc71', fg='white',
                                  font=('Segoe UI', 11, 'bold'),
                                  relief='flat', padx=16, pady=6)
        self.play_btn.pack(fill='x', pady=2)

        self.stop_btn = tk.Button(btn_grid, text="Stop (Not Supported)",
                                  command=self._stop, state='disabled',
                                  bg=COLORS['accent'], fg='white',
                                  font=('Segoe UI', 10, 'bold'),
                                  relief='flat', padx=12, pady=4)
        self.stop_btn.pack(fill='x', pady=2)

        self.home_btn = tk.Button(btn_grid, text="Home Robot",
                                  command=self._send_home,
                                  bg=COLORS['accent2'], fg='white',
                                  font=('Segoe UI', 10),
                                  relief='flat', padx=12, pady=4)
        self.home_btn.pack(fill='x', pady=2)

        pwm_frame = tk.Frame(card, bg=COLORS['bg_card'])
        pwm_frame.pack(fill='x', pady=(8, 0))
        tk.Label(pwm_frame, text="Solenoid PWM",
                 font=('Segoe UI', 10, 'bold'),
                 bg=COLORS['bg_card'], fg=COLORS['text']).pack(anchor='w')
        pwm_row = tk.Frame(pwm_frame, bg=COLORS['bg_card'])
        pwm_row.pack(fill='x', pady=(2, 0))
        self.solenoid_pwm_scale = tk.Scale(
            pwm_row, from_=0, to=100, orient='horizontal',
            variable=self.solenoid_pwm_percent_var,
            bg=COLORS['bg_card'], fg=COLORS['text'],
            troughcolor=COLORS['bg'], highlightthickness=0,
            resolution=1, length=170)
        self.solenoid_pwm_scale.pack(side='left', fill='x', expand=True)
        self.solenoid_pwm_btn = tk.Button(
            pwm_row, text="Apply", command=self._send_solenoid_pwm_from_gui,
            bg=COLORS['accent2'], fg='white',
            font=('Segoe UI', 9), relief='flat', padx=8)
        self.solenoid_pwm_btn.pack(side='left', padx=(6, 0))

        # Playback progress
        self.progress_var = tk.DoubleVar(value=0)
        self.progress = ttk.Progressbar(card, variable=self.progress_var,
                                         maximum=100, mode='determinate')
        self.progress.pack(fill='x', pady=(8, 0))

        self.progress_label = tk.Label(card, text="Ready",
                                        font=('Segoe UI', 9),
                                        bg=COLORS['bg_card'], fg=COLORS['text_dim'])
        self.progress_label.pack(anchor='w', pady=(2, 0))

        # Note sequence panel
        seq_card = tk.Frame(parent, bg=COLORS['bg_card'], padx=12, pady=10)
        seq_card.pack(fill='both', expand=True)

        tk.Label(seq_card, text="Note Sequence", font=('Segoe UI', 12, 'bold'),
                bg=COLORS['bg_card'], fg=COLORS['text']).pack(anchor='w')

        seq_frame = tk.Frame(seq_card, bg=COLORS['bg_card'])
        seq_frame.pack(fill='both', expand=True, pady=(6, 0))

        self.note_listbox = tk.Listbox(seq_frame, bg=COLORS['bg'],
                                        fg=COLORS['text'],
                                        selectbackground=COLORS['accent'],
                                        font=('Consolas', 9), relief='flat',
                                        highlightthickness=0)
        note_scroll = ttk.Scrollbar(seq_frame, orient='vertical',
                                     command=self.note_listbox.yview)
        self.note_listbox.configure(yscrollcommand=note_scroll.set)
        self.note_listbox.pack(side='left', fill='both', expand=True)
        note_scroll.pack(side='right', fill='y')

    def _build_piano_panel(self, parent):
        card = tk.Frame(parent, bg=COLORS['bg_card'], padx=12, pady=10)
        card.pack(fill='x', pady=(0, 8))

        header = tk.Frame(card, bg=COLORS['bg_card'])
        header.pack(fill='x')
        tk.Label(header, text="Piano & Robot Position",
                font=('Segoe UI', 12, 'bold'),
                bg=COLORS['bg_card'], fg=COLORS['text']).pack(side='left')

        # Finger legend
        legend = tk.Frame(header, bg=COLORS['bg_card'])
        legend.pack(side='right')
        for name, color in FINGER_COLORS.items():
            tk.Label(legend, text=f" {name} ", font=('Consolas', 8, 'bold'),
                    bg=color, fg='white').pack(side='left', padx=1)

        self.piano_canvas = tk.Canvas(card, bg=COLORS['bg'], height=200,
                                       highlightthickness=0)
        self.piano_canvas.pack(fill='x', pady=(8, 0))
        self.piano_canvas.bind('<Configure>', lambda e: self._draw_piano())

        # Position info bar
        pos_frame = tk.Frame(card, bg=COLORS['bg_card'])
        pos_frame.pack(fill='x', pady=(6, 0))
        self.pos_label = tk.Label(pos_frame,
                                   text="Motor: 0.0mm  |  Target: 0.0mm  |  State: IDLE",
                                   font=('Consolas', 10),
                                   bg=COLORS['bg_card'], fg=COLORS['text_dim'])
        self.pos_label.pack(side='left')

    def _build_timeline_panel(self, parent):
        card = tk.Frame(parent, bg=COLORS['bg_card'], padx=12, pady=10)
        card.pack(fill='x', pady=(0, 8))

        tk.Label(card, text="Timeline", font=('Segoe UI', 12, 'bold'),
                bg=COLORS['bg_card'], fg=COLORS['text']).pack(anchor='w')

        self.timeline_canvas = tk.Canvas(card, bg=COLORS['bg'], height=100,
                                          highlightthickness=0)
        self.timeline_canvas.pack(fill='x', pady=(6, 0))
        self.timeline_canvas.bind('<Configure>', lambda e: self._draw_timeline())

    def _build_log_panel(self, parent):
        card = tk.Frame(parent, bg=COLORS['bg_card'], padx=12, pady=10)
        card.pack(fill='both', expand=True)

        header = tk.Frame(card, bg=COLORS['bg_card'])
        header.pack(fill='x')
        tk.Label(header, text="Serial Monitor", font=('Segoe UI', 12, 'bold'),
                bg=COLORS['bg_card'], fg=COLORS['text']).pack(side='left')
        tk.Button(header, text="Clear", command=self._clear_log,
                 bg=COLORS['bg'], fg=COLORS['text_dim'],
                 font=('Segoe UI', 8), relief='flat', padx=6).pack(side='right')

        log_frame = tk.Frame(card, bg=COLORS['bg_card'])
        log_frame.pack(fill='both', expand=True, pady=(6, 0))

        self.log_text = tk.Text(log_frame, bg=COLORS['bg'], fg=COLORS['text'],
                                 font=('Consolas', 9), relief='flat',
                                 insertbackground=COLORS['text'], height=6,
                                 highlightthickness=0)
        log_scroll = ttk.Scrollbar(log_frame, orient='vertical',
                                    command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=log_scroll.set)
        self.log_text.pack(side='left', fill='both', expand=True)
        log_scroll.pack(side='right', fill='y')

        # Tag for coloring
        self.log_text.tag_configure('info', foreground=COLORS['text_dim'])
        self.log_text.tag_configure('note', foreground=COLORS['accent'])
        self.log_text.tag_configure('success', foreground=COLORS['success'])
        self.log_text.tag_configure('telemetry', foreground='#4a9eff')

    # ── Serial Connection ─────────────────────────────────────────────────

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _set_robot_ready(self, ready):
        self.firmware_ready = ready
        if not self.connected:
            return

        port = self.port_var.get()
        if ready:
            self.status_label.configure(
                text=f"Connected: {port} (robot ready)",
                fg=COLORS['success'])
        else:
            self.status_label.configure(
                text=f"Connected: {port} (waiting for robot)",
                fg=COLORS['warning'])

    def _estimated_song_duration_ms(self):
        if not self.current_notes:
            return 0
        return int(max(t + d for t, _, d, _ in self.current_notes))

    def _cancel_playback_finish(self):
        if self.playback_finish_after is not None:
            self.root.after_cancel(self.playback_finish_after)
            self.playback_finish_after = None

    def _schedule_playback_finish(self, delay_ms):
        self._cancel_playback_finish()
        self.playback_finish_after = self.root.after(delay_ms, self._playback_finished)

    def _cancel_ready_probe(self):
        if self.ready_probe_after is not None:
            self.root.after_cancel(self.ready_probe_after)
            self.ready_probe_after = None

    def _assume_ready_if_silent(self):
        self.ready_probe_after = None
        if self.connected and not self.firmware_ready and not self.saw_boot_banner:
            self._set_robot_ready(True)
            if not self.is_playing:
                self.progress_label.configure(text="Robot ready")
            self._log("No boot banner seen; assuming robot is already ready.\n", 'info')

    def _coerce_solenoid_pwm_percent(self):
        try:
            percent = int(self.solenoid_pwm_percent_var.get())
        except (tk.TclError, ValueError):
            percent = 100
        percent = max(0, min(100, percent))
        self.solenoid_pwm_percent_var.set(percent)
        return percent

    def _write_solenoid_pwm_command(self, percent):
        if not self.serial_port:
            return
        command = f"PWM {percent}\n".encode("ascii")
        written = self.serial_port.write(command)
        if written != len(command):
            raise IOError(
                f"short PWM command write: expected {len(command)} bytes, wrote {written}"
            )
        self.serial_port.flush()

    def _send_solenoid_pwm_from_gui(self):
        percent = self._coerce_solenoid_pwm_percent()
        if not self.connected or not self.serial_port:
            messagebox.showinfo("Not Connected", "Connect to the robot first.")
            return
        if self.serial_pause_reads:
            messagebox.showinfo("Serial Busy", "Wait for the current serial transfer to finish.")
            return
        try:
            self._write_solenoid_pwm_command(percent)
            self._log(f"Set solenoid pressed PWM to {percent}%\n", 'success')
        except Exception as e:
            self._log(f"PWM command error: {e}\n", 'info')

    def _toggle_connection(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No Port", "Select a serial port first.")
            return
        try:
            self._cancel_playback_finish()
            self._cancel_ready_probe()
            self.serial_port = serial.Serial(port, 115200, timeout=0.1, write_timeout=5)
            self.connected = True
            self.firmware_ready = False
            self.saw_boot_banner = False
            self.serial_running = True
            self.serial_thread = threading.Thread(target=self._serial_reader, daemon=True)
            self.serial_thread.start()

            self.connect_btn.configure(text="Disconnect", bg=COLORS['warning'])
            self._set_robot_ready(False)
            self.progress_label.configure(text="Waiting for robot ready...")
            self._log(f"Connected to {port}\n", 'success')
            self._log("Waiting for robot ready...\n", 'info')
            self.ready_probe_after = self.root.after(1500, self._assume_ready_if_silent)
        except serial.SerialException as e:
            messagebox.showerror("Connection Error", str(e))

    def _disconnect(self):
        self._cancel_playback_finish()
        self._cancel_ready_probe()
        self.serial_running = False
        self.connected = False
        self.firmware_ready = False
        self.is_playing = False
        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None

        self.connect_btn.configure(text="Connect", bg=COLORS['accent'])
        self.status_label.configure(text="Disconnected", fg=COLORS['warning'])
        self.stop_btn.configure(state='disabled')
        self.progress_label.configure(text="Ready")
        self._log("Disconnected\n", 'info')

    def _serial_reader(self):
        """Background thread: reads serial lines and queues them."""
        buf = b""
        while self.serial_running and self.serial_port:
            try:
                if self.serial_pause_reads:
                    time.sleep(0.01)
                    continue
                chunk = self.serial_port.read(256)
                if chunk:
                    buf += chunk
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        text = line.decode('utf-8', errors='replace').strip()
                        if text:
                            self.serial_queue.put(text)
            except Exception:
                break

    def _start_serial_poll(self):
        """Main thread: poll the queue for serial data."""
        try:
            while True:
                line = self.serial_queue.get_nowait()
                self._process_serial_line(line)
        except queue.Empty:
            pass
        self.root.after(50, self._start_serial_poll)

    def _process_serial_line(self, line):
        """Parse telemetry and note events from serial output."""
        if ("Piano robot starting..." in line
                or "Running homing sequence..." in line
                or "Hardware initialised." in line
                or "Press button to home and start" in line
                or "homing..." in line.lower()):
            self.saw_boot_banner = True
            self._set_robot_ready(False)
            self._log(line + '\n', 'info')
            return

        if line == "Homing done. Ready." or line == "Homing done. Robot running.":
            self.saw_boot_banner = True
            self._cancel_ready_probe()
            self._set_robot_ready(True)
            if not self.is_playing:
                self.progress_label.configure(text="Robot ready")
            self._log(line + '\n', 'success')
            return

        midi_match = re.search(
            r'^MIDI:\s+status=(\w+)\s+notes=(\d+)\s+skipped=(\d+)\s+dur=(\d+)ms$',
            line)
        if midi_match:
            status = midi_match.group(1)
            note_count = int(midi_match.group(2))
            duration_ms = int(midi_match.group(4))

            if status == "OK":
                self.progress_label.configure(
                    text=f"Robot accepted {note_count} notes")
                total_ms = max(duration_ms, self._estimated_song_duration_ms())
                self._schedule_playback_finish(max(total_ms + 500, 1500))
            else:
                self._playback_finished(f"Robot rejected MIDI ({status})")

            self._log(line + '\n', 'success' if status == "OK" else 'info')
            return

        # Parse telemetry: [TEL] STATE  pos=X.XXmm  tgt=X.XXmm ...
        tel_match = re.search(
            r'\[TEL\]\s+(\w+)\s+pos=\s*([\d.-]+)mm\s+tgt=\s*([\d.-]+)mm.*fingers=0x([0-9A-Fa-f]+)',
            line)
        if tel_match:
            self.tel_state = tel_match.group(1)
            self.tel_pos = float(tel_match.group(2))
            self.tel_target = float(tel_match.group(3))
            self.tel_fingers = int(tel_match.group(4), 16)
            self.robot_pos_mm = self.tel_pos
            self.robot_target_mm = self.tel_target

            self.pos_label.configure(
                text=f"Motor: {self.tel_pos:.1f}mm  |  Target: {self.tel_target:.1f}mm  "
                     f"|  State: {self.tel_state}  |  Fingers: 0x{self.tel_fingers:02X}")
            self.tel_label.configure(
                text=f"pos={self.tel_pos:.1f}  tgt={self.tel_target:.1f}  [{self.tel_state}]")

            self._update_active_keys()
            self._draw_piano()
            self._log(line + '\n', 'telemetry')
            return

        # Parse note events: --- NOTE X (midi N) ---
        note_match = re.search(r'--- NOTE (\S+) \(midi (\d+)\) ---', line)
        if note_match:
            midi = int(note_match.group(2))
            self.active_keys.add(midi)
            if self.is_playing:
                self.progress_label.configure(text="Playing...")
            self._draw_piano()
            self._log(line + '\n', 'note')

            # Highlight in note list
            for i, n in enumerate(self.current_notes):
                if i > self.current_note_idx and n[1] == midi:
                    self.current_note_idx = i
                    self.note_listbox.selection_clear(0, 'end')
                    self.note_listbox.selection_set(i)
                    self.note_listbox.see(i)
                    if self.current_notes:
                        self.progress_var.set(100 * (i + 1) / len(self.current_notes))
                    break
            self._draw_timeline()
            return

        # Parse finger press/release
        if '[press' in line:
            finger_match = re.search(r'press finger (\w+)', line)
            if finger_match:
                self.active_finger = finger_match.group(1)
                self._draw_piano()
            self._log(line + '\n', 'note')
            return

        if '[release]' in line:
            self.active_finger = None
            self.active_keys.clear()
            self._draw_piano()
            self._log(line + '\n', 'info')
            return

        # Default
        self._log(line + '\n', 'info')

    def _update_active_keys(self):
        """Update active keys from finger bitmask."""
        # We can't easily map fingers to notes without knowing motor position,
        # so we rely on the note parser above for key highlighting.
        pass

    # ── MIDI File Management ──────────────────────────────────────────────

    def _add_midi_file(self):
        paths = filedialog.askopenfilenames(
            title="Select MIDI Files",
            filetypes=[("MIDI files", "*.mid *.midi"), ("All files", "*.*")])
        for p in paths:
            if p not in self.midi_files:
                self.midi_files.append(p)
                name = Path(p).name
                self.file_listbox.insert('end', f"  {name}")

    def _on_file_select(self, event=None):
        sel = self.file_listbox.curselection()
        if not sel:
            return
        idx = sel[0]
        path = self.midi_files[idx]
        try:
            notes = parse_midi_file(path)
            self.current_notes = notes
            self.current_note_idx = -1

            # Update info
            duration = max(n[0] + n[2] for n in notes) if notes else 0
            midi_range = f"{note_name(min(n[1] for n in notes))}–{note_name(max(n[1] for n in notes))}" if notes else "N/A"
            self.file_info.configure(
                text=f"{len(notes)} notes  |  {duration/1000:.1f}s  |  Range: {midi_range}")

            # Populate note list
            self.note_listbox.delete(0, 'end')
            for i, (t, n, d, v) in enumerate(notes):
                name = note_name(n)
                self.note_listbox.insert('end',
                    f" {i+1:3d}  {t/1000:6.2f}s  {name:4s}  {d:4.0f}ms")

            self._draw_timeline()
            self.progress_var.set(0)
            self.progress_label.configure(text=f"Loaded: {Path(path).name}")
        except Exception as e:
            self.file_info.configure(text=f"Error: {e}")

    # ── Upload & Play ─────────────────────────────────────────────────────

    def _upload_and_play(self):
        sel = self.file_listbox.curselection()
        if not sel:
            messagebox.showinfo("No File", "Select a MIDI file first.")
            return
        if not self.connected:
            messagebox.showinfo("Not Connected", "Connect to the robot first.")
            return
        if not self.firmware_ready:
            messagebox.showinfo(
                "Robot Not Ready",
                "Wait for the robot to finish booting and homing before uploading.")
            return

        path = self.midi_files[sel[0]]
        self.is_playing = True
        self.stop_playback = False
        self.current_note_idx = -1
        self._cancel_playback_finish()
        self.play_btn.configure(state='disabled')
        self.stop_btn.configure(state='normal')
        self.progress_var.set(0)
        self.progress_label.configure(text="Uploading...")

        # Send in background thread
        solenoid_pwm_percent = self._coerce_solenoid_pwm_percent()
        thread = threading.Thread(
            target=self._send_midi_thread,
            args=(path, solenoid_pwm_percent),
            daemon=True)
        thread.start()

    def _send_midi_thread(self, path, solenoid_pwm_percent):
        try:
            with open(path, 'rb') as f:
                data = f.read()

            file_len = len(data)
            header = struct.pack('>I', file_len)

            # Pause the monitor while uploading so Windows serial I/O stays
            # stable during binary MIDI transfer.
            self.serial_pause_reads = True
            time.sleep(0.15)
            try:
                self.serial_port.reset_output_buffer()
            except Exception:
                pass
            self._write_solenoid_pwm_command(solenoid_pwm_percent)
            time.sleep(0.05)

            written = self.serial_port.write(header)
            if written != len(header):
                raise IOError(f"short header write: expected {len(header)} bytes, wrote {written}")

            chunk_size = 64
            sent = 0
            time.sleep(0.02)
            while sent < file_len:
                chunk = data[sent:sent + chunk_size]
                chunk_written = self.serial_port.write(chunk)
                if chunk_written != len(chunk):
                    raise IOError(
                        f"short data write at offset {sent}: expected {len(chunk)} bytes, "
                        f"wrote {chunk_written}"
                    )
                sent += chunk_written
                self.root.after(0, lambda p=(100 * sent / file_len): self.progress_var.set(p))
                time.sleep(0.01)

            self.serial_port.flush()
            self.serial_pause_reads = False

            self.root.after(0, lambda: self.progress_var.set(100))
            self.root.after(0, lambda: self.progress_label.configure(
                text="Upload complete. Waiting for robot..."))
            self.root.after(0, lambda: self._log(f"Sent {file_len} bytes\n", 'success'))
            fallback_ms = max(self._estimated_song_duration_ms() + 2000, 10000)
            self.root.after(0, lambda d=fallback_ms: self._schedule_playback_finish(d))

        except Exception as e:
            self.serial_pause_reads = False
            self.root.after(0, lambda: self._log(f"Send error: {e}\n", 'info'))
            self.root.after(0, lambda: self._playback_finished("Upload failed"))

    def _playback_finished(self, status_text=None):
        self._cancel_playback_finish()
        self.is_playing = False
        self.play_btn.configure(state='normal')
        self.stop_btn.configure(state='disabled')
        if status_text is not None:
            self.progress_label.configure(text=status_text)
        elif self.connected and self.firmware_ready:
            self.progress_label.configure(text="Robot ready")
        else:
            self.progress_label.configure(text="Ready")

    def _stop(self):
        self._log(
            "Stop is not implemented in firmware. Wait for the current song to finish "
            "before uploading another one.\n",
            'info'
        )
        if self.is_playing:
            self.progress_label.configure(text="Robot is still playing current song")

    def _send_home(self):
        if not self.connected:
            messagebox.showinfo("Not Connected", "Connect to the robot first.")
            return
        self._log("Home command — robot will home on next restart\n", 'info')

    # ── Piano Drawing ─────────────────────────────────────────────────────

    def _draw_piano(self):
        c = self.piano_canvas
        c.delete('all')
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 50 or h < 50:
            return

        # Layout
        margin_left = 30
        margin_right = 20
        piano_width = w - margin_left - margin_right
        piano_top = 30
        white_key_h = h - piano_top - 20
        black_key_h = white_key_h * 0.62

        # Count white keys in range C2–C5
        num_white = 22  # C2 to C5 = 3 octaves + 1 = 22 white keys
        key_w = piano_width / num_white

        # Draw white keys
        white_idx = 0
        for midi in range(MIDI_MIN, MIDI_MAX + 1):
            if is_black_key(midi):
                continue
            x = margin_left + white_idx * key_w

            # Determine fill color
            fill = COLORS['white_key']
            outline = '#cccccc'
            if midi in self.active_keys:
                fill = COLORS['key_pressed']
                outline = COLORS['accent']

            c.create_rectangle(x, piano_top, x + key_w - 1, piano_top + white_key_h,
                             fill=fill, outline=outline, width=1)

            # Key label (every C)
            if midi % 12 == 0:
                c.create_text(x + key_w / 2, piano_top + white_key_h - 10,
                            text=note_name(midi), font=('Consolas', 7),
                            fill='#666666')
            white_idx += 1

        # Draw black keys
        white_idx = 0
        for midi in range(MIDI_MIN, MIDI_MAX + 1):
            if is_black_key(midi):
                # Black key sits between two white keys; find position
                # The white key to the LEFT of this black key
                left_white_midi = midi - 1
                left_wki = white_key_index(left_white_midi)
                if left_wki < 0:
                    continue
                # Adjust relative to our starting white key
                rel_wki = left_wki - white_key_index(MIDI_MIN)
                bk_w = key_w * 0.65
                x = margin_left + (rel_wki + 1) * key_w - bk_w / 2

                fill = COLORS['black_key']
                if midi in self.active_keys:
                    fill = COLORS['key_pressed']

                c.create_rectangle(x, piano_top, x + bk_w, piano_top + black_key_h,
                                 fill=fill, outline='#333333', width=1)

        # ── Draw robot carriage ──────────────────────────────────────────
        # Convert motor position (mm) to pixel X
        total_mm = num_white * WHITE_KEY_WIDTH_MM
        def mm_to_x(mm):
            return margin_left + (mm / total_mm) * piano_width

        robot_x = mm_to_x(self.tel_pos)
        target_x = mm_to_x(self.tel_target)

        # Draw target indicator
        if abs(self.tel_target - self.tel_pos) > 1.0:
            c.create_line(target_x, 5, target_x, piano_top - 2,
                        fill=COLORS['key_target'], width=2, dash=(4, 2))
            c.create_text(target_x, 2, text="TGT", font=('Consolas', 7),
                        fill=COLORS['key_target'], anchor='s')

        # Draw finger positions on the carriage
        # W1 at robot_x, W2 at +2 keys, W3 at +4 keys
        # B1 between W1 and W2, B2 between W2 and W3
        finger_positions = {
            'W1': robot_x,
            'W2': robot_x + 2 * key_w,
            'W3': robot_x + 4 * key_w,
            'B1': robot_x + 1 * key_w,
            'B2': robot_x + 3 * key_w,
        }

        # Robot body (spans from W1 to W3)
        body_left = robot_x - 3
        body_right = robot_x + 4 * key_w + 3
        c.create_rectangle(body_left, piano_top - 18, body_right, piano_top - 4,
                         fill=COLORS['bg_card'], outline=COLORS['accent'],
                         width=2)

        # Draw each finger
        for fname, fx in finger_positions.items():
            color = FINGER_COLORS[fname]
            is_active = (fname == self.active_finger)
            r = 6 if is_active else 4
            y = piano_top - 11

            if is_active:
                # Draw press indicator line
                c.create_line(fx, y + r, fx, piano_top + 15,
                            fill=color, width=2)

            c.create_oval(fx - r, y - r, fx + r, y + r,
                        fill=color, outline='white' if is_active else color,
                        width=2 if is_active else 1)

            # Finger label
            c.create_text(fx, y, text=fname[0], font=('Consolas', 6, 'bold'),
                        fill='white')

        # Motor position text
        c.create_text(margin_left, h - 6,
                     text=f"0mm", font=('Consolas', 7),
                     fill=COLORS['text_dim'], anchor='w')
        c.create_text(margin_left + piano_width, h - 6,
                     text=f"{total_mm:.0f}mm", font=('Consolas', 7),
                     fill=COLORS['text_dim'], anchor='e')

    # ── Timeline Drawing ──────────────────────────────────────────────────

    def _draw_timeline(self):
        c = self.timeline_canvas
        c.delete('all')
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 50 or h < 30 or not self.current_notes:
            c.create_text(w/2, h/2, text="Load a MIDI file to see timeline",
                        font=('Segoe UI', 10), fill=COLORS['text_dim'])
            return

        notes = self.current_notes
        margin = 40
        plot_w = w - margin * 2
        plot_h = h - 20

        # Time range
        max_time = max(n[0] + n[2] for n in notes)
        if max_time <= 0:
            return

        # Note range
        min_note = min(n[1] for n in notes)
        max_note = max(n[1] for n in notes)
        note_range = max(max_note - min_note, 1)

        # Grid lines
        for midi in range(min_note, max_note + 1):
            y = 10 + plot_h - (midi - min_note) / note_range * plot_h
            if midi % 12 == 0:  # C notes
                c.create_line(margin, y, w - margin, y,
                            fill=COLORS['grid'], width=1)
                c.create_text(margin - 4, y, text=note_name(midi),
                            font=('Consolas', 7), fill=COLORS['text_dim'],
                            anchor='e')

        # Draw notes as rectangles
        for i, (t, midi, dur, vel) in enumerate(notes):
            x1 = margin + (t / max_time) * plot_w
            x2 = margin + ((t + dur) / max_time) * plot_w
            y_center = 10 + plot_h - (midi - min_note) / note_range * plot_h
            bar_h = max(plot_h / note_range * 0.8, 3)

            color = COLORS['accent'] if i <= self.current_note_idx else COLORS['accent2']
            if i == self.current_note_idx:
                color = COLORS['success']

            c.create_rectangle(x1, y_center - bar_h/2, max(x2, x1+2), y_center + bar_h/2,
                             fill=color, outline='', width=0)

        # Playback position line
        if self.current_note_idx >= 0 and self.current_note_idx < len(notes):
            t = notes[self.current_note_idx][0]
            x = margin + (t / max_time) * plot_w
            c.create_line(x, 5, x, h - 5, fill='white', width=2)

    # ── Logging ───────────────────────────────────────────────────────────

    def _log(self, text, tag='info'):
        self.log_text.insert('end', text, tag)
        self.log_text.see('end')
        # Keep log from growing too large
        lines = int(self.log_text.index('end-1c').split('.')[0])
        if lines > 500:
            self.log_text.delete('1.0', '100.0')

    def _clear_log(self):
        self.log_text.delete('1.0', 'end')


# ─── Entry Point ──────────────────────────────────────────────────────────────

def main():
    root = tk.Tk()

    # Try to set DPI awareness on Windows
    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass

    app = PianoRobotGUI(root)

    # Load any .mid files from the tools directory
    tools_dir = Path(__file__).parent
    for mid_file in sorted(tools_dir.glob("*.mid")):
        app.midi_files.append(str(mid_file))
        app.file_listbox.insert('end', f"  {mid_file.name}")

    root.mainloop()


if __name__ == "__main__":
    main()
