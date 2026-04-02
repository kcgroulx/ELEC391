#!/usr/bin/env python3
"""
PID and solenoid control GUI for the ESP32 piano robot.

Requires:
    pip install pyserial

Usage:
    python tools/pid_solenoid_gui.py
"""

from __future__ import annotations

import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

import serial
import serial.tools.list_ports


BAUD_RATE = 115200
SOLENOID_COUNT = 5
TARGET_MM_MIN = -500.0
TARGET_MM_MAX = 500.0


def find_esp32_port() -> str | None:
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        description = (port.description or "").lower()
        if any(token in description for token in ("esp32", "cp210", "usb serial", "uart")):
            return port.device
    if ports:
        return ports[0].device
    return None


class PianoRobotGui:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Piano Robot Control")
        self.root.geometry("960x780")

        self.serial_port: serial.Serial | None = None
        self.reader_thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.rx_queue: queue.Queue[tuple[str, str]] = queue.Queue()

        self.port_var = tk.StringVar()
        self.connection_var = tk.StringVar(value="Disconnected")
        self.target_entry_var = tk.StringVar(value="0.00")
        self.target_scale_var = tk.DoubleVar(value=0.0)
        self.manual_speed_entry_var = tk.StringVar(value="0.00")
        self.manual_speed_scale_var = tk.DoubleVar(value=0.0)

        self.status_mode_var = tk.StringVar(value="PID")
        self.status_target_var = tk.StringVar(value="0.00")
        self.status_actual_var = tk.StringVar(value="0.00")
        self.status_actual_deg_var = tk.StringVar(value="0.0")
        self.status_manual_speed_var = tk.StringVar(value="0.00")
        self.status_encoder_var = tk.StringVar(value="0")
        self.status_arrived_var = tk.StringVar(value="0")
        self.status_fingers_var = tk.StringVar(value="0x00")

        self.solenoid_vars = [tk.BooleanVar(value=False) for _ in range(SOLENOID_COUNT)]
        self.control_widgets: list[tk.Widget] = []

        self._build_ui()
        self.refresh_ports(select_auto=True)
        self.set_connected_state(False)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(50, self.process_incoming)

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(5, weight=1)

        connection_frame = ttk.LabelFrame(self.root, text="Connection", padding=12)
        connection_frame.grid(row=0, column=0, sticky="ew", padx=12, pady=(12, 6))
        connection_frame.columnconfigure(1, weight=1)

        ttk.Label(connection_frame, text="Port").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(connection_frame, textvariable=self.port_var, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=(8, 8))

        refresh_button = ttk.Button(connection_frame, text="Refresh", command=self.refresh_ports)
        refresh_button.grid(row=0, column=2, padx=(0, 8))

        self.connect_button = ttk.Button(connection_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3)

        ttk.Label(connection_frame, textvariable=self.connection_var).grid(
            row=0, column=4, sticky="e", padx=(12, 0)
        )

        position_frame = ttk.LabelFrame(self.root, text="PID Position", padding=12)
        position_frame.grid(row=1, column=0, sticky="ew", padx=12, pady=6)
        position_frame.columnconfigure(1, weight=1)

        ttk.Label(position_frame, text="Target mm").grid(row=0, column=0, sticky="w")
        self.target_entry = ttk.Entry(position_frame, textvariable=self.target_entry_var, width=12)
        self.target_entry.grid(row=0, column=1, sticky="w", padx=(8, 8))
        self.control_widgets.append(self.target_entry)

        move_button = ttk.Button(position_frame, text="Move", command=self.send_target)
        move_button.grid(row=0, column=2, padx=(0, 8))
        self.control_widgets.append(move_button)

        zero_button = ttk.Button(position_frame, text="Zero", command=self.zero_position)
        zero_button.grid(row=0, column=3, padx=(0, 8))
        self.control_widgets.append(zero_button)

        status_button = ttk.Button(position_frame, text="Request Status", command=lambda: self.send_command("STATUS?"))
        status_button.grid(row=0, column=4)
        self.control_widgets.append(status_button)

        self.target_scale = tk.Scale(
            position_frame,
            from_=TARGET_MM_MIN,
            to=TARGET_MM_MAX,
            resolution=0.5,
            orient=tk.HORIZONTAL,
            variable=self.target_scale_var,
            command=self.on_scale_change,
        )
        self.target_scale.grid(row=1, column=0, columnspan=5, sticky="ew", pady=(10, 0))
        self.control_widgets.append(self.target_scale)

        motor_frame = ttk.LabelFrame(self.root, text="Motor Drive", padding=12)
        motor_frame.grid(row=2, column=0, sticky="ew", padx=12, pady=6)
        motor_frame.columnconfigure(1, weight=1)

        ttk.Label(motor_frame, text="Speed").grid(row=0, column=0, sticky="w")
        self.manual_speed_entry = ttk.Entry(motor_frame, textvariable=self.manual_speed_entry_var, width=12)
        self.manual_speed_entry.grid(row=0, column=1, sticky="w", padx=(8, 8))
        self.control_widgets.append(self.manual_speed_entry)

        manual_apply_button = ttk.Button(motor_frame, text="Apply", command=self.send_manual_speed)
        manual_apply_button.grid(row=0, column=2, padx=(0, 8))
        self.control_widgets.append(manual_apply_button)

        manual_stop_button = ttk.Button(motor_frame, text="Stop", command=self.stop_manual_motor)
        manual_stop_button.grid(row=0, column=3)
        self.control_widgets.append(manual_stop_button)

        self.manual_speed_scale = tk.Scale(
            motor_frame,
            from_=-1.0,
            to=1.0,
            resolution=0.01,
            orient=tk.HORIZONTAL,
            variable=self.manual_speed_scale_var,
            command=self.on_manual_speed_scale_change,
        )
        self.manual_speed_scale.grid(row=1, column=0, columnspan=4, sticky="ew", pady=(10, 0))
        self.control_widgets.append(self.manual_speed_scale)

        solenoid_frame = ttk.LabelFrame(self.root, text="Solenoids", padding=12)
        solenoid_frame.grid(row=3, column=0, sticky="ew", padx=12, pady=6)

        for index in range(SOLENOID_COUNT):
            checkbutton = ttk.Checkbutton(
                solenoid_frame,
                text=f"Solenoid {index + 1}",
                variable=self.solenoid_vars[index],
                command=lambda idx=index: self.toggle_solenoid(idx),
            )
            checkbutton.grid(row=0, column=index, padx=(0, 12), sticky="w")
            self.control_widgets.append(checkbutton)

        all_off_button = ttk.Button(solenoid_frame, text="All Off", command=self.all_solenoids_off)
        all_off_button.grid(row=0, column=SOLENOID_COUNT, sticky="e")
        self.control_widgets.append(all_off_button)

        status_frame = ttk.LabelFrame(self.root, text="Live Status", padding=12)
        status_frame.grid(row=4, column=0, sticky="ew", padx=12, pady=6)

        fields = (
            ("Mode", self.status_mode_var),
            ("Target mm", self.status_target_var),
            ("Actual mm", self.status_actual_var),
            ("Actual deg", self.status_actual_deg_var),
            ("Motor cmd", self.status_manual_speed_var),
            ("Encoder", self.status_encoder_var),
            ("Arrived", self.status_arrived_var),
            ("Fingers", self.status_fingers_var),
        )
        for column, (label, variable) in enumerate(fields):
            ttk.Label(status_frame, text=label).grid(row=0, column=column, sticky="w", padx=(0, 8))
            ttk.Label(status_frame, textvariable=variable, width=12).grid(
                row=1, column=column, sticky="w", padx=(0, 16)
            )

        log_frame = ttk.LabelFrame(self.root, text="Serial Log", padding=12)
        log_frame.grid(row=5, column=0, sticky="nsew", padx=12, pady=(6, 12))
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)

        self.log_box = scrolledtext.ScrolledText(log_frame, height=18, state="disabled")
        self.log_box.grid(row=0, column=0, sticky="nsew")

    def log(self, message: str) -> None:
        self.log_box.configure(state="normal")
        self.log_box.insert(tk.END, f"{message}\n")
        self.log_box.see(tk.END)
        self.log_box.configure(state="disabled")

    def refresh_ports(self, select_auto: bool = False) -> None:
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports

        current = self.port_var.get().strip()
        if current and current in ports:
            self.port_var.set(current)
            return

        if select_auto:
            auto_port = find_esp32_port()
            if auto_port:
                self.port_var.set(auto_port)
                return

        if ports:
            self.port_var.set(ports[0])
        else:
            self.port_var.set("")

    def set_connected_state(self, connected: bool) -> None:
        state = "normal" if connected else "disabled"
        for widget in self.control_widgets:
            widget.configure(state=state)

        self.connect_button.configure(text="Disconnect" if connected else "Connect")
        self.port_combo.configure(state="disabled" if connected else "readonly")
        self.connection_var.set("Connected" if connected else "Disconnected")

    def toggle_connection(self) -> None:
        if self.serial_port is None:
            self.connect()
        else:
            self.disconnect()

    def connect(self) -> None:
        port = self.port_var.get().strip() or find_esp32_port()
        if not port:
            messagebox.showerror("No Port", "No serial port found.")
            return

        try:
            self.serial_port = serial.Serial(port, BAUD_RATE, timeout=0.1)
        except serial.SerialException as exc:
            messagebox.showerror("Connection Failed", str(exc))
            return

        self.stop_event.clear()
        self.reader_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.reader_thread.start()
        time.sleep(0.2)

        self.log(f"Connected to {port} @ {BAUD_RATE}")
        self.set_connected_state(True)
        self.send_command("STATUS?")

    def disconnect(self) -> None:
        self.stop_event.set()

        if self.reader_thread is not None and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=0.5)
        self.reader_thread = None

        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except serial.SerialException:
                pass

        self.serial_port = None
        self.set_connected_state(False)
        self.log("Disconnected")

    def read_serial_loop(self) -> None:
        assert self.serial_port is not None
        while not self.stop_event.is_set():
            try:
                line = self.serial_port.readline()
            except serial.SerialException as exc:
                self.rx_queue.put(("error", str(exc)))
                break

            if not line:
                continue

            decoded = line.decode("utf-8", errors="replace").strip()
            if decoded:
                self.rx_queue.put(("line", decoded))

    def process_incoming(self) -> None:
        while not self.rx_queue.empty():
            kind, payload = self.rx_queue.get()
            if kind == "line":
                self.handle_line(payload)
            elif kind == "error":
                self.log(f"Serial error: {payload}")
                self.disconnect()
                break

        self.root.after(50, self.process_incoming)

    def handle_line(self, line: str) -> None:
        self.log(f"< {line}")

        if line.startswith("STATUS "):
            self.update_status(line)

    def update_status(self, line: str) -> None:
        values: dict[str, str] = {}
        for token in line.split()[1:]:
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            values[key] = value

        if "mode" in values:
            self.status_mode_var.set(values["mode"])
        if "target_mm" in values:
            self.status_target_var.set(values["target_mm"])
        if "actual_mm" in values:
            self.status_actual_var.set(values["actual_mm"])
        if "actual_deg" in values:
            self.status_actual_deg_var.set(values["actual_deg"])
        if "manual_speed" in values:
            self.status_manual_speed_var.set(values["manual_speed"])
            try:
                manual_speed = float(values["manual_speed"])
            except ValueError:
                manual_speed = None
            if manual_speed is not None:
                self.manual_speed_scale_var.set(manual_speed)
                self.manual_speed_entry_var.set(f"{manual_speed:.2f}")
        if "encoder" in values:
            self.status_encoder_var.set(values["encoder"])
        if "arrived" in values:
            self.status_arrived_var.set(values["arrived"])
        if "fingers" in values:
            self.status_fingers_var.set(values["fingers"])
            self.update_solenoid_checks(values["fingers"])

    def update_solenoid_checks(self, fingers_value: str) -> None:
        try:
            mask = int(fingers_value, 16)
        except ValueError:
            return

        for index in range(SOLENOID_COUNT):
            self.solenoid_vars[index].set(bool(mask & (1 << index)))

    def on_scale_change(self, value: str) -> None:
        self.target_entry_var.set(f"{float(value):.2f}")

    def on_manual_speed_scale_change(self, value: str) -> None:
        self.manual_speed_entry_var.set(f"{float(value):.2f}")

    def set_target_value(self, target_mm: float) -> None:
        self.target_scale_var.set(target_mm)
        self.target_entry_var.set(f"{target_mm:.2f}")
        self.send_target()

    def zero_position(self) -> None:
        self.target_scale_var.set(0.0)
        self.target_entry_var.set("0.00")
        self.send_command("ZERO")

    def send_target(self) -> None:
        try:
            target_mm = float(self.target_entry_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid Target", "Target millimeters must be a number.")
            return

        self.target_scale_var.set(target_mm)
        self.send_command(f"TARGET_MM {target_mm:.2f}")

    def send_manual_speed(self) -> None:
        try:
            motor_speed = float(self.manual_speed_entry_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid Speed", "Motor speed must be a number between -1.0 and 1.0.")
            return

        self.manual_speed_scale_var.set(motor_speed)
        self.send_command(f"MOTOR {motor_speed:.2f}")

    def stop_manual_motor(self) -> None:
        self.manual_speed_scale_var.set(0.0)
        self.manual_speed_entry_var.set("0.00")
        self.send_command("MOTOR 0.00")

    def toggle_solenoid(self, index: int) -> None:
        state = 1 if self.solenoid_vars[index].get() else 0
        self.send_command(f"SOL {index + 1} {state}")

    def all_solenoids_off(self) -> None:
        for variable in self.solenoid_vars:
            variable.set(False)
        self.send_command("ALL_OFF")

    def send_command(self, command: str) -> None:
        if self.serial_port is None:
            return

        try:
            self.serial_port.write((command + "\n").encode("utf-8"))
            self.serial_port.flush()
        except serial.SerialException as exc:
            self.log(f"Serial write failed: {exc}")
            self.disconnect()
            return

        self.log(f"> {command}")

    def on_close(self) -> None:
        self.disconnect()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    style = ttk.Style(root)
    if "vista" in style.theme_names():
        style.theme_use("vista")
    app = PianoRobotGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
