#!/usr/bin/env python3
from __future__ import annotations

import queue
import re
import sys
import threading
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ModuleNotFoundError:
    serial = None
    list_ports = None


FLOAT_RE = r"-?\d+(?:\.\d+)?"
INT_RE = r"-?\d+"
STATUS_PATTERN = re.compile(
    rf"^STATUS\s+seq=(?P<seq>{INT_RE})\s+t_ms=(?P<t_ms>{INT_RE})\s+"
    rf"kp=(?P<kp>{FLOAT_RE})\s+ki=(?P<ki>{FLOAT_RE})\s+kd=(?P<kd>{FLOAT_RE})\s+deadband_deg=(?P<deadband>{FLOAT_RE})\s+settle_vel_deg_per_s=(?P<settle_velocity>{FLOAT_RE})\s+"
    rf"target_deg=(?P<target>{FLOAT_RE})\s+angle_deg=(?P<angle>{FLOAT_RE})\s+"
    rf"error_deg=(?P<error>{FLOAT_RE})\s+output=(?P<output>{FLOAT_RE})\s+counts=(?P<counts>{INT_RE})$"
)


class PidTunerApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("PianoBot PID Tuner")

        self.serial_port: serial.Serial | None = None
        self.reader_thread: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.rx_queue: queue.Queue[str] = queue.Queue()

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.status_var = tk.StringVar(value="Disconnected")

        self.kp_input_var = tk.StringVar(value="0.005")
        self.ki_input_var = tk.StringVar(value="0")
        self.kd_input_var = tk.StringVar(value="0.000005")
        self.deadband_input_var = tk.StringVar(value="8")
        self.settle_velocity_input_var = tk.StringVar(value="1000")
        self.target_input_var = tk.StringVar(value="1800")

        self.kp_live_var = tk.StringVar(value="0.0")
        self.ki_live_var = tk.StringVar(value="0.0")
        self.kd_live_var = tk.StringVar(value="0.0")
        self.deadband_live_var = tk.StringVar(value="0.0")
        self.settle_velocity_live_var = tk.StringVar(value="0.0")
        self.target_live_var = tk.StringVar(value="0.0")
        self.angle_live_var = tk.StringVar(value="0.0")
        self.error_live_var = tk.StringVar(value="0.0")
        self.output_live_var = tk.StringVar(value="0.0")
        self.counts_live_var = tk.StringVar(value="0")

        self._build_ui()
        self.refresh_ports()
        self.root.after(50, self._poll_rx_queue)

    def _build_ui(self) -> None:
        frame = ttk.Frame(self.root, padding=12)
        frame.grid(sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        frame.columnconfigure(1, weight=1)
        frame.rowconfigure(5, weight=1)

        ttk.Label(frame, text="Port").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=(0, 8))
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", pady=(0, 8))
        ttk.Button(frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=(8, 0), pady=(0, 8))

        ttk.Label(frame, text="Baud").grid(row=1, column=0, sticky="w", padx=(0, 8), pady=(0, 8))
        ttk.Entry(frame, textvariable=self.baud_var, width=12).grid(row=1, column=1, sticky="w", pady=(0, 8))

        self.connect_button = ttk.Button(frame, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=1, column=2, padx=(8, 0), pady=(0, 8))

        controls = ttk.LabelFrame(frame, text="Controls", padding=12)
        controls.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(4, 8))
        for idx in range(6):
            controls.columnconfigure(idx * 2 + 1, weight=1)

        ttk.Label(controls, text="Kp").grid(row=0, column=0, sticky="w")
        ttk.Entry(controls, textvariable=self.kp_input_var, width=12).grid(row=0, column=1, sticky="ew", padx=(0, 8))
        ttk.Label(controls, text="Ki").grid(row=0, column=2, sticky="w")
        ttk.Entry(controls, textvariable=self.ki_input_var, width=12).grid(row=0, column=3, sticky="ew", padx=(0, 8))
        ttk.Label(controls, text="Kd").grid(row=0, column=4, sticky="w")
        ttk.Entry(controls, textvariable=self.kd_input_var, width=12).grid(row=0, column=5, sticky="ew", padx=(0, 8))
        ttk.Label(controls, text="Deadband").grid(row=0, column=6, sticky="w")
        ttk.Entry(controls, textvariable=self.deadband_input_var, width=12).grid(row=0, column=7, sticky="ew", padx=(0, 8))
        ttk.Label(controls, text="Settle Vel").grid(row=0, column=8, sticky="w")
        ttk.Entry(controls, textvariable=self.settle_velocity_input_var, width=12).grid(row=0, column=9, sticky="ew", padx=(0, 8))
        ttk.Button(controls, text="Apply PID", command=self.apply_pid).grid(row=0, column=10, columnspan=2, sticky="ew")

        ttk.Label(controls, text="Target (deg)").grid(row=1, column=0, sticky="w", pady=(10, 0))
        ttk.Entry(controls, textvariable=self.target_input_var, width=12).grid(row=1, column=1, sticky="ew", padx=(0, 8), pady=(10, 0))
        ttk.Button(controls, text="Apply Target", command=self.apply_target).grid(row=1, column=2, columnspan=3, sticky="ew", padx=(0, 8), pady=(10, 0))
        ttk.Button(controls, text="Apply Deadband", command=self.apply_deadband).grid(row=1, column=5, columnspan=3, sticky="ew", padx=(0, 8), pady=(10, 0))
        ttk.Button(controls, text="Apply Settle Vel", command=self.apply_settle_velocity).grid(row=1, column=8, columnspan=2, sticky="ew", padx=(0, 8), pady=(10, 0))
        ttk.Button(controls, text="Zero Position", command=self.zero_position).grid(row=1, column=10, columnspan=2, sticky="ew", pady=(10, 0))

        live = ttk.LabelFrame(frame, text="Live Diagnostics", padding=12)
        live.grid(row=3, column=0, columnspan=3, sticky="ew", pady=(0, 8))
        for idx in range(4):
            live.columnconfigure(idx, weight=1)

        ttk.Label(live, text="Live Kp").grid(row=0, column=0, sticky="w")
        ttk.Label(live, textvariable=self.kp_live_var).grid(row=0, column=1, sticky="w")
        ttk.Label(live, text="Live Ki").grid(row=0, column=2, sticky="w")
        ttk.Label(live, textvariable=self.ki_live_var).grid(row=0, column=3, sticky="w")

        ttk.Label(live, text="Live Kd").grid(row=1, column=0, sticky="w", pady=(8, 0))
        ttk.Label(live, textvariable=self.kd_live_var).grid(row=1, column=1, sticky="w", pady=(8, 0))
        ttk.Label(live, text="Settle Vel").grid(row=1, column=2, sticky="w", pady=(8, 0))
        ttk.Label(live, textvariable=self.settle_velocity_live_var).grid(row=1, column=3, sticky="w", pady=(8, 0))

        ttk.Label(live, text="Target").grid(row=2, column=0, sticky="w", pady=(8, 0))
        ttk.Label(live, textvariable=self.target_live_var).grid(row=2, column=1, sticky="w", pady=(8, 0))
        ttk.Label(live, text="Angle").grid(row=2, column=2, sticky="w", pady=(8, 0))
        ttk.Label(live, textvariable=self.angle_live_var).grid(row=2, column=3, sticky="w", pady=(8, 0))

        ttk.Label(live, text="Error").grid(row=3, column=0, sticky="w", pady=(8, 0))
        ttk.Label(live, textvariable=self.error_live_var).grid(row=3, column=1, sticky="w", pady=(8, 0))
        ttk.Label(live, text="Output").grid(row=3, column=2, sticky="w", pady=(8, 0))
        ttk.Label(live, textvariable=self.output_live_var).grid(row=3, column=3, sticky="w", pady=(8, 0))

        ttk.Label(live, text="Counts").grid(row=4, column=0, sticky="w", pady=(8, 0))
        ttk.Label(live, textvariable=self.counts_live_var).grid(row=4, column=1, sticky="w", pady=(8, 0))
        ttk.Label(live, text="Deadband").grid(row=4, column=2, sticky="w", pady=(8, 0))
        ttk.Label(live, textvariable=self.deadband_live_var).grid(row=4, column=3, sticky="w", pady=(8, 0))

        ttk.Label(frame, textvariable=self.status_var).grid(row=4, column=0, columnspan=3, sticky="w", pady=(0, 8))

        self.log_text = tk.Text(frame, height=16, width=100, state="disabled")
        self.log_text.grid(row=5, column=0, columnspan=3, sticky="nsew")

    def refresh_ports(self) -> None:
        if list_ports is None:
            self.port_combo["values"] = []
            return

        ports = [port.device for port in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def toggle_connection(self) -> None:
        if self.serial_port is None:
            self.connect()
        else:
            self.disconnect()

    def connect(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("PID Tuner", "Select a serial port.")
            return

        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showerror("PID Tuner", "Baud rate must be an integer.")
            return

        try:
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
        except Exception as exc:
            messagebox.showerror("PID Tuner", f"Failed to open {port}: {exc}")
            self.serial_port = None
            return

        self.stop_event.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.connect_button.config(text="Disconnect")
        self.status_var.set(f"Connected to {port} @ {baud}")

    def disconnect(self) -> None:
        self.stop_event.set()
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None
        self.connect_button.config(text="Connect")
        self.status_var.set("Disconnected")

    def _reader_loop(self) -> None:
        assert self.serial_port is not None
        while not self.stop_event.is_set():
            try:
                raw = self.serial_port.readline()
            except Exception as exc:
                self.rx_queue.put(f"ERR serial_read {exc}")
                break

            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                self.rx_queue.put(line)

    def _poll_rx_queue(self) -> None:
        while True:
            try:
                line = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_line(line)

        self.root.after(50, self._poll_rx_queue)

    def _handle_line(self, line: str) -> None:
        status_match = STATUS_PATTERN.match(line)
        if status_match:
            self.kp_live_var.set(status_match.group("kp"))
            self.ki_live_var.set(status_match.group("ki"))
            self.kd_live_var.set(status_match.group("kd"))
            self.deadband_live_var.set(status_match.group("deadband"))
            self.settle_velocity_live_var.set(status_match.group("settle_velocity"))
            self.target_live_var.set(status_match.group("target"))
            self.angle_live_var.set(status_match.group("angle"))
            self.error_live_var.set(status_match.group("error"))
            self.output_live_var.set(status_match.group("output"))
            self.counts_live_var.set(status_match.group("counts"))
            self.status_var.set(
                "Live: "
                f"target {status_match.group('target')} deg | "
                f"deadband {status_match.group('deadband')} deg | "
                f"settle vel {status_match.group('settle_velocity')} deg/s | "
                f"angle {status_match.group('angle')} deg | "
                f"error {status_match.group('error')} deg | "
                f"output {status_match.group('output')} | "
                f"counts {status_match.group('counts')}"
            )

        if line.startswith("READY "):
            self.status_var.set(line)
        elif line.startswith("OK "):
            self.status_var.set(line)
        elif line.startswith("ERR "):
            self.status_var.set(line)

        self._append_log(line)

    def _append_log(self, line: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert("end", line + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _send_line(self, line: str) -> None:
        if self.serial_port is None:
            messagebox.showerror("PID Tuner", "Not connected.")
            return

        try:
            self.serial_port.write((line + "\n").encode("utf-8"))
        except Exception as exc:
            messagebox.showerror("PID Tuner", f"Failed to send command: {exc}")
            self.disconnect()

    def apply_pid(self) -> None:
        try:
            kp = float(self.kp_input_var.get().strip())
            ki = float(self.ki_input_var.get().strip())
            kd = float(self.kd_input_var.get().strip())
        except ValueError:
            messagebox.showerror("PID Tuner", "Kp, Ki, and Kd must be valid numbers.")
            return

        self._send_line(f"PID {kp:.6f} {ki:.6f} {kd:.6f}")

    def apply_target(self) -> None:
        try:
            target_deg = float(self.target_input_var.get().strip())
        except ValueError:
            messagebox.showerror("PID Tuner", "Target angle must be a valid number.")
            return

        self._send_line(f"TARGET {target_deg:.3f}")

    def apply_deadband(self) -> None:
        try:
            deadband_deg = float(self.deadband_input_var.get().strip())
        except ValueError:
            messagebox.showerror("PID Tuner", "Deadband must be a valid number.")
            return

        self._send_line(f"DEADBAND {deadband_deg:.3f}")

    def apply_settle_velocity(self) -> None:
        try:
            settle_velocity_deg_per_sec = float(self.settle_velocity_input_var.get().strip())
        except ValueError:
            messagebox.showerror("PID Tuner", "Settle velocity must be a valid number.")
            return

        self._send_line(f"SETTLEVEL {settle_velocity_deg_per_sec:.3f}")

    def zero_position(self) -> None:
        self._send_line("ZERO")


def main() -> int:
    if serial is None:
        print("pyserial is not installed. Install it with:", file=sys.stderr)
        print(f"  \"{sys.executable}\" -m pip install pyserial", file=sys.stderr)
        return 2

    if not hasattr(serial, "Serial"):
        print(
            "The imported 'serial' module is not pyserial. "
            "Uninstall package 'serial' and install 'pyserial'.",
            file=sys.stderr,
        )
        print(f"Imported module path: {getattr(serial, '__file__', 'unknown')}", file=sys.stderr)
        return 2

    root = tk.Tk()
    app = PidTunerApp(root)

    def on_close() -> None:
        app.disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
