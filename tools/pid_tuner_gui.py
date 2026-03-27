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


PID_PATTERN = re.compile(
    r"^(?:OK\s+)?PID\s+kp=(?P<kp>-?\d+(?:\.\d+)?)\s+ki=(?P<ki>-?\d+(?:\.\d+)?)\s+kd=(?P<kd>-?\d+(?:\.\d+)?)$"
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
        self.kp_var = tk.StringVar()
        self.ki_var = tk.StringVar()
        self.kd_var = tk.StringVar()

        self._build_ui()
        self.refresh_ports()
        self.root.after(100, self._poll_rx_queue)

    def _build_ui(self) -> None:
        frame = ttk.Frame(self.root, padding=12)
        frame.grid(sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        frame.columnconfigure(1, weight=1)
        frame.rowconfigure(4, weight=1)

        ttk.Label(frame, text="Port").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=(0, 8))
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, state="readonly")
        self.port_combo.grid(row=0, column=1, sticky="ew", pady=(0, 8))
        ttk.Button(frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=(8, 0), pady=(0, 8))

        ttk.Label(frame, text="Baud").grid(row=1, column=0, sticky="w", padx=(0, 8), pady=(0, 8))
        ttk.Entry(frame, textvariable=self.baud_var, width=12).grid(row=1, column=1, sticky="w", pady=(0, 8))

        self.connect_button = ttk.Button(frame, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=1, column=2, padx=(8, 0), pady=(0, 8))

        gains = ttk.LabelFrame(frame, text="PID Gains", padding=12)
        gains.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(4, 8))
        for idx in range(3):
            gains.columnconfigure(idx * 2 + 1, weight=1)

        ttk.Label(gains, text="Kp").grid(row=0, column=0, sticky="w")
        ttk.Entry(gains, textvariable=self.kp_var, width=12).grid(row=0, column=1, sticky="ew", padx=(0, 8))
        ttk.Label(gains, text="Ki").grid(row=0, column=2, sticky="w")
        ttk.Entry(gains, textvariable=self.ki_var, width=12).grid(row=0, column=3, sticky="ew", padx=(0, 8))
        ttk.Label(gains, text="Kd").grid(row=0, column=4, sticky="w")
        ttk.Entry(gains, textvariable=self.kd_var, width=12).grid(row=0, column=5, sticky="ew")

        ttk.Button(gains, text="Read From Device", command=self.request_pid).grid(row=1, column=0, columnspan=3, sticky="ew", pady=(10, 0), padx=(0, 8))
        ttk.Button(gains, text="Apply", command=self.apply_pid).grid(row=1, column=3, columnspan=3, sticky="ew", pady=(10, 0))

        ttk.Label(frame, textvariable=self.status_var).grid(row=3, column=0, columnspan=3, sticky="w", pady=(0, 8))

        self.log_text = tk.Text(frame, height=18, width=88, state="disabled")
        self.log_text.grid(row=4, column=0, columnspan=3, sticky="nsew")

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
        self.request_pid()

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
        self.root.after(100, self._poll_rx_queue)

    def _handle_line(self, line: str) -> None:
        match = PID_PATTERN.match(line)
        if match:
            self.kp_var.set(match.group("kp"))
            self.ki_var.set(match.group("ki"))
            self.kd_var.set(match.group("kd"))
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

    def request_pid(self) -> None:
        self._send_line("PID?")

    def apply_pid(self) -> None:
        try:
            kp = float(self.kp_var.get().strip())
            ki = float(self.ki_var.get().strip())
            kd = float(self.kd_var.get().strip())
        except ValueError:
            messagebox.showerror("PID Tuner", "Kp, Ki, and Kd must be valid numbers.")
            return
        self._send_line(f"PID {kp:.6f} {ki:.6f} {kd:.6f}")


def main() -> int:
    if serial is None:
        print("pyserial is not installed. Install it with:", file=sys.stderr)
        print(
            f"  \"{sys.executable}\" -m pip install pyserial",
            file=sys.stderr,
        )
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
