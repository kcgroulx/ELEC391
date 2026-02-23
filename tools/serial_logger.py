#!/usr/bin/env python3
import argparse
import re
import sys
from datetime import datetime

import serial
try:
    from serial.serialutil import SerialException
except Exception:  # fallback for unusual serial module layouts
    SerialException = Exception


COUNT_PATTERN = re.compile(r"enc_position_counts\s*=\s*(-?\d+)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read encoder counts from a serial port, print to terminal, and save to a text file."
    )
    parser.add_argument("--port", required=True, help="Serial port (example: COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--out", default="encoder_counts.txt", help="Output text file path")
    parser.add_argument("--timeout", type=float, default=1.0, help="Serial read timeout in seconds")
    return parser.parse_args()


def extract_value(line: str) -> str:
    match = COUNT_PATTERN.search(line)
    if match:
        return match.group(1)
    return line.strip()


def main() -> int:
    args = parse_args()

    if not hasattr(serial, "Serial"):
        print(
            "The imported 'serial' module is not pyserial. "
            "Uninstall package 'serial' and install 'pyserial'.",
            file=sys.stderr,
        )
        print(f"Imported module path: {getattr(serial, '__file__', 'unknown')}", file=sys.stderr)
        return 2

    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    except SerialException as exc:
        print(f"Failed to open serial port {args.port}: {exc}", file=sys.stderr)
        return 1

    print(f"Listening on {args.port} @ {args.baud} baud")
    print(f"Saving to: {args.out}")

    try:
        with ser, open(args.out, "a", encoding="utf-8") as out_file:
            while True:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                value = extract_value(line)
                timestamp = datetime.now().isoformat(timespec="milliseconds")
                log_line = f"{timestamp}\t{value}"

                print(value)
                out_file.write(log_line + "\n")
                out_file.flush()
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
