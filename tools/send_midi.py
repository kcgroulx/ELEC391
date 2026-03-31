#!/usr/bin/env python3
"""
send_midi.py
============
Sends a MIDI file to the ESP32 piano robot over UART.

USAGE:
    python send_midi.py mysong.mid
    python send_midi.py mysong.mid --port COM4          (Windows)
    python send_midi.py mysong.mid --port /dev/ttyUSB0  (Linux/Mac)
    python send_midi.py mysong.mid --port COM4 --baud 115200

PROTOCOL:
    Sends a 4-byte big-endian file length, then the raw MIDI bytes.
    The ESP32 Midi_receiveUART() function expects exactly this format.

REQUIREMENTS:
    pip install pyserial
"""

import argparse
import serial
import serial.tools.list_ports
import struct
import sys
import time


def find_esp32_port():
    """Try to auto-detect the ESP32 UART port."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        if any(x in desc for x in ["esp32", "cp210", "usb serial", "uart"]):
            return p.device
    # Fall back to first available port
    if ports:
        return ports[0].device
    return None


def send_midi(midi_path: str, port: str, baud: int) -> bool:
    # Read the MIDI file
    try:
        with open(midi_path, "rb") as f:
            midi_data = f.read()
    except FileNotFoundError:
        print(f"Error: file not found: {midi_path}")
        return False

    # Validate it looks like a MIDI file
    if not midi_data.startswith(b"MThd"):
        print("Warning: file does not start with MThd — may not be a valid MIDI file")

    file_len = len(midi_data)
    print(f"File:  {midi_path}")
    print(f"Size:  {file_len} bytes")
    print(f"Port:  {port}  @  {baud} baud")

    # Open serial port
    try:
        ser = serial.Serial(port, baud, timeout=5)
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        return False

    time.sleep(0.1)  # let port settle

    # Send 4-byte big-endian length header
    header = struct.pack(">I", file_len)
    ser.write(header)

    # Send MIDI data in chunks so progress is visible
    chunk_size = 256
    sent = 0
    print("Sending ", end="", flush=True)
    while sent < file_len:
        chunk = midi_data[sent:sent + chunk_size]
        ser.write(chunk)
        sent += len(chunk)
        print(".", end="", flush=True)

    ser.flush()
    print(f"\nDone — sent {sent} bytes")

    # Optional: read back any response from the ESP32 (debug UART output)
    time.sleep(0.5)
    if ser.in_waiting:
        response = ser.read(ser.in_waiting).decode("utf-8", errors="replace")
        print(f"\nESP32 response:\n{response}")

    ser.close()
    return True


def main():
    parser = argparse.ArgumentParser(description="Send MIDI file to ESP32 piano robot")
    parser.add_argument("midi_file", help="Path to .mid file")
    parser.add_argument("--port",  default=None,    help="Serial port (auto-detected if omitted)")
    parser.add_argument("--baud",  default=115200,  type=int, help="Baud rate (default 115200)")
    args = parser.parse_args()

    port = args.port
    if port is None:
        port = find_esp32_port()
        if port is None:
            print("Error: no serial port found. Specify with --port COM5 or --port /dev/ttyUSB0")
            sys.exit(1)
        print(f"Auto-detected port: {port}")

    ok = send_midi(args.midi_file, port, args.baud)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
