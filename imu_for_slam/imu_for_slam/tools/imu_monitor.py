#!/usr/bin/env python3
"""Decode text/HEX IMU packets, calibrate display values, and optionally save raw/paired SI samples."""
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
import struct
import sys
import time

PAYLOAD = struct.Struct("<BBII7fH")
FRAME_SIZE = 42
TEXT_PREFIXES = (b"@IMU,", b"@STATUS,", b"@ERROR,", b"@PONG")
CSV_FIELDS = ("host_time_ns", "sequence", "timestamp_us", "ax_mps2", "ay_mps2",
              "az_mps2", "gx_rads", "gy_rads", "gz_rads", "temperature_c")


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Sample:
    sequence: int
    timestamp_us: int
    values: tuple[float, ...]
    mode: str

    def csv_row(self) -> dict:
        return dict(zip(CSV_FIELDS, (time.time_ns(), self.sequence, self.timestamp_us, *self.values)))


class StreamDecoder:
    """Fixed-length binary framing tolerates FF/FE inside the payload."""
    def __init__(self):
        self.buffer = bytearray()
        self.bad_frames = 0
        self.diagnostics: list[str] = []

    def _valid_binary(self, start: int) -> bool:
        if len(self.buffer) < start + FRAME_SIZE:
            return False
        payload = bytes(self.buffer[start+1:start+41])
        return (self.buffer[start] == 0xFF and self.buffer[start+41] == 0xFE
                and payload[:2] == b"\x01\x01"
                and crc16(payload[:38]) == int.from_bytes(payload[38:40], "little"))

    def feed(self, data: bytes) -> list[Sample]:
        self.buffer.extend(data)
        samples = []
        while self.buffer:
            positions = [p for marker in (b"@", b"\xff")
                         if (p := self.buffer.find(marker)) >= 0]
            if not positions:
                self.buffer.clear()
                break
            start = min(positions)
            if start:
                del self.buffer[:start]
            if self.buffer[0] == 0xFF:
                if len(self.buffer) < FRAME_SIZE:
                    break
                payload = bytes(self.buffer[1:41])
                if not self._valid_binary(0):
                    self.bad_frames += 1
                    del self.buffer[0]  # Slide to another candidate after loss/corruption.
                    continue
                fields = PAYLOAD.unpack(payload)
                values = tuple(fields[4:11])
                del self.buffer[:FRAME_SIZE]
                if all(math.isfinite(v) for v in values):
                    samples.append(Sample(fields[2], fields[3], values, "hex"))
                else:
                    self.bad_frames += 1
            else:
                if not any(self.buffer.startswith(prefix) or prefix.startswith(self.buffer)
                           for prefix in TEXT_PREFIXES):
                    del self.buffer[0]
                    continue
                end = self.buffer.find(b"\n")
                # A reset or dropped byte can truncate a diagnostic before HEX data.
                # Prefer a complete CRC-valid binary frame over an embedded LF.
                candidate = self.buffer.find(b"\xff", 1)
                recovered = False
                while candidate >= 0 and (end < 0 or candidate < end):
                    if self._valid_binary(candidate):
                        del self.buffer[:candidate]
                        self.bad_frames += 1
                        recovered = True
                        break
                    candidate = self.buffer.find(b"\xff", candidate+1)
                if recovered:
                    continue
                if end < 0:
                    if len(self.buffer) > 256:
                        del self.buffer[0]
                        self.bad_frames += 1
                        continue
                    break
                line = bytes(self.buffer[:end+1]).decode("ascii", errors="replace").strip()
                del self.buffer[:end+1]
                if not line.startswith("@IMU,"):
                    self.diagnostics.append(line)
                    continue
                try:
                    fields = line.split(",")
                    if len(fields) != 10:
                        raise ValueError("Wrong number of fields")
                    sequence, timestamp = int(fields[1]), int(fields[2])
                    values = tuple(float(v) for v in fields[3:])
                    if not (0 <= sequence <= 0xFFFFFFFF and 0 <= timestamp <= 0xFFFFFFFF
                            and all(math.isfinite(v) for v in values)):
                        raise ValueError("Invalid sample")
                    samples.append(Sample(sequence, timestamp, values, "text"))
                except ValueError:
                    self.bad_frames += 1
        return samples

    def take_diagnostics(self) -> list[str]:
        messages, self.diagnostics = self.diagnostics, []
        return messages


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="For example /dev/ttyUSB0 or COM5; omitted selects a USB serial port")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--mode", choices=("text", "hex"), default="hex")
    parser.add_argument("--csv", help="Save raw SI samples (uncorrected) to a new CSV file")
    parser.add_argument("--count", type=int, default=0, help="Stop after N samples; 0 keeps running")
    parser.add_argument("--print-every", type=int, default=20, help="Print one sample out of N")
    parser.add_argument("--list", action="store_true", help="List serial ports")
    from calibration_runtime import (CORRECTED_FIELDS, add_calibration_args,
                                     corrected_row, load_calibration)
    add_calibration_args(parser, csv_output=True)
    args = parser.parse_args()
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        print("Install the PC dependency: python -m pip install pyserial", file=sys.stderr)
        return 2
    if args.list:
        for port in list_ports.comports():
            print(port.device, port.description)
        return 0
    if args.count < 0 or args.print_every < 1 or args.baud < 1:
        parser.error("Invalid count, print interval or baud rate")
    from serial_ports import select_serial_port
    try:
        args.port = select_serial_port(args.port)
    except OSError as exc:
        print(f"串口选择失败：{exc}", file=sys.stderr)
        return 1
    try:
        calibration = load_calibration(args)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(f"Calibration loading failed: {exc}")
    decoder, total, previous_sequence = StreamDecoder(), 0, None
    output = None
    corrected_output = None
    corrected_writer = None
    try:
        if args.csv:
            output = open(args.csv, "x", newline="", encoding="utf-8")
            writer = csv.DictWriter(output, fieldnames=CSV_FIELDS)
            writer.writeheader()
        else:
            writer = None
        if args.corrected_csv:
            corrected_output = open(args.corrected_csv, "x", newline="", encoding="utf-8")
            corrected_writer = csv.DictWriter(corrected_output, fieldnames=(*CSV_FIELDS, *CORRECTED_FIELDS))
            corrected_writer.writeheader()
        with serial.Serial(args.port, args.baud, timeout=0.1) as port:
            port.write(b"@PING\r\n")
            port.write(b"@HEX\r\n" if args.mode == "hex" else b"@TEXT\r\n")
            port.write(b"@START\r\n")
            print("Connected. Acceleration: m/s^2; angular rate: rad/s; Ctrl+C to stop.", file=sys.stderr)
            while args.count == 0 or total < args.count:
                samples = decoder.feed(port.read(port.in_waiting or 1))
                for message in decoder.take_diagnostics():
                    print(message, file=sys.stderr)
                for sample in samples:
                    if previous_sequence is not None:
                        gap = (sample.sequence - previous_sequence) & 0xFFFFFFFF
                        if gap != 1:
                            print(f"Sequence discontinuity: {previous_sequence} -> {sample.sequence} "
                                  "(loss, STOP/START or board reset)", file=sys.stderr)
                    previous_sequence = sample.sequence
                    raw_row = sample.csv_row()
                    corrected = calibration.apply(sample) if calibration else sample
                    if writer:
                        writer.writerow(raw_row)
                    if corrected_writer:
                        corrected_writer.writerow(corrected_row(raw_row, corrected))
                    total += 1
                    if total == 1 or total % args.print_every == 0:
                        ax, ay, az, gx, gy, gz, temp = corrected.values
                        print(f"#{sample.sequence} t={sample.timestamp_us}us [{sample.mode}] "
                              f"a=({ax:+.3f},{ay:+.3f},{az:+.3f}) m/s^2 "
                              f"w=({gx:+.6f},{gy:+.6f},{gz:+.6f}) rad/s T={temp:.2f}C")
                    if args.count and total >= args.count:
                        break
            port.write(b"@STOP\r\n")
    except KeyboardInterrupt:
        pass
    except (OSError, serial.SerialException) as exc:
        print(f"Cannot capture data: {exc}", file=sys.stderr)
        return 1
    finally:
        if output:
            output.close()
        if corrected_output:
            corrected_output.close()
        print(f"Decoded {total} samples; rejected candidates: {decoder.bad_frames}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
