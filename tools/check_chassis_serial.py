#!/usr/bin/env python3
"""Check WHEELTEC chassis serial status frame decoding at a given baudrate."""

import argparse
import time

import serial

from car_bringup.protocol import FRAME_HEADER, STATUS_FRAME_SIZE, parse_status_frame


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, required=True)
    parser.add_argument("--seconds", type=float, default=2.0)
    args = parser.parse_args()

    serial_port = serial.Serial(args.port, baudrate=args.baudrate, timeout=0.05)
    buf = bytearray()
    valid_frames = 0
    total_bytes = 0
    deadline = time.time() + args.seconds

    try:
        while time.time() < deadline:
            chunk = serial_port.read(512)
            total_bytes += len(chunk)
            buf.extend(chunk)

            while len(buf) >= STATUS_FRAME_SIZE:
                idx = buf.find(bytes([FRAME_HEADER]))
                if idx < 0:
                    buf.clear()
                    break
                if idx > 0:
                    del buf[:idx]
                if len(buf) < STATUS_FRAME_SIZE:
                    break

                frame = bytes(buf[:STATUS_FRAME_SIZE])
                del buf[:STATUS_FRAME_SIZE]
                if parse_status_frame(frame):
                    valid_frames += 1
    finally:
        serial_port.close()

    print(
        f"baudrate={args.baudrate} bytes={total_bytes} "
        f"valid_frames={valid_frames}"
    )


if __name__ == "__main__":
    main()
