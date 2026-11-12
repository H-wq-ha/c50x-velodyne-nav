#!/usr/bin/env python3
"""200Hz pseudo-terminal -> real receiver CLI -> CSV, without MCU hardware."""
import binascii
import csv
import json
import os
from pathlib import Path
import pty
import struct
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]


def frame(sequence):
    # Independent producer follows the documented v2 layout.
    elapsed = sequence * 5000
    payload = struct.pack("<IQQII7fII", sequence, 900000 + elapsed,
                          1767225600000000 + elapsed, elapsed, elapsed // 1000000 + 1,
                          0.0, 0.0, 9.80665, 0.1, -0.2, 0.3, 36.5, 7, 0)
    body = b"\x02\x01\x40\x00" + payload
    return b"\xa5\x5a" + body + struct.pack("<H", binascii.crc_hqx(body, 0xFFFF))


class SerialPipelineTest(unittest.TestCase):
    def test_receiver_200hz_and_crc_recovery(self):
        try:
            import serial  # noqa: F401
        except ImportError:
            self.skipTest("pyserial required for pseudo-terminal integration")
        master, slave = pty.openpty()
        try:
            with tempfile.TemporaryDirectory(prefix="imu-serial-") as directory:
                output = Path(directory) / "samples.csv"
                process = subprocess.Popen([sys.executable, str(ROOT / "tools/imu_receive.py"),
                                            "--port", os.ttyname(slave), "--duration", "3",
                                            "--csv", str(output)], stdout=subprocess.PIPE,
                                           stderr=subprocess.PIPE, text=True)
                try:
                    time.sleep(0.3)
                    damaged = bytearray(frame(0))
                    damaged[40] ^= 1
                    os.write(master, b"noise" + bytes(damaged))
                    start = time.monotonic()
                    for sequence in range(400):
                        remaining = start + sequence * 0.005 - time.monotonic()
                        if remaining > 0:
                            time.sleep(remaining)
                        data = frame(sequence)
                        # Exercise partial writes at frame boundaries.
                        os.write(master, data[:11])
                        os.write(master, data[11:])
                    stdout, stderr = process.communicate(timeout=5)
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.communicate()
                self.assertEqual(process.returncode, 0, stderr)
                with output.open(newline="") as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual(len(rows), 400)
                self.assertEqual([int(row["sequence"]) for row in rows], list(range(400)))
                stamps = [int(row["epoch_us"]) for row in rows]
                self.assertTrue(all(b - a == 5000 for a, b in zip(stamps, stamps[1:])))
                reports = [json.loads(line) for line in stdout.splitlines()]
                self.assertGreaterEqual(max(report["crc_errors"] for report in reports), 1)
                self.assertEqual(max(report["sequence_gaps"] for report in reports), 0)
        finally:
            os.close(master)
            os.close(slave)


if __name__ == "__main__":
    unittest.main(verbosity=2)
