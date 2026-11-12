#!/usr/bin/env python3
"""Failure detection, device protocol, recorder, and plotting checks."""
import binascii
import csv
import importlib.util
import os
from pathlib import Path
import pty
import queue
import struct
import sys
import tempfile
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from imu_lidar_scope import ImuReceiver, Recorder, draw, event
from lidar_receive import validate_vlp16_packet


def frame(kind, payload):
    body = bytes((2, kind, 64, 0)) + payload
    return b"\xa5\x5a" + body + struct.pack("<H", binascii.crc_hqx(body, 0xFFFF))


def sample(sequence):
    return frame(1, struct.pack("<IQQII7fII", sequence, 900000 + sequence * 5000,
                               1767225600000000 + sequence * 5000, sequence * 5000, 1,
                               0., 0., 9.8, 0.1, 0.2, 0.3, 28.0, 111, 0))


def packet():
    data = bytearray(1206)
    for offset in range(0, 1200, 100):
        data[offset:offset + 4] = b"\xff\xee" + struct.pack("<H", 12345)
    data[1200:1204] = struct.pack("<I", 123456)
    data[1204:] = b"\x37\x22"
    return data


class ReceptionScopeTests(unittest.TestCase):
    def test_lidar_packet_rejection(self):
        data = packet()
        self.assertEqual(validate_vlp16_packet(data), (True, "ok"))
        self.assertFalse(validate_vlp16_packet(data[:-1])[0])
        broken = data.copy()
        broken[100] = 0
        self.assertEqual(validate_vlp16_packet(broken), (False, "block_header"))
        broken = data.copy()
        broken[2:4] = struct.pack("<H", 36000)
        self.assertEqual(validate_vlp16_packet(broken), (False, "azimuth_range"))
        data[1200:1204] = struct.pack("<I", 3600000000)
        self.assertEqual(validate_vlp16_packet(data), (False, "hardware_time_range"))

    def test_real_serial_reader_status_crc_gap_and_timeout(self):
        master, slave = pty.openpty()
        incoming = queue.Queue()
        stop = threading.Event()
        reader = ImuReceiver(incoming.put, stop, os.ttyname(slave), start_stream=False)
        thread = threading.Thread(target=reader.run)
        thread.start()
        try:
            time.sleep(0.1)
            corrupt = bytearray(sample(3))
            corrupt[30] ^= 1
            os.write(master, frame(2, struct.pack("<16I", *range(16)))
                     + sample(1) + sample(2) + bytes(corrupt) + sample(5))
            time.sleep(0.25)
        finally:
            stop.set()
            thread.join(timeout=2)
            os.close(master)
            os.close(slave)
        self.assertIsNone(reader.error)
        items = []
        while not incoming.empty():
            items.append(incoming.get_nowait())
        self.assertEqual([item["sequence"] for item in items if item["event"] == "data"], [1, 2, 5])
        self.assertEqual(sum(item["event"] == "status" for item in items), 1)
        self.assertEqual(sum(item["event"] == "crc_error" for item in items), 1)
        gaps = [item for item in items if item["event"] == "sequence_gap"]
        self.assertEqual(gaps[0]["details"]["missing"], 2)
        self.assertTrue(any(item["event"] == "timeout" for item in items))
        self.assertTrue(all(item["hardware_ns"] is None for item in items if not item["success"]))

    def test_record_all_events_but_not_status_impulses(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = Recorder(directory)
            recorder.append(event("imu", "status", True, "device_status", flags=109))
            for index, source in enumerate(("imu", "lidar")):
                for number in range(3):
                    item = event(source, "data", True, "ok", 1792328000000000000 + number * 5000000,
                                 1000000000 + number * 5000000)
                    item["hardware_ns"] = item["receive_ns"] + 1000
                    recorder.append(item)
            recorder.append(event("imu", "crc_error", False, "crc_mismatch", 1792328000020000000))
            recorder.flush()
            self.assertEqual(len(recorder.events), 7)
            self.assertEqual(recorder.summary()["sources"]["imu"]["crc_errors"], 1)
            self.assertEqual(recorder.summary()["sources"]["imu"]["receive_hz"], 200)
            figure = plt.figure(figsize=(10, 5))
            for kind in ("receive", "hardware"):
                draw(recorder, figure, kind, 1)
                output = Path(directory) / (kind + ".png")
                figure.savefig(output)
                self.assertGreater(output.stat().st_size, 10000)
            plt.close(figure)
            recorder.stream.close()
            with (Path(directory) / "events.csv").open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 8)
            self.assertEqual(rows[-1]["success"], "0")


if __name__ == "__main__":
    unittest.main(verbosity=2)
