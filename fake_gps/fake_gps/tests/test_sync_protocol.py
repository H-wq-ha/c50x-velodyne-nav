#!/usr/bin/env python3
"""Exercise firmware C clock/packet code against independent Python references."""
import binascii
import ctypes as ct
from datetime import datetime, timezone
import importlib.util
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("imu_receive", ROOT / "tools/imu_receive.py")
receiver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(receiver)


class GpsTime(ct.Structure):
    _fields_ = [("year", ct.c_uint16)] + [(key, ct.c_uint8) for key in
                                            ("month", "day", "hour", "minute", "second")]


class Clock(ct.Structure):
    _fields_ = [(key, ct.c_uint64) for key in ("epoch_start_us", "first_pps_us", "last_pps_us")] + [
        ("pps_count", ct.c_uint32), ("interval_error_max_us", ct.c_uint32), ("valid", ct.c_bool)]


class Stamp(ct.Structure):
    _fields_ = [("monotonic_us", ct.c_uint64), ("epoch_us", ct.c_uint64),
                ("hour_us", ct.c_uint32), ("pps_count", ct.c_uint32)]


class Sample(ct.Structure):
    _fields_ = [("sequence", ct.c_uint32), ("monotonic_us", ct.c_uint64),
                ("epoch_us", ct.c_uint64)] + [(key, ct.c_uint32) for key in
                ("hour_us", "pps_count", "flags", "lost_samples")] + [
                ("accel_mps2", ct.c_float * 3), ("gyro_rads", ct.c_float * 3),
                ("temperature_c", ct.c_float)]


class SharedClockTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="shared-clock-")
        library = Path(cls.temporary.name) / "clock.so"
        subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-fPIC", "-shared",
                        "-O2", "-I", str(ROOT / "include"),
                        *[str(ROOT / "src" / file) for file in
                          ("gps_time.c", "clock_sync.c", "imu_protocol.c")],
                        "-o", str(library)], check=True)
        cls.lib = ct.CDLL(str(library))
        cls.lib.Clock_EpochUs.argtypes = [ct.POINTER(GpsTime)]
        cls.lib.Clock_EpochUs.restype = ct.c_uint64
        cls.lib.Clock_ExtendCapture.argtypes = [ct.c_uint64, ct.c_uint16, ct.c_bool]
        cls.lib.Clock_ExtendCapture.restype = ct.c_uint64
        cls.lib.Clock_OnPps.argtypes = [ct.POINTER(Clock), ct.c_uint64]
        cls.lib.Clock_Stamp.argtypes = [ct.POINTER(Clock), ct.c_uint64, ct.POINTER(Stamp)]
        cls.lib.Clock_Stamp.restype = ct.c_bool
        cls.lib.IMU_PackBinary.argtypes = [ct.POINTER(Sample), ct.c_void_p]
        cls.lib.IMU_PackStatus.argtypes = [ct.c_void_p, ct.c_void_p]
        cls.lib.IMU_Crc16.argtypes = [ct.c_void_p, ct.c_size_t]
        cls.lib.IMU_Crc16.restype = ct.c_uint16

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def frame(self, sequence=123):
        sample = Sample(sequence=sequence, monotonic_us=(1 << 32) + 1234,
                        epoch_us=1767225600123456, hour_us=123456, pps_count=1,
                        flags=7, lost_samples=2, accel_mps2=(1.0, -2.0, 9.80665),
                        gyro_rads=(-0.1, 0.2, 0.3), temperature_c=36.5)
        data = ct.create_string_buffer(72)
        self.lib.IMU_PackBinary(ct.byref(sample), data)
        return data.raw

    def test_calendar_epoch_and_leap_days(self):
        for parts in [(2000, 1, 1, 0, 0, 0), (2000, 2, 29, 12, 34, 56),
                      (2026, 1, 1, 0, 0, 0), (2028, 2, 29, 23, 59, 59),
                      (2099, 12, 31, 23, 59, 59)]:
            expected = int(datetime(*parts, tzinfo=timezone.utc).timestamp()) * 1000000
            self.assertEqual(self.lib.Clock_EpochUs(ct.byref(GpsTime(*parts))), expected)
        self.assertEqual(self.lib.Clock_EpochUs(ct.byref(GpsTime(2026, 2, 29, 0, 0, 0))), 0)

    def test_16bit_overflow_capture_order(self):
        base = (1 << 32) + 65536
        self.assertEqual(self.lib.Clock_ExtendCapture(base, 65530, True), base + 65530)
        self.assertEqual(self.lib.Clock_ExtendCapture(base, 5, True), base + 65536 + 5)
        self.assertEqual(self.lib.Clock_ExtendCapture(base, 5, False), base + 5)
        self.assertEqual(self.lib.Clock_ExtendCapture(base, 0, True), base + 65536)

    def test_first_pps_hour_rollover_and_long_uptime(self):
        epoch = int(datetime(2026, 1, 1, 23, 59, 59, tzinfo=timezone.utc).timestamp()) * 1000000
        clock = Clock(epoch_start_us=epoch)
        stamp = Stamp()
        self.assertFalse(self.lib.Clock_Stamp(ct.byref(clock), 900000, ct.byref(stamp)))
        self.lib.Clock_OnPps(ct.byref(clock), 900000)
        self.assertFalse(self.lib.Clock_Stamp(ct.byref(clock), 899999, ct.byref(stamp)))
        for delta in (0, 4999, 5000, 999999, 1000000, (1 << 32) + 123, 3 * 86400 * 1000000):
            self.assertTrue(self.lib.Clock_Stamp(ct.byref(clock), 900000 + delta, ct.byref(stamp)))
            self.assertEqual(stamp.epoch_us, epoch + delta)
            self.assertEqual(stamp.hour_us, (epoch + delta) % 3600000000)
            self.assertEqual(stamp.pps_count, delta // 1000000 + 1)
        self.lib.Clock_OnPps(ct.byref(clock), 1900000)
        self.lib.Clock_OnPps(ct.byref(clock), 2900001)
        self.assertEqual(clock.interval_error_max_us, 1)

    def test_crc_and_wire_layout(self):
        self.assertEqual(self.lib.IMU_Crc16(b"123456789", 9), 0x29B1)
        frame = self.frame()
        self.assertEqual(frame[:6], b"\xa5\x5a\x02\x01\x40\x00")
        self.assertEqual(int.from_bytes(frame[70:], "little"), binascii.crc_hqx(frame[2:70], 0xFFFF))
        record, = receiver.FrameParser().feed(frame)
        self.assertEqual(record["sequence"], 123)
        self.assertEqual(record["monotonic_us"], (1 << 32) + 1234)
        self.assertEqual(record["epoch_us"], 1767225600123456)
        self.assertEqual(record["hour_us"], 123456)
        self.assertAlmostEqual(record["accel_mps2"][2], 9.80665, places=5)
        self.assertEqual(record["lost_samples"], 2)
        words = (ct.c_uint32 * 16)(*range(16))
        data = ct.create_string_buffer(72)
        self.lib.IMU_PackStatus(words, data)
        record, = receiver.FrameParser().feed(data.raw)
        self.assertEqual(record["kind"], "status")
        self.assertEqual(record["nmea_sent"], 15)

    def test_stream_fragmentation_corruption_and_resync(self):
        frame = self.frame()
        for split in range(73):
            parser = receiver.FrameParser()
            output = parser.feed(frame[:split]) + parser.feed(frame[split:])
            self.assertEqual(len(output), 1)
        damaged = bytearray(frame)
        damaged[31] ^= 1
        parser = receiver.FrameParser()
        records = []
        stream = b"garbage\xa5" + bytes(damaged) + frame[:21] + self.frame(124) + self.frame(125)
        for byte in stream:
            records += parser.feed(bytes([byte]))
        self.assertEqual([record["sequence"] for record in records], [124, 125])
        self.assertGreaterEqual(parser.crc_errors, 1)
        self.assertLess(len(parser.buffer), 72)

    def test_lidar_compatible_hour_selection(self):
        hour = 3_600_000_000_000
        base = int(datetime(2026, 10, 18, 12, tzinfo=timezone.utc).timestamp()) * 1000000000
        self.assertEqual(receiver.lidar_stamp_ns(123456, base + 500000000), base + 123456000)
        self.assertEqual(receiver.lidar_stamp_ns(3599999999, base + 1000000), base - 1000)
        self.assertEqual(receiver.lidar_stamp_ns(1, base + hour - 1000000), base + hour + 1000)
        with self.assertRaises(ValueError):
            receiver.lidar_stamp_ns(3600000000, base)


if __name__ == "__main__":
    unittest.main(verbosity=2)
