#!/usr/bin/env python3
"""Host checks using Python's calendar as an independent reference.

Run: python3 tests/run_checks.py
No board, serial connection, or third-party Python dependency is needed.
"""
import ctypes as ct
from datetime import datetime, timedelta
from functools import reduce
import operator
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class Time(ct.Structure):
    _fields_ = [("year", ct.c_uint16)] + [
        (name, ct.c_uint8) for name in ("month", "day", "hour", "minute", "second")
    ]


def from_datetime(value):
    return Time(value.year, value.month, value.day,
                value.hour, value.minute, value.second)


def as_tuple(value):
    return tuple(getattr(value, field) for field, _ in Time._fields_)


class FirmwareLogicChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory(prefix="fake-gps-check-")
        library = Path(cls.temporary.name) / "gps_time.so"
        subprocess.run([
            "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wconversion",
            "-fPIC", "-shared", "-O2", "-I", str(ROOT / "include"),
            str(ROOT / "src/gps_time.c"), "-o", str(library),
        ], check=True)
        cls.lib = ct.CDLL(str(library))
        cls.lib.gps_time_valid.argtypes = [ct.POINTER(Time)]
        cls.lib.gps_time_valid.restype = ct.c_bool
        cls.lib.gps_time_advance.argtypes = [ct.POINTER(Time)]
        cls.lib.gps_rmc_format.argtypes = [ct.c_void_p, ct.c_size_t,
                                         ct.POINTER(Time), ct.c_char]
        cls.lib.gps_rmc_format.restype = ct.c_size_t
        cls.lib.gps_tx_window.argtypes = [ct.c_uint32]
        cls.lib.gps_tx_window.restype = ct.c_bool

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def sentence(self, time, status=b"A"):
        buffer = ct.create_string_buffer(83)
        count = self.lib.gps_rmc_format(buffer, 83, ct.byref(time), status)
        self.assertGreater(count, 0)
        self.assertEqual(count, len(buffer.value))
        return buffer.value

    def test_every_day_rollover_2000_to_2099(self):
        date = datetime(2000, 1, 1, 23, 59, 59)
        end = datetime(2100, 1, 1)
        count = 0
        while date < end:
            time = from_datetime(date)
            self.assertTrue(self.lib.gps_time_valid(ct.byref(time)))
            self.lib.gps_time_advance(ct.byref(time))
            self.assertEqual(as_tuple(time), as_tuple(from_datetime(date + timedelta(seconds=1))))
            date += timedelta(days=1)
            count += 1
        self.assertEqual(count, 36525)
        # NMEA's two-digit year cannot represent the next century unambiguously.
        self.assertFalse(self.lib.gps_time_valid(ct.byref(time)))

    def test_seconds_and_sentence_progression_across_new_year(self):
        date = datetime(2027, 12, 31, 23, 59, 58)
        time = from_datetime(date)
        for index in range(86404):
            self.assertEqual(as_tuple(time), as_tuple(from_datetime(date)))
            if index < 5 or index % 3600 == 0:
                sentence = self.sentence(time)
                fields = sentence[1:sentence.index(b"*")].split(b",")
                self.assertEqual(fields[1], date.strftime("%H%M%S.000").encode())
                self.assertEqual(fields[9], date.strftime("%d%m%y").encode())
            self.lib.gps_time_advance(ct.byref(time))
            date += timedelta(seconds=1)

    def test_invalid_dates_and_status(self):
        invalid = [Time(1999, 1, 1, 0, 0, 0), Time(2100, 1, 1, 0, 0, 0),
                   Time(2026, 0, 1, 0, 0, 0), Time(2026, 13, 1, 0, 0, 0),
                   Time(2026, 2, 29, 0, 0, 0), Time(2026, 4, 31, 0, 0, 0),
                   Time(2026, 1, 0, 0, 0, 0), Time(2026, 1, 1, 24, 0, 0),
                   Time(2026, 1, 1, 0, 60, 0), Time(2026, 1, 1, 0, 0, 60)]
        for time in invalid:
            self.assertFalse(self.lib.gps_time_valid(ct.byref(time)))
            buffer = ct.create_string_buffer(b"old", 83)
            self.assertEqual(self.lib.gps_rmc_format(buffer, 83, ct.byref(time), b"A"), 0)
            self.assertEqual(buffer.value, b"")
        self.assertFalse(self.lib.gps_time_valid(None))
        time = Time(2000, 2, 29, 0, 0, 0)
        self.assertTrue(self.lib.gps_time_valid(ct.byref(time)))
        buffer = ct.create_string_buffer(83)
        self.assertEqual(self.lib.gps_rmc_format(buffer, 83, ct.byref(time), b"X"), 0)

    def test_checksum_crlf_nul_and_buffer_boundaries(self):
        time = Time(2026, 10, 18, 12, 34, 56)
        for status in (b"A", b"V"):
            sentence = self.sentence(time, status)
            self.assertTrue(sentence.startswith(b"$GPRMC,123456.000," + status + b","))
            self.assertTrue(sentence.endswith(b"\r\n"))
            self.assertNotIn(b"\x00", sentence)
            body, suffix = sentence[1:].split(b"*")
            checksum = reduce(operator.xor, body, 0)
            self.assertEqual(suffix, f"{checksum:02X}\r\n".encode())
            self.assertIn(b",181026,,,", body)
            self.assertLessEqual(len(sentence), 82)
        length = len(sentence)
        for capacity in range(1, 90):
            storage = (ct.c_ubyte * (capacity + 2))(*([0xA5] * (capacity + 2)))
            pointer = ct.cast(ct.byref(storage, 1), ct.c_void_p)
            count = self.lib.gps_rmc_format(pointer, capacity, ct.byref(time), b"V")
            self.assertEqual((storage[0], storage[capacity + 1]), (0xA5, 0xA5))
            if capacity <= length:
                self.assertEqual(count, 0)
                self.assertEqual(storage[1], 0)
            else:
                self.assertEqual(count, length)
                self.assertEqual(storage[length + 1], 0)
        self.assertEqual(self.lib.gps_rmc_format(None, 0, None, b"A"), 0)

    def test_transmit_window_and_worst_case_wire_time(self):
        allowed = [tick for tick in range(10000) if self.lib.gps_tx_window(tick)]
        self.assertEqual(allowed, list(range(1000, 2000)))
        self.assertFalse(self.lib.gps_tx_window(0xFFFFFFFF))
        # Relative to preceding PPS: 200..300ms launch. 9600 baud, 8N1.
        longest_wire_ms = 82 * 10 * 1000 / 9600
        self.assertGreaterEqual(200 - 100, 50)  # gap after falling PPS
        self.assertLess(300 + longest_wire_ms, 500)  # old firmware limit
        self.assertGreater(1000 - 300 - longest_wire_ms, 300)  # Rev-F limit

    def test_built_vector_tables(self):
        nm = Path.home() / ".platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-nm"
        if not nm.exists():
            self.skipTest("PlatformIO ARM tools unavailable; build verification skipped")
        required = {"NMI_Handler": 2, "HardFault_Handler": 3, "SysTick_Handler": 15,
                    "SVC_Handler": 11, "PendSV_Handler": 14,
                    "DMA1_Channel4_IRQHandler": 16 + 14,
                    "DMA1_Channel7_IRQHandler": 16 + 17,
                    "TIM2_IRQHandler": 16 + 28, "TIM4_IRQHandler": 16 + 30,
                    "TIM1_CC_IRQHandler": 16 + 27, "USART1_IRQHandler": 16 + 37,
                    "USART2_IRQHandler": 16 + 38}
        for env in ("genericSTM32F103C8", "genericSTM32F103C8_hsi"):
            directory = ROOT / ".pio/build" / env
            if not (directory / "firmware.elf").exists():
                self.skipTest(f"Build {env} first to verify its vector table")
            result = subprocess.check_output([str(nm), "--defined-only",
                                              str(directory / "firmware.elf")], text=True)
            symbols = {}
            for line in result.splitlines():
                items = line.split()
                if len(items) == 3:
                    symbols[items[2]] = (int(items[0], 16), items[1])
            image = (directory / "firmware.bin").read_bytes()
            for name, index in required.items():
                address, kind = symbols[name]
                self.assertEqual(kind, "T", f"{env}: {name} is still a weak default")
                self.assertEqual(struct.unpack_from("<I", image, index * 4)[0], address | 1)


if __name__ == "__main__":
    if shutil.which("gcc") is None:
        raise SystemExit("Host gcc is required")
    unittest.main(verbosity=2)
