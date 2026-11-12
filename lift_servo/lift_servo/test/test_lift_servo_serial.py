import importlib.util
import pathlib
import sys
import unittest


MODULE_PATH = pathlib.Path(__file__).with_name("lift_servo_serial.py")
SPEC = importlib.util.spec_from_file_location("lift_servo_serial", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class FakeSerial:
    def __init__(self, **kwargs):
        self.responses = {
            "PING": [b"OK PING\r\n"],
            "STATUS": [
                b"STATUS state=IDLE dir=FWD enabled=1 alarm=0 pend=1 "
                b"speed_pps=500 sent=20 mode=FINITE remaining=0 "
                b"watchdog_ms=0 reason=MOVE_COMPLETE\r\n"
            ],
            "MOVE REV 10 50": [b"OK MOVE\r\n"],
            "RUN FWD 100": [b"ERR SAFETY_LATCHED clear_required\r\n"],
        }
        self.pending = []
        self.writes = []
        self.closed = False

    def write(self, data):
        self.writes.append(data)
        command = data.decode("ascii").strip()
        self.pending.extend(self.responses.get(command, []))
        return len(data)

    def readline(self):
        return self.pending.pop(0) if self.pending else b""

    def flush(self):
        pass

    def close(self):
        self.closed = True


class LiftServoClientTests(unittest.TestCase):
    def make_client(self):
        return MODULE.LiftServoClient(
            "FAKE", timeout=0.05, serial_factory=lambda **kwargs: FakeSerial(**kwargs)
        )

    def test_ping_uses_newline_framing(self):
        with self.make_client() as client:
            response = client.ping()
            self.assertEqual(response.primary, "OK PING")
            self.assertEqual(client._serial.writes[-1], b"PING\n")

    def test_status_is_parsed(self):
        with self.make_client() as client:
            status = client.status()
            self.assertEqual(status["state"], "IDLE")
            self.assertEqual(status["speed_pps"], "500")
            self.assertEqual(status["reason"], "MOVE_COMPLETE")

    def test_move_command(self):
        with self.make_client() as client:
            response = client.move("rev", 10, 50)
            self.assertEqual(response.primary, "OK MOVE")
            self.assertEqual(client._serial.writes[-1], b"MOVE REV 10 50\n")

    def test_device_error_is_raised(self):
        with self.make_client() as client:
            with self.assertRaises(MODULE.DeviceRejected):
                client.run("FWD", 100)

    def test_local_ranges_are_checked(self):
        with self.make_client() as client:
            with self.assertRaises(ValueError):
                client.set_speed(20_001)
            with self.assertRaises(ValueError):
                client.move("FWD", 0, 10)
            with self.assertRaises(ValueError):
                client.set_watchdog(60_001)


if __name__ == "__main__":
    unittest.main()
