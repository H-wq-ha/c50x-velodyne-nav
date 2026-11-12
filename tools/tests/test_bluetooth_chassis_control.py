"""Offline tests; the integration tests open only a newly-created pseudo terminal."""

import contextlib
import io
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time
import types
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bluetooth_chassis_control as bt


class FirmwareLink:
    """Small peer for the UART4 parameter echo and two-A mode activation."""

    def __init__(self, echo=True):
        self.echo = echo
        self.speed = 500
        self.parameter = None
        self.previous = None
        self.received = bytearray()
        self.pending = bytearray()
        self.activation_speeds = []
        self.app_mode = False

    def send(self, payload):
        self.received.extend(payload)
        for byte in payload:
            if byte == ord("A") and self.previous == ord("A") and not self.app_mode:
                self.activation_speeds.append(self.speed)
                self.app_mode = True
            self.previous = byte
            if byte == ord("{"):
                self.parameter = bytearray(b"{")
            elif self.parameter is not None:
                self.parameter.append(byte)
                if byte == ord("}"):
                    command = bytes(self.parameter)
                    self.parameter = None
                    if command == b"{0:P}":
                        if self.echo:
                            self.pending.extend(
                                f"{{C{self.speed}:100:50:20:20:2000:1000:1500:50}}$".encode())
                    else:
                        self.speed = int(command[3:-1])

    def receive(self):
        pending = bytes(self.pending)
        self.pending.clear()
        return pending


class ProtocolTests(unittest.TestCase):
    def test_exact_parameter_and_stop_bytes(self):
        self.assertEqual(bt.speed_packet(0.1), b"{0:100}")
        self.assertEqual(bt.speed_packet(0), b"{0:0}")
        self.assertEqual(bt.REQUEST_PARAMETERS, b"{0:P}")
        self.assertEqual(bt.STOP, bytes.fromhex("4b 00 49 00"))
        for value in (-0.1, 1.01, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                bt.speed_packet(value)

    def test_turn_and_strafe_pages_are_different(self):
        self.assertEqual(bt.action_packet("forward", "4wd"), b"IA")
        self.assertEqual(bt.action_packet("backward", "4wd"), b"IE")
        self.assertEqual(bt.action_packet("left", "4wd"), b"IG")
        self.assertEqual(bt.action_packet("right", "diff"), b"IC")
        self.assertEqual(bt.action_packet("left", "mecanum"), b"KG")
        self.assertEqual(bt.action_packet("right", "omni"), b"KC")
        self.assertEqual(bt.action_packet("strafe-left", "mecanum"), b"IG")
        self.assertEqual(bt.action_packet("strafe-right", "omni"), b"IC")
        self.assertEqual(bt.action_packet("forward-left", "ackermann"), b"IH")
        with self.assertRaises(ValueError):
            bt.action_packet("left", "ackermann")
        with self.assertRaises(ValueError):
            bt.action_packet("strafe-left", "4wd")

    def test_fragmentation_noise_and_resynchronization(self):
        parser = bt.Telemetry()
        self.assertEqual(parser.feed(b"noise{A1:2:"), [])
        self.assertEqual(parser.feed(b"90:-12}$junk{B3:-4:5}$"),
                         [("A", (1, 2, 90, -12)), ("B", (3, -4, 5))])
        self.assertEqual(parser.feed(b"{bad{B0:0:0}$"), [("B", (0, 0, 0))])
        self.assertEqual(parser.feed(b"{A1:2}$ {B1:x:3}$"), [])
        parser.feed(b"{" + b"9" * 500)
        self.assertLessEqual(len(parser.buffer), 256)
        self.assertEqual(parser.feed(b"{B1:2:3}$"), [("B", (1, 2, 3))])
        self.assertEqual(parser.feed(b"{B" + b"9" * 500 + b":0:0}$"), [])

    def test_zero_speed_is_confirmed_before_activation(self):
        peer = FirmwareLink()
        control = bt.Controller(peer, "4wd", 0.1)
        with contextlib.redirect_stdout(io.StringIO()):
            control.initialize(0.1)
        self.assertEqual(peer.activation_speeds, [0])
        self.assertEqual(peer.speed, 100)
        self.assertLess(peer.received.index(b"{0:0}{0:P}"), peer.received.index(b"AA"))
        control.send_action("forward")
        self.assertTrue(peer.received.endswith(bytes.fromhex("4b 00 49 00 49 41")))
        control.stop()
        self.assertEqual(peer.speed, 0)
        self.assertTrue(peer.received.endswith(bt.STOP * 3 + b"{0:0}"))

    def test_missing_parameter_echo_never_activates(self):
        peer = FirmwareLink(echo=False)
        with self.assertRaises(TimeoutError):
            bt.Controller(peer, "4wd", 0.03).initialize(0.1)
        self.assertNotIn(b"AA", peer.received)
        self.assertNotIn(b"{0:100}", peer.received)

    def test_feedback_timeout(self):
        control = bt.Controller(FirmwareLink(), "4wd", 0.01)
        control.telemetry.last_received = time.monotonic() - 1
        with self.assertRaises(TimeoutError):
            control.poll(check_link=True)

    def test_reject_invalid_arguments_before_connecting(self):
        for argv in (
            ["--dry-run", "move", "forward", "--duration", "nan"],
            ["--dry-run", "move", "forward", "--speed", "inf"],
            ["--dry-run", "--chassis", "4wd", "move", "strafe-left"],
            ["--address", "invalid", "stop"],
        ):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                bt.parse_args(argv)


class FakeBlueZ:
    """Exercise the BLE adapter without accessing the system bus or a radio."""

    address = "2A:A2:19:08:05:45"
    device_path = "/org/bluez/hci0/dev_2A_A2_19_08_05_45"
    service_path = device_path + "/service000e"
    characteristic_path = service_path + "/char000f"

    def __init__(self, discover=True, characteristic_uuid=bt.BLE_CHARACTERISTIC):
        self.queue = []
        self.connected = False
        self.closed = False
        self.notifying = False
        self.discovering = False
        self.discovered = discover
        self.callback = None
        self.writes = []
        self.characteristic_uuid = characteristic_uuid
        self.peer = FirmwareLink()
        self.disconnects = 0

    def objects(self):
        result = {"/org/bluez/hci0": {"org.bluez.Adapter1": {"Powered": True}}}
        if self.discovered:
            result.update({
                self.device_path: {"org.bluez.Device1": {}},
                self.service_path: {"org.bluez.GattService1": {"UUID": bt.BLE_SERVICE}},
                self.characteristic_path: {"org.bluez.GattCharacteristic1": {
                    "Service": self.service_path, "UUID": self.characteristic_uuid,
                    "Flags": ["write", "write-without-response", "notify"]}},
            })
        return result

    def connect(self, **kwargs):
        self.connected = True

    def disconnect(self, **kwargs):
        self.connected = False
        self.disconnects += 1

    def start_scan(self):
        self.discovering = True
        self.discovered = True

    def stop_scan(self, **kwargs):
        self.discovering = False

    def notify(self, payload):
        self.queue.append(lambda: self.callback(
            "org.bluez.GattCharacteristic1", {"Value": payload}, [],
            path=self.characteristic_path))

    def write(self, payload, options, **kwargs):
        self.writes.append((self.characteristic_path, bytes(payload), options))
        self.peer.send(bytes(payload))
        response = self.peer.receive()
        if response:
            # Simulate one parameter response fragmented across notifications.
            self.notify(response[:7])
            self.notify(response[7:])

    def interface(self, path, name):
        if name.endswith("ObjectManager"):
            return types.SimpleNamespace(GetManagedObjects=self.objects)
        if name.endswith("Adapter1"):
            return types.SimpleNamespace(SetDiscoveryFilter=lambda options: None,
                                         StartDiscovery=self.start_scan, StopDiscovery=self.stop_scan)
        if name.endswith("Properties"):
            return types.SimpleNamespace(Get=lambda iface, key:
                                         self.connected if key == "Connected" else True)
        if name.endswith("Device1"):
            return types.SimpleNamespace(Connect=self.connect, Disconnect=self.disconnect)
        if name.endswith("GattCharacteristic1"):
            return types.SimpleNamespace(
                StartNotify=lambda **kw: setattr(self, "notifying", True),
                StopNotify=lambda **kw: setattr(self, "notifying", False),
                WriteValue=self.write)
        raise AssertionError(name)

    def add_signal_receiver(self, callback, **kwargs):
        self.callback = callback
        return types.SimpleNamespace(remove=lambda: setattr(self, "callback", None))

    def close(self):
        self.closed = True

    def modules(self):
        dbus = types.ModuleType("dbus")
        dbus.SystemBus = lambda **kw: self
        dbus.Interface = lambda path, iface: self.interface(path, iface)
        dbus.Array = lambda payload, **kw: list(payload)
        dbus.Dictionary = lambda options, **kw: options
        self.get_object = lambda service, path, **kw: path
        glib = types.ModuleType("dbus.mainloop.glib")
        glib.DBusGMainLoop = lambda **kw: None
        context = types.SimpleNamespace(pending=lambda: bool(self.queue),
                                        iteration=lambda block: self.queue.pop(0)())
        gi_repository = types.ModuleType("gi.repository")
        gi_repository.GLib = types.SimpleNamespace(
            MainContext=types.SimpleNamespace(default=lambda: context))
        return {"dbus": dbus, "dbus.mainloop": types.ModuleType("dbus.mainloop"),
                "dbus.mainloop.glib": glib, "gi": types.ModuleType("gi"),
                "gi.repository": gi_repository}


@unittest.skipUnless(sys.platform.startswith("linux"), "BLE transport uses Linux BlueZ")
class BLETests(unittest.TestCase):
    def test_notify_is_read_only_and_close_releases_resources(self):
        peer = FakeBlueZ(discover=False)
        with mock.patch.dict(sys.modules, peer.modules()), contextlib.redirect_stdout(io.StringIO()):
            link = bt.BLELink(peer.address, 0.1)
            self.assertFalse(peer.discovering)
            peer.notify(b"{A0:0:44:1}$")
            self.assertEqual(link.receive(), b"{A0:0:44:1}$")
            self.assertEqual(peer.writes, [])
            link.close()
            link.close()  # Cleanup must also tolerate a second call.
        self.assertFalse(peer.connected)
        self.assertFalse(peer.notifying)
        self.assertTrue(peer.closed)
        self.assertEqual(peer.disconnects, 1)

    def test_bidirectional_protocol_uses_ffe1_and_zero_speed_activation(self):
        peer = FakeBlueZ()
        with mock.patch.dict(sys.modules, peer.modules()), contextlib.redirect_stdout(io.StringIO()):
            link = bt.BLELink(peer.address, 0.1)
            try:
                controller = bt.Controller(link, "4wd", 0.1)
                controller.initialize(0.1)
                controller.send_action("forward")
                controller.stop()
            finally:
                link.close()
        self.assertEqual(peer.peer.activation_speeds, [0])
        self.assertEqual(peer.peer.speed, 0)
        self.assertTrue(peer.writes)
        self.assertTrue(all(path == peer.characteristic_path and options == {"type": "request"}
                            for path, data, options in peer.writes))

    def test_unknown_characteristic_is_never_written(self):
        peer = FakeBlueZ(characteristic_uuid="0000ffe2-0000-1000-8000-00805f9b34fb")
        with mock.patch.dict(sys.modules, peer.modules()), self.assertRaises(RuntimeError):
            bt.BLELink(peer.address, 0.1)
        self.assertEqual(peer.writes, [])
        self.assertFalse(peer.connected)
        self.assertTrue(peer.closed)

    def test_disconnect_refuses_further_commands(self):
        peer = FakeBlueZ()
        with mock.patch.dict(sys.modules, peer.modules()), contextlib.redirect_stdout(io.StringIO()):
            link = bt.BLELink(peer.address, 0.1)
            peer.callback("org.bluez.Device1", {"Connected": False}, [], path=peer.device_path)
            with self.assertRaises(ConnectionError):
                link.send(b"A")
            with self.assertRaises(ConnectionError):
                link.receive()
            link.close()
        self.assertEqual(peer.writes, [])


@unittest.skipUnless(sys.platform.startswith("linux"), "pseudo terminal tests need Linux")
class SerialIntegrationTests(unittest.TestCase):
    def run_peer(self, command, echo=True, interrupt=False):
        import pty

        master, slave = pty.openpty()
        peer = FirmwareLink(echo=echo)
        script = Path(bt.__file__).resolve()
        process = subprocess.Popen(
            [sys.executable, str(script), "--port", os.ttyname(slave),
             "--timeout", "0.3"] + command,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        deadline = time.monotonic() + 5
        interrupted = False
        try:
            while process.poll() is None:
                if time.monotonic() > deadline:
                    self.fail("模拟串口测试超时")
                if select.select([master], [], [], 0.02)[0]:
                    peer.send(os.read(master, 4096))
                    reply = peer.receive()
                    if reply:
                        os.write(master, reply)
                    if interrupt and not interrupted and peer.activation_speeds and peer.speed == 100:
                        process.send_signal(signal.SIGTERM)
                        interrupted = True
            # Read final bytes buffered before the child closed its serial port.
            while select.select([master], [], [], 0)[0]:
                peer.send(os.read(master, 4096))
            out, err = process.communicate(timeout=1)
            return process.returncode, peer, out, err
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate()
            os.close(master)
            os.close(slave)

    def test_timed_motion_and_final_stop_over_serial(self):
        code, peer, out, err = self.run_peer(
            ["move", "forward", "--speed", "0.1", "--duration", "0.12"])
        self.assertEqual(code, 0, err)
        self.assertEqual(peer.activation_speeds, [0])
        self.assertIn(b"{0:100}{0:P}", peer.received)
        self.assertIn(bt.STOP + b"IA", peer.received)
        self.assertEqual(peer.speed, 0)
        self.assertTrue(peer.received.endswith(bt.STOP * 3 + b"{0:0}"))

    def test_no_echo_refuses_motion_over_serial(self):
        code, peer, out, err = self.run_peer(
            ["move", "forward", "--duration", "0.1"], echo=False)
        self.assertEqual(code, 1)
        self.assertIn("未收到速度参数回读", err)
        self.assertNotIn(b"AA", peer.received)
        self.assertEqual(peer.speed, 0)

    def test_status_is_read_only(self):
        code, peer, out, err = self.run_peer(["status", "--duration", "0.05"])
        self.assertEqual(code, 1)  # No peer telemetry was injected.
        self.assertEqual(peer.received, b"")

    def test_sigterm_sends_stop(self):
        code, peer, out, err = self.run_peer(
            ["move", "forward", "--duration", "3"], interrupt=True)
        self.assertEqual(code, 130, err)
        self.assertEqual(peer.speed, 0)
        self.assertTrue(peer.received.endswith(bt.STOP * 3 + b"{0:0}"))


if __name__ == "__main__":
    unittest.main()
