#!/usr/bin/env python3
"""Control the existing UART4 Bluetooth APP protocol without ROS2.

Use Linux BLE (--ble), a Bluetooth SPP serial port, or an RFCOMM socket.
No physical connection is opened by --dry-run. See bluetooth_chassis_control.md.
"""

import argparse
import math
import re
import select
import signal
import socket
import sys
import time


# UART4_IRQHandler: K clears TurnFlag/DirectionFlag; I selects joystick mode.
STOP = b"K\x00I\x00"
REQUEST_PARAMETERS = b"{0:P}"
BLE_SERVICE = "0000ffe0-0000-1000-8000-00805f9b34fb"
BLE_CHARACTERISTIC = "0000ffe1-0000-1000-8000-00805f9b34fb"
DIRECTIONS = {
    "forward": b"A", "forward-right": b"B", "right": b"C",
    "backward-right": b"D", "backward": b"E", "backward-left": b"F",
    "left": b"G", "forward-left": b"H",
    "strafe-left": b"G", "strafe-right": b"C",
}
KEYS = dict(zip("wsadqezcjl", (
    "forward", "backward", "left", "right", "forward-left", "forward-right",
    "backward-left", "backward-right", "strafe-left", "strafe-right",
)))


def speed_packet(speed):
    """APP parameter 0 is integer mm/s; do not request Flash saving."""
    if not math.isfinite(speed) or not 0 <= speed <= 1.0:
        raise ValueError("速度必须在 0～1.0 m/s 之间")
    return ("{0:%d}" % round(speed * 1000)).encode("ascii")


def action_packet(action, chassis):
    """Return page selector + direction; actual kinematics run on the STM32."""
    if action == "stop":
        return STOP
    if action not in DIRECTIONS:
        raise ValueError("未知动作: " + action)
    omni = chassis in ("mecanum", "omni")
    if action.startswith("strafe-") and not omni:
        raise ValueError("只有 mecanum / omni 车型支持横移")
    if action in ("left", "right"):
        if chassis == "ackermann":
            raise ValueError("阿克曼底盘不能原地旋转，请使用 forward-left / forward-right")
        return (b"K" if omni else b"I") + DIRECTIONS[action]
    return b"I" + DIRECTIONS[action]


class Telemetry:
    """Extract fragmented {A...}$ / {B...}$ / {C...}$ APP messages."""

    def __init__(self):
        self.buffer = bytearray()
        self.latest = {}
        self.last_received = None

    def feed(self, chunk):
        self.buffer.extend(chunk)
        records = []
        while self.buffer:
            start = self.buffer.find(b"{")
            if start < 0:
                self.buffer.clear()
                break
            del self.buffer[:start]
            end = self.buffer.find(b"}$")
            nested = self.buffer.find(b"{", 1)
            if nested >= 0 and (end < 0 or nested < end):
                del self.buffer[:nested]
                continue
            if end < 0:
                if len(self.buffer) > 256:
                    self.buffer.clear()
                break
            body = bytes(self.buffer[1:end])
            del self.buffer[:end + 2]
            if len(body) > 256 or not re.fullmatch(rb"[ABC]-?\d+(?::-?\d+)*", body):
                continue
            kind = chr(body[0])
            values = tuple(int(v) for v in body[1:].split(b":"))
            if len(values) != {"A": 4, "B": 3, "C": 9}[kind]:
                continue
            self.latest[kind] = values
            self.last_received = time.monotonic()
            records.append((kind, values))
        return records


class SerialLink:
    def __init__(self, port, baudrate):
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("请先安装依赖: python3 -m pip install pyserial") from exc
        options = dict(baudrate=baudrate, timeout=0, write_timeout=0.3)
        if sys.platform.startswith("linux"):
            options["exclusive"] = True
        self.device = serial.Serial(port, **options)

    def send(self, payload):
        if self.device.write(payload) != len(payload):
            raise OSError("串口写入不完整")

    def receive(self):
        return self.device.read(min(self.device.in_waiting, 4096))

    def close(self):
        self.device.close()


class RFCOMMLink:
    def __init__(self, address, channel):
        if not all(hasattr(socket, n) for n in ("AF_BLUETOOTH", "BTPROTO_RFCOMM")):
            raise RuntimeError("此 Python 未提供 RFCOMM 支持，请使用 --port 蓝牙虚拟串口")
        self.device = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM,
                                    socket.BTPROTO_RFCOMM)
        try:
            self.device.settimeout(5)
            self.device.connect((address, channel))
            self.device.settimeout(0.3)
        except BaseException:
            self.device.close()
            raise

    def send(self, payload):
        self.device.sendall(payload)

    def receive(self):
        if not select.select([self.device], [], [], 0)[0]:
            return b""
        data = self.device.recv(4096)
        if not data:
            raise ConnectionError("蓝牙连接已断开")
        return data

    def close(self):
        self.device.close()


class BLELink:
    """BlueZ GATT transport for the verified WHEELTEC-IOS FFE0/FFE1 service.

    Keep a private D-Bus connection so discovery and notification ownership are
    released on close. FFE2 is deliberately unused: it is not our UART endpoint.
    """

    def __init__(self, address, connect_timeout=15):
        if not sys.platform.startswith("linux"):
            raise RuntimeError("--ble 当前使用 Linux BlueZ；其他系统可使用 SPP 虚拟串口")
        try:
            import dbus
            from dbus.mainloop.glib import DBusGMainLoop
            from gi.repository import GLib
        except ImportError as exc:
            raise RuntimeError("BLE 需要系统 D-Bus/GLib 支持，请使用 /usr/bin/python3；"
                               "缺失时安装 python3-dbus 和 python3-gi") from exc
        self.dbus = dbus
        self.context = GLib.MainContext.default()
        self.bus = None
        self.device = None
        self.characteristic = None
        self.subscription = None
        self.adapter = None
        self.discovering = False
        self.owns_connection = False
        self.notifying = False
        self.disconnected = False
        self.overflow = False
        self.buffer = bytearray()
        try:
            DBusGMainLoop(set_as_default=True)
            self.bus = dbus.SystemBus(private=True)
            self.manager = self.interface("/", "org.freedesktop.DBus.ObjectManager")
            objects = self.manager.GetManagedObjects()
            adapters = sorted(str(p) for p, i in objects.items() if "org.bluez.Adapter1" in i)
            if not adapters:
                raise RuntimeError("没有找到蓝牙适配器，请检查 BlueZ 服务和蓝牙电源")
            adapter_path = adapters[0]
            if not objects[adapter_path]["org.bluez.Adapter1"].get("Powered", False):
                raise RuntimeError("蓝牙适配器未开启，请先开启主机蓝牙")
            self.device_path = adapter_path + "/dev_" + address.upper().replace(":", "_")
            self.adapter = self.interface(adapter_path, "org.bluez.Adapter1")
            if self.device_path not in objects:
                self.adapter.SetDiscoveryFilter(dbus.Dictionary({"Transport": "le"}, signature="sv"))
                self.adapter.StartDiscovery()
                self.discovering = True
                self.wait_for(lambda: self.device_path in self.manager.GetManagedObjects(),
                              connect_timeout, "未发现指定 BLE 设备，请确认车辆供电且未被其他设备占用")
            properties = self.interface(self.device_path, "org.freedesktop.DBus.Properties")
            if properties.Get("org.bluez.Device1", "Connected"):
                raise RuntimeError("设备已被本机其他会话连接，请先关闭原连接再使用工具")
            self.device = self.interface(self.device_path, "org.bluez.Device1")
            # Even a timed-out Connect can have established a link; clean it up.
            self.owns_connection = True
            self.device.Connect(timeout=connect_timeout)
            self.wait_for(lambda: bool(properties.Get("org.bluez.Device1", "ServicesResolved")),
                          connect_timeout, "BLE 服务发现超时")
            objects = self.manager.GetManagedObjects()
            services = {str(p) for p, i in objects.items()
                        if str(p).startswith(self.device_path + "/")
                        and str(i.get("org.bluez.GattService1", {}).get("UUID", "")).lower() == BLE_SERVICE}
            matches = [(str(p), i["org.bluez.GattCharacteristic1"]) for p, i in objects.items()
                       if "org.bluez.GattCharacteristic1" in i
                       and str(i["org.bluez.GattCharacteristic1"].get("Service", "")) in services
                       and str(i["org.bluez.GattCharacteristic1"].get("UUID", "")).lower() == BLE_CHARACTERISTIC]
            if len(matches) != 1:
                raise RuntimeError("未找到 WHEELTEC 的 FFE0/FFE1 透传特征，拒绝向其他特征发送指令")
            self.characteristic_path, info = matches[0]
            flags = set(str(f) for f in info.get("Flags", []))
            if not {"write", "notify"}.issubset(flags):
                raise RuntimeError("FFE1 缺少 write / notify 能力，无法进行双向控制")
            self.characteristic = self.interface(self.characteristic_path, "org.bluez.GattCharacteristic1")
            self.subscription = self.bus.add_signal_receiver(
                self.on_properties, dbus_interface="org.freedesktop.DBus.Properties",
                signal_name="PropertiesChanged", path_keyword="path")
            self.characteristic.StartNotify(timeout=5)
            self.notifying = True
            print(f"BLE 已连接 {address.upper()}，使用 FFE1 透传。", flush=True)
        except Exception as exc:
            self.close()
            raise RuntimeError("BLE 连接失败: " + str(exc)) from exc
        except BaseException:
            self.close()
            raise
        finally:
            self.stop_discovery()

    def interface(self, path, name):
        obj = self.bus.get_object("org.bluez", path, introspect=False)
        return self.dbus.Interface(obj, name)

    def pump(self):
        for _ in range(128):
            if not self.context.pending():
                break
            self.context.iteration(False)

    def wait_for(self, predicate, timeout, message):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            if predicate():
                return
            time.sleep(0.05)
        raise TimeoutError(message)

    def on_properties(self, interface, values, invalid, path=None):
        if str(path) == self.device_path and interface == "org.bluez.Device1":
            if "Connected" in values and not values["Connected"]:
                self.disconnected = True
        if (str(path) == self.characteristic_path
                and interface == "org.bluez.GattCharacteristic1" and "Value" in values):
            chunk = bytes(values["Value"])
            if len(self.buffer) + len(chunk) > 65536:
                self.overflow = True
            else:
                self.buffer.extend(chunk)

    def send(self, payload):
        self.pump()
        if self.disconnected:
            raise ConnectionError("BLE 连接已断开，无法发送指令")
        if len(payload) > 20:
            raise ValueError("单次 BLE APP 指令不得超过 20 字节")
        try:
            self.characteristic.WriteValue(
                self.dbus.Array(payload, signature="y"),
                self.dbus.Dictionary({"type": "request"}, signature="sv"), timeout=1)
        except Exception as exc:
            raise OSError("BLE 指令写入失败: " + str(exc)) from exc

    def receive(self):
        self.pump()
        if self.disconnected:
            raise ConnectionError("BLE 连接已断开")
        if self.overflow:
            raise OSError("BLE 接收缓冲区溢出，结束控制")
        data = bytes(self.buffer)
        self.buffer.clear()
        return data

    def stop_discovery(self):
        if self.discovering:
            try:
                self.adapter.StopDiscovery(timeout=3)
            except Exception:
                pass
            self.discovering = False

    def close(self):
        self.stop_discovery()
        if self.notifying:
            try:
                self.characteristic.StopNotify(timeout=3)
            except Exception:
                pass
            self.notifying = False
        if self.subscription is not None:
            self.subscription.remove()
            self.subscription = None
        if self.owns_connection:
            try:
                self.device.Disconnect(timeout=3)
            except Exception:
                pass
            self.owns_connection = False
        if self.bus is not None:
            self.bus.close()
            self.bus = None


class Controller:
    def __init__(self, link, chassis, timeout):
        self.link = link
        self.chassis = chassis
        self.timeout = timeout
        self.telemetry = Telemetry()
        self.action = "stop"
        self.last_display = 0.0

    def poll(self, check_link=False):
        records = self.telemetry.feed(self.link.receive())
        now = time.monotonic()
        if records and now - self.last_display >= 1.0:
            if "A" in self.telemetry.latest:
                left, right, battery, gyro_z = self.telemetry.latest["A"]
                print(f"反馈: 轮速显示值={left}/{right}，电量显示值={battery}% ，"
                      f"陀螺仪Z原始值={gyro_z}", flush=True)
                self.last_display = now
        last = self.telemetry.last_received
        if check_link and (last is None or now - last > self.timeout):
            raise TimeoutError("未持续收到 APP 反馈，结束控制并尝试停车")
        return records

    def set_speed(self, speed):
        # Drain old parameter reports before asking for a fresh echo.
        self.poll()
        self.link.send(speed_packet(speed) + REQUEST_PARAMETERS)
        expected = round(speed * 1000)
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            for kind, values in self.poll():
                if kind == "C" and values[0] == expected:
                    return
            time.sleep(0.01)
        raise TimeoutError("未收到速度参数回读；请确认连接的是 UART4 蓝牙模块，"
                           "底盘已完成约 13 秒上电自检")

    def initialize(self, speed):
        self.link.send(STOP)
        # Consecutive A activates APP mode. Prove speed is zero before sending A.
        self.set_speed(0)
        self.link.send(b"AA" + STOP)
        time.sleep(0.1)
        self.set_speed(speed)
        print(f"已发送 APP 模式切换序列；速度回读 {speed:.3f} m/s。"
              "固件未提供控制模式确认。", flush=True)

    def send_action(self, action):
        packet = action_packet(action, self.chassis)
        if action == "stop":
            self.link.send(STOP)
        elif action != self.action:
            # Omni Get_APPcmd leaves Vz unchanged in translation cases. Allow
            # the 100 Hz task to process STOP before setting another direction.
            self.link.send(STOP)
            time.sleep(0.03)
            self.link.send(packet)
        else:
            self.link.send(packet[-1:])
        self.action = action

    def stop(self):
        # A stop is a motion command, not the board's hardware emergency stop.
        for _ in range(3):
            self.link.send(STOP)
            time.sleep(0.05)
        self.link.send(speed_packet(0))


def timed_move(controller, args):
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        controller.poll(check_link=True)
        controller.send_action(args.action)
        time.sleep(min(1 / args.rate, max(0, deadline - time.monotonic())))


def keyboard_control(controller, args):
    import termios
    import tty

    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    action, last_key, speed = "stop", 0.0, args.speed
    print("W/S 前后；A/D 旋转；Q/E 前进左/右转；Z/C 后退左/右转；"
          "J/L 横移；+/- 调速；空格停车；Ctrl+C 退出。")
    print("持续收到方向按键才保持运动；无按键 0.35 秒后停车。", flush=True)
    try:
        tty.setcbreak(fd)
        while True:
            ready = select.select([sys.stdin], [], [], 1 / args.rate)[0]
            if ready:
                key = sys.stdin.read(1).lower()
                if not key or key == "\x03":
                    break
                if key in KEYS:
                    candidate = KEYS[key]
                    try:
                        action_packet(candidate, args.chassis)
                    except ValueError as exc:
                        print(str(exc), flush=True)
                        action = "stop"
                    else:
                        action, last_key = candidate, time.monotonic()
                elif key in ("+", "=", "-"):
                    action = "stop"
                    controller.send_action("stop")
                    speed = round(min(1.0, max(0.0, speed + (-0.05 if key == "-" else 0.05))), 3)
                    controller.set_speed(speed)
                    print(f"速度: {speed:.3f} m/s", flush=True)
                else:
                    action = "stop"
            if time.monotonic() - last_key >= 0.35:
                action = "stop"
            controller.poll(check_link=True)
            controller.send_action(action)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("必须是大于零的有限数值")
    return number


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    connection = parser.add_mutually_exclusive_group()
    connection.add_argument("--port", help="蓝牙虚拟串口，例如 /dev/rfcomm0 或 COM6")
    connection.add_argument("--address", help="蓝牙 SPP 地址，例如 AA:BB:CC:DD:EE:FF")
    connection.add_argument("--ble", metavar="ADDRESS", help="BLE 地址，例如实测 WHEELTEC-IOS 的 2A:A2:19:08:05:45")
    parser.add_argument("--connect-timeout", type=positive_float, default=15.0, help="BLE 扫描和连接超时秒数，默认 15")
    parser.add_argument("--channel", type=int, default=1, help="RFCOMM 服务通道，默认 1，按设备修改")
    parser.add_argument("--baudrate", type=int, default=9600, help="虚拟串口速率，默认 9600")
    parser.add_argument("--chassis", choices=("4wd", "diff", "ackermann", "mecanum", "omni"), default="4wd")
    parser.add_argument("--timeout", type=positive_float, default=2.0, help="反馈超时秒数，默认 2")
    parser.add_argument("--rate", type=positive_float, default=20.0, help="指令发送频率，默认 20 Hz")
    parser.add_argument("--dry-run", action="store_true", help="显示协议字节，不连接设备")
    sub = parser.add_subparsers(dest="command", required=True)
    move = sub.add_parser("move", help="按指定动作运行一段时间后停车")
    move.add_argument("action", choices=tuple(DIRECTIONS))
    move.add_argument("--speed", type=float, default=0.1, help="遥控速度基准 m/s，默认 0.1，范围 0～1")
    move.add_argument("--duration", type=positive_float, default=2.0, help="运行秒数，默认 2")
    keyboard = sub.add_parser("keyboard", help="终端键盘遥控，需 Linux/macOS 交互终端")
    keyboard.add_argument("--speed", type=float, default=0.1)
    sub.add_parser("stop", help="重复发送 APP 停车指令并将遥控速度设为零")
    status = sub.add_parser("status", help="只读 APP 回传，不发送任何指令")
    status.add_argument("--duration", type=positive_float, default=5.0)
    args = parser.parse_args(argv)
    try:
        if not 1 <= args.channel <= 30 or args.baudrate <= 0:
            raise ValueError("RFCOMM 通道应为 1～30，串口速率必须大于零")
        if not 5 <= args.rate <= 50:
            raise ValueError("发送频率必须在 5～50 Hz 之间")
        address = args.ble or args.address
        if address and not re.fullmatch(r"(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}", address):
            raise ValueError("蓝牙地址格式应为 AA:BB:CC:DD:EE:FF")
        if hasattr(args, "speed"):
            speed_packet(args.speed)
        if args.command == "move":
            action_packet(args.action, args.chassis)
        if not args.dry_run and not (args.port or args.address or args.ble):
            raise ValueError("请指定 --ble、--port 或 --address；预览使用 --dry-run")
        if args.command == "keyboard" and not args.dry_run:
            if sys.platform == "win32" or not sys.stdin.isatty():
                raise ValueError("keyboard 需要 Linux/macOS 交互终端；可使用 move 定时运动")
    except ValueError as exc:
        parser.error(str(exc))
    return args


def preview(args):
    if args.command == "status":
        print("只接收 {A...}$ / {B...}$ / {C...}$，不发送指令")
        return
    steps = [("停车", STOP)]
    if args.command in ("move", "keyboard"):
        steps += [("设为零速并请求回读", speed_packet(0) + REQUEST_PARAMETERS),
                  ("确认零速后切换 APP 模式并停车", b"AA" + STOP),
                  ("设置运行速度并请求回读", speed_packet(args.speed) + REQUEST_PARAMETERS)]
        if args.command == "move":
            steps.append(("方向切换前停车并等待 30 ms", STOP))
            steps.append((f"{args.action}，持续 {args.duration:g} 秒，{args.rate:g} Hz",
                          action_packet(args.action, args.chassis)))
        else:
            steps.append(("示例：W 前进", STOP + action_packet("forward", args.chassis)))
    steps += [("退出时重复停车三次", STOP), ("遥控速度归零", speed_packet(0))]
    for label, payload in steps:
        print(f"{label}: {payload!r} | HEX {payload.hex(' ')}")


def main(argv=None):
    args = parse_args(argv)
    if args.dry_run:
        preview(args)
        return 0
    controller = None
    result = 0
    old_term = signal.getsignal(signal.SIGTERM)
    old_int = signal.getsignal(signal.SIGINT)

    def terminate(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    try:
        if args.ble:
            link = BLELink(args.ble, args.connect_timeout)
        elif args.port:
            link = SerialLink(args.port, args.baudrate)
        else:
            link = RFCOMMLink(args.address, args.channel)
        controller = Controller(link, args.chassis, args.timeout)
        if args.command == "status":
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                controller.poll()
                time.sleep(0.01)
            if controller.telemetry.last_received is None:
                raise TimeoutError("未收到有效 APP 反馈；请检查蓝牙服务、UART4 连接及串口速率")
        elif args.command != "stop":
            controller.initialize(args.speed)
            if args.command == "move":
                timed_move(controller, args)
            else:
                keyboard_control(controller, args)
    except KeyboardInterrupt:
        print("收到退出请求。", flush=True)
        result = 130
    except (OSError, RuntimeError, ValueError) as exc:
        print("错误: " + str(exc), file=sys.stderr)
        result = 1
    finally:
        if controller is not None:
            try:
                if args.command != "status":
                    # Finish the bounded stop sequence even on another Ctrl+C/SIGTERM.
                    signal.signal(signal.SIGINT, signal.SIG_IGN)
                    signal.signal(signal.SIGTERM, signal.SIG_IGN)
                    controller.stop()
                    print("停车和零速指令已写入连接；固件无停车确认帧。", flush=True)
            except OSError as exc:
                print(f"停车指令发送失败，无法确认底盘已停止: {exc}", file=sys.stderr)
                result = 1
            finally:
                controller.link.close()
        signal.signal(signal.SIGTERM, old_term)
        signal.signal(signal.SIGINT, old_int)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
