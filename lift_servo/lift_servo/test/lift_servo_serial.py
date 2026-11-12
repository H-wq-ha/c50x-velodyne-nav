#!/usr/bin/env python3
"""STM32 lift_servo 串口控制与联调工具。"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Callable, Iterable

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover
    raise SystemExit("缺少 pyserial，请执行：python3 -m pip install pyserial") from exc


DEFAULT_BAUD = 115200
MIN_SPEED_PPS = 1
MAX_SPEED_PPS = 20_000
MAX_WATCHDOG_MS = 60_000
MAX_PULSES = 0xFFFFFFFF


class CommunicationError(RuntimeError):
    """串口超时或响应格式错误。"""


class DeviceRejected(RuntimeError):
    """STM32 返回 ERR。"""


@dataclass(frozen=True)
class Response:
    command: str
    lines: tuple[str, ...]

    @property
    def primary(self) -> str:
        return self.lines[-1] if self.lines else ""


def available_ports() -> list:
    """返回稳定排序后的串口信息。"""
    return sorted(list_ports.comports(), key=lambda port: port.device)


def choose_port(requested: str | None) -> str:
    if requested:
        return requested

    candidates = [
        port
        for port in available_ports()
        if port.vid is not None
        or port.device.startswith("/dev/ttyACM")
        or port.device.startswith("/dev/ttyUSB")
    ]
    if len(candidates) == 1:
        return candidates[0].device
    if not candidates:
        raise CommunicationError(
            "未发现 USB 串口，请用 --port 指定，例如 --port /dev/ttyACM0"
        )
    devices = ", ".join(port.device for port in candidates)
    raise CommunicationError(f"发现多个 USB 串口，请用 --port 指定：{devices}")


def _bounded_int(name: str, value: int, minimum: int, maximum: int) -> int:
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} 必须在 {minimum}..{maximum} 范围内")
    return value


def normalize_direction(direction: str) -> str:
    normalized = direction.upper()
    if normalized not in {"FWD", "REV"}:
        raise ValueError("方向只能是 FWD 或 REV")
    return normalized


class LiftServoClient:
    """与 lift_servo V1 文本协议通信。"""

    def __init__(
        self,
        port: str,
        baud: int = DEFAULT_BAUD,
        timeout: float = 2.0,
        serial_factory: Callable[..., object] = serial.Serial,
    ) -> None:
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._serial = serial_factory(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=1.0,
        )

    def close(self) -> None:
        self._serial.close()

    def __enter__(self) -> "LiftServoClient":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def _read_line(self) -> str | None:
        raw = self._serial.readline()
        if not raw:
            return None
        return raw.decode("ascii", errors="replace").strip()

    def drain_startup_messages(self, quiet_time: float = 0.12) -> tuple[str, ...]:
        """读取上电 READY/STATUS 或上一条命令遗留消息。"""
        lines: list[str] = []
        quiet_deadline = time.monotonic() + quiet_time
        while time.monotonic() < quiet_deadline:
            line = self._read_line()
            if line:
                lines.append(line)
                quiet_deadline = time.monotonic() + quiet_time
        return tuple(lines)

    def execute(self, command: str) -> Response:
        command = " ".join(command.strip().upper().split())
        if not command or "\n" in command or "\r" in command:
            raise ValueError("命令不能为空或包含换行")

        # 防止把设备刚上电的 READY/STATUS 当成当前命令响应。
        self.drain_startup_messages()
        self._serial.write((command + "\n").encode("ascii"))
        self._serial.flush()

        deadline = time.monotonic() + self.timeout
        lines: list[str] = []
        expected_help_lines = 4 if command == "HELP" else 0
        while time.monotonic() < deadline:
            line = self._read_line()
            if not line:
                continue
            lines.append(line)

            if line.startswith("ERR "):
                raise DeviceRejected(line)
            if command == "STATUS" and line.startswith("STATUS "):
                return Response(command, tuple(lines))
            if command == "HELP" and len([x for x in lines if x.startswith("CMDS ")]) >= expected_help_lines:
                return Response(command, tuple(lines))
            if command not in {"STATUS", "HELP"} and line.startswith("OK "):
                return Response(command, tuple(lines))

        received = " | ".join(lines) if lines else "无数据"
        raise CommunicationError(f"等待 {command!r} 响应超时，已收到：{received}")

    def ping(self) -> Response:
        return self.execute("PING")

    def status(self) -> dict[str, str]:
        response = self.execute("STATUS")
        fields: dict[str, str] = {}
        for token in response.primary.split()[1:]:
            if "=" in token:
                key, value = token.split("=", 1)
                fields[key] = value
        if "state" not in fields:
            raise CommunicationError(f"STATUS 响应格式错误：{response.primary}")
        return fields

    def enable(self) -> Response:
        return self.execute("ENABLE")

    def disable(self) -> Response:
        return self.execute("DISABLE")

    def stop(self) -> Response:
        return self.execute("STOP")

    def emergency_stop(self) -> Response:
        return self.execute("ESTOP")

    def clear(self) -> Response:
        return self.execute("CLEAR")

    def set_speed(self, speed_pps: int) -> Response:
        speed = _bounded_int("speed_pps", speed_pps, MIN_SPEED_PPS, MAX_SPEED_PPS)
        return self.execute(f"SPEED {speed}")

    def set_watchdog(self, timeout_ms: int) -> Response:
        timeout = _bounded_int("watchdog_ms", timeout_ms, 0, MAX_WATCHDOG_MS)
        return self.execute(f"WATCHDOG {timeout}")

    def run(self, direction: str, speed_pps: int | None = None) -> Response:
        direction = normalize_direction(direction)
        suffix = ""
        if speed_pps is not None:
            speed = _bounded_int("speed_pps", speed_pps, MIN_SPEED_PPS, MAX_SPEED_PPS)
            suffix = f" {speed}"
        return self.execute(f"RUN {direction}{suffix}")

    def move(
        self, direction: str, pulses: int, speed_pps: int | None = None
    ) -> Response:
        direction = normalize_direction(direction)
        pulse_count = _bounded_int("pulses", pulses, 1, MAX_PULSES)
        suffix = ""
        if speed_pps is not None:
            speed = _bounded_int("speed_pps", speed_pps, MIN_SPEED_PPS, MAX_SPEED_PPS)
            suffix = f" {speed}"
        return self.execute(f"MOVE {direction} {pulse_count}{suffix}")


def format_status(fields: dict[str, str]) -> str:
    order = (
        "state", "dir", "enabled", "alarm", "pend", "speed_pps", "sent",
        "mode", "remaining", "watchdog_ms", "reason",
    )
    return "\n".join(f"{key:>13}: {fields[key]}" for key in order if key in fields)


def print_response(response: Response) -> None:
    for line in response.lines:
        print(line)


def print_ports() -> None:
    ports = available_ports()
    shown = 0
    for port in ports:
        if port.vid is not None or port.device.startswith(("/dev/ttyACM", "/dev/ttyUSB")):
            print(f"{port.device}\t{port.description}\t{port.hwid}")
            shown += 1
    if shown == 0:
        print("未发现 USB 串口")


def interactive_shell(client: LiftServoClient) -> None:
    print("已连接。输入固件命令；输入 quit 退出，Ctrl+C 会先发送 STOP。")
    print("建议先执行 STATUS，再用 MOVE FWD 10 10 / MOVE REV 10 10 低速测试。")
    while True:
        try:
            line = input("lift-servo> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            try:
                print_response(client.stop())
            except Exception as exc:  # best-effort stop on local interruption
                print(f"警告：STOP 发送失败：{exc}", file=sys.stderr)
            return
        if not line:
            continue
        if line.lower() in {"quit", "exit"}:
            return
        try:
            if line.upper() == "STATUS":
                print(format_status(client.status()))
            else:
                print_response(client.execute(line))
        except (CommunicationError, DeviceRejected, ValueError) as exc:
            print(f"错误：{exc}", file=sys.stderr)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="lift_servo STM32 串口控制工具")
    parser.add_argument("--port", help="串口，例如 /dev/ttyACM0；只有一个USB串口时可省略")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="波特率，默认115200")
    parser.add_argument("--timeout", type=float, default=2.0, help="响应超时秒数，默认2秒")
    subparsers = parser.add_subparsers(dest="action")

    subparsers.add_parser("ports", help="列出USB串口")
    for action in ("ping", "status", "enable", "disable", "stop", "estop", "clear", "help"):
        subparsers.add_parser(action)

    speed_parser = subparsers.add_parser("speed")
    speed_parser.add_argument("pps", type=int)

    watchdog_parser = subparsers.add_parser("watchdog")
    watchdog_parser.add_argument("milliseconds", type=int)

    run_parser = subparsers.add_parser("run")
    run_parser.add_argument("direction", choices=("FWD", "REV", "fwd", "rev"))
    run_parser.add_argument("pps", type=int, nargs="?")

    move_parser = subparsers.add_parser("move")
    move_parser.add_argument("direction", choices=("FWD", "REV", "fwd", "rev"))
    move_parser.add_argument("pulses", type=int)
    move_parser.add_argument("pps", type=int, nargs="?")

    raw_parser = subparsers.add_parser("raw")
    raw_parser.add_argument("command", nargs=argparse.REMAINDER)
    subparsers.add_parser("shell")
    return parser


def dispatch(client: LiftServoClient, args: argparse.Namespace) -> None:
    action = args.action or "shell"
    if action == "shell":
        interactive_shell(client)
    elif action == "status":
        print(format_status(client.status()))
    elif action == "ping":
        print_response(client.ping())
    elif action == "enable":
        print_response(client.enable())
    elif action == "disable":
        print_response(client.disable())
    elif action == "stop":
        print_response(client.stop())
    elif action == "estop":
        print_response(client.emergency_stop())
    elif action == "clear":
        print_response(client.clear())
    elif action == "help":
        print_response(client.execute("HELP"))
    elif action == "speed":
        print_response(client.set_speed(args.pps))
    elif action == "watchdog":
        print_response(client.set_watchdog(args.milliseconds))
    elif action == "run":
        print_response(client.run(args.direction, args.pps))
        print("连续运动已启动；请及时执行 stop，建议同时启用 WATCHDOG。")
    elif action == "move":
        print_response(client.move(args.direction, args.pulses, args.pps))
    elif action == "raw":
        if not args.command:
            raise ValueError("raw 后必须提供命令")
        print_response(client.execute(" ".join(args.command)))
    else:  # pragma: no cover
        raise ValueError(f"未知操作：{action}")


def main(argv: Iterable[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.action == "ports":
        print_ports()
        return 0

    try:
        port = choose_port(args.port)
        print(f"连接 {port} @ {args.baud} 8N1", file=sys.stderr)
        with LiftServoClient(port, args.baud, args.timeout) as client:
            dispatch(client, args)
        return 0
    except (serial.SerialException, CommunicationError, DeviceRejected, ValueError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
