"""USB 串口自动选择，供接收、绘图和标定采集共用；选择时不打开设备。"""
from __future__ import annotations

import sys


def select_serial_port(requested: str | None = None) -> str:
    """显式串口优先；唯一 USB 串口自动选择，多个候选时在终端选择。"""
    if requested and requested.lower() != "auto":
        return requested

    from serial.tools import list_ports

    candidates = {}
    for port in list_ports.comports():
        device = port.device.lower()
        usb_name = device.startswith(("/dev/ttyusb", "/dev/ttyacm", "/dev/cu.usb",
                                      "/dev/tty.usb", "/dev/cu.wchusb", "/dev/tty.wchusb"))
        if (port.vid is not None or port.pid is not None
                or "USB" in (port.hwid or "").upper() or usb_name):
            candidates[port.device] = port
    ports = sorted(candidates.values(), key=lambda port: port.device)
    if not ports:
        raise OSError("未检测到 USB 串口，请检查连接；其他类型的串口请用 --port 手动指定。")
    if len(ports) == 1:
        port = ports[0]
        print(f"自动选择串口：{port.device}（{port.description}）", file=sys.stderr)
        return port.device

    print("检测到多个 USB 串口：", file=sys.stderr)
    for index, port in enumerate(ports, 1):
        identity = f"，序列号 {port.serial_number}" if port.serial_number else ""
        print(f"  {index}. {port.device}  {port.description}{identity}", file=sys.stderr)
    if not sys.stdin.isatty():
        raise OSError("存在多个 USB 串口；非交互运行请用 --port 明确指定设备。")
    while True:
        print(f"输入串口序号 1–{len(ports)}：", end="", file=sys.stderr, flush=True)
        try:
            choice = input().strip()
        except (EOFError, KeyboardInterrupt) as exc:
            raise OSError("串口选择已取消。") from exc
        if choice.isdigit() and 1 <= int(choice) <= len(ports):
            selected = ports[int(choice) - 1]
            print(f"选择串口：{selected.device}", file=sys.stderr)
            return selected.device
        print("序号无效，请重新输入。", file=sys.stderr)
