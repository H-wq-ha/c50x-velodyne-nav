"""
WHEELTEC C50X 串口底盘控制工具

通过 USART1 或 USART3 (460800, 8N1) 发送控制指令控制底盘运动。
同时读取状态帧显示实时反馈。

控制协议:
  11字节控制帧:
  [0]   帧头 0x7B
  [1]   命令 (0x00=正常控制)
  [2]   保留字节 (0x00)
  [3-4] X轴速度 (高8位, 低8位, 有符号16位, 单位mm/s)
  [5-6] Y轴速度
  [7-8] Z轴角速度
  [9]   BCC 异或校验 (字节0~8)
  [10]  帧尾 0x7D

串口配置:
  - USART1: 460800, 8N1
  - USART3: 460800, 8N1

注意事项:
  - 底盘启动后有约10秒控制延迟 (CONTROL_DELAY=1000@100Hz)
  - 发送的速度将叠加在遥控器等已有的控制信号上
  - 停止: 发送 Vx=0, Vy=0, Vz=0
"""

import serial
import serial.tools.list_ports
import struct
import time
import threading
import sys
from enum import IntEnum
from typing import Optional, Tuple

# ==================== 协议常量 ====================
FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
FRAME_TAIL_AUTO_CHARGE = 0x7F
CONTROL_FRAME_SIZE = 11
STATUS_FRAME_SIZE = 24
BAUDRATE = 460800


class Command(IntEnum):
    """控制命令类型"""
    NORMAL_CONTROL = 0x00   # 正常速度控制
    AUTO_RECHARGE = 0x01    # 自动回充 (差速/麦轮/四驱/全向)
    AUTO_RECHARGE_AKM = 0x02  # 自动回充 + 导航 (阿克曼)
    DOCK_SPEED = 0x03       # 设置回充对接速度


def calc_bcc(data: bytes) -> int:
    """计算 BCC 异或校验和"""
    bcc = 0
    for b in data:
        bcc ^= b
    return bcc


def read_s16(high: int, low: int) -> int:
    """有符号16位大端解码"""
    val = (high << 8) | low
    if val >= 0x8000:
        val -= 0x10000
    return val


def build_control_frame(vx_mm_s: int, vy_mm_s: int, vz_mm_s: int,
                        command: int = Command.NORMAL_CONTROL) -> bytes:
    """
    构建 11 字节控制帧

    参数:
        vx_mm_s: X轴速度 (mm/s, 范围约-3500~+3500)
        vy_mm_s: Y轴速度 (mm/s)
        vz_mm_s: Z轴角速度 (mm/s, 阿克曼车型当作转向值)
        command: 命令类型

    返回:
        11字节控制帧

    示例:
        # 以 0.5m/s 前进
        frame = build_control_frame(500, 0, 0)

        # 以 -0.3m/s 后退, 同时以 0.2m/s 左移
        frame = build_control_frame(-300, 200, 0)

        # 原地顺时针旋转
        frame = build_control_frame(0, 0, 300)
    """
    packet = bytearray(CONTROL_FRAME_SIZE)
    packet[0] = FRAME_HEADER
    packet[1] = command & 0xFF
    packet[2] = 0x00  # 保留

    # 限制速度范围为有符号16位
    def clamp(val, lo=-32768, hi=32767):
        return max(lo, min(hi, val))

    for i, val in enumerate([clamp(vx_mm_s), clamp(vy_mm_s), clamp(vz_mm_s)]):
        packet[3 + i * 2] = (val >> 8) & 0xFF      # 高位
        packet[4 + i * 2] = val & 0xFF             # 低位

    packet[9] = calc_bcc(packet[:9])  # BCC
    packet[10] = FRAME_TAIL

    return bytes(packet)


def send_speed(ser: serial.Serial, vx: float, vy: float, vz: float):
    """
    发送速度指令 (单位为 m/s)

    参数:
        ser: 已打开的串口对象
        vx: X轴速度 (m/s, 正=前进)
        vy: Y轴速度 (m/s, 正=左移)
        vz: Z轴角速度 (m/s, 正=逆时针旋转)
    """
    vx_mm = int(vx * 1000)
    vy_mm = int(vy * 1000)
    vz_mm = int(vz * 1000)
    frame = build_control_frame(vx_mm, vy_mm, vz_mm)
    ser.write(frame)


def stop_robot(ser: serial.Serial):
    """发送停止指令"""
    send_speed(ser, 0, 0, 0)
    print("[控制] 已发送停止指令")


# ==================== 串口相关工具 ====================

def list_serial_ports() -> list:
    """列出可用串口"""
    return [(p.device, p.description) for p in serial.tools.list_ports.comports()]


def find_wheeltec_port() -> Optional[str]:
    """自动查找 WHEELTEC 串口"""
    for device, desc in list_serial_ports():
        if any(kw in desc.lower() for kw in ['usb', 'serial', 'uart', 'ch340', 'cp210', 'ftdi']):
            return device
    return None


def open_serial(port: str, baudrate: int = BAUDRATE) -> serial.Serial:
    """打开串口"""
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1,
    )
    return ser


# ==================== 键盘控制 ====================

def keyboard_control(ser: serial.Serial, speed: float = 0.5):
    """
    键盘控制底盘移动

    按键:
        W/↑   前进
        S/↓   后退
        A/←   左移
        D/→   右移
        Q/E   旋转
        Space 停止
        +/-   调节速度
        ESC   退出
    """
    try:
        import msvcrt  # Windows
        is_windows = True
    except ImportError:
        try:
            import termios
            import tty
            is_windows = False
        except ImportError:
            print("错误: 不支持的平台，需要 msvcrt (Windows) 或 termios (Linux)")
            return

    current_speed = speed
    vel_x, vel_y, vel_z = 0.0, 0.0, 0.0

    print("=" * 50)
    print("WHEELTEC C50X 键盘控制模式")
    print("=" * 50)
    print(f"当前速度: {current_speed:.1f} m/s")
    print("")
    print("  W/↑     - 前进")
    print("  S/↓     - 后退")
    print("  A/←     - 左移")
    print("  D/→     - 右移")
    print("  Q       - 逆时针旋转")
    print("  E       - 顺时针旋转")
    print("  Space   - 停止")
    print("  +/=     - 加速 (+0.1 m/s)")
    print("  -/_     - 减速 (-0.1 m/s)")
    print("  ESC/Ctrl+C - 退出")
    print("")
    print("提示: 底盘启动后有约10秒控制延迟")
    print("=" * 50)

    def print_status():
        nonlocal vel_x, vel_y, vel_z
        status = f"Vx={vel_x:+.2f}  Vy={vel_y:+.2f}  Vz={vel_z:+.2f}  | 速度档位={current_speed:.1f} m/s"
        print(f"\r{status}", end="", flush=True)

    if is_windows:
        print("\n开始控制 (按 ESC 退出):")

        while True:
            if msvcrt.kbhit():
                key = msvcrt.getch()

                if key == b'\x1b':  # ESC
                    stop_robot(ser)
                    print("\n退出键盘控制")
                    break

                elif key in (b'w', b'W'):        # 前进
                    vel_x = current_speed
                elif key in (b's', b'S'):        # 后退
                    vel_x = -current_speed
                elif key in (b'a', b'A'):        # 左移
                    vel_y = current_speed
                elif key in (b'd', b'D'):        # 右移
                    vel_y = -current_speed
                elif key in (b'q', b'Q'):        # 逆时针
                    vel_z = current_speed
                elif key in (b'e', b'E'):        # 顺时针
                    vel_z = -current_speed
                elif key == b' ':                # 停止
                    vel_x = vel_y = vel_z = 0.0

                elif key == b'+' or key == b'=':  # 加速
                    current_speed = min(3.0, current_speed + 0.1)
                    if vel_x != 0: vel_x = current_speed if vel_x > 0 else -current_speed
                    if vel_y != 0: vel_y = current_speed if vel_y > 0 else -current_speed
                    if vel_z != 0: vel_z = current_speed if vel_z > 0 else -current_speed
                elif key == b'-' or key == b'_':  # 减速
                    current_speed = max(0.1, current_speed - 0.1)
                    if vel_x != 0: vel_x = current_speed if vel_x > 0 else -current_speed
                    if vel_y != 0: vel_y = current_speed if vel_y > 0 else -current_speed
                    if vel_z != 0: vel_z = current_speed if vel_z > 0 else -current_speed

                # 方向键 (箭头键是两字节序列: 0xE0 + 方向码)
                elif key == b'\xe0':
                    arrow = msvcrt.getch()
                    if arrow == b'H':      # ↑ 前进
                        vel_x = current_speed
                    elif arrow == b'P':    # ↓ 后退
                        vel_x = -current_speed
                    elif arrow == b'K':    # ← 左移
                        vel_y = current_speed
                    elif arrow == b'M':    # → 右移
                        vel_y = -current_speed

                send_speed(ser, vel_x, vel_y, vel_z)
                print_status()

            time.sleep(0.05)

    else:
        # Linux 环境下可使用终端原始模式
        import termios
        import tty
        import select

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(sys.stdin.fileno())
            print("\n开始控制 (按 q 退出):")

            while True:
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    key = sys.stdin.read(1)

                    if key == '\x1b':  # ESC 或方向键起始
                        # 尝试读取后续字符 (方向键是 \x1b[A 等)
                        if select.select([sys.stdin], [], [], 0.05)[0]:
                            key += sys.stdin.read(2)
                        else:
                            stop_robot(ser)
                            print("\n退出键盘控制")
                            break

                    if key == 'q' or key == 'Q':
                        stop_robot(ser)
                        print("\n退出键盘控制")
                        break
                    elif key in ('w', 'W'):
                        vel_x = current_speed
                    elif key in ('s', 'S'):
                        vel_x = -current_speed
                    elif key in ('a', 'A'):
                        vel_y = current_speed
                    elif key in ('d', 'D'):
                        vel_y = -current_speed
                    elif key in ('z', 'Z'):
                        vel_z = current_speed
                    elif key in ('c', 'C'):
                        vel_z = -current_speed
                    elif key == ' ':
                        vel_x = vel_y = vel_z = 0.0
                    elif key in ('+', '='):
                        current_speed = min(3.0, current_speed + 0.1)
                    elif key in ('-', '_'):
                        current_speed = max(0.1, current_speed - 0.1)
                    elif key == '\x1b[A':  # ↑
                        vel_x = current_speed
                    elif key == '\x1b[B':  # ↓
                        vel_x = -current_speed
                    elif key == '\x1b[D':  # ←
                        vel_y = current_speed
                    elif key == '\x1b[C':  # →
                        vel_y = -current_speed

                    send_speed(ser, vel_x, vel_y, vel_z)
                    print_status()

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


# ==================== 序列控制 (预设动作) ====================

def sequence_control(ser: serial.Serial):
    """演示控制序列: 前进 -> 左移 -> 旋转 -> 停止"""
    print("\n[序列] 开始演示控制序列")

    steps = [
        (0.0, 0.0, 0.0, 0.5, "准备..."),
        (0.5, 0.0, 0.0, 2.0, "前进 0.5 m/s, 持续 2秒"),
        (0.0, 0.0, 0.0, 1.0, "停止"),
        (0.0, 0.5, 0.0, 2.0, "左移 0.5 m/s, 持续 2秒"),
        (0.0, 0.0, 0.0, 1.0, "停止"),
        (0.0, 0.0, 0.5, 2.0, "逆时针旋转 0.5, 持续 2秒"),
        (0.0, 0.0, 0.0, 1.0, "停止"),
        (0.3, 0.3, 0.0, 2.0, "前进+左移 复合运动 (0.3, 0.3)"),
        (0.0, 0.0, 0.0, 1.0, "停止"),
    ]

    for vx, vy, vz, duration, desc in steps:
        print(f"  -> {desc}")
        send_speed(ser, vx, vy, vz)
        time.sleep(duration)

    stop_robot(ser)
    print("[序列] 演示完成")


# ==================== 命令行入口 ====================

def main():
    import argparse

    parser = argparse.ArgumentParser(description='WHEELTEC C50X 串口底盘控制')
    parser.add_argument('port', nargs='?', default=None, help='串口名称 (如 COM3)')
    parser.add_argument('--baudrate', type=int, default=BAUDRATE, help=f'波特率 (默认 {BAUDRATE})')
    parser.add_argument('--mode', choices=['keyboard', 'sequence', 'direct'], default='keyboard',
                        help='控制模式: keyboard(键盘), sequence(预设序列), direct(命令行参数)')
    parser.add_argument('--vx', type=float, default=0.0, help='X速度 (m/s, direct模式)')
    parser.add_argument('--vy', type=float, default=0.0, help='Y速度 (m/s, direct模式)')
    parser.add_argument('--vz', type=float, default=0.0, help='Z角速度 (m/s, direct模式)')
    parser.add_argument('--time', type=float, default=2.0, help='持续秒数 (direct模式, 默认2秒)')
    parser.add_argument('--speed', type=float, default=0.5, help='键盘模式初始速度 (m/s, 默认0.5)')
    parser.add_argument('--list', action='store_true', help='列出可用串口')
    args = parser.parse_args()

    if args.list:
        print("可用串口:")
        for device, desc in list_serial_ports():
            print(f"  {device}: {desc}")
        return

    port = args.port
    if not port:
        port = find_wheeltec_port()
        if not port:
            print("错误: 未指定串口，且无法自动检测。请:")
            print("  1. 使用 --list 查看可用串口")
            print("  2. 指定串口: python serial_test_control.py COM3")
            return
        print(f"自动检测到串口: {port}")

    try:
        ser = open_serial(port, args.baudrate)
    except serial.SerialException as e:
        print(f"错误: 无法打开串口 {port}: {e}")
        return

    print(f"[串口] 已连接 {port} @ {args.baudrate} baud")
    print(f"[提示] 底盘启动后有约10秒控制延迟，请确保底盘已上电并完成初始化")

    try:
        if args.mode == 'keyboard':
            keyboard_control(ser, speed=args.speed)
        elif args.mode == 'sequence':
            sequence_control(ser)
        elif args.mode == 'direct':
            print(f"[控制] 发送 vx={args.vx}, vy={args.vy}, vz={args.vz}, 持续 {args.time} 秒")
            send_speed(ser, args.vx, args.vy, args.vz)
            time.sleep(args.time)
            stop_robot(ser)
            print("[控制] 完成")
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        stop_robot(ser)
        ser.close()
        print("[串口] 已断开")


if __name__ == '__main__':
    main()
