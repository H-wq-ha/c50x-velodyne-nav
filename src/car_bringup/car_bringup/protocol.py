"""
WHEELTEC C50X 底盘串口通信协议层

从 serial_read.py 和 serial_test_control.py 提取，无 ROS 依赖。
"""
import serial
import serial.tools.list_ports
import math
import time
from dataclasses import dataclass
from typing import Optional

# ==================== 协议常量 ====================
FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
STATUS_FRAME_SIZE = 24
CONTROL_FRAME_SIZE = 11
BAUDRATE = 460800

# IMU 转换系数（与已烧录固件源码保持一致）
# 加速度计: ±2g, 16位 → m/s² : 1672 = 32768 / 2 / 9.8
ACCEL_SCALE = 1.0 / 1672.0
# 陀螺仪: ±500°/s, 16位 → rad/s : 500/32768 * π/180
GYRO_SCALE = 500.0 / 32768.0 * math.pi / 180.0


@dataclass
class RobotStatus:
    """底盘状态数据"""
    flag_stop: int = 0
    vel_x: float = 0.0      # m/s
    vel_y: float = 0.0      # m/s
    vel_z: float = 0.0      # rad/s（从角速度 mm/s 转来）
    accel_x: int = 0        # 原始值
    accel_y: int = 0
    accel_z: int = 0
    gyro_x: int = 0         # 原始值
    gyro_y: int = 0
    gyro_z: int = 0
    voltage: float = 0.0    # V
    timestamp: float = 0.0


def calc_bcc(data: bytes) -> int:
    """XOR 校验和"""
    bcc = 0
    for b in data:
        bcc ^= b
    return bcc


def read_s16(high: int, low: int) -> int:
    """有符号 16 位大端解码"""
    val = (high << 8) | low
    return val - 0x10000 if val >= 0x8000 else val


def parse_status_frame(data: bytes) -> Optional[RobotStatus]:
    """解析 24 字节状态帧 → RobotStatus"""
    if len(data) != STATUS_FRAME_SIZE:
        return None
    if data[0] != FRAME_HEADER or data[23] != FRAME_TAIL:
        return None
    if calc_bcc(data[:22]) != data[22]:
        return None

    return RobotStatus(
        flag_stop=data[1],
        vel_x=read_s16(data[2], data[3]) / 1000.0,
        vel_y=read_s16(data[4], data[5]) / 1000.0,
        vel_z=read_s16(data[6], data[7]) / 1000.0,
        accel_x=read_s16(data[8], data[9]),
        accel_y=read_s16(data[10], data[11]),
        accel_z=read_s16(data[12], data[13]),
        gyro_x=read_s16(data[14], data[15]),
        gyro_y=read_s16(data[16], data[17]),
        gyro_z=read_s16(data[18], data[19]),
        voltage=read_s16(data[20], data[21]) / 1000.0,
        timestamp=time.time(),
    )


def build_control_frame(vx_mm_s: int, vy_mm_s: int, vz_mm_s: int,
                        command: int = 0x00) -> bytes:
    """构建 11 字节控制帧"""
    def clamp(v, lo=-32768, hi=32767):
        return max(lo, min(hi, v))

    packet = bytearray(CONTROL_FRAME_SIZE)
    packet[0] = FRAME_HEADER
    packet[1] = command & 0xFF
    packet[2] = 0x00
    vals = [clamp(vx_mm_s), clamp(vy_mm_s), clamp(vz_mm_s)]
    for i, val in enumerate(vals):
        packet[3 + i*2] = (val >> 8) & 0xFF
        packet[4 + i*2] = val & 0xFF
    packet[9] = calc_bcc(packet[:9])
    packet[10] = FRAME_TAIL
    return bytes(packet)


def send_speed(ser: serial.Serial, vx: float, vy: float, vz: float):
    """发送速度指令（单位 m/s）"""
    vx_mm = int(vx * 1000)
    vy_mm = int(vy * 1000)
    vz_mm = int(vz * 1000)
    ser.write(build_control_frame(vx_mm, vy_mm, vz_mm))


def stop_robot(ser: serial.Serial):
    """发送停止指令"""
    send_speed(ser, 0, 0, 0)


def open_serial(port: str, baudrate: int = BAUDRATE) -> serial.Serial:
    """打开串口"""
    return serial.Serial(
        port=port, baudrate=baudrate,
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE, timeout=0.1,
    )


def find_wheeltec_port() -> Optional[str]:
    """自动检测 WHEELTEC 串口"""
    for port in serial.tools.list_ports.comports():
        desc = f'{port.device} {port.description}'
        if any(kw in desc.lower() for kw in
               ['usb', 'serial', 'uart', 'ch340', 'cp210', 'ftdi']):
            return port.device
    return None
