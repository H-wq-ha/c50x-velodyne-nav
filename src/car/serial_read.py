"""
WHEELTEC C50X 串口状态读取工具

从 USART1 (115200, 8N1) 读取底盘状态数据帧。
协议格式：
  - 状态帧: 24字节 (帧头0x7B + 22字节数据 + BCC异或校验 + 帧尾0x7D)
  - 速度单位: mm/s (有符号16位大端)
  - 电压: 原始值/1000 得到实际电压

串口配置:
  - USART1: PA9(TX) PA10(RX), 115200, 8N1
  - USART3: PB10(TX) PB11(RX), 115200, 8N1 (默认ROS口)
"""

import serial
import serial.tools.list_ports
import struct
import time
import threading
from dataclasses import dataclass
from typing import Optional, Callable

# ==================== 协议常量 ====================
FRAME_HEADER = 0x7B  # 帧头 '{'
FRAME_TAIL = 0x7D    # 帧尾 '}'
STATUS_FRAME_SIZE = 24  # 状态帧总长度
BAUDRATE = 115200
RATE_HZ = 20  # 底盘发送频率 20Hz


@dataclass
class RobotStatus:
    """底盘状态数据结构"""
    flag_stop: int = 0           # 电机停止标志 (0=使能, 1=停止)
    vel_x: float = 0.0           # X轴速度 (m/s)
    vel_y: float = 0.0           # Y轴速度 (m/s)
    vel_z: float = 0.0           # Z轴角速度 (m/s)
    accel_x: int = 0             # 加速度计X (原始值)
    accel_y: int = 0             # 加速度计Y (原始值)
    accel_z: int = 0             # 加速度计Z (原始值)
    gyro_x: int = 0              # 陀螺仪X (原始值)
    gyro_y: int = 0              # 陀螺仪Y (原始值)
    gyro_z: int = 0              # 陀螺仪Z (原始值)
    voltage: float = 0.0         # 电池电压 (V)
    timestamp: float = 0.0       # 接收时间戳

    def __str__(self) -> str:
        return (
            f"[{self.timestamp:.1f}] "
            f"Vx={self.vel_x:+.2f} Vy={self.vel_y:+.2f} Vz={self.vel_z:+.2f} "
            f"电压={self.voltage:.2f}V "
            f"停止={'是' if self.flag_stop else '否'}"
        )


def calc_bcc(data: bytes) -> int:
    """计算 BCC 异或校验和"""
    bcc = 0
    for b in data:
        bcc ^= b
    return bcc


def list_serial_ports() -> list:
    """列出可用串口"""
    ports = serial.tools.list_ports.comports()
    return [(p.device, p.description) for p in ports]


def read_s16(high: int, low: int) -> int:
    """将有符号16位大端字节解码为整数 (mm/s)"""
    val = (high << 8) | low
    if val >= 0x8000:
        val -= 0x10000
    return val


def parse_status_frame(data: bytes) -> Optional[RobotStatus]:
    """
    解析24字节状态帧

    字节分布:
      [0]   帧头 0x7B
      [1]   FlagStop
      [2-3] Vel X (高8位, 低8位)
      [4-5] Vel Y
      [6-7] Vel Z
      [8-9] 加速度 X
      [10-11] 加速度 Y
      [12-13] 加速度 Z
      [14-15] 陀螺仪 X
      [16-17] 陀螺仪 Y
      [18-19] 陀螺仪 Z
      [20-21] 电压 (V * 1000)
      [22]  BCC 校验
      [23] 帧尾 0x7D
    """
    if len(data) != STATUS_FRAME_SIZE:
        return None
    if data[0] != FRAME_HEADER or data[23] != FRAME_TAIL:
        return None

    # 校验 BCC (字节 0~21 异或)
    if calc_bcc(data[:22]) != data[22]:
        return None

    status = RobotStatus(
        flag_stop=data[1],
        vel_x=read_s16(data[2], data[3]) / 1000.0,    # mm/s -> m/s
        vel_y=read_s16(data[4], data[5]) / 1000.0,
        vel_z=read_s16(data[6], data[7]) / 1000.0,
        accel_x=read_s16(data[8], data[9]),
        accel_y=read_s16(data[10], data[11]),
        accel_z=read_s16(data[12], data[13]),
        gyro_x=read_s16(data[14], data[15]),
        gyro_y=read_s16(data[16], data[17]),
        gyro_z=read_s16(data[18], data[19]),
        voltage=read_s16(data[20], data[21]) / 1000.0,  # -> V
        timestamp=time.time(),
    )
    return status


class SerialReader:
    """
    WHEELTEC C50X 串口读取器

    用法:
        reader = SerialReader('COM3')
        reader.start()

        # 获取最新状态
        status = reader.get_status()
        if status:
            print(status)

        reader.stop()
    """

    def __init__(self, port: str, baudrate: int = BAUDRATE, callback: Optional[Callable[[RobotStatus], None]] = None):
        self.port = port
        self.baudrate = baudrate
        self.callback = callback
        self._serial: Optional[serial.Serial] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._latest_status: Optional[RobotStatus] = None
        self._buf = bytearray()

    def start(self):
        """启动串口读取"""
        if self._running:
            return

        self._serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        print(f"[串口] 已连接 {self.port} @ {self.baudrate} baud")

    def stop(self):
        """停止串口读取"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        if self._serial and self._serial.is_open:
            self._serial.close()
            print("[串口] 已断开")

    def get_status(self) -> Optional[RobotStatus]:
        """获取最新状态"""
        return self._latest_status

    def send_raw(self, data: bytes):
        """发送原始数据到串口"""
        if self._serial and self._serial.is_open:
            self._serial.write(data)

    def _read_loop(self):
        """后台读取线程"""
        while self._running:
            try:
                if not self._serial or not self._serial.is_open:
                    break

                # 读取可用数据
                n = self._serial.in_waiting
                if n > 0:
                    chunk = self._serial.read(n)
                    self._buf.extend(chunk)
                    self._parse_buffer()

                time.sleep(0.005)  # 5ms 轮询间隔
            except serial.SerialException as e:
                print(f"[串口错误] {e}")
                break
            except Exception as e:
                print(f"[错误] {e}")

    def _parse_buffer(self):
        """从缓冲区解析状态帧"""
        while len(self._buf) >= STATUS_FRAME_SIZE:
            # 查找帧头
            header_idx = self._buf.find(bytes([FRAME_HEADER]))
            if header_idx < 0:
                self._buf.clear()
                break

            # 丢弃帧头前的无用数据
            if header_idx > 0:
                self._buf = self._buf[header_idx:]
                continue

            # 检查是否有完整的一帧
            if len(self._buf) < STATUS_FRAME_SIZE:
                break

            frame = bytes(self._buf[:STATUS_FRAME_SIZE])
            status = parse_status_frame(frame)
            if status:
                self._latest_status = status
                if self.callback:
                    self.callback(status)

            # 移除已处理的一帧
            self._buf = self._buf[STATUS_FRAME_SIZE:]


def find_wheeltec_port() -> Optional[str]:
    """
    自动查找 WHEELTEC 串口 (通过描述信息)
    返回第一个找到的串口名称，未找到则返回 None
    """
    for device, desc in list_serial_ports():
        # 常见 USB 转串口描述，可根据实际调整
        if any(kw in desc.lower() for kw in ['usb', 'serial', 'uart', 'ch340', 'cp210', 'ftdi']):
            return device
    return None


# ==================== 命令行入口 ====================

def main():
    import argparse

    parser = argparse.ArgumentParser(description='WHEELTEC C50X 串口状态读取')
    parser.add_argument('port', nargs='?', default=None, help='串口名称 (如 COM3, /dev/ttyUSB0)')
    parser.add_argument('--baudrate', type=int, default=BAUDRATE, help=f'波特率 (默认 {BAUDRATE})')
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
            print("  2. 指定串口: python serial_read.py COM3")
            return
        print(f"自动检测到串口: {port}")

    # 回调方式: 每收到一帧数据就打印
    def on_status(status: RobotStatus):
        print(status)

    reader = SerialReader(port, args.baudrate, callback=on_status)
    try:
        reader.start()
        print("\n正在接收数据 (Ctrl+C 退出)...\n")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        reader.stop()


if __name__ == '__main__':
    main()
