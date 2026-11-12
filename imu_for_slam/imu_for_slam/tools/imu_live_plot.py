#!/usr/bin/env python3
"""实时绘制IMU：总图和六轴图同时显示原始/校正曲线，横轴为MCU时间秒值。

运行：python3 tools/imu_live_plot.py（自动识别 USB 串口）；也可指定 --port /dev/ttyACM0
依赖：python3 -m pip install pyserial matplotlib
旧版115200固件：增加 --baud 115200
--window控制可见时间窗口，--fps只控制绘图刷新，不改变IMU采样频率。
"""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import math
import os
from pathlib import Path
from queue import Empty, Full, Queue
import sys
import threading
import time

# 复用已定义的TEXT/HEX协议及CRC解码；不重复维护包结构。
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from imu_monitor import Sample, StreamDecoder
from serial_ports import select_serial_port
from calibration_runtime import add_calibration_args, load_calibration

CHANNELS = ("AX", "AY", "AZ", "GX", "GY", "GZ")
COLORS = ("tab:red", "tab:green", "tab:blue", "tab:orange", "tab:purple", "tab:brown")
UINT32_MASK = 0xFFFFFFFF
UINT32_HALF = 0x80000000


@dataclass(frozen=True)
class SamplePair:
    """同一次采样的两个版本一起排队、丢弃，避免曲线错位。"""

    raw: Sample
    corrected: Sample | None


class SerialReader(threading.Thread):
    """串口线程只解包入队，Matplotlib操作全部留在主线程。"""

    def __init__(self, port, calibration=None):
        super().__init__(name="imu-serial-reader", daemon=True)
        self.port = port
        self.calibration = calibration
        self.samples: Queue[SamplePair] = Queue(maxsize=4096)
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.received = 0
        self.bad_frames = 0
        self.queue_drops = 0
        self.sequence_gaps = 0
        self.last_receive_time = 0.0
        self.diagnostic = "Waiting for IMU data"
        self.error = ""

    def status(self) -> dict:
        with self.lock:
            return {
                "received": self.received,
                "bad_frames": self.bad_frames,
                "queue_drops": self.queue_drops,
                "sequence_gaps": self.sequence_gaps,
                "last_receive_time": self.last_receive_time,
                "diagnostic": self.diagnostic,
                "error": self.error,
            }

    def run(self) -> None:
        decoder = StreamDecoder()
        previous_sequence = None
        try:
            while not self.stop_event.is_set():
                data = self.port.read(min(self.port.in_waiting or 1, 4096))
                batch = decoder.feed(data)
                for message in decoder.take_diagnostics():
                    with self.lock:
                        self.diagnostic = message
                    print(message, file=sys.stderr)
                for sample in batch:
                    corrected = self.calibration.apply(sample) if self.calibration is not None else None
                    pair = SamplePair(sample, corrected)
                    # 绘图卡顿时丢弃最老的待绘制样本，接收线程不等待GUI。
                    try:
                        self.samples.put_nowait(pair)
                    except Full:
                        removed = False
                        try:
                            self.samples.get_nowait()
                            removed = True
                        except Empty:
                            pass
                        try:
                            self.samples.put_nowait(pair)
                        except Full:
                            # 只有当前线程生产；仍保留防御性处理。
                            removed = True
                        if removed:
                            with self.lock:
                                self.queue_drops += 1
                    with self.lock:
                        self.received += 1
                        self.last_receive_time = time.monotonic()
                        if previous_sequence is not None:
                            if (sample.sequence - previous_sequence) & UINT32_MASK != 1:
                                self.sequence_gaps += 1
                    previous_sequence = sample.sequence
                with self.lock:
                    self.bad_frames = decoder.bad_frames
        except Exception as exc:
            if not self.stop_event.is_set():
                with self.lock:
                    self.error = str(exc)
                print(f"串口接收中断：{exc}", file=sys.stderr)


class TimestampSeconds:
    """将uint32微秒转为秒：正常回绕展开；板子复位时通知绘图清空旧段。"""

    def __init__(self):
        self.previous_time = None
        self.previous_sequence = None
        self.wrap_offset_us = 0

    def convert(self, sample: Sample) -> tuple[float, bool]:
        reset = False
        if self.previous_time is not None:
            time_step = (sample.timestamp_us - self.previous_time) & UINT32_MASK
            sequence_step = (sample.sequence - self.previous_sequence) & UINT32_MASK
            if time_step >= UINT32_HALF or sequence_step >= UINT32_HALF:
                # 回退/复位：不把新板端时钟伪接成旧时钟的延续。
                self.wrap_offset_us = 0
                reset = True
            elif sample.timestamp_us < self.previous_time:
                self.wrap_offset_us += 1 << 32
        self.previous_time = sample.timestamp_us
        self.previous_sequence = sample.sequence
        return (self.wrap_offset_us + sample.timestamp_us) / 1_000_000.0, reset


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="串口，例如 /dev/ttyACM0、/dev/ttyUSB0 或 COM5；省略时自动识别")
    parser.add_argument("--list", action="store_true", help="列出串口")
    parser.add_argument("--baud", type=int, default=460800, help="波特率，默认460800")
    parser.add_argument("--mode", choices=("hex", "text"), default="hex",
                        help="请求固件的格式，默认hex；接收器自动识别两种包")
    parser.add_argument("--no-command", action="store_true", help="只接收，不发送格式或START命令")
    parser.add_argument("--window", type=float, default=10.0, help="可见时间窗口秒数，默认10")
    parser.add_argument("--fps", type=float, default=30.0, help="绘图目标帧率，默认30，不影响采集率")
    parser.add_argument("--max-points", type=int, default=12000,
                        help="每条曲线保留的最大点数，默认12000，限制内存")
    parser.add_argument("--backend", help="可选GUI后端，例如TkAgg或QtAgg")
    add_calibration_args(parser)
    args = parser.parse_args()
    if args.baud <= 0 or args.max_points < 2:
        parser.error("baud必须大于0，max-points至少为2")
    if not math.isfinite(args.window) or args.window <= 0:
        parser.error("window必须是大于0的有限秒数")
    if not math.isfinite(args.fps) or not 1 <= args.fps <= 120:
        parser.error("fps必须在1至120之间")
    return args


def main() -> int:
    args = parse_args()
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        print("请先安装依赖：python3 -m pip install pyserial matplotlib", file=sys.stderr)
        return 2
    if args.list:
        ports = list(list_ports.comports())
        for port in ports:
            print(port.device, port.description)
        if not ports:
            print("未发现串口。")
        return 0
    try:
        args.port = select_serial_port(args.port)
    except OSError as exc:
        print(f"串口选择失败：{exc}", file=sys.stderr)
        return 1
    try:
        calibration = load_calibration(args)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"标定参数加载失败：{exc}", file=sys.stderr)
        return 2
    try:
        import matplotlib
        if args.backend:
            matplotlib.use(args.backend)
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.ticker import MaxNLocator, ScalarFormatter
    except ImportError as exc:
        print(f"绘图依赖不可用：{exc}\n请安装：python3 -m pip install matplotlib", file=sys.stderr)
        return 2
    if matplotlib.get_backend().lower() in {"agg", "pdf", "ps", "svg", "pgf", "cairo", "template"}:
        print("当前Matplotlib后端不能显示动画窗口，请在桌面环境中运行，或指定 --backend TkAgg。",
              file=sys.stderr)
        return 2

    port = None
    reader = None
    figure = None
    try:
        # POSIX独占防止两个程序同时消费同一串口；Windows不传该参数。
        options = {"exclusive": True} if os.name == "posix" else {}
        port = serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=1.0,
                             bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                             stopbits=serial.STOPBITS_ONE, xonxoff=False,
                             rtscts=False, dsrdtr=False, **options)
        if not args.no_command:
            commands = b"@PING\r\n" + (b"@HEX\r\n" if args.mode == "hex" else b"@TEXT\r\n") + b"@START\r\n"
            if port.write(commands) != len(commands):
                raise OSError("控制命令未完整发送")

        figure = plt.figure(figsize=(15, 10))
        display_mode = "RAW + CORRECTED" if calibration else "RAW ONLY"
        figure.canvas.manager.set_window_title(f"IMU: {display_mode} overview + six axes")
        grid = figure.add_gridspec(3, 3, height_ratios=(1.3, 1, 1))
        overview_accel = figure.add_subplot(grid[0, :])
        overview_gyro = overview_accel.twinx()
        overview_accel.set_title("All six channels | dashed: raw; solid: corrected"
                                 if calibration else "All six channels | raw data")
        overview_accel.set_ylabel("Acceleration (m/s²)")
        overview_gyro.set_ylabel("Angular rate (rad/s)")
        overview_raw_lines, overview_corrected_lines = [], []
        for index, name in enumerate(CHANNELS):
            axis = overview_accel if index < 3 else overview_gyro
            line, = axis.plot([], [], color=COLORS[index],
                              linestyle="--", linewidth=0.9, alpha=0.55,
                              label=f"{name} raw")
            overview_raw_lines.append(line)
            if calibration:
                line, = axis.plot([], [], color=COLORS[index], linestyle="-", linewidth=1.2,
                                  label=f"{name} corrected")
                overview_corrected_lines.append(line)
        legend_handles = ([line for pair in zip(overview_raw_lines, overview_corrected_lines)
                           for line in pair] if calibration else overview_raw_lines)
        overview_accel.legend(handles=legend_handles,
                              loc="upper left", ncol=6, fontsize=8, framealpha=0.75)
        individual_axes, individual_raw_lines, individual_corrected_lines, value_labels = [], [], [], []
        for index, name in enumerate(CHANNELS):
            axis = figure.add_subplot(grid[1 + index // 3, index % 3], sharex=overview_accel)
            axis.set_title(name)
            axis.set_ylabel("Acceleration (m/s²)" if index < 3 else "Angular rate (rad/s)")
            line, = axis.plot([], [], color="0.45" if calibration else COLORS[index],
                              linestyle="--", linewidth=0.9,
                              alpha=0.65, label="Raw")
            individual_axes.append(axis)
            individual_raw_lines.append(line)
            if calibration:
                line, = axis.plot([], [], color=COLORS[index], linestyle="-", linewidth=1.2,
                                  label="Corrected")
                individual_corrected_lines.append(line)
            axis.legend(loc="upper left", fontsize=8, framealpha=0.75)
            value_labels.append(axis.text(0.98, 0.94, "--", transform=axis.transAxes,
                                          ha="right", va="top", color=COLORS[index], fontsize=8,
                                          bbox=dict(facecolor="white", alpha=0.75, edgecolor="none")))
        all_axes = [overview_accel, overview_gyro, *individual_axes]
        for axis in all_axes:
            # 每个轴单独使用locator；纵轴范围和分度值都随可见数据更新。
            axis.yaxis.set_major_locator(MaxNLocator(nbins=5, min_n_ticks=3))
            axis.yaxis.set_major_formatter(ScalarFormatter(useOffset=False))
            axis.margins(y=0.15)
            axis.set_xlim(0, args.window)
        for axis in [overview_accel, *individual_axes]:
            axis.set_xlabel("MCU timestamp (s)")
            axis.xaxis.set_major_formatter(ScalarFormatter(useOffset=False))
            axis.grid(True, alpha=0.25)
        headline = figure.suptitle("Waiting for IMU packets", fontsize=12)
        footer = figure.text(0.5, 0.01, "Timestamp: MCU microseconds / 1e6; Y limits and ticks update automatically.",
                             ha="center", fontsize=9)
        figure.subplots_adjust(left=0.07, right=0.92, top=0.91, bottom=0.08, hspace=0.65, wspace=0.32)

        reader = SerialReader(port, calibration)
        reader.start()
        clock = TimestampSeconds()
        timestamps = deque(maxlen=args.max_points)
        raw_channels = [deque(maxlen=args.max_points) for _ in CHANNELS]
        corrected_channels = [deque(maxlen=args.max_points) for _ in CHANNELS] if calibration else []
        first_time = None
        resets = 0
        last_sequence = None

        def update(_frame):
            nonlocal first_time, resets, last_sequence
            changed = False
            # 有界队列一次全部消费；接收200Hz与绘图约30fps互不绑定。
            for _ in range(reader.samples.maxsize):
                try:
                    pair = reader.samples.get_nowait()
                except Empty:
                    break
                sample = pair.raw
                seconds, reset = clock.convert(sample)
                if reset:
                    timestamps.clear()
                    for values in (*raw_channels, *corrected_channels):
                        values.clear()
                    first_time = None
                    resets += 1
                if first_time is None:
                    first_time = seconds
                timestamps.append(seconds)
                for values, value in zip(raw_channels, sample.values[:6]):
                    values.append(value)
                if pair.corrected is not None:
                    for values, value in zip(corrected_channels, pair.corrected.values[:6]):
                        values.append(value)
                while len(timestamps) > 1 and timestamps[0] < seconds - args.window:
                    timestamps.popleft()
                    for values in (*raw_channels, *corrected_channels):
                        values.popleft()
                last_sequence = sample.sequence
                changed = True
            if changed:
                x = list(timestamps)
                for index, values in enumerate(raw_channels):
                    y = list(values)
                    overview_raw_lines[index].set_data(x, y)
                    individual_raw_lines[index].set_data(x, y)
                    label = f"Raw: {y[-1]:+.6f}"
                    if calibration:
                        corrected_y = list(corrected_channels[index])
                        overview_corrected_lines[index].set_data(x, corrected_y)
                        individual_corrected_lines[index].set_data(x, corrected_y)
                        label += f"\nCorrected: {corrected_y[-1]:+.6f}"
                    value_labels[index].set_text(label)
                right = max(first_time + args.window, x[-1])
                overview_accel.set_xlim(right - args.window, right)
                for axis in all_axes:
                    axis.set_autoscaley_on(True)
                    axis.relim()
                    axis.autoscale_view(scalex=False, scaley=True)
            status = reader.status()
            if status["error"]:
                state = "DISCONNECTED: " + status["error"]
            elif not status["last_receive_time"] or time.monotonic() - status["last_receive_time"] > 5:
                state = "WAITING: check sensor status / baud / START"
            else:
                state = "LIVE"
            headline.set_text(
                f"{state} {display_mode} | RX={status['received']}  bad={status['bad_frames']}  "
                f"seq gaps={status['sequence_gaps']}  plot drops={status['queue_drops']}  resets={resets}"
            )
            footer.set_text(f"MCU timestamp (s) | seq={last_sequence} | window={args.window:g}s | "
                            f"{str(status['diagnostic'])[:100]}")
            return (*overview_raw_lines, *overview_corrected_lines,
                    *individual_raw_lines, *individual_corrected_lines,
                    *value_labels, headline, footer)

        figure.canvas.mpl_connect("close_event", lambda _event: reader.stop_event.set())
        # 保留动画引用直到show返回。范围/刻度动态更新，不使用只重绘曲线的blit。
        animation = FuncAnimation(figure, update, interval=1000.0 / args.fps,
                                  blit=False, cache_frame_data=False)
        print(f"正在绘图：{args.port}，{args.baud} baud；一个总图+六个轴图，"
              f"{'原始虚线与校正实线同时显示' if calibration else '仅显示原始数据'}。关闭窗口或Ctrl+C退出。",
              file=sys.stderr)
        plt.show(block=True)
        if animation.event_source is not None:
            animation.event_source.stop()
        return 1 if reader.status()["error"] else 0
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"无法启动或更新实时绘图：{exc}", file=sys.stderr)
        return 1
    finally:
        if reader is not None:
            reader.stop_event.set()
            reader.join(timeout=1.0)
        if port is not None:
            port.close()
        if figure is not None:
            plt.close(figure)


if __name__ == "__main__":
    raise SystemExit(main())
