#!/usr/bin/env python3
"""校正后的 IMU 实时中点预积分，在 Matplotlib 中绘制红色三维轨迹。

启动后先保持静止（默认 3 秒），初始化重力方向和残余陀螺零偏。
世界系 Z 向上，起点位置/速度为零，初始航向人为选定，不能由重力确定。
预积分累计局部 delta_R/delta_v/delta_p，再结合锚点状态和重力恢复世界系状态。
只实现惯性状态传播，不包含 SLAM 的外部观测、因子图优化或偏置雅可比。
按 R 清空并重新初始化；按 Q 或关闭窗口退出。不创建数据、文档或日志文件。

中点更新的参考结构：VINS-Mono IntegrationBase::midPointIntegration。
https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/vins_estimator/src/factor/integration_base.h
"""
from __future__ import annotations

import argparse
from collections import deque
import math
import os
from pathlib import Path
import sys
import threading
import time

import numpy as np

from calibration_runtime import Calibration, DEFAULT_CALIBRATION
from imu_monitor import Sample, StreamDecoder
from serial_ports import select_serial_port

UINT32_MASK = 0xFFFFFFFF
UINT32_HALF = 0x80000000


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)))


def so3_exp(rotation_vector: np.ndarray) -> np.ndarray:
    """Rodrigues 公式：IMU 系角增量变成有限旋转，不能逐轴累计欧拉角。"""
    angle_sq = float(rotation_vector @ rotation_vector)
    cross = skew(rotation_vector)
    if angle_sq < 1e-10:
        a = 1.0 - angle_sq / 6.0 + angle_sq * angle_sq / 120.0
        b = 0.5 - angle_sq / 24.0 + angle_sq * angle_sq / 720.0
    else:
        angle = math.sqrt(angle_sq)
        a = math.sin(angle) / angle
        b = (1.0 - math.cos(angle)) / angle_sq
    return np.eye(3) + a * cross + b * (cross @ cross)


def gravity_alignment(mean_force: np.ndarray) -> np.ndarray:
    """将初始静止比力方向旋到世界 +Z；初始 yaw 选为零。"""
    fx, fy, fz = mean_force
    roll = math.atan2(fy, fz)
    pitch = math.atan2(-fx, math.hypot(fy, fz))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    rx = np.array(((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)))
    ry = np.array(((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp)))
    return ry @ rx


class MidpointPreintegrator:
    """在锚点 IMU 系中累计不含重力的增量；偏置已在输入处扣除。"""

    def __init__(self) -> None:
        self.delta_rotation = np.eye(3)
        self.delta_velocity = np.zeros(3)
        self.delta_position = np.zeros(3)
        self.duration = 0.0

    def push(self, dt: float, force0: np.ndarray, omega0: np.ndarray,
             force1: np.ndarray, omega1: np.ndarray) -> None:
        rotation0 = self.delta_rotation
        rotation1 = rotation0 @ so3_exp(0.5 * (omega0 + omega1) * dt)
        # 两端比力先旋到同一个锚点坐标系，再取平均。
        acceleration = 0.5 * (rotation0 @ force0 + rotation1 @ force1)
        self.delta_position += self.delta_velocity * dt + 0.5 * acceleration * dt * dt
        self.delta_velocity += acceleration * dt
        self.delta_rotation = rotation1
        self.duration += dt


class InertialState:
    """静止初始化 -> 局部预积分 -> 世界系传播；不将静止加速度全部扣为零。"""

    def __init__(self, gravity: float, init_seconds: float, frame_seconds: float,
                 max_dt: float) -> None:
        self.gravity = gravity
        self.init_seconds = init_seconds
        self.frame_seconds = frame_seconds
        self.max_dt = max_dt
        self.restart_count = 0
        self.reset()

    def reset(self, reason: str = "Keep still for initialization") -> None:
        self.previous: Sample | None = None
        self.init_window: deque = deque()
        self.init_time = 0.0
        self.ready = False
        self.rotation = np.eye(3)
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.elapsed = 0.0
        self.anchor_rotation = np.eye(3)
        self.anchor_position = np.zeros(3)
        self.anchor_velocity = np.zeros(3)
        self.preintegration = MidpointPreintegrator()
        self.status = reason

    def _initialize(self, sample: Sample, dt: float) -> bool:
        self.init_time += dt
        values = np.asarray(sample.values[:6], dtype=float)
        self.init_window.append((self.init_time, values))
        # 使用滚动静止窗口；移动后可以自然等待最近一段重新稳定。
        while (len(self.init_window) > 2
               and self.init_time - self.init_window[1][0] >= self.init_seconds):
            self.init_window.popleft()
        duration = self.init_time - self.init_window[0][0]
        self.status = f"Keep still: {min(duration, self.init_seconds):.1f}/{self.init_seconds:.1f} s"
        if duration < self.init_seconds or len(self.init_window) < 20:
            return False
        readings = np.array([entry[1] for entry in self.init_window])
        mean = readings.mean(axis=0)
        std = readings.std(axis=0, ddof=1)
        force_norm = float(np.linalg.norm(mean[:3]))
        if (np.max(std[:3]) > 0.08 or np.max(std[3:]) > 0.004
                or np.linalg.norm(mean[3:]) > 0.03
                or abs(force_norm - self.gravity) > 0.5):
            self.status = "Waiting for stable, calibrated stationary data"
            return False
        self.rotation = gravity_alignment(mean[:3])
        self.anchor_rotation = self.rotation.copy()
        # 标定参数只应用一次；这里估计的是校正后剩余的本次启动零偏。
        self.gyro_bias = mean[3:].copy()
        self.ready = True
        self.init_window.clear()
        self.status = "Integrating corrected IMU (pure inertial drift)"
        print(f"静止初始化完成：比力模长 {force_norm:.6f} m/s²；"
              f"残余陀螺零偏 {np.rad2deg(self.gyro_bias)} °/s。", flush=True)
        return True

    def process(self, sample: Sample) -> tuple[bool, bool]:
        """返回 (轨迹有新点, 需要清空旧轨迹)。dt 只取 MCU 微秒时间戳。"""
        clear_path = False
        dt = 0.0
        if self.previous is not None:
            sequence_step = (sample.sequence - self.previous.sequence) & UINT32_MASK
            time_step = (sample.timestamp_us - self.previous.timestamp_us) & UINT32_MASK
            if sequence_step == 0 and time_step == 0:
                return False, False
            if (sequence_step != 1 or time_step == 0 or time_step >= UINT32_HALF
                    or time_step / 1e6 > self.max_dt):
                self.restart_count += 1
                print("序号/时间不连续：清空轨迹，请重新保持静止初始化。", file=sys.stderr)
                self.reset("Data discontinuity: keep still to reinitialize")
                clear_path = True
            else:
                dt = time_step / 1e6  # 模运算也正确处理 uint32 时间自然回绕。
        previous = self.previous
        self.previous = sample
        if not self.ready:
            return self._initialize(sample, dt), clear_path
        if previous is None or dt <= 0.0:
            return False, clear_path
        old = np.asarray(previous.values[:6], dtype=float)
        new = np.asarray(sample.values[:6], dtype=float)
        self.preintegration.push(dt, old[:3], old[3:] - self.gyro_bias,
                                 new[:3], new[3:] - self.gyro_bias)
        block = self.preintegration
        gravity_world = np.array((0.0, 0.0, -self.gravity))
        self.rotation = self.anchor_rotation @ block.delta_rotation
        self.velocity = (self.anchor_velocity + gravity_world * block.duration
                         + self.anchor_rotation @ block.delta_velocity)
        self.position = (self.anchor_position + self.anchor_velocity * block.duration
                         + 0.5 * gravity_world * block.duration ** 2
                         + self.anchor_rotation @ block.delta_position)
        self.elapsed += dt
        if block.duration >= self.frame_seconds:
            # 提交一个时间锚点后重新预积分；保留累计世界系位姿/速度。
            u, _, vh = np.linalg.svd(self.rotation)
            orthogonal = u @ vh
            if np.linalg.det(orthogonal) < 0.0:
                u[:, -1] *= -1.0
                orthogonal = u @ vh
            self.rotation = orthogonal
            self.anchor_rotation = self.rotation.copy()
            self.anchor_velocity = self.velocity.copy()
            self.anchor_position = self.position.copy()
            self.preintegration = MidpointPreintegrator()
        return True, clear_path


class IntegrationWorker(threading.Thread):
    """在串口线程逐个积分所有有效样本；GUI 只读取轨迹快照。"""

    def __init__(self, args, calibration: Calibration) -> None:
        super().__init__(name="imu-preintegration", daemon=True)
        self.args = args
        self.calibration = calibration
        self.state = InertialState(args.gravity, args.init_seconds, args.frame_seconds, args.max_dt)
        self.path: deque = deque(maxlen=args.max_points)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.reset_event = threading.Event()
        self.received = 0
        self.bad_frames = 0
        self.last_receive = time.monotonic()
        self.error = ""

    def snapshot(self) -> dict:
        with self.lock:
            return dict(path=list(self.path), position=self.state.position.copy(),
                        velocity=self.state.velocity.copy(), elapsed=self.state.elapsed,
                        ready=self.state.ready, status=self.state.status,
                        received=self.received, bad_frames=self.bad_frames,
                        resets=self.state.restart_count, last_receive=self.last_receive,
                        error=self.error)

    def run(self) -> None:
        import serial

        decoder = StreamDecoder()
        try:
            with serial.Serial(
                self.args.port, self.args.baud, timeout=0.1, write_timeout=1.0,
                xonxoff=False, rtscts=False, dsrdtr=False,
                **({"exclusive": True} if os.name == "posix" else {}),
            ) as port:
                started = False
                try:
                    port.reset_input_buffer()
                    command = b"@STOP\r\n@HEX\r\n@START\r\n"
                    if port.write(command) != len(command):
                        raise OSError("串口启动命令未完整发送")
                    started = True
                    print(f"已连接 {self.args.port}；请先保持静止 {self.args.init_seconds:g} 秒。",
                          flush=True)
                    while not self.stop_event.is_set():
                        samples = decoder.feed(port.read(min(port.in_waiting or 1, 4096)))
                        for message in decoder.take_diagnostics():
                            print(message, file=sys.stderr)
                        for raw in samples:
                            if self.stop_event.is_set():
                                break
                            corrected = self.calibration.apply(raw)
                            if not np.isfinite(corrected.values).all():
                                raise ValueError("校正数据包含非有限值，请检查标定参数")
                            with self.lock:
                                if self.reset_event.is_set():
                                    self.state.reset()
                                    self.path.clear()
                                    self.reset_event.clear()
                                point_added, clear = self.state.process(corrected)
                                if clear:
                                    self.path.clear()
                                if point_added:
                                    self.path.append(self.state.position.copy())
                                self.received += 1
                                self.last_receive = time.monotonic()
                        with self.lock:
                            self.bad_frames = decoder.bad_frames
                finally:
                    if started:
                        try:
                            port.write(b"@STOP\r\n")
                        except (OSError, serial.SerialException):
                            pass
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
            print(f"积分采集停止：{exc}", file=sys.stderr)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="串口；省略时自动选择 USB 串口")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION,
                        help="六轴 SI 标定参数 JSON，默认脚本旁的 imu_calibration.json")
    parser.add_argument("--init-seconds", type=float, default=3.0, help="静止初始化时间，默认3秒")
    parser.add_argument("--gravity", type=float, default=9.80665, help="重力大小，m/s²")
    parser.add_argument("--frame-seconds", type=float, default=1.0, help="预积分锚点间隔，默认1秒")
    parser.add_argument("--max-dt", type=float, default=0.05, help="允许的最大相邻样本间隔，秒")
    parser.add_argument("--fps", type=float, default=25.0, help="绘图频率，不影响积分采样率")
    parser.add_argument("--max-points", type=int, default=50000, help="保留的最新轨迹点数")
    args = parser.parse_args()
    values = (args.init_seconds, args.gravity, args.frame_seconds, args.max_dt, args.fps)
    if args.baud <= 0 or args.max_points < 2 or any(not math.isfinite(x) or x <= 0 for x in values):
        parser.error("时间、重力、频率和波特率必须为正，max-points 必须至少为2")
    if args.init_seconds < 0.2 or args.max_dt >= UINT32_HALF / 1e6:
        parser.error("init-seconds 至少0.2秒，max-dt 必须小于uint32计时半周期")
    return parser, args


def main() -> int:
    parser, args = parse_args()
    try:
        import serial  # noqa: F401 -- 启动线程前检查依赖。
        import matplotlib.pyplot as plt
        calibration = Calibration(args.calibration)
        args.port = select_serial_port(args.port)
    except ImportError as exc:
        print(f"缺少运行依赖：{exc}；需要 numpy、pyserial、matplotlib。", file=sys.stderr)
        return 2
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))

    fig = plt.figure(figsize=(10, 8))
    axes = fig.add_subplot(111, projection="3d")
    axes.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)",
             title="Corrected IMU - 3D midpoint preintegration")
    axes.set_box_aspect((1, 1, 1))
    axes.view_init(elev=25, azim=-55)
    axes.grid(True)
    trajectory, = axes.plot([], [], [], color="red", linewidth=1.6, label="IMU trajectory")
    endpoint, = axes.plot([], [], [], marker="o", color="red", markersize=4)
    axes.scatter([0], [0], [0], color="black", marker="x", label="Origin")
    axes.legend(loc="upper right")
    status = fig.text(0.03, 0.03, "", family="monospace", fontsize=9)
    fig.subplots_adjust(bottom=0.19)
    worker = IntegrationWorker(args, calibration)

    def update() -> None:
        snap = worker.snapshot()
        points = np.array(snap["path"])
        if len(points):
            # 只下采样显示，积分已经在串口线程逐样本完成。
            display = points[::max(1, math.ceil(len(points) / 5000))]
            display = np.vstack((display, points[-1]))
            trajectory.set_data_3d(display[:, 0], display[:, 1], display[:, 2])
            endpoint.set_data_3d([points[-1, 0]], [points[-1, 1]], [points[-1, 2]])
            extent_points = np.vstack((points, np.zeros(3)))
            lower, upper = extent_points.min(axis=0), extent_points.max(axis=0)
            center = 0.5 * (lower + upper)
            radius = max(0.5, float(np.max(upper - lower)) * 0.6)
        else:
            trajectory.set_data_3d([], [], [])
            endpoint.set_data_3d([], [], [])
            center, radius = np.zeros(3), 0.5
        axes.set_xlim(center[0] - radius, center[0] + radius)
        axes.set_ylim(center[1] - radius, center[1] + radius)
        axes.set_zlim(center[2] - radius, center[2] + radius)
        message = snap["error"] or snap["status"]
        if not snap["error"] and time.monotonic() - snap["last_receive"] > 3.0:
            message = "No recent IMU data; integration is waiting"
        p, v = snap["position"], snap["velocity"]
        status.set_text(
            f"{message}\n"
            f"t={snap['elapsed']:.2f}s  p=({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})m  "
            f"|v|={np.linalg.norm(v):.3f}m/s\n"
            f"samples={snap['received']}  bad CRC/frames={snap['bad_frames']}  "
            f"restarts={snap['resets']}  |  R: reset   Q: quit")
        fig.canvas.draw_idle()

    timer = fig.canvas.new_timer(interval=max(1, round(1000 / args.fps)))
    timer.add_callback(update)

    def close(_event=None) -> None:
        timer.stop()
        worker.stop_event.set()

    def key(event) -> None:
        if event.key and event.key.lower() == "r":
            worker.reset_event.set()
            print("请求重新初始化：请保持静止。", flush=True)
        elif event.key and event.key.lower() == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("close_event", close)
    fig.canvas.mpl_connect("key_press_event", key)
    print(f"标定参数：{calibration.path}\n"
          "图中单位为米；纯IMU积分会漂移。先静止初始化，再开始移动。", flush=True)
    worker.start()
    timer.start()
    try:
        plt.show()
    except KeyboardInterrupt:
        plt.close(fig)
    finally:
        close()
        worker.join(timeout=3.0)
    snap = worker.snapshot()
    return 1 if snap["error"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
