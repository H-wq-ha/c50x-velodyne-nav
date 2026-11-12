#!/usr/bin/env python3
"""PyShoe 实时零速辅助 IMU 轨迹：校正数据、中点传播、SHOE 检测、EKF 零速更新。

先静止3秒；世界Z向上，初始位置/速度为零，初始航向人为选定。
确认静止时锁定位置并将速度置零；移动时继续惯性传播。不使用相机、雷达或轮速。
纯IMU不能区分理想匀速运动与静止，自动模式适用于频繁停下的短时手持实验。
R重新初始化；M强制移动；S确认静止；A恢复自动检测；Q退出。
不创建采集数据、日志、测试或额外说明文件。

上游：utiasSTARS/pyshoe，提交 6bbc4df4eb52aa5d52e29da582a4c5273eb4aced。
实际调用 PyShoe Localizer.SHOE/state_update/corrector；实时输入和姿态重力更新在此适配。
上游局部修改：延迟加载学习模型依赖、NumPy兼容、位置更新、Joseph协方差更新。
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

ROOT = Path(__file__).resolve().parent
TOOLS = ROOT.parent / "imu_for_slam" / "tools"
sys.path.insert(0, str(ROOT / "pyshoe"))
sys.path.insert(0, str(TOOLS))
from ins_tools.EKF import Localizer
from ins_tools.geometry_helpers import mat2quat, quat2mat, quat2euler
from calibration_runtime import Calibration, DEFAULT_CALIBRATION
from imu_monitor import Sample, StreamDecoder
from serial_ports import select_serial_port
from imu_preintegration import gravity_alignment, skew, so3_exp

MASK = 0xFFFFFFFF
HALF = 0x80000000


class ZeroVelocityINS:
    """PyShoe九维误差状态滤波，增加在线输入、重力观测及明确的静止位置保持。"""

    def __init__(self, init_seconds=3.0, window_seconds=0.4, dwell_seconds=0.3,
                 gravity=9.80665, enter_threshold=18.0, exit_threshold=35.0,
                 max_dt=0.05):
        self.init_seconds = init_seconds
        self.window_seconds = window_seconds
        self.dwell_seconds = dwell_seconds
        self.gravity = gravity
        self.enter_threshold = enter_threshold
        self.exit_threshold = exit_threshold
        self.max_dt = max_dt
        self.resets = 0
        self.mode = "auto"
        self.reset()

    def reset(self):
        self.previous = None
        self.initial = deque()
        self.window = deque()
        self.clock = 0.0
        self.elapsed = 0.0
        self.ready = False
        self.stationary = True
        self.quiet_since = None
        self.motion_since = None
        self.score = float("nan")
        self.last_detection = -1.0
        self.last_gravity_update = -1.0
        self.state = np.zeros(9)
        self.rotation = np.eye(3)
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.covariance = np.eye(9) * 1e-4
        self.gyro_bias = np.zeros(3)
        self.accel_bias = np.zeros(3)
        self.parked_position = np.zeros(3)
        self.world_acceleration = np.zeros(3)
        self.stationary_updates = 0
        self.moving_updates = 0
        self.status = "Keep still to initialize"
        self.navigator = None
        self.detector = None

    def _initialize(self, sample, dt):
        self.clock += dt
        self.initial.append((self.clock, np.asarray(sample.values[:6], dtype=float)))
        while len(self.initial) > 2 and self.clock - self.initial[1][0] >= self.init_seconds:
            self.initial.popleft()
        duration = self.clock - self.initial[0][0]
        self.status = f"Keep still: {min(duration, self.init_seconds):.1f}/{self.init_seconds:.1f}s"
        if duration < self.init_seconds or len(self.initial) < 20:
            return False
        readings = np.array([item[1] for item in self.initial])
        mean, std = readings.mean(axis=0), readings.std(axis=0, ddof=1)
        norm = float(np.linalg.norm(mean[:3]))
        if (np.max(std[:3]) > 0.08 or np.max(std[3:]) > 0.004
                or np.linalg.norm(mean[3:]) > 0.03 or abs(norm - self.gravity) > 0.5):
            self.status = "Waiting for stable stationary measurements"
            return False
        self.rotation = gravity_alignment(mean[:3])
        self.quaternion = mat2quat(self.rotation)
        self.state[6:9] = quat2euler(self.quaternion, "sxyz")
        self.gyro_bias = mean[3:].copy()
        # 仅补偿初始重力方向上的模长残差，不声称单姿态估出了完整三轴加速度零偏。
        self.accel_bias = (norm - self.gravity) * mean[:3] / norm
        self.sigma_a = max(0.005, float(np.sqrt(np.mean(std[:3] ** 2))))
        self.sigma_w = max(0.0004, float(np.sqrt(np.mean(std[3:] ** 2))))
        h = np.zeros((3, 9))
        h[:, 3:6] = np.eye(3)
        config = {"g": -self.gravity, "H": h, "R": np.eye(3) * 0.01 ** 2,
                  "var_a": self.sigma_a ** 2, "var_w": self.sigma_w ** 2}
        self.navigator = Localizer(config, np.empty((0, 6)))
        # 上游惯导采用Z向下；实时传播改为Z向上，但SHOE模长必须使用正的g。
        self.detector = Localizer(dict(config, g=self.gravity), np.empty((0, 6)))
        self.process_noise = np.diag([max(0.03, 2*self.sigma_a)**2]*3
                                     + [max(0.002, 2*self.sigma_w)**2]*3)
        self.covariance = np.diag([0.001**2]*3 + [0.01**2]*3
                                 + [math.radians(0.5)**2]*3)
        for timestamp, value in self.initial:
            corrected = value.copy()
            corrected[:3] -= self.accel_bias
            corrected[3:] -= self.gyro_bias
            self.window.append((timestamp, corrected))
        self.initial.clear()
        self.ready = True
        self.status = "STATIONARY - position held"
        print(f"初始化完成：静态模长={norm:.6f}；噪声σa={self.sigma_a:.6f} m/s²，"
              f"σw={self.sigma_w:.6f} rad/s。静止时位置保持。", flush=True)
        return True

    def _detect(self, imu):
        self.window.append((self.clock, imu.copy()))
        while len(self.window) > 2 and self.clock - self.window[1][0] >= self.window_seconds:
            self.window.popleft()
        world_a = self.rotation @ imu[:3] + np.array([0., 0., -self.gravity])
        # 短暂连续超限才退出静止，避免将单个随机噪声尖峰当成起步。
        instant_motion = (np.linalg.norm(imu[3:]) > max(0.012, 6*self.sigma_w)
                          or np.linalg.norm(world_a) > max(0.06, 5*self.sigma_a))
        if instant_motion:
            if self.motion_since is None:
                self.motion_since = self.clock
        else:
            self.motion_since = None
        motion_now = (self.motion_since is not None
                      and self.clock - self.motion_since >= 0.015)
        if self.clock - self.last_detection >= 0.025:
            self.last_detection = self.clock
            data = np.array([item[1] for item in self.window])
            if len(data) >= 5 and np.linalg.norm(data[:, :3].mean(axis=0)) > 1.0:
                self.detector.imudata = data
                self.score = float(self.detector.SHOE(W=len(data))[0])
        if self.mode == "moving":
            wanted = False
            self.quiet_since = None
        elif self.mode == "stationary":
            wanted = True
        elif self.stationary:
            wanted = not motion_now and np.isfinite(self.score) and self.score < self.exit_threshold
            if not wanted:
                self.quiet_since = None
        else:
            quiet = (not motion_now and np.isfinite(self.score)
                     and self.score < self.enter_threshold)
            if quiet:
                if self.quiet_since is None:
                    self.quiet_since = self.clock
            else:
                self.quiet_since = None
            wanted = (self.quiet_since is not None
                      and self.clock - self.quiet_since >= self.dwell_seconds)
        if wanted and not self.stationary:
            self.parked_position = self.state[:3].copy()
            print("检测到静止：零速更新并保持当前位置。", flush=True)
        elif not wanted and self.stationary:
            print("检测到移动：恢复惯性轨迹传播。", flush=True)
        self.stationary = wanted

    def _gravity_update(self, mean_force):
        force_world = self.rotation @ mean_force
        h = np.zeros((3, 9))
        h[:, 6:9] = -skew(force_world)
        residual = np.array([0., 0., self.gravity]) - force_world
        noise = np.eye(3) * max(0.03, 2*self.sigma_a)**2
        p = self.covariance
        gain = np.linalg.solve(h @ p @ h.T + noise, h @ p).T
        correction = gain @ residual
        self.state[:6] += correction[:6]
        self.rotation = so3_exp(correction[6:9]) @ self.rotation
        self.quaternion = mat2quat(self.rotation)
        self.state[6:9] = quat2euler(self.quaternion, "sxyz")
        a = np.eye(9) - gain @ h
        self.covariance = a @ p @ a.T + gain @ noise @ gain.T
        self.covariance = 0.5 * (self.covariance + self.covariance.T)

    def process(self, sample):
        """返回(新增轨迹点, 清空轨迹)。姿态/协方差仍可在静止期更新，位置不变。"""
        dt, clear = 0.0, False
        if self.previous is not None:
            seq_step = (sample.sequence - self.previous.sequence) & MASK
            ticks = (sample.timestamp_us - self.previous.timestamp_us) & MASK
            if seq_step == 0 and ticks == 0:
                return False, False
            if seq_step != 1 or ticks == 0 or ticks >= HALF or ticks/1e6 > self.max_dt:
                self.resets += 1
                self.reset()
                clear = True
                print("数据不连续：重新静止初始化，不跨缺失数据积分。", file=sys.stderr)
            else:
                dt = ticks / 1e6
        previous = self.previous
        self.previous = sample
        if not self.ready:
            return self._initialize(sample, dt), clear
        if previous is None or dt <= 0:
            return False, clear
        self.clock += dt
        self.elapsed += dt
        old = np.asarray(previous.values[:6], dtype=float).copy()
        imu = np.asarray(sample.values[:6], dtype=float).copy()
        old[:3] -= self.accel_bias
        imu[:3] -= self.accel_bias
        old[3:] -= self.gyro_bias
        imu[3:] -= self.gyro_bias
        self._detect(imu)
        rotation0 = self.rotation
        rotation1 = rotation0 @ so3_exp(0.5*(old[3:]+imu[3:])*dt)
        acceleration = (0.5*(rotation0 @ old[:3] + rotation1 @ imu[:3])
                        + np.array([0., 0., -self.gravity]))
        self.world_acceleration = acceleration
        self.state[:3] += self.state[3:6]*dt + 0.5*acceleration*dt*dt
        self.state[3:6] += acceleration*dt
        self.rotation = rotation1
        self.quaternion = mat2quat(rotation1)
        self.state[6:9] = quat2euler(self.quaternion, "sxyz")
        f, g = self.navigator.state_update(imu, mat2quat(rotation0), dt)
        self.covariance = f @ self.covariance @ f.T + g @ self.process_noise @ g.T
        self.covariance = 0.5*(self.covariance + self.covariance.T)
        if self.stationary:
            self.state, self.covariance, self.quaternion = self.navigator.corrector(
                self.state, self.covariance, self.rotation)
            self.rotation = quat2mat(self.quaternion)
            if self.clock - self.last_gravity_update >= 0.05:
                self.last_gravity_update = self.clock
                mean_force = np.mean([item[1][:3] for item in self.window], axis=0)
                self._gravity_update(mean_force)
            # 零速假设成立时，硬保持可保证静态显示不再积出位移。
            self.state[:3] = self.parked_position
            self.state[3:6] = 0.0
            alpha = -math.expm1(-dt/10.0)
            self.gyro_bias += alpha*(np.asarray(sample.values[3:6])-self.gyro_bias)
            self.stationary_updates += 1
            self.status = "STATIONARY - position held"
            return False, clear
        self.moving_updates += 1
        self.status = "MOVING - inertial estimate"
        return True, clear


class SerialINS(threading.Thread):
    def __init__(self, args, calibration):
        super().__init__(name="pyshoe-live", daemon=True)
        self.args, self.calibration = args, calibration
        self.engine = ZeroVelocityINS(args.init_seconds, args.window, args.dwell,
                                      args.gravity, args.enter_threshold,
                                      args.exit_threshold, args.max_dt)
        self.engine.mode = args.mode
        self.points = deque(maxlen=args.max_points)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.command = None
        self.received = 0
        self.bad_frames = 0
        self.error = ""
        self.last_receive = time.monotonic()

    def snapshot(self):
        with self.lock:
            e = self.engine
            return dict(points=list(self.points), position=e.state[:3].copy(),
                        velocity=e.state[3:6].copy(), acceleration=e.world_acceleration.copy(),
                        ready=e.ready, stationary=e.stationary, status=e.status, mode=e.mode,
                        elapsed=e.elapsed, score=e.score, static_updates=e.stationary_updates,
                        moving_updates=e.moving_updates, received=self.received,
                        bad_frames=self.bad_frames, error=self.error,
                        last_receive=self.last_receive)

    def request(self, command):
        with self.lock:
            self.command = command

    def run(self):
        import serial
        decoder = StreamDecoder()
        try:
            with serial.Serial(self.args.port, self.args.baud, timeout=0.1, write_timeout=1,
                               xonxoff=False, rtscts=False, dsrdtr=False,
                               **({"exclusive": True} if os.name == "posix" else {})) as port:
                started = False
                try:
                    port.reset_input_buffer()
                    command = b"@STOP\r\n@HEX\r\n@START\r\n"
                    if port.write(command) != len(command):
                        raise OSError("启动命令发送不完整")
                    started = True
                    print(f"已连接 {self.args.port}；请保持静止完成初始化。", flush=True)
                    while not self.stop_event.is_set():
                        samples = decoder.feed(port.read(min(port.in_waiting or 1, 4096)))
                        for message in decoder.take_diagnostics():
                            print(message, file=sys.stderr)
                        for raw in samples:
                            if self.stop_event.is_set():
                                break
                            corrected = self.calibration.apply(raw)
                            if not np.isfinite(corrected.values).all():
                                raise ValueError("校正数据含非有限值")
                            with self.lock:
                                if self.command == "reset":
                                    self.engine.reset()
                                    self.points.clear()
                                elif self.command is not None:
                                    self.engine.mode = self.command
                                self.command = None
                                added, clear = self.engine.process(corrected)
                                if clear:
                                    self.points.clear()
                                if added and (not self.points or np.linalg.norm(
                                        self.engine.state[:3]-self.points[-1]) > 1e-6):
                                    self.points.append(self.engine.state[:3].copy())
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
            print(f"采集停止：{exc}", file=sys.stderr)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="省略时自动选择 USB 串口")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--init-seconds", type=float, default=3.0)
    parser.add_argument("--window", type=float, default=0.4, help="静止检测窗口，秒")
    parser.add_argument("--dwell", type=float, default=0.3, help="持续满足静止条件的时间，秒")
    parser.add_argument("--gravity", type=float, default=9.80665)
    parser.add_argument("--enter-threshold", type=float, default=18.0)
    parser.add_argument("--exit-threshold", type=float, default=35.0)
    parser.add_argument("--max-dt", type=float, default=0.05)
    parser.add_argument("--mode", choices=("auto", "moving", "stationary"), default="auto")
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--max-points", type=int, default=30000)
    args = parser.parse_args()
    positive = (args.init_seconds, args.window, args.dwell, args.gravity,
                args.enter_threshold, args.exit_threshold, args.max_dt, args.fps)
    if (args.baud <= 0 or args.max_points < 2 or any(not math.isfinite(v) or v <= 0 for v in positive)
            or args.init_seconds < 0.2 or args.window < 0.05
            or args.enter_threshold >= args.exit_threshold or args.max_dt >= HALF/1e6):
        parser.error("参数必须为正，初始化至少0.2秒，窗口至少0.05秒，退出阈值大于进入阈值")
    return parser, args


def main():
    parser, args = parse_args()
    try:
        import serial
        import matplotlib.pyplot as plt
        calibration = Calibration(args.calibration)
        args.port = select_serial_port(args.port)
    except ImportError as exc:
        print(f"运行依赖缺失：{exc}；需要numpy、pyserial、matplotlib。", file=sys.stderr)
        return 2
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    fig = plt.figure(figsize=(11, 8))
    axes = fig.add_subplot(111, projection="3d")
    axes.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)",
             title="PyShoe - corrected IMU with zero-velocity updates")
    axes.set_box_aspect((1, 1, 1))
    axes.view_init(elev=25, azim=-55)
    line, = axes.plot([], [], [], color="red", linewidth=1.7, label="Trajectory")
    dot, = axes.plot([], [], [], "o", color="red", markersize=5)
    axes.scatter([0], [0], [0], color="black", marker="x", label="Origin")
    axes.legend(loc="upper right")
    info = fig.text(0.025, 0.03, "", family="monospace", fontsize=9)
    fig.subplots_adjust(bottom=0.22)
    worker = SerialINS(args, calibration)

    def update():
        data = worker.snapshot()
        points = np.array(data["points"])
        if len(points):
            visible = points[::max(1, math.ceil(len(points)/3000))]
            visible = np.vstack((visible, points[-1]))
            line.set_data_3d(visible[:, 0], visible[:, 1], visible[:, 2])
            extent = np.vstack((points, np.zeros(3)))
            low, high = extent.min(axis=0), extent.max(axis=0)
            center = (low+high)*0.5
            radius = max(0.5, float(np.max(high-low))*0.6)
        else:
            line.set_data_3d([], [], [])
            center, radius = np.zeros(3), 0.5
        p, v, a = data["position"], data["velocity"], data["acceleration"]
        dot.set_data_3d([p[0]], [p[1]], [p[2]])
        for setter, coordinate in zip((axes.set_xlim, axes.set_ylim, axes.set_zlim), center):
            setter(coordinate-radius, coordinate+radius)
        message = data["error"] or data["status"]
        if not data["error"] and time.monotonic()-data["last_receive"] > 3:
            message = "No recent IMU data"
        info.set_text(
            f"{message} | mode={data['mode']} | SHOE={data['score']:.2f}\n"
            f"t={data['elapsed']:.2f}s  p=({p[0]:+.5f}, {p[1]:+.5f}, {p[2]:+.5f})m\n"
            f"|v|={np.linalg.norm(v):.6f}m/s  |a_world|={np.linalg.norm(a):.6f}m/s2  "
            f"static/moving updates={data['static_updates']}/{data['moving_updates']}\n"
            f"samples={data['received']}  bad frames={data['bad_frames']}  path points={len(points)}\n"
            "R: reset | M: force moving | S: confirm stationary | A: auto | Q: quit")
        dot.set_color("green" if data["ready"] and data["stationary"] else "red")
        fig.canvas.draw_idle()

    timer = fig.canvas.new_timer(interval=max(1, round(1000/args.fps)))
    timer.add_callback(update)

    def close(_event=None):
        timer.stop()
        worker.stop_event.set()

    def key(event):
        commands = {"r": "reset", "m": "moving", "s": "stationary", "a": "auto"}
        if event.key and event.key.lower() in commands:
            worker.request(commands[event.key.lower()])
        elif event.key and event.key.lower() == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", key)
    fig.canvas.mpl_connect("close_event", close)
    print(f"PyShoe零速辅助轨迹；标定参数：{calibration.path}\n"
          "确认静止时保持位置；移动段仍可能漂移。慢速匀速实验可按M，停稳后按S。", flush=True)
    worker.start()
    timer.start()
    try:
        plt.show()
    except KeyboardInterrupt:
        plt.close(fig)
    finally:
        close()
        worker.join(timeout=3)
    return 1 if worker.snapshot()["error"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
