#!/usr/bin/env python3
"""通过 imu_for_slam 的串口协议采集 SI 数据并标定；不要求修改 STM32 固件。"""
from __future__ import annotations

import argparse
import csv
import importlib.util
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parent
DEFAULT_TOOLS = ROOT.parents[1] / "imu_for_slam" / "imu_for_slam" / "tools"
SI_FIELDS = ("ax_mps2", "ay_mps2", "az_mps2", "gx_rads", "gy_rads", "gz_rads")
UINT32_MASK = 0xFFFFFFFF


def load_tool_module(tools_path, name):
    path = Path(tools_path) / (name + ".py")
    if not path.is_file():
        raise ValueError(f"未找到串口工具模块：{path}；请通过 --protocol-tools 指定 tools 目录")
    spec = importlib.util.spec_from_file_location("stm32_calibration_" + name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclass 要求模块已经注册。
    spec.loader.exec_module(module)
    return module


def load_protocol(tools_path):
    return load_tool_module(tools_path, "imu_monitor")


def session_path(path):
    return Path(str(path) + ".session.json")


def record(args):
    import serial

    if args.baud <= 0:
        raise ValueError("波特率必须大于 0")
    protocol = load_protocol(args.protocol_tools)
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists() or session_path(output).exists():
        raise ValueError(f"输出已存在，请使用新文件名：{output}")
    if args.positions < 12 or min(args.t_init, args.hold, args.move, args.t_wait) <= 0:
        raise ValueError("至少采集 12 个姿态，所有时间参数必须大于 0")
    if args.t_wait >= args.hold:
        raise ValueError("静止检测窗口 --t-wait 必须小于每个姿态的静置时间 --hold")
    selector = load_tool_module(args.protocol_tools, "serial_ports")
    args.port = selector.select_serial_port(args.port)

    # 由 MCU 时间推进提示，避免主机接收时刻抖动改变记录的初始静止时长。
    phases = [(args.t_init, f"姿态 1/{args.positions}：保持静止 {args.t_init:g} 秒")]
    deadline = args.t_init
    for index in range(2, args.positions + 1):
        deadline += args.move
        phases.append((deadline, f"转到姿态 {index}/{args.positions}：用 {args.move:g} 秒平稳转动"))
        deadline += args.hold
        phases.append((deadline, f"姿态 {index}/{args.positions}：保持静止 {args.hold:g} 秒"))
    print(f"采集 {args.positions} 个姿态，预计 {deadline:g} 秒；串口 {args.port}，{args.baud} baud。")
    print("先让传感器预热至读数稳定，关闭其他占用串口的程序。采集过程中不要复位板子。")
    print("前 6 个姿态建议依次让 Z+、Z-、X+、X-、Y+、Y- 方向朝上，然后增加不同倾斜姿态。")
    input("摆好第一个姿态，按 Enter 开始：")

    decoder = protocol.StreamDecoder()
    count, elapsed, phase = 0, 0.0, 0
    previous_timestamp = None
    previous_sequence = None
    sequence_breaks = 0
    completed = False
    error = None
    try:
        with output.open("x", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=protocol.CSV_FIELDS)
            writer.writeheader()
            with serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=1.0) as port:
                try:
                    # 与 serial_receive.py 相同的 ASCII 控制命令；数据使用带 CRC 的 HEX 包。
                    port.write(b"@STOP\r\n")
                    port.flush()
                    time.sleep(0.1)
                    port.reset_input_buffer()
                    port.write(b"@PING\r\n@HEX\r\n@START\r\n")
                    last_sample_host_time = time.monotonic()
                    last_flush_time = last_sample_host_time
                    print(phases[0][1], flush=True)
                    while not completed:
                        chunk = port.read(min(port.in_waiting or 1, 4096))
                        for sample in decoder.feed(chunk):
                            if previous_timestamp is not None:
                                dt = ((sample.timestamp_us - previous_timestamp) & UINT32_MASK) / 1e6
                                if not 0 < dt < 1:
                                    raise ValueError("MCU 时间异常或板子复位；已保存的数据可用于排查，请重新采集")
                                elapsed += dt
                                if ((sample.sequence - previous_sequence) & UINT32_MASK) != 1:
                                    sequence_breaks += 1
                            previous_timestamp = sample.timestamp_us
                            previous_sequence = sample.sequence
                            writer.writerow(sample.csv_row())
                            count += 1
                            last_sample_host_time = time.monotonic()
                            while phase < len(phases) and elapsed >= phases[phase][0]:
                                phase += 1
                                if phase < len(phases):
                                    print(phases[phase][1], flush=True)
                                else:
                                    completed = True
                            if completed:
                                break
                        for message in decoder.take_diagnostics():
                            print(f"板端：{message}", flush=True)
                        if time.monotonic() - last_sample_host_time > 5:
                            raise ValueError("5 秒没有有效 IMU 数据，请检查串口、波特率和传感器状态")
                        if time.monotonic() - last_flush_time > 1:
                            stream.flush()
                            last_flush_time = time.monotonic()
                finally:
                    try:
                        port.write(b"@STOP\r\n")
                        port.flush()
                    except (OSError, serial.SerialException):
                        pass
    except KeyboardInterrupt:
        error = "用户中断，采集未完成"
    except (OSError, ValueError) as exc:
        error = str(exc)
    metadata = dict(
        completed=completed, error=error, units="SI", port=args.port, baud=args.baud,
        t_init=args.t_init, t_wait=args.t_wait, hold=args.hold, move=args.move,
        positions=args.positions, expected_duration_s=deadline,
        duration_s=elapsed, samples=count, sequence_breaks=sequence_breaks,
        decoder_bad_frames=decoder.bad_frames,
    )
    with session_path(output).open("x", encoding="utf-8") as stream:
        json.dump(metadata, stream, indent=2, ensure_ascii=False)
    print(f"保存 {count} 条样本：{output}")
    print(f"有效时长 {elapsed:.3f} 秒，序号不连续 {sequence_breaks} 次，拒绝异常包 {decoder.bad_frames} 个。")
    if error:
        print(error, file=sys.stderr)
    return 0 if completed and sequence_breaks == 0 else 1


def load_si_csv(path, fs):
    """拒绝丢包和复位，用 MCU 时间将轻微采样抖动插值到均匀时间网格。"""
    import numpy as np

    samples, stamps, sequences = [], [], []
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = set(SI_FIELDS) | {"sequence", "timestamp_us"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError("需要 serial_receive.py --csv 或本工具 record 生成的 SI CSV 文件")
        for row in reader:
            stamps.append(int(row["timestamp_us"]))
            sequences.append(int(row["sequence"]))
            samples.append([float(row[key]) for key in SI_FIELDS])
    if len(samples) < 2:
        raise ValueError("有效样本不足")
    if not all(0 <= x <= UINT32_MASK for x in stamps + sequences):
        raise ValueError("序号或 MCU 时间超出 uint32 范围")
    data = np.asarray(samples, dtype=np.float64)
    if not np.isfinite(data).all():
        raise ValueError("数据包含 NaN 或无穷大")
    sequence_steps = np.diff(np.asarray(sequences, dtype=np.int64)) & UINT32_MASK
    if np.any(sequence_steps != 1):
        raise ValueError("样本序号不连续，存在丢包或复位；请重新采集连续数据")
    dt = (np.diff(np.asarray(stamps, dtype=np.int64)) & UINT32_MASK) / 1e6
    if np.any(dt <= 0) or np.any(dt > 3 / fs):
        raise ValueError("时间戳重复、复位或采样间隔过大；请检查采样率并重新采集")
    measured_fs = (len(data) - 1) / np.sum(dt)
    if abs(measured_fs / fs - 1) > 0.05:
        raise ValueError(f"实际频率 {measured_fs:.3f} Hz 与 --fs {fs:g} Hz 不一致")
    seconds = np.r_[0.0, np.cumsum(dt)]
    grid = np.arange(int(np.floor(seconds[-1] * fs)) + 1) / fs
    uniform = np.column_stack([np.interp(grid, seconds, data[:, axis]) for axis in range(6)])
    summary = dict(original_samples=len(data), uniform_samples=len(uniform),
                   measured_fs_hz=float(measured_fs), target_fs_hz=fs,
                   duration_s=float(seconds[-1]), max_interval_s=float(np.max(dt)))
    return uniform, summary


def inspect_data(data, fs, t_init, t_wait, gravity):
    """报告静止段和方向覆盖；六方向检查用于本工具的完整内参标定。"""
    import numpy as np
    from ImuCalibrationModules import imu_calibration as imu

    initial = data[:int(t_init * fs)-1]
    if len(initial) < 2:
        raise ValueError("初始静止段太短")
    noise = np.sum(np.var(initial[:, :3], axis=0)**2)
    best = None
    for multiplier in (2, 5, 10, 20, 50, 100, 199):
        starts, ends = imu.find_static_imu_intervals(data[:, :3], fs, t_wait, multiplier * noise)
        long_enough = ends - starts >= max(1, int(0.5 * fs))
        means = imu.compute_accel_averages(starts[long_enough], ends[long_enough], data[:, :3])
        unique = []
        for mean in means:
            norm = np.linalg.norm(mean)
            if norm < 0.5 * gravity:
                continue
            direction = mean / norm
            # Cluster within five degrees for a readable pose count.
            if not any(np.dot(direction, previous) > np.cos(np.deg2rad(5)) for previous in unique):
                unique.append(direction)
        candidate = (len(unique), len(means), len(starts), multiplier, means)
        if best is None or candidate[:2] > best[:2]:
            best = candidate
    count, stable, detected, multiplier, means = best
    missing = []
    if len(means):
        directions = means / np.maximum(np.linalg.norm(means, axis=1, keepdims=True), 1e-12)
        for axis, name in enumerate("XYZ"):
            if np.max(directions[:, axis]) < 0.65:
                missing.append(name + "+")
            if np.min(directions[:, axis]) > -0.65:
                missing.append(name + "-")
    else:
        missing = [axis + sign for axis in "XYZ" for sign in ("+", "-")]
    initial_std = np.std(initial[:, :3], axis=0)
    # Initial noise must be representative of an actual stationary hold.
    initial_stable = bool(np.max(initial_std) < 0.05 * gravity)
    report = dict(
        detected_static_segments=detected, stable_segments=stable,
        distinct_attitudes_approx=count, threshold_multiplier=multiplier,
        missing_gravity_directions=missing, required_direction_component=0.65,
        initial_accel_std_mps2=initial_std.tolist(), initial_stable=initial_stable,
        initial_gyro_mean_rads=np.mean(initial[:, 3:], axis=0).tolist(),
        initial_gyro_std_rads=np.std(initial[:, 3:], axis=0).tolist(),
        accel_min_mps2=np.min(data[:, :3], axis=0).tolist(),
        accel_max_mps2=np.max(data[:, :3], axis=0).tolist(),
        static_accel_means_mps2=means.tolist(),
        ready_for_full_calibration=count >= 10 and not missing and initial_stable,
    )
    print(f"静止段 {detected} 个，至少 0.5 秒的静止段 {stable} 个，约 {count} 个不同姿态。")
    if missing:
        print("完整内参标定的方向覆盖检查未通过，缺少：" + "、".join(missing))
    if not initial_stable:
        print("开头静止段有明显运动，请重新采集。")
    print("开头静止角速度均值 (rad/s)：", np.mean(initial[:, 3:], axis=0))
    return report


def inspect_recording(args):
    path = Path(args.input).resolve()
    metadata = json.loads(session_path(path).read_text(encoding="utf-8")) if session_path(path).exists() else {}
    t_init = args.t_init if args.t_init is not None else metadata.get("t_init")
    t_wait = args.t_wait if args.t_wait is not None else metadata.get("t_wait", 3.0)
    if t_init is None or min(t_init, t_wait, args.fs, args.gravity) <= 0:
        raise ValueError("请提供正确的 --t-init、--t-wait、--fs 和 --gravity")
    data, quality = load_si_csv(path, args.fs)
    if t_init + 2 * t_wait >= quality["duration_s"]:
        raise ValueError("有效记录时长不足")
    report = inspect_data(data, args.fs, t_init, t_wait, args.gravity)
    report.update(input=str(path), quality=quality, completed=metadata.get("completed"))
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"诊断报告：{output.resolve()}")
    return 0


def calibrate(args):
    import numpy as np
    from ImuCalibrationModules import imu_calibration as imu

    if args.fs <= 0 or args.gravity <= 0 or args.iterations < 3:
        raise ValueError("采样率、重力值必须大于 0；阈值搜索次数至少为 3")
    outdir = Path(args.output).resolve()
    outputs = [outdir / name for name in ("params.json", "params_acc.csv", "params_gyro.csv",
                                        "calibration_input.csv", "calibrated.csv")]
    if any(p.exists() for p in outputs):
        raise ValueError("结果目录已有同名文件，请选择新 --output 目录")
    path = Path(args.input).resolve()
    metadata = {}
    if session_path(path).exists():
        metadata = json.loads(session_path(path).read_text(encoding="utf-8"))
        if not metadata.get("completed"):
            raise ValueError("此采集会话未完成，请按提示重新采集")
    t_init = args.t_init if args.t_init is not None else metadata.get("t_init")
    t_wait = args.t_wait if args.t_wait is not None else metadata.get("t_wait", 3.0)
    if t_init is None or min(t_init, t_wait) <= 0:
        raise ValueError("请提供开头实际静止时长 --t-init（秒），或使用带会话文件的 record 数据")
    data, quality = load_si_csv(path, args.fs)
    if t_init + 2 * t_wait >= quality["duration_s"] or int(t_wait * args.fs / 2) < 1:
        raise ValueError("记录时长不足或检测窗口太小")
    print(f"数据检查通过：{quality['duration_s']:.2f} 秒，实际 {quality['measured_fs_hz']:.3f} Hz。", flush=True)
    diagnostic = inspect_data(data, args.fs, t_init, t_wait, args.gravity)
    outdir.mkdir(parents=True, exist_ok=True)
    (outdir / "diagnostics.json").write_text(json.dumps(diagnostic, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    if not diagnostic["ready_for_full_calibration"]:
        raise ValueError("完整标定数据检查未通过；请先采集 X/Y/Z 各轴正反朝上的静止姿态，再增加倾斜姿态")
    print("开始标定；加速度计和陀螺仪输入已为 SI 单位，比例因子初值均为 1。", flush=True)
    acc, gyro = imu.calibrate_imu_from_data(
        t_init, t_wait, data, args.fs,
        theta_init_acc=np.array([0., 0., 0., 1., 1., 1., 0., 0., 0.]),
        theta_init_gyro=np.array([0., 0., 0., 0., 0., 0., 1., 1., 1.]),
        g=args.gravity, n_iteration=args.iterations, spanish=False,
    )
    if not np.isfinite(acc).all() or not np.isfinite(gyro).all():
        raise ValueError("标定结果包含非有限值，请检查数据与静止姿态")
    corrected_acc = imu.apply_accel_calibration(acc, data[:, :3])
    corrected_gyro = imu.apply_gyro_calibration(gyro[:9], data[:, 3:] - gyro[9:])
    outdir.mkdir(parents=True, exist_ok=True)
    result = dict(
        input=str(path), quality=quality, fs=args.fs, t_init=t_init, t_wait=t_wait,
        gravity_mps2=args.gravity, units=dict(acceleration="m/s^2", angular_rate="rad/s"),
        accelerometer=dict(parameters=acc.tolist(), misalignment=acc[:3].tolist(),
                           scale=acc[3:6].tolist(), additive_bias_mps2=acc[6:].tolist(),
                           model="a_corrected = T_a @ diag(scale) @ (a_measured + additive_bias)"),
        gyroscope=dict(parameters=gyro.tolist(), misalignment=gyro[:6].tolist(),
                       scale=gyro[6:9].tolist(), subtractive_bias_rads=gyro[9:].tolist(),
                       model="w_corrected = T_g @ diag(scale) @ (w_measured - subtractive_bias)"),
    )
    (outdir / "params.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    np.savetxt(outdir / "params_acc.csv", acc, delimiter=",")
    np.savetxt(outdir / "params_gyro.csv", gyro, delimiter=",")
    with (outdir / "calibration_input.csv").open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerows([["Fs", args.fs], ["Logging Type", 0], ["Initialization time", t_init],
                         ["Waiting time", t_wait], ["ax", "ay", "az", "gx", "gy", "gz"]])
        writer.writerows(data)
    np.savetxt(outdir / "calibrated.csv", np.column_stack((np.arange(len(data))/args.fs,
               corrected_acc, corrected_gyro)), delimiter=",", comments="",
               header="time_s,ax_mps2,ay_mps2,az_mps2,gx_rads,gy_rads,gz_rads")
    print("加速度计比例因子：", acc[3:6])
    print("加速度计加性修正偏置 (m/s²)：", acc[6:])
    print("陀螺仪比例因子：", gyro[6:9])
    print("陀螺仪需减去的零偏 (rad/s)：", gyro[9:])
    print(f"结果保存至：{outdir}")
    if args.plots:
        imu.show_data(data, args.fs, acc, gyro, "STM32 IMU", spanish=False)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    rec = commands.add_parser("record", help="复用 imu_for_slam 解码器，按提示采集连续串口数据")
    rec.add_argument("--port", help="省略或设置 auto 时自动选择 USB 串口；也可手动指定设备名")
    rec.add_argument("--baud", type=int, default=460800)
    rec.add_argument("--protocol-tools", type=Path, default=DEFAULT_TOOLS)
    rec.add_argument("--output", default="data/stm32_session.csv")
    rec.add_argument("--positions", type=int, default=40)
    rec.add_argument("--t-init", type=float, default=30.0)
    rec.add_argument("--hold", type=float, default=6.0)
    rec.add_argument("--move", type=float, default=4.0)
    rec.add_argument("--t-wait", type=float, default=3.0, help="静止检测滑动窗口长度（秒）")
    rec.set_defaults(run=record)
    cal = commands.add_parser("calibrate", help="标定 record 或 serial_receive.py 保存的 SI CSV")
    cal.add_argument("--input", required=True)
    cal.add_argument("--output", default="data/stm32_calibration")
    cal.add_argument("--fs", type=float, default=200.0)
    cal.add_argument("--t-init", type=float)
    cal.add_argument("--t-wait", type=float)
    cal.add_argument("--gravity", type=float, default=9.80665)
    cal.add_argument("--iterations", type=int, default=200)
    cal.add_argument("--plots", action="store_true", help="完成后显示标定前后图表")
    cal.set_defaults(run=calibrate)
    diag = commands.add_parser("inspect", help="检查静止段、重复姿态、六方向覆盖及静止角速度均值")
    diag.add_argument("--input", required=True)
    diag.add_argument("--output", default="data/diagnostics.json")
    diag.add_argument("--fs", type=float, default=200.0)
    diag.add_argument("--t-init", type=float)
    diag.add_argument("--t-wait", type=float)
    diag.add_argument("--gravity", type=float, default=9.80665)
    diag.set_defaults(run=inspect_recording)
    args = parser.parse_args()
    try:
        return args.run(args)
    except (OSError, ValueError, RuntimeError, ImportError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
