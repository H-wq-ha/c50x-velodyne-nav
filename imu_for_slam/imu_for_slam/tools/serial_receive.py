#!/usr/bin/env python3
"""接收 STM32 IMU 串口数据；默认应用 calibration03，支持原始/校正对照 CSV。"""
from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
import sys
import time

# 与 tools 中的采集工具共用协议解析，防止两份包结构定义出现差异。
# 使用脚本自身位置定位，允许从其他工作目录启动本程序。
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from imu_monitor import CSV_FIELDS, Sample, StreamDecoder
from serial_ports import select_serial_port
from calibration_runtime import (CORRECTED_FIELDS, add_calibration_args,
                                 corrected_row, load_calibration)


def print_sample(sample: Sample) -> None:
    """values 的顺序为 ax、ay、az、gx、gy、gz、温度，固件已转为标准单位。"""
    ax, ay, az, gx, gy, gz, temperature = sample.values
    print(
        f"序号={sample.sequence}  MCU时间={sample.timestamp_us} us  [{sample.mode.upper()}]\n"
        f"  加速度: X={ax:+.3f}  Y={ay:+.3f}  Z={az:+.3f} m/s²\n"
        f"  角速度: X={gx:+.6f}  Y={gy:+.6f}  Z={gz:+.6f} rad/s\n"
        f"  温度: {temperature:.3f} °C",
        flush=True,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="列出电脑上的串口")
    parser.add_argument("--port", help="串口名，例如 /dev/ttyUSB0 或 COM5；省略时自动识别 USB 串口")
    parser.add_argument("--baud", type=int, default=460800, help="波特率，默认 460800")
    parser.add_argument("--mode", choices=("text", "hex"), default="hex",
                        help="请求固件使用的发送格式，默认 hex；接收仍自动识别两种格式")
    parser.add_argument("--no-command", action="store_true",
                        help="仅接收，不发送 PING、格式切换或 START 命令")
    parser.add_argument("--stats", action="store_true", help="每秒发送 STATS，显示板端频率与任务统计")
    parser.add_argument("--csv", help="保存全部原始 SI 样本（未校正）到新 CSV 文件")
    parser.add_argument("--count", type=int, default=0,
                        help="接收指定数量的有效样本后退出，0 表示持续接收")
    parser.add_argument("--print-every", type=int, default=1,
                        help="每 N 个有效样本显示一次，默认逐条显示；不影响 CSV 保存")
    add_calibration_args(parser, csv_output=True)
    args = parser.parse_args()
    if args.baud <= 0 or args.count < 0 or args.print_every < 1:
        parser.error("baud 必须大于 0，count 不能为负，print-every 必须至少为 1")
    if args.stats and args.no_command:
        parser.error("--stats 会发送命令，不能与 --no-command 同时使用")

    # 延迟导入：查看 --help 或导入解析器不需要安装 pyserial。
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        print("缺少 pyserial，请执行：python3 -m pip install pyserial", file=sys.stderr)
        return 2

    if args.list:
        ports = list(list_ports.comports())
        for port in ports:
            print(f"{port.device}  {port.description}")
        if not ports:
            print("未发现串口，请连接 USB-TTL 转换器。")
        return 0

    try:
        args.port = select_serial_port(args.port)
    except OSError as exc:
        print(f"串口选择失败：{exc}", file=sys.stderr)
        return 1

    try:
        calibration = load_calibration(args)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.error(f"标定参数加载失败：{exc}")

    decoder = StreamDecoder()
    total = 0
    previous_sequence = None
    output = None
    writer = None
    corrected_output = None
    corrected_writer = None
    try:
        # x 模式避免覆盖以前采集的数据。
        if args.csv:
            output = open(args.csv, "x", newline="", encoding="utf-8")
            writer = csv.DictWriter(output, fieldnames=CSV_FIELDS)
            writer.writeheader()

        if args.corrected_csv:
            corrected_output = open(args.corrected_csv, "x", newline="", encoding="utf-8")
            corrected_writer = csv.DictWriter(corrected_output, fieldnames=(*CSV_FIELDS, *CORRECTED_FIELDS))
            corrected_writer.writeheader()

        with serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
            write_timeout=1.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
            **({"exclusive": True} if os.name == "posix" else {}),
        ) as port:
            if not args.no_command:
                # HEX 模式下控制命令及状态回复仍然使用 ASCII 文本。
                commands = b"@PING\r\n" + (
                    b"@HEX\r\n" if args.mode == "hex" else b"@TEXT\r\n"
                ) + b"@START\r\n"
                if port.write(commands) != len(commands):
                    raise OSError("控制命令未完整发送")

            print(f"已打开 {args.port}，{args.baud} baud / 8N1；Ctrl+C 结束。",
                  file=sys.stderr)
            last_sample_time = time.monotonic()
            last_stats_time = last_sample_time
            waiting_reported = False
            while args.count == 0 or total < args.count:
                if args.stats and time.monotonic() - last_stats_time >= 1:
                    if port.write(b"@STATS\r\n") != 8:
                        raise OSError("统计查询未完整发送")
                    last_stats_time = time.monotonic()
                # read 的结果可能只有半包，也可能包含多包；不能按一次读取切包。
                data = port.read(min(port.in_waiting or 1, 4096))
                samples = decoder.feed(data)
                for message in decoder.take_diagnostics():
                    print(f"板端状态: {message}", file=sys.stderr)

                for sample in samples:
                    if previous_sequence is not None:
                        delta = (sample.sequence - previous_sequence) & 0xFFFFFFFF
                        if delta != 1:
                            print(
                                f"序号不连续: {previous_sequence} → {sample.sequence}"
                                "（可能是丢包、STOP/START 或板子复位）",
                                file=sys.stderr,
                            )
                    previous_sequence = sample.sequence
                    total += 1
                    raw_row = sample.csv_row()
                    corrected = calibration.apply(sample) if calibration else sample
                    if writer is not None:
                        writer.writerow(raw_row)
                    if corrected_writer is not None:
                        corrected_writer.writerow(corrected_row(raw_row, corrected))
                    if total == 1 or total % args.print_every == 0:
                        print_sample(corrected)
                    last_sample_time = time.monotonic()
                    waiting_reported = False
                    if args.count and total >= args.count:
                        break

                if not waiting_reported and time.monotonic() - last_sample_time >= 5:
                    print("5 秒没有有效 IMU 数据，请检查接线、板端状态及是否已 START。",
                          file=sys.stderr)
                    waiting_reported = True
            if not args.no_command:
                port.write(b"@STOP\r\n")
    except KeyboardInterrupt:
        print("\n接收已停止。", file=sys.stderr)
    except (OSError, serial.SerialException) as exc:
        print(f"串口接收失败: {exc}", file=sys.stderr)
        return 1
    finally:
        if output is not None:
            output.close()
        if corrected_output is not None:
            corrected_output.close()
        print(f"接收有效样本 {total} 条；拒绝异常候选包 {decoder.bad_frames} 个。",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
