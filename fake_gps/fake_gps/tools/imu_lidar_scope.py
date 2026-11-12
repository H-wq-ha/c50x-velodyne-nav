#!/usr/bin/env python3
"""Record IMU v2 serial + VLP-16 reception and draw matplotlib impulse plots.

Default x-axis is application receive time; an additional figure uses the
hardware time counters. Success=1, detected failure/timeout=0. No missing
packet is assigned a fabricated arrival time. ROS mode does not bind UDP 2368.
"""
import argparse
from collections import Counter, deque
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import threading
import time
from zoneinfo import ZoneInfo

from imu_receive import FrameParser, lidar_stamp_ns
from lidar_receive import LidarReceiver


def event(source, event_kind, success, reason, receive_ns=None, monotonic_ns=None, **details):
    return dict(source=source, event=event_kind, success=int(success), reason=reason,
                receive_ns=receive_ns if receive_ns is not None else time.time_ns(),
                monotonic_ns=monotonic_ns if monotonic_ns is not None else time.monotonic_ns(),
                hardware_ns=None, sequence=None, details=details)


class ImuReceiver:
    def __init__(self, emit, stop, port, baud=460800, timeout=0.1, start_stream=True):
        self.emit, self.stop, self.port, self.baud = emit, stop, port, baud
        self.timeout, self.start_stream = timeout, start_stream
        self.error = None
        self.parser = FrameParser()

    def run(self):
        previous = None
        try:
            import serial
            with serial.Serial(self.port, self.baud, timeout=0.02, write_timeout=1, exclusive=True) as stream:
                if self.start_stream:
                    stream.write(b"@START\r\n@STATS\r\n")
                last_good = time.monotonic()
                while not self.stop.is_set():
                    data = stream.read(max(1, min(stream.in_waiting, 4096)))
                    received, mono = time.time_ns(), time.monotonic_ns()
                    bad_before = self.parser.crc_errors
                    records = self.parser.feed(data)
                    for _ in range(self.parser.crc_errors - bad_before):
                        self.emit(event("imu", "crc_error", False, "crc_mismatch", received, mono))
                    for record in records:
                        if record["kind"] == "status":
                            self.emit(event("imu", "status", True, "device_status", received, mono, **record))
                            continue
                        last_good = time.monotonic()
                        good = record["flags"] & 3 == 3 and 0 <= record["hour_us"] < 3_600_000_000
                        reason = "ok" if good else "invalid_time_flags"
                        if good and not all(math.isfinite(value) for value in (
                                *record["accel_mps2"], *record["gyro_rads"], record["temperature_c"])):
                            good, reason = False, "nonfinite_measurement"
                        current = event("imu", "data", good, reason,
                                        received, mono, **record)
                        current["sequence"] = record["sequence"]
                        if good:
                            current["hardware_ns"] = lidar_stamp_ns(record["hour_us"], received)
                        if previous is not None:
                            if record["monotonic_us"] <= previous["monotonic_us"]:
                                self.emit(event("imu", "clock_reset", False, "device_time_restarted", received, mono))
                            else:
                                delta = (record["sequence"] - previous["sequence"]) & 0xFFFFFFFF
                                if 1 < delta < 0x80000000:
                                    self.emit(event("imu", "sequence_gap", False, "missing_sequence_detected",
                                                    received, mono, missing=delta - 1))
                        previous = record
                        self.emit(current)
                    now = time.monotonic()
                    if now - last_good >= self.timeout:
                        self.emit(event("imu", "timeout", False, "no_imu_frame_timeout", received, mono))
                        last_good = now
        except Exception as error:
            self.error = f"{type(error).__name__}: {error}"
            self.emit(event("imu", "receiver_error", False, self.error))


class Recorder:
    def __init__(self, directory, max_events=50000):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.stream = (self.directory / "events.csv").open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.stream)
        self.writer.writerow(("source", "receive_ns", "receive_beijing", "monotonic_ns", "hardware_ns",
                              "success", "event", "reason", "sequence", "details_json"))
        self.events = deque(maxlen=max_events)
        self.counts = {name: Counter() for name in ("imu", "lidar")}
        self.latest = {}
        self.first = {}
        self.last = {}
        self.first_hardware_ns = None
        self.first_receive_ns = None

    def append(self, item):
        source = item["source"]
        counts = self.counts[source]
        counts[item["event"]] += 1
        if item["event"] == "status":
            self.latest["imu_status"] = item["details"]
        else:
            self.events.append(item)
            if self.first_receive_ns is None:
                self.first_receive_ns = item["receive_ns"]
            if item["event"] == "data" and item["success"]:
                counts["success"] += 1
                self.first.setdefault(source, item["monotonic_ns"])
                self.last[source] = item["monotonic_ns"]
                self.latest[source] = item
                if self.first_hardware_ns is None and item["hardware_ns"] is not None:
                    self.first_hardware_ns = item["hardware_ns"]
            elif not item["success"]:
                counts["failure_events"] += 1
                counts["missing_sequences"] += item["details"].get("missing", 0)
        date = datetime.fromtimestamp(item["receive_ns"] / 1e9, ZoneInfo("Asia/Shanghai")).isoformat(timespec="microseconds")
        self.writer.writerow((source, item["receive_ns"], date, item["monotonic_ns"], item["hardware_ns"],
                              item["success"], item["event"], item["reason"], item["sequence"],
                              json.dumps(item["details"], ensure_ascii=False)))

    def summary(self):
        result = dict(sources={}, latest_status=self.latest.get("imu_status"),
                      receive_axis="application_receive_time", hardware_axis="nearest_host_hour_plus_device_hour_us")
        for source, counts in self.counts.items():
            duration = (self.last.get(source, 0) - self.first.get(source, 0)) / 1e9
            result["sources"][source] = dict(counts, receive_hz=round((counts["success"] - 1) / duration, 4)
                                              if duration > 0 and counts["success"] > 1 else None,
                                              crc_errors=counts["crc_error"], timeouts=counts["timeout"],
                                              failure_events=counts["failure_events"],
                                              missing_sequences=counts["missing_sequences"])
        result["latest_samples"] = {key: value for key, value in self.latest.items() if key != "imu_status"}
        return result

    def flush(self):
        self.stream.flush()
        (self.directory / "summary.json").write_text(json.dumps(self.summary(), ensure_ascii=False, indent=2), encoding="utf-8")


def draw(recorder, figure, axis_kind="receive", window=1.0):
    axes = figure.axes
    if len(axes) != 2:
        axes = figure.subplots(2, 1, sharex=True)
    key = "receive_ns" if axis_kind == "receive" else "hardware_ns"
    values = [item[key] for item in recorder.events if item[key] is not None]
    if not values:
        return
    right = max(values)
    left = right - int(window * 1e9) if window > 0 else min(values)
    origin = left
    for axis, source, color in zip(axes, ("imu", "lidar"), ("#2563eb", "#059669")):
        axis.clear()
        items = [item for item in recorder.events if item["source"] == source and item[key] is not None
                 and left <= item[key] <= right]
        good = [(item[key] - origin) / 1e9 for item in items if item["success"] and item["event"] == "data"]
        bad = [(item[key] - origin) / 1e9 for item in items if not item["success"]]
        if good:
            axis.vlines(good, 0, 1, colors=color, linewidth=0.8, alpha=0.85, label="valid reception")
            axis.scatter(good, [1] * len(good), s=8, color=color, zorder=3)
        if bad:
            axis.scatter(bad, [0] * len(bad), marker="x", s=25, color="#dc2626", label="detected failure / timeout", zorder=4)
        counts = recorder.summary()["sources"][source]
        rate = counts["receive_hz"]
        rate_label = f"{rate:.2f} Hz" if rate is not None else "waiting"
        lidar_unit = recorder.latest.get("lidar", {}).get("details", {}).get("unit", "scan")
        lidar_label = "VLP-16 | complete ROS scans" if lidar_unit == "scan" else "VLP-16 | raw UDP datagrams"
        label = "IMU | CRC-checked serial samples" if source == "imu" else lidar_label
        axis.set_title(f"{label}   ·   {rate_label}   ·   valid {counts.get('success', 0)}   ·   failures {counts.get('failure_events', 0)}",
                       fontsize=10, loc="left", pad=10)
        axis.set_ylim(-0.15, 1.22)
        axis.set_yticks((0, 1))
        axis.set_yticklabels(("0  failure", "1  success"))
        axis.grid(axis="x", alpha=0.18)
        axis.spines['top'].set_visible(False)
        axis.spines['right'].set_visible(False)
        if good or bad:
            axis.legend(loc="upper right", fontsize=8, framealpha=0.9)
    axes[-1].set_xlim(0, max((right - left) / 1e9, 0.1))
    label = datetime.fromtimestamp(origin / 1e9, ZoneInfo("Asia/Shanghai")).isoformat(timespec="milliseconds")
    description = "Host receive time" if axis_kind == "receive" else "Device time, mapped to nearest host hour"
    axes[-1].set_xlabel(f"{description}: seconds after {label}")
    figure.suptitle("IMU and LiDAR reception impulses" if axis_kind == "receive" else "IMU and LiDAR hardware-time impulses",
                    fontsize=14, fontweight="bold")
    figure.subplots_adjust(left=0.1, right=0.98, bottom=0.14, top=0.86, hspace=0.55)


def default_port():
    import serial.tools.list_ports
    ports = [port.device for port in serial.tools.list_ports.comports()
             if port.vid is not None and ("ttyACM" in port.device or "ttyUSB" in port.device)]
    if len(ports) == 1:
        return ports[0]
    raise ValueError("Specify --port: no unique USB-UART device was found")


def main(argv=None):
    argument = argparse.ArgumentParser(description=__doc__)
    argument.add_argument("--port", help="Auto-selects only when exactly one USB serial device exists")
    argument.add_argument("--baud", type=int, default=460800)
    argument.add_argument("--lidar-source", choices=("ros", "udp"), default="ros")
    argument.add_argument("--topic", default="/velodyne_packets")
    argument.add_argument("--lidar-port", type=int, default=2368)
    argument.add_argument("--device-ip", default="192.168.1.201")
    argument.add_argument("--duration", type=float, default=0, help="Seconds; 0 runs until Ctrl-C or window close")
    argument.add_argument("--window", type=float, default=1.0, help="Displayed time span, seconds; 0 shows retained session")
    argument.add_argument("--headless", action="store_true", help="Save matplotlib PNGs without opening a window")
    argument.add_argument("--no-start", action="store_true", help="Do not send @START to the IMU")
    argument.add_argument("--output", type=Path, default=None)
    args = argument.parse_args(argv)
    if args.duration < 0 or args.window < 0:
        argument.error("duration/window must be nonnegative")
    try:
        port = args.port or default_port()
    except (ImportError, ValueError) as error:
        argument.error(str(error))
    directory = args.output or Path(__file__).resolve().parents[1] / "output" / (
        "reception_" + datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S"))
    import matplotlib
    if args.headless:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    stop = threading.Event()
    incoming = queue.Queue()
    recorder = Recorder(directory)
    imu = ImuReceiver(incoming.put, stop, port, args.baud, start_stream=not args.no_start)
    lidar = LidarReceiver(incoming.put, stop, args.lidar_source, args.topic, args.device_ip, port=args.lidar_port)
    threads = [threading.Thread(target=item.run, daemon=True) for item in (imu, lidar)]
    figure = plt.figure(figsize=(14, 6.5), facecolor="white")
    figure.canvas.mpl_connect("close_event", lambda _: stop.set())
    for thread in threads:
        thread.start()
    if not args.headless:
        plt.show(block=False)
    started = last_report = last_draw = time.monotonic()
    try:
        while not stop.is_set() and (args.duration <= 0 or time.monotonic() - started < args.duration):
            for _ in range(2000):
                try:
                    recorder.append(incoming.get_nowait())
                except queue.Empty:
                    break
            now = time.monotonic()
            if not args.headless and now - last_draw >= 0.2:
                draw(recorder, figure, "receive", args.window)
                figure.canvas.draw_idle()
                figure.canvas.flush_events()
                last_draw = now
            if now - last_report >= 1:
                recorder.flush()
                print(json.dumps(recorder.summary()["sources"], ensure_ascii=False), flush=True)
                last_report = now
            if imu.error or lidar.error:
                break
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for thread in threads:
            thread.join(timeout=2)
        while not incoming.empty():
            recorder.append(incoming.get_nowait())
        recorder.flush()
        for kind in ("receive", "hardware"):
            draw(recorder, figure, kind, args.window)
            figure.savefig(directory / f"{kind}_impulses.png", dpi=160, facecolor="white")
            draw(recorder, figure, kind, 0)
            figure.savefig(directory / f"{kind}_overview.png", dpi=160, facecolor="white")
        recorder.stream.close()
        plt.close(figure)
    print(json.dumps(dict(output=str(directory), errors=dict(imu=imu.error, lidar=lidar.error),
                          **recorder.summary()), ensure_ascii=False), flush=True)
    return 1 if imu.error or lidar.error or any(recorder.counts[name]["success"] == 0 for name in ("imu", "lidar")) else 0


if __name__ == "__main__":
    raise SystemExit(main())
