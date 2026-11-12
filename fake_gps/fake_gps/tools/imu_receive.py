#!/usr/bin/env python3
"""Receive GPS-synchronized IMU v2 packets, save CSV, optionally publish ROS 2.

Only opening a serial port requires pyserial. Frame parsing uses the standard
library and is also importable by the host regression checks.
"""
import argparse
import binascii
import csv
import json
import struct
import sys
import time

MAGIC = b"\xa5\x5a"
FRAME_SIZE = 72
SAMPLE = struct.Struct("<IQQII7fII")
STATUS_NAMES = (
    "uptime_ms", "sysclk_hz", "flags", "who_am_i", "drdy_count",
    "imu_reads", "samples_queued", "lost_samples", "imu_io_errors",
    "sample_queue_drops", "tx_drops", "read_max_us", "read_rate_millihz",
    "send_rate_millihz", "pps_count", "nmea_sent",
)


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0
        self.discarded_bytes = 0

    def feed(self, data):
        self.buffer.extend(data)
        records = []
        while True:
            start = self.buffer.find(MAGIC)
            if start < 0:
                keep = 1 if self.buffer.endswith(MAGIC[:1]) else 0
                self.discarded_bytes += len(self.buffer) - keep
                if keep:
                    self.buffer[:] = self.buffer[-1:]
                else:
                    self.buffer.clear()
                break
            if start:
                self.discarded_bytes += start
                del self.buffer[:start]
            if len(self.buffer) < 6:
                break
            if self.buffer[2] != 2 or self.buffer[3] not in (1, 2) or self.buffer[4:6] != b"\x40\0":
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < FRAME_SIZE:
                break
            frame = bytes(self.buffer[:FRAME_SIZE])
            crc = binascii.crc_hqx(frame[2:70], 0xFFFF)
            if crc != int.from_bytes(frame[70:72], "little"):
                self.crc_errors += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            del self.buffer[:FRAME_SIZE]
            if frame[3] == 2:
                record = dict(zip(STATUS_NAMES, struct.unpack_from("<16I", frame, 6)))
                record["kind"] = "status"
            else:
                fields = SAMPLE.unpack_from(frame, 6)
                record = dict(kind="imu", sequence=fields[0], monotonic_us=fields[1],
                              epoch_us=fields[2], hour_us=fields[3], pps_count=fields[4],
                              accel_mps2=fields[5:8], gyro_rads=fields[8:11],
                              temperature_c=fields[11], flags=fields[12], lost_samples=fields[13])
            records.append(record)
        return records


def lidar_stamp_ns(hour_us, nominal_ns):
    """Match Velodyne's nearest-host-hour policy, preserving integer precision.

    This deliberately does not use the simulated calendar date for ROS lidar
    compatibility. Both LiDAR and IMU must use the same policy.
    """
    if not 0 <= hour_us < 3_600_000_000:
        raise ValueError("Invalid microseconds-past-hour")
    hour_ns = 3_600_000_000_000
    value = nominal_ns // hour_ns * hour_ns + hour_us * 1000
    if nominal_ns - value > hour_ns // 2:
        value += hour_ns
    elif value - nominal_ns > hour_ns // 2:
        value -= hour_ns
    return value


class RosPublisher:
    def __init__(self, topic, frame_id, time_mode):
        import rclpy
        from sensor_msgs.msg import Imu
        from rclpy.qos import qos_profile_sensor_data
        self.rclpy = rclpy
        self.message_type = Imu
        self.frame_id = frame_id
        self.time_mode = time_mode
        rclpy.init(args=[])
        self.node = rclpy.create_node("stm32_shared_clock_imu")
        self.publisher = self.node.create_publisher(Imu, topic, qos_profile_sensor_data)

    def publish(self, record):
        # bit0 time valid, bit1 hardware DRDY capture.
        if record["flags"] & 3 != 3:
            return
        timestamp = (lidar_stamp_ns(record["hour_us"], self.node.get_clock().now().nanoseconds)
                     if self.time_mode == "lidar" else record["epoch_us"] * 1000)
        message = self.message_type()
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(timestamp, 1_000_000_000)
        message.header.frame_id = self.frame_id
        message.orientation_covariance[0] = -1.0  # No orientation estimate in a six-axis raw packet.
        for axis, value in zip(("x", "y", "z"), record["gyro_rads"]):
            setattr(message.angular_velocity, axis, float(value))
        for axis, value in zip(("x", "y", "z"), record["accel_mps2"]):
            setattr(message.linear_acceleration, axis, float(value))
        self.publisher.publish(message)

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def main(argv=None):
    argument = argparse.ArgumentParser(description=__doc__)
    argument.add_argument("--port", help="USB-UART device, e.g. /dev/ttyUSB0")
    argument.add_argument("--baud", type=int, default=460800)
    argument.add_argument("--list-ports", action="store_true")
    argument.add_argument("--duration", type=float, default=0, help="Seconds; zero means continuous")
    argument.add_argument("--csv", help="CSV output path")
    argument.add_argument("--command", choices=("PING", "START", "STOP", "STATS"))
    argument.add_argument("--ros", action="store_true", help="Publish sensor_msgs/Imu using ROS 2")
    argument.add_argument("--topic", default="/imu/stm32_raw")
    argument.add_argument("--frame-id", default="imu_link")
    argument.add_argument("--ros-time", choices=("lidar", "epoch"), default="lidar")
    args = argument.parse_args(argv)
    try:
        import serial
        import serial.tools.list_ports
    except ImportError:
        argument.error("pyserial is required; use the Python environment with pyserial installed")
    if args.list_ports:
        for port in serial.tools.list_ports.comports():
            print(port.device, port.description)
        return 0
    if not args.port:
        argument.error("--port is required (or use --list-ports)")
    parser = FrameParser()
    output = open(args.csv, "w", newline="", encoding="utf-8") if args.csv else None
    writer = csv.writer(output) if output else None
    if writer:
        writer.writerow(("sequence", "monotonic_us", "epoch_us", "hour_us", "pps_count",
                         "ax_mps2", "ay_mps2", "az_mps2", "gx_rads", "gy_rads", "gz_rads",
                         "temperature_c", "flags", "lost_samples"))
    ros = None
    stream = None
    count = gaps = resets = 0
    previous = None
    last_status = None
    started = last_report = time.monotonic()
    interval_count = 0
    try:
        if args.ros:
            ros = RosPublisher(args.topic, args.frame_id, args.ros_time)
        stream = serial.Serial(args.port, args.baud, timeout=0.1, write_timeout=1)
        if args.command:
            stream.write(("@" + args.command + "\r\n").encode())
        while args.duration <= 0 or time.monotonic() - started < args.duration:
            for record in parser.feed(stream.read(max(1, min(stream.in_waiting, 4096)))):
                if record["kind"] == "status":
                    last_status = record
                    continue
                count += 1
                interval_count += 1
                if previous:
                    if record["monotonic_us"] <= previous["monotonic_us"]:
                        resets += 1
                    else:
                        advance = (record["sequence"] - previous["sequence"]) & 0xFFFFFFFF
                        if 1 < advance < 0x80000000:
                            gaps += advance - 1
                previous = record
                if writer:
                    writer.writerow((record["sequence"], record["monotonic_us"], record["epoch_us"],
                                     record["hour_us"], record["pps_count"], *record["accel_mps2"],
                                     *record["gyro_rads"], record["temperature_c"], record["flags"],
                                     record["lost_samples"]))
                if ros:
                    ros.publish(record)
            now = time.monotonic()
            if now - last_report >= 1:
                report = dict(host_receive_hz=round(interval_count / (now - last_report), 2),
                              received=count, sequence_gaps=gaps, resets=resets,
                              crc_errors=parser.crc_errors, latest=previous, status=last_status)
                print(json.dumps(report, ensure_ascii=False), flush=True)
                last_report = now
                interval_count = 0
                if output:
                    output.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if stream:
            stream.close()
        if output:
            output.close()
        if ros:
            ros.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
