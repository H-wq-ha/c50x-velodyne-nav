#!/usr/bin/env python3
"""Receive VLP-16 scans through ROS 2 or raw datagrams through an unused UDP port.

ROS mode observes the running driver without competing for UDP 2368. A ROS
event is one complete VelodyneScan, not one of its constituent UDP packets.
"""
import argparse
import csv
import json
import socket
import threading
import time

from imu_receive import lidar_stamp_ns


def validate_vlp16_packet(data):
    if len(data) != 1206:
        return False, "packet_length"
    if any(data[offset:offset + 2] != b"\xff\xee" for offset in range(0, 1200, 100)):
        return False, "block_header"
    if any(int.from_bytes(data[offset + 2:offset + 4], "little") >= 36000
           for offset in range(0, 1200, 100)):
        return False, "azimuth_range"
    if int.from_bytes(data[1200:1204], "little") >= 3_600_000_000:
        return False, "hardware_time_range"
    return True, "ok"


def receive_event(success, reason, data=None, receive_ns=None, monotonic_ns=None, **details):
    receive_ns = time.time_ns() if receive_ns is None else receive_ns
    monotonic_ns = time.monotonic_ns() if monotonic_ns is None else monotonic_ns
    event = dict(source="lidar", event="data", success=int(success), reason=reason,
                 receive_ns=receive_ns, monotonic_ns=monotonic_ns,
                 hardware_ns=None, sequence=None, details=details)
    if success and data is not None:
        hour_us = int.from_bytes(data[1200:1204], "little")
        event["hardware_ns"] = lidar_stamp_ns(hour_us, receive_ns)
        details["hour_us"] = hour_us
        details["return_mode"] = data[1204]
        details["product_id"] = data[1205]
    return event


class LidarReceiver:
    def __init__(self, emit, stop, source="ros", topic="/velodyne_packets",
                 device_ip="192.168.1.201", bind="0.0.0.0", port=2368, timeout=0.5):
        self.emit, self.stop, self.source = emit, stop, source
        self.topic, self.device_ip, self.bind, self.port = topic, device_ip, bind, port
        self.timeout = timeout
        self.error = None

    def run(self):
        try:
            if self.source == "ros":
                self._ros()
            else:
                self._udp()
        except Exception as error:
            self.error = f"{type(error).__name__}: {error}"
            event = receive_event(False, self.error)
            event["event"] = "receiver_error"
            self.emit(event)

    def _ros(self):
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data
            from velodyne_msgs.msg import VelodyneScan
        except ImportError as error:
            raise RuntimeError("ROS mode needs ROS 2 Humble and velodyne_msgs; source ROS setup.bash first") from error
        context = Context()
        rclpy.init(args=[], context=context)
        node = rclpy.create_node("vlp16_reception_scope", context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        last = time.monotonic()

        def callback(message):
            nonlocal last
            received, mono = time.time_ns(), time.monotonic_ns()
            last = time.monotonic()
            good, reason = bool(message.packets), "empty_scan"
            for packet in message.packets:
                good, reason = validate_vlp16_packet(bytes(packet.data))
                if not good:
                    break
            final = bytes(message.packets[-1].data) if message.packets else None
            event = receive_event(good, reason, final, received, mono,
                                  packets=len(message.packets), frame_id=message.header.frame_id,
                                  ros_stamp_ns=message.header.stamp.sec * 1_000_000_000
                                  + message.header.stamp.nanosec, unit="scan")
            self.emit(event)

        subscription = node.create_subscription(VelodyneScan, self.topic, callback, qos_profile_sensor_data)
        try:
            while not self.stop.is_set():
                executor.spin_once(timeout_sec=0.05)
                now = time.monotonic()
                if now - last >= self.timeout:
                    event = receive_event(False, "no_scan_timeout", unit="scan")
                    event["event"] = "timeout"
                    self.emit(event)
                    last = now
        finally:
            executor.shutdown()
            node.destroy_subscription(subscription)
            node.destroy_node()
            context.try_shutdown()

    def _udp(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as stream:
            try:
                stream.bind((self.bind, self.port))
            except OSError as error:
                raise RuntimeError(f"Cannot bind UDP {self.port}; use ROS mode while velodyne_driver is running") from error
            stream.settimeout(self.timeout)
            while not self.stop.is_set():
                try:
                    data, address = stream.recvfrom(65535)
                    received, mono = time.time_ns(), time.monotonic_ns()
                except socket.timeout:
                    event = receive_event(False, "no_datagram_timeout", unit="udp_packet")
                    event["event"] = "timeout"
                    self.emit(event)
                    continue
                if self.device_ip and address[0] != self.device_ip:
                    continue
                good, reason = validate_vlp16_packet(data)
                self.emit(receive_event(good, reason, data, received, mono, bytes=len(data),
                                        device_ip=address[0], unit="udp_packet"))


def main():
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument("--source", choices=("ros", "udp"), default="ros")
    args.add_argument("--topic", default="/velodyne_packets")
    args.add_argument("--device-ip", default="192.168.1.201")
    args.add_argument("--port", type=int, default=2368)
    args.add_argument("--duration", type=float, default=10)
    args.add_argument("--csv", default="lidar_reception.csv")
    options = args.parse_args()
    stop = threading.Event()
    count = 0
    writer_stream = open(options.csv, "w", newline="", encoding="utf-8")
    writer = csv.writer(writer_stream)
    writer.writerow(("receive_ns", "hardware_ns", "success", "event", "reason", "details"))

    def emit(event):
        nonlocal count
        writer.writerow((event["receive_ns"], event["hardware_ns"], event["success"],
                         event["event"], event["reason"], json.dumps(event["details"])))
        if event["success"]:
            count += 1

    receiver = LidarReceiver(emit, stop, source=options.source, topic=options.topic,
                             device_ip=options.device_ip, port=options.port)
    thread = threading.Thread(target=receiver.run)
    thread.start()
    try:
        started = time.monotonic()
        while thread.is_alive() and (options.duration <= 0 or time.monotonic() - started < options.duration):
            stop.wait(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        thread.join(timeout=2)
        writer_stream.close()
    print(json.dumps(dict(success=count, error=receiver.error, csv=options.csv)))
    return 1 if receiver.error or count == 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
