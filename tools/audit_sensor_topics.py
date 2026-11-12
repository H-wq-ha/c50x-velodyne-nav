#!/usr/bin/env python3
"""Lightweight independent observer for locating diagnostic/topic delivery loss."""
import argparse
import json
from pathlib import Path
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, Imu
from velodyne_msgs.msg import VelodyneScan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=48)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--reliable', action='store_true')
    args = ap.parse_args()
    rclpy.init(args=[])
    node = rclpy.create_node('independent_reception_audit')
    values = {'imu':[], 'scans':[], 'clouds':[]}
    qos = QoSProfile(depth=2000, reliability=(ReliabilityPolicy.RELIABLE if args.reliable
                                            else ReliabilityPolicy.BEST_EFFORT))

    def callback(key):
        def record(msg):
            values[key].append([msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec, time.monotonic_ns()])
        return record

    for msg_type, key, topic in [(Imu, 'imu', '/calibration/imu'),
                                 (VelodyneScan, 'scans', '/calibration/velodyne_packets'),
                                 (PointCloud2, 'clouds', '/calibration/velodyne_points')]:
        node.create_subscription(msg_type, topic, callback(key), qos)
    start = time.monotonic()
    while time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=.05)
    values['publisher_qos'] = {}
    for topic in ('/calibration/imu', '/calibration/velodyne_packets', '/calibration/velodyne_points'):
        values['publisher_qos'][topic] = [str(info.qos_profile) for info in node.get_publishers_info_by_topic(topic)]
    args.output.write_text(json.dumps(values))
    print({key:len(values[key]) for key in ('imu', 'scans', 'clouds')}, flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
