#!/usr/bin/env python3
"""Observe a running calibration without opening serial or moving the rig."""
import argparse
import json
import math
import struct
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=20)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = Node('check_calibration_hardware_topics')
    report = dict(imu=0, clouds=0, odometry=0, errors=[])
    previous = {}

    def check_stamp(msg, key):
        t = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if key in previous and t <= previous[key]:
            report['errors'].append(f'{key}: timestamp not increasing')
        previous[key] = t
        report[key + '_latest_ns'] = t

    def imu(msg):
        check_stamp(msg, 'imu')
        report['imu'] += 1
        if not all(math.isfinite(v) for v in (msg.angular_velocity.x, msg.angular_velocity.y,
                    msg.angular_velocity.z, msg.linear_acceleration.x, msg.linear_acceleration.y,
                    msg.linear_acceleration.z)):
            report['errors'].append('IMU not finite')

    def cloud(msg):
        check_stamp(msg, 'clouds')
        report['clouds'] += 1
        fields = {f.name: f for f in msg.fields}
        if 'ring' not in fields or 'time' not in fields or fields['time'].datatype != 7:
            report['errors'].append('Point cloud missing ring / FLOAT32 time')
            return
        data = bytes(msg.data)
        offsets = [struct.unpack_from('<f', data, row*msg.row_step+col*msg.point_step+fields['time'].offset)[0]
                   for row in range(msg.height) for col in range(msg.width)]
        if not offsets or not all(math.isfinite(t) and 0 <= t <= 0.2 for t in offsets):
            report['errors'].append('Point firing offsets invalid')
        else:
            report['point_time_range_s'] = [min(offsets), max(offsets)]

    def odom(msg):
        report['odometry'] += 1
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not all(math.isfinite(x) for x in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
            report['errors'].append('Odometry not finite')

    node.create_subscription(Imu, '/calibration/imu', imu, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/calibration/velodyne_points', cloud, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/aft_mapped_to_init', odom, qos_profile_sensor_data)
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if min(report['imu'], report['clouds'], report['odometry']) == 0:
        report['errors'].append('Missing one or more required topics')
    if 'imu' in previous and 'clouds' in previous:
        report['latest_input_delta_s'] = (previous['imu']-previous['clouds'])/1e9
        if abs(report['latest_input_delta_s']) > 0.5:
            report['errors'].append('Inputs do not share a nearby epoch')
    report['passed'] = not report['errors']
    text = json.dumps(report, indent=2)
    print(text)
    if args.output:
        args.output.write_text(text)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
