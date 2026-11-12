#!/usr/bin/env python3
"""Convert only calibration inputs to ROS1, indexed by hardware header time."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
from rosbags.rosbag2 import Reader
from rosbags.rosbag1 import Writer, Reader as Reader1
from rosbags.typesys import Stores, get_typestore


def sha256(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(chunk)
    return result.hexdigest()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('source', type=Path)
    p.add_argument('destination', type=Path)
    p.add_argument('--imu-csv', type=Path, help='Export unchanged corrected IMU for diagnostics')
    args = p.parse_args()
    types = get_typestore(Stores.ROS2_HUMBLE)
    ros1_types = get_typestore(Stores.ROS1_NOETIC)
    selected = {'/calibration/imu', '/calibration/velodyne_points'}
    evidence = {}
    imu_rows = []
    with Reader(args.source) as reader, Writer(args.destination) as writer:
        connections = [c for c in reader.connections if c.topic in selected]
        assert {c.topic for c in connections} == selected
        out = {c.id: writer.add_connection(c.topic, c.msgtype, typestore=ros1_types)
               for c in connections}
        for c, received, raw in reader.messages(connections=connections):
            msg = types.deserialize_cdr(raw, c.msgtype)
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            if args.imu_csv and c.topic == '/calibration/imu':
                imu_rows.append([stamp, received,
                    *[getattr(msg.linear_acceleration, k) for k in 'xyz'],
                    *[getattr(msg.angular_velocity, k) for k in 'xyz']])
            data = types.cdr_to_ros1(raw, c.msgtype)
            # Round-trip verification includes stamp, fields, covariances and data.
            restored = types.ros1_to_cdr(data, c.msgtype)
            assert restored == raw, (c.topic, stamp)
            writer.write(out[c.id], stamp, data)
            if c.topic not in evidence:
                evidence[c.topic] = dict(count=0, first_stamp_ns=stamp,
                                         last_stamp_ns=stamp, nonincreasing=0)
            e = evidence[c.topic]
            if e['count'] and stamp <= e['last_stamp_ns']:
                e['nonincreasing'] += 1
            e['count'] += 1
            e['last_stamp_ns'] = stamp
    if any(value['nonincreasing'] for value in evidence.values()):
        raise ValueError('输入硬件时间未严格递增，请检查录包后重新采集')
    with Reader1(args.destination) as reader:
        assert {c.topic: c.msgcount for c in reader.connections} == {
            k: v['count'] for k, v in evidence.items()}
        for connection in reader.connections:
            assert connection.digest == ros1_types.generate_msgdef(connection.msgtype)[1]
            evidence[connection.topic]['ros1_md5'] = connection.digest
        # Validate using the independent ROS1 type definitions as well.
        early_clouds = 0
        first_imu = evidence['/calibration/imu']['first_stamp_ns']
        for connection, timestamp, raw in reader.messages():
            ros1_types.deserialize_ros1(raw, connection.msgtype)
            if connection.topic == '/calibration/velodyne_points' and timestamp < first_imu:
                early_clouds += 1
    evidence['clouds_before_first_imu'] = early_clouds
    evidence['source'] = str(args.source.resolve())
    evidence['destination'] = str(args.destination.resolve())
    evidence['time_policy'] = 'ROS1 record time = unchanged hardware header stamp; no IMU recalibration'
    evidence['roundtrip_all_messages_byte_identical'] = True
    evidence['all_messages_deserialized_with_ros1_noetic_types'] = True
    evidence['source_sha256'] = {
        f.name: sha256(f)
        for f in args.source.glob('*.db3')}
    args.destination.with_suffix('.conversion.json').write_text(json.dumps(evidence, indent=2))
    if args.imu_csv:
        args.imu_csv.parent.mkdir(parents=True, exist_ok=True)
        with args.imu_csv.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['stamp_ns', 'receive_ns', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'])
            writer.writerows(imu_rows)
    print(json.dumps(evidence, indent=2))


if __name__ == '__main__':
    main()
