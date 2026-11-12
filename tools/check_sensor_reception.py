#!/usr/bin/env python3
"""Exercise the existing shared-clock frontend and record diagnostic evidence.

Run after sourcing /opt/ros/humble/setup.bash. No motion/SLAM commands are sent.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import urllib.request
from datetime import datetime
from zoneinfo import ZoneInfo

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from velodyne_msgs.msg import VelodyneScan

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tool/calibration'))
from hardware_frontend import HardwareFrontend
from hardware_core import packet_hour_us, lidar_epoch_us

STATUS_NAMES = ('uptime_ms sysclk_hz flags who_am_i drdy_count imu_reads samples_queued '
                'lost_samples imu_io_errors sample_queue_drops tx_drops read_max_us '
                'read_rate_millihz send_rate_millihz pps_count nmea_sent').split()


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def hardware_web():
    result = {}
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    for name in ('status', 'settings'):
        try:
            with opener.open('http://192.168.1.201/cgi/' + name + '.json', timeout=2) as f:
                result[name] = json.load(f)
        except Exception as e:
            result[name] = {'error': str(e)}
    return result


class Diagnostic(HardwareFrontend):
    def __init__(self, args):
        super().__init__(args)
        self.records, self.statuses, self.scans, self.clouds, self.imus = [], [], [], [], []
        self.packet_times = []
        self.errors = []
        self.latest_points = None
        self.nmea = []
        original_feed = self.parser.feed

        def feed(data):
            records = original_feed(data)
            host_ns, mono_ns = time.time_ns(), time.monotonic_ns()
            for record in records:
                if record['kind'] == 'imu':
                    self.records.append(dict(record, receive_ns=host_ns, receive_monotonic_ns=mono_ns))
                else:
                    self.statuses.append(dict(zip(STATUS_NAMES, record['values']), receive_ns=host_ns))
            return records

        self.parser.feed = feed
        monitor_qos = QoSProfile(depth=20000, reliability=(ReliabilityPolicy.BEST_EFFORT
                                 if args.best_effort_monitor else ReliabilityPolicy.RELIABLE))
        self.create_subscription(VelodyneScan, args.packets_topic, self.raw_scan, monitor_qos)
        self.create_subscription(PointCloud2, '/calibration/velodyne_points', self.cloud, monitor_qos)
        self.create_subscription(Imu, '/calibration/imu', self.imu, monitor_qos)
        self.gps_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gps_socket.bind(('', 8308))
        self.gps_socket.setblocking(False)
        self.create_timer(0.01, self.gps)

    def imu(self, msg):
        self.imus.append([ns(msg.header.stamp), time.monotonic_ns(),
                          *[getattr(msg.linear_acceleration, a) for a in 'xyz'],
                          *[getattr(msg.angular_velocity, a) for a in 'xyz']])

    def gps(self):
        while True:
            try:
                data, sender = self.gps_socket.recvfrom(2048)
            except BlockingIOError:
                break
            start = data.find(b'$GPRMC,')
            sentence = data[start:].split(b'\x00')[0].decode('ascii', errors='replace').strip() if start >= 0 else ''
            self.nmea.append(dict(receive_ns=time.time_ns(), sender=sender[0], length=len(data),
                                  sentence=sentence, hex=data.hex()))

    def raw_scan(self, msg):
        if self.correction.last_epoch_us is None:
            return
        try:
            times = [lidar_epoch_us(packet_hour_us(p.data), self.correction.last_epoch_us) for p in msg.packets]
            self.packet_times.extend(times)
            azimuths = [int.from_bytes(bytes(p.data[2:4]), 'little') / 100 for p in msg.packets]
            self.scans.append([times[0], times[-1], time.monotonic_ns(), ns(msg.header.stamp),
                               len(times), azimuths[0], azimuths[-1],
                               ns(msg.packets[0].stamp), ns(msg.packets[-1].stamp)])
        except Exception as e:
            self.errors.append(str(e))

    def cloud(self, msg):
        formats = {1:'i1', 2:'u1', 3:'i2', 4:'u2', 5:'i4', 6:'u4', 7:'f4', 8:'f8'}
        dtype = np.dtype({'names':[f.name for f in msg.fields],
                          'formats':[('>' if msg.is_bigendian else '<') + formats[f.datatype] for f in msg.fields],
                          'offsets':[f.offset for f in msg.fields], 'itemsize':msg.point_step})
        p = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                       strides=(msg.row_step, msg.point_step)).reshape(-1)
        required = {'x','y','z','ring','time','intensity'}
        if not required.issubset(dtype.names):
            self.errors.append('Missing point fields: ' + str(dtype.names))
            return
        xyz = np.column_stack([p[a] for a in 'xyz'])
        finite = np.isfinite(xyz).all(axis=1)
        t, rings = p['time'], p['ring']
        ranges = np.linalg.norm(xyz[finite], axis=1)
        if not len(ranges):
            self.errors.append('Empty valid cloud')
            return
        self.clouds.append([ns(msg.header.stamp), time.monotonic_ns(), len(p), int(finite.sum()),
                            float(t.min()), float(t.max()), int(rings.min()), int(rings.max()),
                            len(np.unique(rings)), float(ranges.min()), float(ranges.max()),
                            int(np.isfinite(t).sum()), int(((t < 0) | (t > 0.2)).sum())])
        self.latest_points = {a:p[a].copy() for a in required}
        if len(self.clouds) == 1:
            np.savez_compressed(self.directory / 'first_cloud.npz', **self.latest_points)

    def save_diagnostics(self):
        for name, values in [('imu_samples', self.records), ('firmware_status', self.statuses),
                             ('nmea_packets', self.nmea)]:
            with (self.directory / (name + '.jsonl')).open('w') as f:
                for value in values:
                    f.write(json.dumps(value) + '\n')
        for name, header, values in [
            ('raw_scans', 'first_epoch_us last_epoch_us receive_monotonic_ns driver_header_ns packets first_azimuth last_azimuth driver_first_packet_ns driver_last_packet_ns', self.scans),
            ('clouds', 'stamp_ns receive_monotonic_ns points finite_xyz min_time max_time min_ring max_ring ring_count range_min range_max finite_times invalid_times', self.clouds),
            ('published_imu', 'stamp_ns receive_monotonic_ns ax ay az gx gy gz', self.imus),
            ('packet_times', 'epoch_us', [[t] for t in self.packet_times])]:
            with (self.directory / (name + '.csv')).open('w') as f:
                w = csv.writer(f)
                w.writerow(header.split())
                w.writerows(values)
        if self.latest_points:
            np.savez_compressed(self.directory / 'last_cloud.npz', **self.latest_points)
        (self.directory / 'diagnostic_errors.json').write_text(json.dumps(self.errors, indent=2))
        self.gps_socket.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=75)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', default='/dev/serial/by-id/usb-FIREPHX_USB_SER_FX2348N-if00')
    parser.add_argument('--best-effort-monitor', action='store_true',
                        help='Reproduce lossy topic monitoring; default requests reliable delivery')
    args = parser.parse_args()
    args.baud = 460800
    args.packets_topic = '/velodyne_packets'
    args.accel_calibration = str(ROOT / 'tool/calibration/accel_calibration03.json')
    args.output = args.output.resolve()
    # Frontend owns its new output directory; preserve logs alongside it until then.
    args.output.parent.mkdir(parents=True, exist_ok=True)
    logs, processes = [], []
    node = None
    before = hardware_web()
    started = datetime.now(ZoneInfo('Asia/Shanghai')).isoformat()
    try:
        driver = ['/opt/ros/humble/lib/velodyne_driver/velodyne_driver_node', '--ros-args',
                  '-r', '__node:=reception_velodyne_driver']
        for key, value in dict(model='VLP16', device_ip='192.168.1.201', port=2368, rpm=600.0,
                               frame_id='velodyne', gps_time=True, time_offset=0.0,
                               timestamp_first_packet=True).items():
            driver += ['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}']
        decoder = ['/opt/ros/humble/lib/velodyne_pointcloud/velodyne_transform_node', '--ros-args',
                   '-r', '__node:=reception_velodyne_transform',
                   '-r', 'velodyne_packets:=/calibration/velodyne_packets',
                   '-r', 'velodyne_points:=/calibration/velodyne_points']
        for key, value in dict(calibration='/opt/ros/humble/share/velodyne_pointcloud/params/VLP16db.yaml',
                               model='VLP16', min_range=0.5, max_range=50.0,
                               organize_cloud=False).items():
            decoder += ['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}']
        rclpy.init(args=[])
        node = Diagnostic(args)
        for name, cmd in [('driver', driver), ('decoder', decoder)]:
            log = (args.output / (name + '.log')).open('w')
            logs.append(log)
            processes.append(subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        while rclpy.ok() and time.monotonic() - node.started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.03)
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('Driver or decoder exited; inspect logs')
    except Exception as e:
        if node:
            node.failure = str(e)
        print('Capture failure:', e, flush=True)
        return 1
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
        for log in logs:
            log.close()
        if node:
            node.save_diagnostics()
            node.close()
            (args.output / 'capture_meta.json').write_text(json.dumps(dict(
                started_beijing=started, finished_beijing=datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),
                requested_duration_s=args.duration, monitor_reliability=(
                    'best_effort' if args.best_effort_monitor else 'reliable'),
                web_before=before, web_after=hardware_web()), indent=2))
        if rclpy.ok():
            rclpy.shutdown()
    print('Saved:', args.output, flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
