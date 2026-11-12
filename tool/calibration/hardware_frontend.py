#!/usr/bin/env python3
"""Publish hardware-stamped, bias-corrected IMU and Velodyne scans for calibration."""
import argparse
import copy
import csv
import hashlib
import json
import signal
import time
from datetime import datetime
from pathlib import Path

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
from velodyne_msgs.msg import VelodyneScan

from hardware_core import FrameParser, ImuCorrection, lidar_epoch_us, packet_hour_us
from capture_quality import CaptureReadiness, validate_cloud

HERE = Path(__file__).resolve().parent


def stamp(message, microseconds):
    message.sec, remainder = divmod(microseconds, 1_000_000)
    message.nanosec = remainder * 1000


class HardwareFrontend(Node):
    def __init__(self, args):
        super().__init__('calibration_hardware_frontend')
        self.args = args
        self.parser = FrameParser()
        self.correction = ImuCorrection(args.accel_calibration, tolerate_gaps=args.tolerant)
        self.readiness = CaptureReadiness(args.min_valid_seconds, args.min_clouds) if args.quality_guidance else None
        self.last_guidance = None
        self.point_count = self.dropped_scans = self.dropped_clouds = 0
        self.last_point_us = None
        self.last_warning = 0.
        self.imu_count = self.scan_count = 0
        self.last_scan_us = None
        self.last_receive = None
        self.bias_saved = False
        self.failure = None
        self.status_record = None
        self.directory = Path(args.output).resolve()
        self.directory.mkdir(parents=True, exist_ok=False)
        self.events_file = (self.directory/'quality_events.jsonl').open('w')
        self.serial_file = (self.directory/'serial_rx.bin').open('wb') if args.tolerant else None
        self.raw_file = (self.directory / 'imu_raw_and_corrected.csv').open('w', newline='')
        self.writer = csv.writer(self.raw_file)
        self.writer.writerow(['sequence', 'epoch_us', 'monotonic_us', 'hour_us', 'flags',
                              'ax', 'ay', 'az', 'gx', 'gy', 'gz',
                              'corrected_ax', 'corrected_ay', 'corrected_az',
                              'corrected_gx', 'corrected_gy', 'corrected_gz'])
        self.lidar_file = (self.directory / 'lidar_hardware_time.csv').open('w', newline='')
        self.lidar_writer = csv.writer(self.lidar_file)
        self.lidar_writer.writerow(['scan', 'first_epoch_us', 'last_epoch_us', 'packets'])
        self.imu_pub = self.create_publisher(Imu, '/calibration/imu', 2000)
        self.scan_pub = self.create_publisher(VelodyneScan, '/calibration/velodyne_packets', 20)
        sensor_input_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(VelodyneScan, args.packets_topic, self.scan_callback,
                                 sensor_input_qos)
        if args.points_input:
            self.points_pub = self.create_publisher(PointCloud2, '/calibration/velodyne_points', 20)
            self.create_subscription(PointCloud2, args.points_input, self.points_callback,
                                     sensor_input_qos)
        self.stream = serial.Serial(args.port, args.baud, timeout=0, exclusive=True)
        self.stream.reset_input_buffer()
        self.stream.write(b'@START\r\n@STATS\r\n')
        self.started = time.monotonic()
        self.create_timer(0.002, self.read_serial)
        self.create_timer(1.0, self.report)
        self.get_logger().info('前 10 秒请保持整套装置静止；正在使用 STM32 硬件时间估计角速度均值。')

    def read_serial(self):
        data = self.stream.read(min(self.stream.in_waiting, 32768))
        if self.serial_file:
            self.serial_file.write(data)
        previous_crc = self.parser.crc_errors
        for record in self.parser.feed(data):
            if record['kind'] == 'status':
                self.status_record = record['values']
                continue
            previous = self.correction.previous
            previous_missing = self.correction.missing_samples
            previous_firmware = self.correction.firmware_losses
            try:
                values = self.correction.accept(record)
            except ValueError as error:
                self.event('imu_fatal', reason=str(error), sequence=record['sequence'],
                    epoch_us=record['epoch_us'], previous_sequence=previous['sequence'] if previous else None,
                    previous_epoch_us=previous['epoch_us'] if previous else None)
                raise
            if self.correction.last_drop_reason:
                self.event('imu_rejected', reason=self.correction.last_drop_reason,
                           sequence=record['sequence'], epoch_us=record['epoch_us'])
                continue
            self.last_receive = time.monotonic()
            if self.correction.missing_samples != previous_missing or self.correction.firmware_losses != previous_firmware:
                self.event('imu_gap', previous_sequence=previous['sequence'],
                    sequence=record['sequence'], epoch_us=record['epoch_us'],
                    gap_us=record['epoch_us']-previous['epoch_us'],
                    missing=self.correction.missing_samples-previous_missing,
                    firmware_losses=self.correction.firmware_losses-previous_firmware)
            self.writer.writerow([record['sequence'], record['epoch_us'], record['monotonic_us'],
                                  record['hour_us'], record['flags'], *record['accel'], *record['gyro'],
                                  *(values[0] + values[1] if values else ('',) * 6)])
            if values is None:
                continue
            if not self.bias_saved:
                summary = self.correction.summary()
                summary['calibration_path'] = str(Path(self.args.accel_calibration).resolve())
                summary['calibration_sha256'] = hashlib.sha256(Path(self.args.accel_calibration).read_bytes()).hexdigest()
                (self.directory / 'applied_calibration.json').write_text(json.dumps(summary, indent=2))
                self.bias_saved = True
                self.get_logger().info(f'10 秒静止检查通过，角速度零偏 rad/s={self.correction.bias_gyro}。' +
                                       ('已开始发布标定输入，请再静止 5 秒后在平地驾驶，左右转弯并加减速。'
                                        if self.args.motion_mode == 'planar' else
                                        '已开始发布标定输入，请再静止 5 秒后做三轴运动。'))
            message = Imu()
            stamp(message.header.stamp, record['epoch_us'])
            message.header.frame_id = 'imu_link'
            message.orientation_covariance[0] = -1.0
            for axis, a, g in zip('xyz', *values):
                setattr(message.linear_acceleration, axis, a)
                setattr(message.angular_velocity, axis, g)
            self.imu_pub.publish(message)
            self.imu_count += 1
            if self.readiness:
                self.readiness.imu(record['epoch_us'], *values)
        if self.parser.crc_errors > previous_crc and self.args.tolerant:
            self.event('crc_rejected', count=self.parser.crc_errors-previous_crc,
                       total=self.parser.crc_errors)
        if (self.parser.startup_crc_errors is not None
                and self.parser.crc_errors > self.parser.startup_crc_errors and not self.args.tolerant):
            raise RuntimeError('串口 CRC 错误；本次标定停止，请检查连接后重启')
        if time.monotonic() - (self.last_receive or self.started) > 2:
            raise RuntimeError('连续 2 秒没有收到 IMU 硬件数据，请检查 v2 固件和串口')

    def scan_callback(self, raw_scan):
        # ROS may dispatch a queued LiDAR callback before the serial timer after
        # a scheduling pause. Drain pending hardware samples before deciding
        # that the IMU has stopped; the cached receive time alone is insufficient.
        self.read_serial()
        if not self.correction.ready:
            return
        if self.last_receive is None or time.monotonic() - self.last_receive > 0.5:
            raise RuntimeError('IMU 数据已过期，停止发布雷达标定数据')
        if not raw_scan.packets:
            if self.args.tolerant:
                self.dropped_scans += 1
                self.event('scan_rejected', reason='空雷达扫描')
                return
            raise RuntimeError('收到空雷达扫描')
        scan = copy.deepcopy(raw_scan)
        try:
            hours = [packet_hour_us(p.data) for p in scan.packets]
        except ValueError as error:
            if not self.args.tolerant: raise
            self.dropped_scans += 1
            self.event('scan_rejected', reason=str(error))
            return
        packet_times = [lidar_epoch_us(h, self.correction.last_epoch_us) for h in hours]
        if any(b <= a for a, b in zip(packet_times, packet_times[1:])):
            if self.args.tolerant:
                self.dropped_scans += 1
                self.event('scan_rejected', reason='扫描内雷达包重复或乱序')
                return
            raise RuntimeError('雷达包硬件时间未递增')
        if packet_times[-1] - packet_times[0] > 200_000:
            if self.args.tolerant:
                self.dropped_scans += 1
                self.event('scan_rejected', reason='单帧雷达时长异常')
                return
            raise RuntimeError('单帧雷达时长异常')
        if self.last_scan_us is not None and packet_times[0] <= self.last_scan_us:
            if self.args.tolerant and packet_times[0] == self.last_scan_us:
                self.dropped_scans += 1
                self.event('scan_rejected', reason='重复扫描')
                return
            raise RuntimeError('雷达硬件时间倒退；检查授时并重新启动')
        for packet, t in zip(scan.packets, packet_times):
            stamp(packet.stamp, t)
        stamp(scan.header.stamp, packet_times[0])
        self.last_scan_us = packet_times[0]
        self.scan_pub.publish(scan)
        self.scan_count += 1
        self.lidar_writer.writerow([self.scan_count, packet_times[0], packet_times[-1], len(packet_times)])

    def event(self, kind, **values):
        self.events_file.write(json.dumps(dict(kind=kind, **values), ensure_ascii=False)+'\n')
        if kind.endswith('_fatal'):
            self.events_file.flush()
            self.get_logger().error(f'严重采集故障：{values}')
            return
        now = time.monotonic()
        if now-self.last_warning >= 1:
            self.get_logger().warning(f'采集告警，继续记录有效数据：{kind} {values}；未补帧或改写时间。')
            self.last_warning = now

    def points_callback(self, message):
        self.read_serial()
        t = message.header.stamp.sec*1_000_000+message.header.stamp.nanosec//1000
        if self.last_point_us is not None and t < self.last_point_us:
            raise RuntimeError('解码点云硬件时间倒退，停止采集')
        try:
            if self.last_point_us == t:
                raise ValueError('重复点云')
            if self.correction.last_epoch_us is None or abs(t-self.correction.last_epoch_us) > 500_000:
                raise ValueError('点云相对最新有效 IMU 已过期')
            eigenvalues = validate_cloud(message)
        except ValueError as error:
            self.dropped_clouds += 1
            self.event('cloud_rejected', reason=str(error), epoch_us=t)
            return
        self.points_pub.publish(message)
        self.point_count += 1
        self.last_point_us = t
        if self.readiness:
            self.readiness.cloud(t, eigenvalues)

    def report(self):
        self.raw_file.flush()
        self.lidar_file.flush()
        self.events_file.flush()
        if self.serial_file: self.serial_file.flush()
        quality = dict(accepted_samples=self.correction.accepted_samples,
            missing_samples=self.correction.missing_samples,
            invalid_samples=self.correction.invalid_samples,
            duplicate_samples=self.correction.duplicate_samples,
            firmware_losses=self.correction.firmware_losses,
            crc_errors=max(0,self.parser.crc_errors-(self.parser.startup_crc_errors or 0)),
            largest_gap_us=self.correction.largest_gap_us)
        summary = dict(ready=self.correction.ready, imu_published=self.imu_count,
                       stationary_check=self.correction.stationary_metrics,
                       scans_published=self.scan_count, crc_errors=self.parser.crc_errors,
                       startup_crc_errors=self.parser.startup_crc_errors,
                       latest_imu_epoch_us=self.correction.last_epoch_us,
                       latest_lidar_epoch_us=self.last_scan_us, device_status=self.status_record,
                       failure=self.failure, tolerant=self.args.tolerant,
                       quality=quality, pointclouds_published=self.point_count,
                       scans_rejected=self.dropped_scans, pointclouds_rejected=self.dropped_clouds)
        if self.readiness:
            guidance = self.readiness.summary(quality)
            # Bad clouds/scans are filtered as whole frames; they also count in quality.
            rejection_ratio = max(self.dropped_clouds/max(1,self.point_count+self.dropped_clouds),
                self.dropped_scans/max(1,self.scan_count+self.dropped_scans))
            if rejection_ratio > .01:
                guidance['reasons'].append('雷达扫描/点云坏帧比例超过 1%')
                guidance['data_sufficient'] = False
            if self.failure:
                guidance['data_sufficient'] = False
                guidance['reasons'].append(self.failure)
            summary['capture_readiness'] = guidance
            (self.directory/'readiness.json').write_text(json.dumps(guidance, ensure_ascii=False, indent=2))
            sufficient = guidance['data_sufficient']
            if sufficient and self.last_guidance is not True:
                self.get_logger().info('【可以结束采集】数据量、输入质量和三轴运动建议条件已满足。'
                    '可按 Ctrl+C 保存，再运行离线标定；这不是标定精度验收。')
            elif not sufficient and self.last_guidance is True:
                self.get_logger().warning('采集建议条件不再满足，请继续补采：'+'；'.join(guidance['reasons']))
            elif self.correction.ready and self.imu_count and int(time.monotonic()-self.started)%5 == 0:
                self.get_logger().info(f'采集进度：有效 {guidance["valid_seconds"]:.0f}/{guidance["target_seconds"]:g} 秒，'
                    f'点云 {self.point_count}/{guidance["target_clouds"]}，'
                    f'三轴转动 {guidance["rotation_progress_percent"]}；'
                    +'；'.join(guidance['reasons']))
            self.last_guidance = sufficient
        (self.directory / 'status.json').write_text(json.dumps(summary, indent=2))
        if not self.correction.ready:
            elapsed = ((self.correction.last_epoch_us - self.correction.first_us) / 1e6
                       if self.correction.first_us is not None else 0)
            self.get_logger().info(f'静止采集 {elapsed:.1f}/10.0 秒')

    def close(self):
        self.report()
        self.raw_file.close()
        self.lidar_file.close()
        self.events_file.close()
        if self.serial_file: self.serial_file.close()
        self.stream.close()
        self.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--baud', type=int, default=460800)
    parser.add_argument('--packets-topic', default='/velodyne_packets')
    parser.add_argument('--accel-calibration', default=str(HERE / 'accel_calibration03.json'))
    parser.add_argument('--output', default=str(HERE.parent / 'runs' / datetime.now().strftime('%Y%m%d_%H%M%S')))
    parser.add_argument('--duration', type=float, default=0)
    parser.add_argument('--motion-mode', choices=['spatial', 'planar'], default='spatial')
    parser.add_argument('--tolerant', action='store_true', help='过滤孤立坏帧，允许不超过 50ms 的真实采样缺口')
    parser.add_argument('--quality-guidance', action='store_true')
    parser.add_argument('--points-input', default='', help='验证后再转发到 /calibration/velodyne_points')
    parser.add_argument('--min-valid-seconds', type=float, default=90.)
    parser.add_argument('--min-clouds', type=int, default=600)
    args = parser.parse_args()
    if args.quality_guidance and not args.points_input:
        parser.error('--quality-guidance 需要 --points-input 提供解码点云')
    rclpy.init(args=[])
    node = None
    result = 0
    try:
        node = HardwareFrontend(args)
        while rclpy.ok() and (not args.duration or time.monotonic() - node.started < args.duration):
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        if node:
            node.failure = str(error)
            node.get_logger().error(str(error))
        else:
            print(str(error), flush=True)
        result = 1
    finally:
        # ros2 launch may relay SIGINT after the terminal already delivered it.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
