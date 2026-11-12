"""Fail-fast data-stream watchdog for the one-click sensor/SLAM launch."""

import json
import os
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Imu, PointCloud2
from std_msgs.msg import String
import tf2_ros


def is_successful_semantic_heartbeat(payload):
    """Return true only for a completed, map-ready semantic frame."""
    try:
        return json.loads(payload).get('status') == 'ok'
    except (TypeError, ValueError, AttributeError):
        return False


class MappingHealthMonitor(Node):
    def __init__(self):
        super().__init__('one_click_mapping_health')
        self.declare_parameter('run_dir', '')
        self.declare_parameter('use_chassis_imu', True)
        self.declare_parameter('startup_timeout_s', 90.0)
        self.declare_parameter('stream_timeout_s', 10.0)
        self.run_dir = os.path.abspath(os.path.expanduser(
            self.get_parameter('run_dir').value))
        self.use_imu = self.get_parameter('use_chassis_imu').value
        self.startup_timeout = float(self.get_parameter('startup_timeout_s').value)
        self.stream_timeout = float(self.get_parameter('stream_timeout_s').value)
        if self.startup_timeout <= 0 or self.stream_timeout <= 0:
            raise ValueError('watchdog timeouts must be positive')

        self.started = time.monotonic()
        self.last_seen = {}
        self.counts = {}
        self.state = 'waiting'
        self.map_known_cells = 0
        self.map_dimensions = None
        self.last_semantic_status = None
        self.semantic_failures = 0
        self.last_waiting_write = self.started
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._subscribe('/velodyne_points', PointCloud2, 'pointcloud')
        self._subscribe('/camera/camera/color/camera_info', CameraInfo, 'camera_info')
        self._subscribe('/semantic/laptop/observations', String, 'semantic_observation')
        self._subscribe('/odom', Odometry, 'odom')
        if self.use_imu:
            self._subscribe('/imu_raw', Imu, 'chassis_imu_raw')
            self._subscribe('/imu/data', Imu, 'chassis_imu_filtered')

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.timer = self.create_timer(1.0, self._check_health)
        self._write_state('waiting')
        self.get_logger().info(
            f'Waiting up to {self.startup_timeout:.0f}s for live lidar, camera '
            'calibration, processed synchronized RGB-D, IMU, odometry, map and TF')

    def _subscribe(self, topic, message_type, key):
        callback = self._on_stream
        if key == 'semantic_observation':
            callback = self._on_semantic_observation
        self.create_subscription(
            message_type, topic,
            lambda message, key=key, callback=callback: callback(key, message),
            qos_profile_sensor_data,
            raw=(key == 'pointcloud'),
        )

    def _on_stream(self, key, _message):
        self.last_seen[key] = time.monotonic()
        self.counts[key] = self.counts.get(key, 0) + 1

    def _on_semantic_observation(self, key, message):
        try:
            status = json.loads(message.data).get('status')
        except (TypeError, ValueError, AttributeError):
            status = None
        self.last_semantic_status = status
        if is_successful_semantic_heartbeat(message.data):
            self._on_stream(key, message)
        else:
            self.semantic_failures += 1

    def _on_map(self, message):
        self.last_seen['map'] = time.monotonic()
        self.counts['map'] = self.counts.get('map', 0) + 1
        known = sum(1 for value in message.data if value >= 0)
        if known > self.map_known_cells:
            self.map_known_cells = known
            self.map_dimensions = [message.info.width, message.info.height]

    def _required_streams(self):
        streams = [
            'pointcloud', 'camera_info',
            'semantic_observation', 'odom',
        ]
        if self.use_imu:
            streams.extend(['chassis_imu_raw', 'chassis_imu_filtered'])
        return streams

    def _missing_streams(self, now):
        missing = []
        for stream in self._required_streams():
            last = self.last_seen.get(stream)
            if last is None:
                missing.append(f'{stream}:no_sample')
            elif now - last > self.stream_timeout:
                missing.append(f'{stream}:stale_{now-last:.1f}s')
        if self.map_known_cells <= 0:
            missing.append('map:no_known_cells')
        if not self.tf_buffer.can_transform(
                'map', 'camera_color_optical_frame', Time()):
            missing.append('tf:map_to_camera_color_optical_frame')
        return missing

    def _check_health(self):
        now = time.monotonic()
        missing = self._missing_streams(now)
        if self.state == 'waiting':
            if not missing:
                self.state = 'ready'
                self.ready_at_monotonic_s = now - self.started
                self._write_state('ready')
                self.get_logger().info(
                    'Live sensor/semantic/SLAM readiness confirmed: '
                    f'{self.map_known_cells} known occupancy cells; real map→camera TF available')
                return
            if now - self.started > self.startup_timeout:
                self.state = 'failed'
                self._write_state('failed', missing)
                self.get_logger().fatal(
                    'Sensor/SLAM readiness timeout; missing: ' + ', '.join(missing))
            elif now - self.last_waiting_write >= 5.0:
                self._write_state('waiting', missing)
                self.last_waiting_write = now
            return

        if self.state == 'ready' and missing:
            self.state = 'failed'
            self._write_state('failed', missing)
            self.get_logger().fatal(
                'A required live stream stopped; stopping the one-click stack: '
                + ', '.join(missing))

    def _write_state(self, state, missing=None):
        payload = {
            'state': state,
            'use_chassis_imu': self.use_imu,
            'elapsed_s': round(time.monotonic() - self.started, 3),
            'startup_timeout_s': self.startup_timeout,
            'stream_timeout_s': self.stream_timeout,
            'sample_counts': dict(self.counts),
            'semantic_last_status': self.last_semantic_status,
            'semantic_non_ok_observations': self.semantic_failures,
            'map_known_cells': self.map_known_cells,
            'map_dimensions': self.map_dimensions,
            'missing': missing or [],
        }
        path = os.path.join(self.run_dir, 'health.json')
        temporary_path = path + '.tmp'
        with open(temporary_path, 'w', encoding='utf-8') as stream:
            json.dump(payload, stream, indent=2)
            stream.write('\n')
        os.replace(temporary_path, path)


def main(args=None):
    rclpy.init(args=args)
    node = MappingHealthMonitor()
    try:
        # Keep shutdown on this main thread. Calling rclpy.shutdown() from a
        # timer callback can deadlock while the single-threaded executor waits
        # for that same callback to finish.
        while rclpy.ok() and node.state != 'failed':
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # rclpy can raise while a queued subscription callback is unwinding
        # after Ctrl+C has already shut down its context. Keep genuine runtime
        # failures visible while treating only that shutdown race as normal.
        if rclpy.ok():
            raise
    finally:
        failed = node.state == 'failed'
        if not failed:
            node._write_state('stopped')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
