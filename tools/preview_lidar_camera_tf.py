#!/usr/bin/env python3
"""Preview the camera mount, or the known whole-vehicle TF chain, in RViz."""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


ROOT = Path(__file__).resolve().parents[1]


def mount_geometry(roll, pitch, yaw):
    """Return R_parent_from_child and quaternion xyzw for ROS fixed-axis RPY."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    hr, hp, hy = roll / 2, pitch / 2, yaw / 2
    c1, c2, c3 = math.cos(hr), math.cos(hp), math.cos(hy)
    s1, s2, s3 = math.sin(hr), math.sin(hp), math.sin(hy)
    quaternion = (s1*c2*c3-c1*s2*s3, c1*s2*c3+s1*c2*s3,
                  c1*c2*s3-s1*s2*c3, c1*c2*c3+s1*s2*s3)
    return rotation, quaternion


class MountPreview(Node):
    def __init__(self, args):
        super().__init__('lidar_camera_mount_preview')
        self.args = args
        self.origin = (args.x, args.y, args.z)
        angles = [math.radians(v) for v in (args.roll_deg, args.pitch_deg, args.yaw_deg)]
        self.rotation, self.quaternion = mount_geometry(*angles)
        self.forward = tuple(self.rotation[i][0] for i in range(3))
        self.broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'velodyne'
        transform.child_frame_id = 'camera_link'
        transform.transform.translation.x = args.x
        transform.transform.translation.y = args.y
        transform.transform.translation.z = args.z
        q = transform.transform.rotation
        q.x, q.y, q.z, q.w = self.quaternion
        transforms = [transform]
        if args.whole_vehicle:
            for parent, child, xyz, yaw in [
                ('base_link', 'imu_link', (0., 0., 0.), 0.),
                ('imu_link', 'velodyne', (args.lidar_x, args.lidar_y, args.lidar_z),
                 math.radians(args.lidar_yaw_deg)),
            ]:
                edge = TransformStamped()
                edge.header.stamp = transform.header.stamp
                edge.header.frame_id, edge.child_frame_id = parent, child
                edge.transform.translation.x, edge.transform.translation.y, edge.transform.translation.z = xyz
                _, quat = mount_geometry(0., 0., yaw)
                edge.transform.rotation.x, edge.transform.rotation.y, edge.transform.rotation.z, edge.transform.rotation.w = quat
                transforms.append(edge)
            self.get_logger().info('Whole-vehicle preview: base_link -> imu_link -> velodyne -> camera_link. '
                                   'base_link and imu_link coincide as configured in the real-robot launch.')
        self.broadcaster.sendTransform(transforms)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, '/lidar_camera_tf_preview/markers', qos)
        self.timer = self.create_timer(1.0, self.publish_scene)
        self.publish_scene()
        self.get_logger().info(
            f'PREVIEW TF velodyne -> camera_link: t={self.origin} m; '
            f'RPY=({args.roll_deg}, {args.pitch_deg}, {args.yaw_deg}) deg; '
            f'quaternion xyzw={self.quaternion}; forward={self.forward}')
        self.get_logger().info('Mount values are preliminary. Arrow lengths are visual aids. '
                               'No LiDAR, camera, SLAM, or motor driver is started. '
                               'Stop this preview before starting other TF publishers for the same child frames.')

    def publish_scene(self):
        if self.args.whole_vehicle:
            self.publish_vehicle_scene()
            return
        markers = []

        def marker(kind, color, frame='velodyne'):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'mount_preview'
            m.id = len(markers)
            m.type = kind
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = color
            markers.append(m)
            return m

        def point(v):
            return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))

        def line(points, color, width=0.002, kind=Marker.LINE_STRIP):
            m = marker(kind, color)
            m.scale.x = width
            m.points = [point(v) for v in points]
            return m

        def label(text, position, color=(0.95, 0.95, 0.95, 1.0), size=0.018):
            m = marker(Marker.TEXT_VIEW_FACING, color)
            m.pose.position = point(position)
            m.scale.z = size
            m.text = text

        def offset(vector, length=1.0):
            return tuple(self.origin[i] + length * vector[i] for i in range(3))

        cyan, yellow, purple = (0.1, 0.9, 1.0, 1.0), (1.0, 0.8, 0.1, 1.0), (0.95, 0.35, 1.0, 1.0)
        # Show the physical separation at its true scale, not an exploded view.
        for position, color in [((0.0, 0.0, 0.0), (1., 1., 1., 1.)), (self.origin, purple)]:
            m = marker(Marker.SPHERE, color)
            m.pose.position = point(position)
            m.scale.x = m.scale.y = m.scale.z = 0.012
        line([(0., 0., 0.), (self.args.x, 0., 0.),
              (self.args.x, self.args.y, 0.), self.origin], purple)
        label('velodyne', (-0.085, -0.035, 0.11))
        label('camera_link', (0.05, 0.14, 0.045), purple)
        label(f't = [{self.args.x:+.4f}, {self.args.y:+.4f}, {self.args.z:+.4f}] m',
              (-0.04, -0.13, 0.01), purple, 0.015)
        label('TF axes: X red / Y green / Z blue', (0.08, 0.0, 0.235), size=0.016)
        label(f'Preliminary mount: pitch {self.args.pitch_deg:+g} deg (down)',
              (0.10, 0.0, 0.205), size=0.016)

        ray_length = 0.4
        tip = offset(self.forward, ray_length)
        ray = line([offset(self.forward, 0.105), tip], cyan, 0.004, Marker.ARROW)
        ray.scale.y, ray.scale.z = 0.012, 0.023
        label('Lens forward = +X camera = +Z optical',
              (tip[0] + 0.045, tip[1], tip[2] - 0.035), cyan, 0.014)
        projection = (self.forward[0], self.forward[1], 0.0)
        projection_tip = offset(projection, ray_length)
        projected = line([self.origin, projection_tip], yellow, 0.0025, Marker.ARROW)
        projected.scale.y, projected.scale.z = 0.009, 0.018
        label('XY projection: LiDAR +X heading',
              (projection_tip[0] + 0.05, projection_tip[1], projection_tip[2] + 0.025), yellow, 0.014)
        # Dashed vertical line between the ray endpoint and its XY projection.
        dashes = []
        for i in range(0, 20, 2):
            for fraction in (i / 20, (i + 1) / 20):
                dashes.append(tuple(projection_tip[k] + fraction * (tip[k] - projection_tip[k])
                                    for k in range(3)))
        line(dashes, yellow, 0.0015, Marker.LINE_LIST)
        # Angle arc in the vertical plane of the camera's horizontal heading.
        yaw, pitch = math.radians(self.args.yaw_deg), math.radians(self.args.pitch_deg)
        arc = [offset((math.cos(pitch*i/30)*math.cos(yaw),
                       math.cos(pitch*i/30)*math.sin(yaw), -math.sin(pitch*i/30)), 0.15)
               for i in range(31)]
        line(arc, yellow)
        label(f'{self.args.pitch_deg:g} deg',
              offset((math.cos(pitch/2)*math.cos(yaw), math.cos(pitch/2)*math.sin(yaw),
                      -math.sin(pitch/2)), 0.19), yellow, 0.018)
        self.publisher.publish(MarkerArray(markers=markers))

    def publish_vehicle_scene(self):
        markers = []

        def point(v):
            return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))

        def marker(kind, frame, color):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id = 'vehicle_preview', len(markers)
            m.type, m.action = kind, Marker.ADD
            m.pose.orientation.w = 1.
            m.color.r, m.color.g, m.color.b, m.color.a = color
            markers.append(m)
            return m

        def label(text, position, color=(0.95, 0.95, 0.95, 1.), frame='base_link', size=.045):
            m = marker(Marker.TEXT_VIEW_FACING, frame, color)
            m.text, m.scale.z, m.pose.position = text, size, point(position)

        def line(points, color, frame='base_link', width=.008, kind=Marker.LINE_STRIP):
            m = marker(kind, frame, color)
            m.scale.x, m.points = width, [point(v) for v in points]
            return m

        def arrow(start, end, color, frame='base_link'):
            m = line([start, end], color, frame, .012, Marker.ARROW)
            m.scale.y, m.scale.z = .035, .07

        cyan = (.1, .9, 1., 1.)
        yellow = (1., .8, .1, 1.)
        purple = (.95, .35, 1., 1.)
        lidar = (self.args.lidar_x, self.args.lidar_y, self.args.lidar_z)
        yaw = math.radians(self.args.lidar_yaw_deg)
        cy, sy = math.cos(yaw), math.sin(yaw)

        def in_base(v):
            return (lidar[0] + cy*v[0] - sy*v[1],
                    lidar[1] + sy*v[0] + cy*v[1], lidar[2] + v[2])

        camera = in_base(self.origin)
        forward = (cy*self.forward[0]-sy*self.forward[1],
                   sy*self.forward[0]+cy*self.forward[1], self.forward[2])
        body = marker(Marker.CUBE, 'base_link', (.3, .45, .65, .42))
        body.pose.position.z = -.012
        body.scale.x, body.scale.y, body.scale.z = .62, .65, .02
        line([(.31, .325, 0.), (.31, -.325, 0.), (-.31, -.325, 0.),
              (-.31, .325, 0.), (.31, .325, 0.)], (.55, .7, .85, 1.))
        imu = marker(Marker.CUBE, 'imu_link', cyan)
        imu.scale.x = imu.scale.y = imu.scale.z = .055
        laser = marker(Marker.CYLINDER, 'velodyne', (.3, .7, 1., .55))
        laser.scale.x = laser.scale.y = .11
        laser.scale.z = .075
        cam = marker(Marker.CUBE, 'camera_link', (.9, .35, .95, .65))
        cam.scale.x, cam.scale.y, cam.scale.z = .035, .085, .028
        # Parent-to-child connection is a TF relation, not a physical support pole.
        line([(0., 0., 0.), lidar, camera], (.6, .65, .7, .7), width=.006)
        arrow((0., 0., .035), (.7, 0., .035), (1., .25, .2, 1.))
        label('Vehicle forward: +X base', (.48, -.10, .11), (1., .4, .3, 1.), size=.035)
        label('base_link = imu_link', (.04, .12, .18), cyan)
        label('Nav2 footprint: 0.62 x 0.65 m', (.04, -.18, -.10), size=.033)
        label('velodyne: yaw 180 deg', (-.67, .15, 1.22), (.5, .8, 1., 1.), size=.04)
        label(f'camera_link: down {self.args.pitch_deg:g} deg', (-.93, -.18, 1.12), purple, size=.04)
        label(f'camera in base: [{camera[0]:+.4f}, {camera[1]:+.4f}, {camera[2]:+.4f}] m',
              (-.82, -.20, .92), purple, size=.03)
        label('Known vehicle TF | axes: X red, Y green, Z blue',
              (-.25, 0., 1.62), size=.045)
        label('base_link -> imu_link -> velodyne -> camera_link',
              (-.25, 0., 1.52), size=.038)
        label('Sensor icons illustrative; grid at base Z=0, not a measured ground frame',
              (-.2, 0., 1.43), size=.025)
        tip = tuple(camera[i] + .85*forward[i] for i in range(3))
        arrow((.12, 0., 0.), (.85, 0., 0.), cyan, 'camera_link')
        label('Lens forward (+Z optical)', (tip[0], tip[1], tip[2]-.10), cyan, size=.035)
        projection = (tip[0], tip[1], camera[2])
        arrow(camera, projection, yellow)
        label('Horizontal heading: -X base / +X LiDAR',
              (projection[0]-.04, projection[1], projection[2]+.12), yellow, size=.03)
        dashes = []
        for i in range(0, 20, 2):
            for f in (i/20, (i+1)/20):
                dashes.append(tuple(projection[k]+f*(tip[k]-projection[k]) for k in range(3)))
        line(dashes, yellow, width=.004, kind=Marker.LINE_LIST)
        self.publisher.publish(MarkerArray(markers=markers))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, default=0.096769992)
    parser.add_argument('--y', type=float, default=0.017500000)
    parser.add_argument('--z', type=float, default=-0.064108820)
    parser.add_argument('--roll-deg', type=float, default=0.)
    parser.add_argument('--pitch-deg', type=float, default=50.09677973469077)
    parser.add_argument('--yaw-deg', type=float, default=0.)
    parser.add_argument('--whole-vehicle', action='store_true', help='Include base_link, imu_link, and the existing real-robot LiDAR TF')
    parser.add_argument('--lidar-x', type=float, default=-.0718)
    parser.add_argument('--lidar-y', type=float, default=.006)
    parser.add_argument('--lidar-z', type=float, default=1.0345)
    parser.add_argument('--lidar-yaw-deg', type=float, default=180.)
    parser.add_argument('--rviz', action='store_true', help='Open the prepared RViz scene; closing RViz also stops this preview')
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (args.x, args.y, args.z, args.roll_deg, args.pitch_deg, args.yaw_deg,
                                        args.lidar_x, args.lidar_y, args.lidar_z, args.lidar_yaw_deg)):
        parser.error('Transform values must be finite')
    rclpy.init(args=[])
    node = MountPreview(args)
    rviz = None
    try:
        if args.rviz:
            config_name = 'vehicle_tf_preview.rviz' if args.whole_vehicle else 'lidar_camera_tf_preview.rviz'
            rviz = subprocess.Popen(['rviz2', '-d', str(ROOT / 'src/velodyne_bringup/config' / config_name)])
        while rclpy.ok() and (rviz is None or rviz.poll() is None):
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        if rviz is not None and rviz.poll() is None:
            rviz.terminate()
            try:
                rviz.wait(timeout=5)
            except subprocess.TimeoutExpired:
                rviz.kill()
                rviz.wait()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
