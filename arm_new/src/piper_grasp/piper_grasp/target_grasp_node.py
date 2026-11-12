#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Vision-guided PiPER grasp planner.

Stages:
  1. wait for a stable `cell phone` detection in /vision/detections_3d_base
  2. object pose is already expressed in robot base frame (vision node)
  3. add the gripper length offset along the tool approach axis
  4. plan the arm group with MoveIt (plan_only=True)
  5. if planning succeeds and execute==true, execute the returned
     trajectory through /execute_trajectory

The arm group end-effector is `link6` (the gripper_base and fingers are
fixed to link6).  The PiPER two-finger TCP is 0.1358 m along link6 +Z,
so link6_target = tcp_target - tool_z * gripper_length.
"""

import math
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from sensor_msgs.msg import JointState
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
    WorkspaceParameters,
)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection3DArray

from piper_vision.calibration import matrix_to_quat


def _error_name(value):
    for name, item in vars(MoveItErrorCodes).items():
        if isinstance(item, int) and item == value:
            return name
    return str(value)


class TargetGraspNode(Node):

    def __init__(self):
        super().__init__('target_grasp_node')

        # target / stability
        self.detections_topic = self._p('detections_topic',
                                        '/vision/detections_3d_base')
        self.target_label = self._p('target_label', 'cell phone')
        self.min_score = float(self._p('min_score', 0.25))
        self.stable_frames = int(self._p('stable_frames', 6))
        self.stability_tolerance = float(
            self._p('stability_tolerance', 0.01))
        self.min_target_distance = float(
            self._p('min_target_distance', 0.10))
        self.max_target_distance = float(
            self._p('max_target_distance', 0.70))

        # frames / planning group
        self.base_frame = self._p('base_frame', 'base_link')
        self.arm_group = self._p('arm_group', 'arm')
        self.eef_link = self._p('eef_link', 'link6')

        # gripper geometry: link6/gripper_base -> TCP
        self.gripper_length = float(self._p('gripper_length', 0.1358))
        # extra TCP offset along tool +Z: 0 = object surface,
        # negative = above the object (safer pre-grasp), positive = deeper
        self.tcp_offset = float(self._p('tcp_offset', 0.0))
        self.grasp_yaw = float(self._p('grasp_yaw', 0.0))

        # planner / execution
        self.execute = bool(self._p('execute', False))
        self.position_tolerance = float(
            self._p('position_tolerance', 0.01))
        self.orientation_tolerance = float(
            self._p('orientation_tolerance', 0.35))
        self.planning_time = float(self._p('planning_time', 10.0))
        self.num_planning_attempts = int(
            self._p('num_planning_attempts', 10))
        self.velocity_scaling = float(
            self._p('velocity_scaling', 0.10))
        self.acceleration_scaling = float(
            self._p('acceleration_scaling', 0.10))
        # If true, refuse to execute unless the real piper driver is
        # publishing /joint_states_single (i.e. CAN feedback is alive).
        self.require_piper = bool(self._p('require_piper', False))
        self.workspace_min = np.array([-0.8, -0.8, -0.2])
        self.workspace_max = np.array([0.8, 0.8, 1.0])

        # state
        self._buffer = deque(maxlen=max(10, self.stable_frames * 2))
        self._last_target_time = 0.0
        self._state = 'WAIT_TARGET'
        self._finished = False
        self._object_base = None
        self._tcp_target = None
        self._link6_target = None
        self._target_rotation = None
        self._target_quat = None
        self._last_stage_log = 0.0
        self._last_piper_feedback = 0.0

        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.exec_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory')

        self.create_subscription(
            Detection3DArray, self.detections_topic,
            self.detections_callback, 10)
        self.create_subscription(
            JointState, '/joint_states_single',
            self._piper_feedback_callback, qos_profile_sensor_data)

        self.object_pub = self.create_publisher(
            PoseStamped, '/grasp/object_pose', 10)
        self.link6_pub = self.create_publisher(
            PoseStamped, '/grasp/link6_target', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/grasp/target_markers', 10)

        self.create_timer(0.5, self.tick)
        self.create_timer(1.0, self.publish_target_visuals)

        self.get_logger().info(
            f'[STAGE 0] grasp planner ready: target="{self.target_label}", '
            f'group={self.arm_group}, eef={self.eef_link}, '
            f'gripper_length={self.gripper_length:.4f} m, '
            f'tcp_offset={self.tcp_offset:+.3f} m, '
            f'execute={self.execute}, require_piper={self.require_piper}')
        self.get_logger().info(
            '[STAGE 0] waiting for /move_action and stable vision target...')

    def _p(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    # ------------------------------------------------------------------ #
    # vision
    # ------------------------------------------------------------------ #
    def _piper_feedback_callback(self, msg):
        self._last_piper_feedback = time.monotonic()

    def detections_callback(self, msg):
        candidates = []
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            if hyp.class_id != self.target_label:
                continue
            if hyp.score < self.min_score:
                continue
            p = det.bbox.center.position
            dist = math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
            if not (self.min_target_distance <= dist <=
                    self.max_target_distance):
                continue
            candidates.append((float(hyp.score), p))

        if candidates:
            score, p = max(candidates, key=lambda item: item[0])
            self._buffer.append((p.x, p.y, p.z, score))
            self._last_target_time = time.monotonic()
        elif time.monotonic() - self._last_target_time > 1.5:
            self._buffer.clear()

    def _stable_target(self):
        if len(self._buffer) < self.stable_frames:
            return None
        arr = np.array([[x, y, z] for x, y, z, _ in self._buffer])
        spread = arr.max(axis=0) - arr.min(axis=0)
        if float(np.max(spread)) > self.stability_tolerance:
            return None
        median = np.median(arr, axis=0)
        score = float(np.median([s for _, _, _, s in self._buffer]))
        return median, score

    # ------------------------------------------------------------------ #
    # main state machine
    # ------------------------------------------------------------------ #
    def tick(self):
        if self._finished or self._state != 'WAIT_TARGET':
            return

        target = self._stable_target()
        if target is None:
            now = time.monotonic()
            if now - self._last_stage_log > 5.0:
                self.get_logger().info(
                    f'[STAGE 1] waiting for stable "{self.target_label}" '
                    f'detections ({len(self._buffer)}/{self.stable_frames})')
                self._last_stage_log = now
            return

        if not self.move_client.server_is_ready():
            self.get_logger().info(
                '[STAGE 4] /move_action not ready; waiting...',
                throttle_duration_sec=5.0)
            return

        object_base, score = target
        self.get_logger().info(
            f'[STAGE 2] stable target: label="{self.target_label}", '
            f'score={score:.3f}, base_xyz=('
            f'{object_base[0]:+.4f}, {object_base[1]:+.4f}, '
            f'{object_base[2]:+.4f}) m')
        self._plan(object_base, score)

    def _build_pose(self, object_base):
        # Top-down grasp: tool +Z points down (-Z base).
        cy, sy = math.cos(self.grasp_yaw), math.sin(self.grasp_yaw)
        rz = np.array([
            [cy, -sy, 0.0],
            [sy, cy, 0.0],
            [0.0, 0.0, 1.0],
        ])
        rx_pi = np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
        ])
        rotation = rz @ rx_pi
        tool_z = rotation[:, 2]

        tcp_target = np.asarray(object_base) + tool_z * self.tcp_offset
        link6_target = tcp_target - tool_z * self.gripper_length
        quat = matrix_to_quat(rotation)
        return rotation, quat, tcp_target, link6_target

    def _plan(self, object_base, score):
        rotation, quat, tcp_target, link6_target = self._build_pose(
            object_base)

        self._object_base = np.asarray(object_base)
        self._target_rotation = rotation
        self._target_quat = quat
        self._tcp_target = tcp_target
        self._link6_target = link6_target

        self.get_logger().info(
            f'[STAGE 3] TCP target=({tcp_target[0]:+.4f}, '
            f'{tcp_target[1]:+.4f}, {tcp_target[2]:+.4f}) m; '
            f'link6 target=({link6_target[0]:+.4f}, '
            f'{link6_target[1]:+.4f}, {link6_target[2]:+.4f}) m; '
            f'quat=({quat[0]:+.3f},{quat[1]:+.3f},{quat[2]:+.3f},'
            f'{quat[3]:+.3f})')
        self.publish_target_visuals()

        pose = Pose()
        pose.position.x = float(link6_target[0])
        pose.position.y = float(link6_target[1])
        pose.position.z = float(link6_target[2])
        pose.orientation = Quaternion(
            x=float(quat[0]), y=float(quat[1]),
            z=float(quat[2]), w=float(quat[3]))

        pos = PositionConstraint()
        pos.header.frame_id = self.base_frame
        pos.link_name = self.eef_link
        pos.target_point_offset = Vector3(x=0.0, y=0.0, z=0.0)
        pos.constraint_region = BoundingVolume()
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [
            2.0 * self.position_tolerance,
            2.0 * self.position_tolerance,
            2.0 * self.position_tolerance,
        ]
        pos.constraint_region.primitives = [box]
        pos.constraint_region.primitive_poses = [pose]
        pos.weight = 1.0

        ori = OrientationConstraint()
        ori.header.frame_id = self.base_frame
        ori.link_name = self.eef_link
        ori.orientation = pose.orientation
        ori.absolute_x_axis_tolerance = self.orientation_tolerance
        ori.absolute_y_axis_tolerance = self.orientation_tolerance
        ori.absolute_z_axis_tolerance = self.orientation_tolerance
        ori.weight = 1.0

        constraints = Constraints()
        constraints.name = 'grasp_target'
        constraints.position_constraints = [pos]
        constraints.orientation_constraints = [ori]

        request = MotionPlanRequest()
        request.group_name = self.arm_group
        request.num_planning_attempts = self.num_planning_attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling
        request.workspace_parameters = WorkspaceParameters()
        request.workspace_parameters.header.frame_id = self.base_frame
        request.workspace_parameters.min_corner = Vector3(
            x=float(self.workspace_min[0]),
            y=float(self.workspace_min[1]),
            z=float(self.workspace_min[2]))
        request.workspace_parameters.max_corner = Vector3(
            x=float(self.workspace_max[0]),
            y=float(self.workspace_max[1]),
            z=float(self.workspace_max[2]))
        request.goal_constraints = [constraints]

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False

        self._state = 'PLANNING'
        self.get_logger().info(
            '[STAGE 4] sending MoveGroup plan request: '
            f'group={self.arm_group}, pos_tol={self.position_tolerance} m, '
            f'ori_tol={self.orientation_tolerance} rad, '
            f'planning_time={self.planning_time}s')
        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self._plan_goal_response)

    # ------------------------------------------------------------------ #
    # planning callbacks
    # ------------------------------------------------------------------ #
    def _plan_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'[STAGE 4] plan goal failed: {exc}')
            self._state = 'WAIT_TARGET'
            return
        if not goal_handle.accepted:
            self.get_logger().error('[STAGE 4] plan goal rejected')
            self._state = 'WAIT_TARGET'
            return
        self.get_logger().info('[STAGE 4] plan goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._plan_result)

    def _plan_result(self, future):
        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'[STAGE 5] plan result failed: {exc}')
            self._state = 'WAIT_TARGET'
            return

        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f'[STAGE 5] PLANNING FAILED: code={code} '
                f'({_error_name(code)})')
            self._state = 'WAIT_TARGET'
            return

        traj = result.planned_trajectory
        points = traj.joint_trajectory.points
        duration = 0.0
        if points:
            last = points[-1].time_from_start
            duration = last.sec + last.nanosec * 1e-9
        self.get_logger().info(
            f'[STAGE 5] PLANNING SUCCESS: points={len(points)}, '
            f'duration={duration:.2f}s, planning_time='
            f'{result.planning_time:.2f}s')
        self.get_logger().info(
            f'[STAGE 5] trajectory joints='
            f'{list(traj.joint_trajectory.joint_names)}')

        if not self.execute:
            self._state = 'DONE'
            self._finished = True
            self.get_logger().info(
                '[STAGE 6] plan_only=True: trajectory NOT executed. '
                'Re-run with execute:=true to execute it.')
            return

        self._execute_trajectory(traj)

    # ------------------------------------------------------------------ #
    # execution callbacks
    # ------------------------------------------------------------------ #
    def _execute_trajectory(self, trajectory):
        if self.require_piper:
            age = time.monotonic() - self._last_piper_feedback
            if age > 1.0:
                self.get_logger().error(
                    f'[STAGE 6] piper feedback timeout ({age:.1f}s): '
                    '/joint_states_single is not alive. Refusing to '
                    'execute on the real arm. Start piper driver and '
                    'check CAN.')
                self._state = 'WAIT_TARGET'
                return
        if not self.exec_client.server_is_ready():
            self.get_logger().error(
                '[STAGE 6] /execute_trajectory not ready')
            self._state = 'WAIT_TARGET'
            return
        self._state = 'EXECUTING'
        self.get_logger().info(
            '[STAGE 6] executing the planned trajectory...')
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        future = self.exec_client.send_goal_async(goal)
        future.add_done_callback(self._exec_goal_response)

    def _exec_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'[STAGE 6] execute goal failed: {exc}')
            self._state = 'WAIT_TARGET'
            return
        if not goal_handle.accepted:
            self.get_logger().error('[STAGE 6] execute goal rejected')
            self._state = 'WAIT_TARGET'
            return
        self.get_logger().info('[STAGE 6] execute goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._exec_result)

    def _exec_result(self, future):
        try:
            result = future.result().result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'[STAGE 7] execute result failed: {exc}')
            self._state = 'WAIT_TARGET'
            return
        code = result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(
                '[STAGE 7] EXECUTION SUCCESS: arm trajectory completed')
        else:
            self.get_logger().error(
                f'[STAGE 7] EXECUTION FAILED: code={code} '
                f'({_error_name(code)})')
        self._state = 'DONE'
        self._finished = True

    # ------------------------------------------------------------------ #
    # visualization
    # ------------------------------------------------------------------ #
    def publish_target_visuals(self):
        if self._object_base is None:
            return

        now = self.get_clock().now().to_msg()

        obj = PoseStamped()
        obj.header.stamp = now
        obj.header.frame_id = self.base_frame
        obj.pose.position = Point(
            x=float(self._object_base[0]),
            y=float(self._object_base[1]),
            z=float(self._object_base[2]))
        obj.pose.orientation.w = 1.0
        self.object_pub.publish(obj)

        link6 = PoseStamped()
        link6.header.stamp = now
        link6.header.frame_id = self.base_frame
        link6.pose.position = Point(
            x=float(self._link6_target[0]),
            y=float(self._link6_target[1]),
            z=float(self._link6_target[2]))
        link6.pose.orientation = Quaternion(
            x=float(self._target_quat[0]),
            y=float(self._target_quat[1]),
            z=float(self._target_quat[2]),
            w=float(self._target_quat[3]))
        self.link6_pub.publish(link6)

        markers = MarkerArray()
        delete = Marker()
        delete.header = Header(stamp=now, frame_id=self.base_frame)
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)

        p_obj = Marker()
        p_obj.header = Header(stamp=now, frame_id=self.base_frame)
        p_obj.ns = 'grasp_target'
        p_obj.id = 1
        p_obj.type = Marker.SPHERE
        p_obj.action = Marker.ADD
        p_obj.pose.position = Point(
            x=float(self._object_base[0]),
            y=float(self._object_base[1]),
            z=float(self._object_base[2]))
        p_obj.pose.orientation.w = 1.0
        p_obj.scale.x = p_obj.scale.y = p_obj.scale.z = 0.03
        p_obj.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.9)
        markers.markers.append(p_obj)

        p_link6 = Marker()
        p_link6.header = Header(stamp=now, frame_id=self.base_frame)
        p_link6.ns = 'grasp_target'
        p_link6.id = 2
        p_link6.type = Marker.SPHERE
        p_link6.action = Marker.ADD
        p_link6.pose.position = Point(
            x=float(self._link6_target[0]),
            y=float(self._link6_target[1]),
            z=float(self._link6_target[2]))
        p_link6.pose.orientation.w = 1.0
        p_link6.scale.x = p_link6.scale.y = p_link6.scale.z = 0.02
        p_link6.color = ColorRGBA(r=0.1, g=0.4, b=1.0, a=0.9)
        markers.markers.append(p_link6)

        arrow = Marker()
        arrow.header = Header(stamp=now, frame_id=self.base_frame)
        arrow.ns = 'grasp_target'
        arrow.id = 3
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.points = [
            Point(x=float(self._link6_target[0]),
                  y=float(self._link6_target[1]),
                  z=float(self._link6_target[2])),
            Point(x=float(self._tcp_target[0]),
                  y=float(self._tcp_target[1]),
                  z=float(self._tcp_target[2])),
        ]
        arrow.scale.x = 0.008
        arrow.scale.y = 0.016
        arrow.scale.z = 0.016
        arrow.color = ColorRGBA(r=0.1, g=1.0, b=0.1, a=0.9)
        markers.markers.append(arrow)

        # link6 target frame axes
        axis_colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9),
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9),
            ColorRGBA(r=0.0, g=0.2, b=1.0, a=0.9),
        ]
        for i in range(3):
            axis = Marker()
            axis.header = Header(stamp=now, frame_id=self.base_frame)
            axis.ns = 'grasp_target'
            axis.id = 10 + i
            axis.type = Marker.ARROW
            axis.action = Marker.ADD
            axis.points = [
                Point(x=float(self._link6_target[0]),
                      y=float(self._link6_target[1]),
                      z=float(self._link6_target[2])),
                Point(
                    x=float(self._link6_target[0] +
                            self._target_rotation[0, i] * 0.08),
                    y=float(self._link6_target[1] +
                            self._target_rotation[1, i] * 0.08),
                    z=float(self._link6_target[2] +
                            self._target_rotation[2, i] * 0.08)),
            ]
            axis.scale.x = 0.006
            axis.scale.y = 0.012
            axis.scale.z = 0.012
            axis.color = axis_colors[i]
            markers.markers.append(axis)

        text = Marker()
        text.header = Header(stamp=now, frame_id=self.base_frame)
        text.ns = 'grasp_target'
        text.id = 20
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position = Point(
            x=float(self._tcp_target[0]),
            y=float(self._tcp_target[1]),
            z=float(self._tcp_target[2]) + 0.06)
        text.pose.orientation.w = 1.0
        text.scale.z = 0.04
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.text = (
            f'{self.target_label}\n'
            f'tcp=({self._tcp_target[0]:.2f},'
            f'{self._tcp_target[1]:.2f},{self._tcp_target[2]:.2f})\n'
            f'link6=({self._link6_target[0]:.2f},'
            f'{self._link6_target[1]:.2f},{self._link6_target[2]:.2f})\n'
            f'gripper={self.gripper_length:.3f} m')
        markers.markers.append(text)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = TargetGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
