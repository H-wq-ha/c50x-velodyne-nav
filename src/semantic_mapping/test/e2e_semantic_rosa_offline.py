#!/usr/bin/env python3
"""Chassis-off end-to-end check of the ROSA semantic navigation chain.

A fake environment (localization TF, static map, camera info, Nav2 lifecycle
services and action servers) is published on an isolated ROS domain, the real
installed ``semantic_navigator`` runs against it, and the ROSA DeepSeek agent
drives it through the versioned JSON protocol. No serial port, no real Nav2,
no chassis motion: nothing subscribes to act on /cmd_vel.

Run from the workspace root after building the package:

  source /opt/ros/humble/setup.bash
  source install/setup.bash
  /usr/bin/python3 src/semantic_mapping/test/e2e_semantic_rosa_offline.py

Phases:
  A  raw protocol: list_objects -> start -> status -> cancel (no LLM)
  B  real LLM navigation: run_semantic_agent.sh --enable-navigation --once
  C  real LLM read-only question must not start a goal

Evidence is written to output/e2e_semantic_rosa_<timestamp>/.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import signal
import sqlite3
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer
from rclpy.action.server import CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

WORKSPACE = Path(__file__).resolve().parents[3]
ROSA_RUN = WORKSPACE.parent / "ROSA" / "run_semantic_agent.sh"

TERMINAL_STATES = {"canceled", "succeeded", "failed", "rejected"}


def log(message: str) -> None:
    print(f"[e2e {time.strftime('%H:%M:%S')}] {message}", flush=True)


def mat_mul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def mat_vec(m, v):
    return tuple(sum(m[i][j] * v[j] for j in range(3)) for i in range(3))


def rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def ry(pitch):
    c, s = math.cos(pitch), math.sin(pitch)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def mat_to_quat(m):
    """Shepperd's method; only used for well-formed rotation matrices."""
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return x / norm, y / norm, z / norm, w / norm


def composed_base_to_camera_tf() -> TransformStamped:
    """base_link -> camera_color_optical_frame, composed from the launch chain.

    Mirrors semantic_navigation.launch.py: base_link->imu_link (yaw=pi),
    imu_link->velodyne (yaw=pi, t=(-0.0718, 0.006, 1.0345)) and the calibrated
    velodyne->camera_link transform, plus the standard optical-frame rotation.
    """
    r1 = rz(math.pi)
    r2 = rz(math.pi)
    r3 = ry(2.0 * math.atan2(0.4233835445, 0.9059505363))
    r_optical = ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0))
    # Explicit step-by-step composition (kept simple on purpose):
    t12 = mat_vec(r1, (-0.0718, 0.006, 1.0345))
    r12 = mat_mul(r1, r2)
    t_velodyne = mat_vec(r12, (0.096769992, 0.017500000, -0.064108820))
    t_cam = tuple(t_velodyne[i] + t12[i] for i in range(3))
    r_cam = mat_mul(mat_mul(r12, r3), r_optical)
    qx, qy, qz, qw = mat_to_quat(r_cam)

    tf = TransformStamped()
    tf.header.frame_id = "base_link"
    tf.child_frame_id = "camera_color_optical_frame"
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = t_cam
    tf.transform.rotation.x, tf.transform.rotation.y = qx, qy
    tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
    return tf


def build_static_map(width=480, height=400, resolution=0.05, origin=(-12.0, -10.0)):
    msg = OccupancyGrid()
    msg.info.resolution = resolution
    msg.info.width = width
    msg.info.height = height
    msg.info.origin.position.x, msg.info.origin.position.y = origin
    msg.info.origin.orientation.w = 1.0
    data = [0] * (width * height)
    border = 2
    for row in range(height):
        for col in range(width):
            if row < border or col < border or row >= height - border or col >= width - border:
                data[row * width + col] = 100
    msg.data = data
    return msg


def build_camera_info() -> CameraInfo:
    info = CameraInfo()
    info.width, info.height = 640, 480
    info.distortion_model = "plumb_bob"
    info.d = [0.0] * 5
    info.k = [604.0, 0.0, 321.8, 0.0, 604.0, 253.1, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [604.0, 0.0, 321.8, 0.0, 0.0, 604.0, 253.1, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


class FakeNavEnvironment:
    """Fake localization, map, camera and Nav2 servers; records everything."""

    def __init__(self):
        self.node = rclpy.create_node("e2e_fake_env")
        self.group = ReentrantCallbackGroup()

        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.static_broadcaster = StaticTransformBroadcaster(self.node)
        static = composed_base_to_camera_tf()
        static.header.stamp = self.node.get_clock().now().to_msg()
        self.static_broadcaster.sendTransform(static)
        log(f"static base_link->camera_color_optical_frame "
            f"t=({static.transform.translation.x:.4f}, {static.transform.translation.y:.4f}, "
            f"{static.transform.translation.z:.4f})")

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.node.create_publisher(OccupancyGrid, "/static_map", map_qos)
        self.map_msg = build_static_map()
        self.camera_pub = self.node.create_publisher(
            CameraInfo, "/camera/camera/color/camera_info", qos_profile_sensor_data)
        self.camera_info = build_camera_info()

        self.lifecycle_servers = []
        for name in ("planner_server", "controller_server", "bt_navigator", "velocity_smoother"):
            self.lifecycle_servers.append(self.node.create_service(
                GetState, f"/{name}/get_state", self._get_state_cb,
                callback_group=self.group))

        self.path_server = ActionServer(
            self.node, ComputePathToPose, "/compute_path_to_pose",
            execute_callback=self._plan_execute, cancel_callback=self._cancel_accept,
            callback_group=self.group)
        self.nav_server = ActionServer(
            self.node, NavigateToPose, "/navigate_to_pose",
            execute_callback=self._nav_execute, cancel_callback=self._cancel_accept,
            callback_group=self.group)

        self.nav_goals = []           # [{"goal": (x, y, yaw), "terminal": str|None}]
        self.nav_goal_lock = threading.Lock()
        self.nav_succeed_delay = 2.0  # goals after the first succeed after this delay

        self.requests_seen = []
        self.request_payloads = []
        self.response_payloads = {}
        self.status_events = []
        transient = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, "/semantic_nav/request", self._request_cb, 20)
        self.node.create_subscription(String, "/semantic_nav/response", self._response_tap_cb, 20)
        self.node.create_subscription(String, "/semantic_nav/task_status", self._status_cb, transient)

        self.stop_event = threading.Event()
        self.threads = [
            threading.Thread(target=self._publish_loop, args=(self.tf_pub_loop,), daemon=True),
            threading.Thread(target=self._publish_loop, args=(self.map_pub_loop,), daemon=True),
            threading.Thread(target=self._publish_loop, args=(self.camera_pub_loop,), daemon=True),
        ]
        for thread in self.threads:
            thread.start()

    # -- publishing loops ------------------------------------------------
    def _publish_loop(self, fn):
        while not self.stop_event.is_set() and rclpy.ok():
            try:
                fn()
            except Exception as exc:
                print(f"[fake env] publish error: {exc}", file=sys.stderr, flush=True)
            time.sleep(0.05)

    def tf_pub_loop(self):
        stamp = self.node.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "map"
        tf.child_frame_id = "base_link"
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf)

    def map_pub_loop(self):
        self.map_pub.publish(self.map_msg)

    def camera_pub_loop(self):
        info = build_camera_info()
        info.header.stamp = self.node.get_clock().now().to_msg()
        info.header.frame_id = "camera_color_optical_frame"
        self.camera_pub.publish(info)

    # -- lifecycle fake --------------------------------------------------
    def _get_state_cb(self, request, response):
        response.current_state = LifecycleState(id=3, label="active")
        return response

    # -- action servers --------------------------------------------------
    @staticmethod
    def _cancel_accept(goal_handle):
        return CancelResponse.ACCEPT

    def _plan_execute(self, goal_handle):
        goal = goal_handle.request.goal
        path = RosPath()
        path.header.frame_id = "map"
        path.header.stamp = self.node.get_clock().now().to_msg()
        start = PoseStamped()
        start.header.frame_id = "map"
        start.header.stamp = path.header.stamp
        path.poses = [start, goal]
        result = ComputePathToPose.Result()
        result.path = path
        goal_handle.succeed()
        return result

    def _nav_execute(self, goal_handle):
        pose = goal_handle.request.pose  # NavigateToPose_Goal.pose is a PoseStamped
        with self.nav_goal_lock:
            index = len(self.nav_goals)
            self.nav_goals.append({
                "x": pose.pose.position.x, "y": pose.pose.position.y,
                "yaw": math.atan2(2 * (pose.pose.orientation.w * pose.pose.orientation.z),
                                  1 - 2 * pose.pose.orientation.z ** 2),
                "terminal": None,
            })
        delay = 0.0 if index == 0 else self.nav_succeed_delay
        deadline = time.monotonic() + (120.0 if index == 0 else delay)
        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                with self.nav_goal_lock:
                    self.nav_goals[index]["terminal"] = "CANCELED"
                return NavigateToPose.Result()
            time.sleep(0.05)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            with self.nav_goal_lock:
                self.nav_goals[index]["terminal"] = "CANCELED"
            return NavigateToPose.Result()
        goal_handle.succeed()
        with self.nav_goal_lock:
            self.nav_goals[index]["terminal"] = "SUCCEEDED"
        return NavigateToPose.Result()

    # -- protocol taps ---------------------------------------------------
    def _request_cb(self, message):
        try:
            payload = json.loads(message.data)
        except ValueError:
            return
        if isinstance(payload, dict):
            self.requests_seen.append(payload.get("op"))
            self.request_payloads.append(payload)

    def _response_tap_cb(self, message):
        try:
            payload = json.loads(message.data)
        except ValueError:
            return
        if isinstance(payload, dict) and isinstance(payload.get("request_id"), str):
            self.response_payloads[payload["request_id"]] = payload

    def _status_cb(self, message):
        try:
            payload = json.loads(message.data)
        except ValueError:
            return
        if isinstance(payload, dict):
            self.status_events.append(payload)

    def start_executor(self):
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(self.node)
        threading.Thread(target=executor.spin, daemon=True).start()

    def shutdown(self):
        self.stop_event.set()


class RawProtocolClient:
    """Minimal independent protocol client used as a cross-check."""

    def __init__(self, node):
        self.node = node
        self.pub = node.create_publisher(String, "/semantic_nav/request", 20)
        self.lock = threading.Lock()
        self.responses = {}
        self.events = {}
        node.create_subscription(String, "/semantic_nav/response", self._response_cb, 20)

    def _response_cb(self, message):
        try:
            data = json.loads(message.data)
        except ValueError:
            return
        request_id = data.get("request_id") if isinstance(data, dict) else None
        if isinstance(request_id, str):
            with self.lock:
                self.responses[request_id] = data
                event = self.events.get(request_id)
            if event is not None:
                event.set()

    def request(self, payload, timeout=6.0):
        request_id = payload["request_id"]
        event = threading.Event()
        with self.lock:
            self.events[request_id] = event
        self.pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        event.wait(timeout)
        with self.lock:
            self.events.pop(request_id, None)
            return self.responses.pop(request_id, None)

    def list_objects(self, client_id):
        return self.request({
            "schema_version": 1, "request_id": f"req-{uuid.uuid4().hex[:12]}",
            "op": "list_objects", "client_id": client_id,
            "offset": 0, "limit": 100})

    def preview(self, client_id, map_id, session_id, class_label, object_id):
        return self.request({
            "schema_version": 1, "request_id": f"req-{uuid.uuid4().hex[:12]}",
            "op": "preview_navigation", "client_id": client_id,
            "map_id": map_id, "server_session_id": session_id,
            "class_label": class_label, "object_id": object_id}, timeout=20.0)

    def start(self, client_id, task_id, map_id, session_id, class_label, object_id):
        return self.request({
            "schema_version": 1, "request_id": f"req-{uuid.uuid4().hex[:12]}",
            "op": "start", "client_id": client_id, "task_id": task_id,
            "map_id": map_id, "server_session_id": session_id,
            "class_label": class_label, "object_id": object_id})

    def status(self, client_id, task_id, map_id, session_id):
        return self.request({
            "schema_version": 1, "request_id": f"req-{uuid.uuid4().hex[:12]}",
            "op": "status", "client_id": client_id, "task_id": task_id,
            "map_id": map_id, "server_session_id": session_id})

    def cancel(self, client_id, task_id, map_id, session_id):
        return self.request({
            "schema_version": 1, "request_id": f"req-{uuid.uuid4().hex[:12]}",
            "op": "cancel", "client_id": client_id, "task_id": task_id,
            "map_id": map_id, "server_session_id": session_id})


class Checks:
    def __init__(self):
        self.results = []

    def check(self, name, ok, detail=""):
        self.results.append({"name": name, "ok": bool(ok), "detail": str(detail)[:400]})
        log(f"{'PASS' if ok else 'FAIL'}  {name}" + (f"  [{detail}]" if detail and not ok else ""))
        return bool(ok)

    @property
    def passed(self):
        return all(item["ok"] for item in self.results)


def wait_for_navigator(node, timeout=40.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and rclpy.ok():
        if (node.count_publishers("/semantic_nav/response") > 0 and
                node.count_subscribers("/semantic_nav/request") > 0):
            return True
        time.sleep(0.2)
    return False


def wait_status_event(env, task_id, states, timeout=45.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for event in env.status_events:
            if event.get("task_id") == task_id and event.get("state") in states:
                return event
        time.sleep(0.1)
    return None


def run_cli(env, args, timeout, log_path):
    command = ["bash", str(ROSA_RUN), *args]
    log("运行: " + " ".join(command))
    started = time.monotonic()
    try:
        completed = subprocess.run(command, capture_output=True, text=True,
                                   timeout=timeout, cwd=str(ROSA_RUN.parent))
        rc, stdout, stderr = completed.returncode, completed.stdout, completed.stderr
    except subprocess.TimeoutExpired as exc:
        rc, stdout, stderr = None, exc.stdout or "", exc.stderr or ""
    elapsed = time.monotonic() - started
    log_path.write_text(
        f"# command: {' '.join(command)}\n# rc={rc} elapsed={elapsed:.1f}s\n"
        f"--- stdout ---\n{stdout}\n--- stderr ---\n{stderr}", encoding="utf-8")
    log(f"CLI rc={rc} elapsed={elapsed:.1f}s; 输出见 {log_path.name}")
    return rc, stdout, stderr, elapsed


def phase_a(env, client, checks, map_id):
    """list_objects -> read-only preview -> start -> status -> cancel."""
    client_id = f"e2e-raw-{uuid.uuid4().hex[:8]}"
    response = client.list_objects(client_id)
    if not checks.check("A1 list_objects ok", response and response.get("ok"),
                        json.dumps(response, ensure_ascii=False)[:300]):
        return None
    result = response.get("result") or {}
    objects = result.get("objects") or []
    checks.check("A2 map_id 与会话返回", result.get("map_id") == map_id and
                 bool(result.get("server_session_id")))
    checks.check("A3 对象清单包含已确认 bottle:1",
                 any(item.get("class_label") == "bottle" and
                     item.get("object_id") == 1 and item.get("count", 0) >= 3
                     for item in objects),
                 json.dumps(objects, ensure_ascii=False))
    checks.check("A3a 上下文含鲜定位/地图状态/显式分页",
                 (result.get("context", {}).get("robot", {}).get("fresh") is True and
                  result.get("context", {}).get("static_map", {}).get("received") is True and
                  result.get("pagination", {}).get("truncated") is False),
                 json.dumps(result.get("context"), ensure_ascii=False)[:300])
    timestamped = all("first_stamp_ns" in item and "last_stamp_ns" in item
                      for item in objects)
    relative = all(isinstance(item.get("relative"), dict) for item in objects)
    checks.check("A3b 对象含观测时间及relative数据状态",
                 timestamped and relative,
                 json.dumps(objects, ensure_ascii=False)[:300])
    session_id = result.get("server_session_id")

    with env.nav_goal_lock:
        goals_before_preview = len(env.nav_goals)
    preview = client.preview(client_id, map_id, session_id, "bottle", 1)
    checks.check("A3c preview纯读且获得ComputePath路径",
                 preview and preview.get("ok") and
                 preview.get("result", {}).get("status") == "path_found" and
                 preview.get("result", {}).get("reachable") is True and
                 preview.get("result", {}).get("path_length_m", 0) >= 0,
                 json.dumps(preview, ensure_ascii=False)[:500])
    with env.nav_goal_lock:
        goals_after_preview = len(env.nav_goals)
    checks.check("A3d preview未创建任务/未发送NavigateToPose",
                 goals_after_preview == goals_before_preview and
                 not any(event.get("task_id", "").startswith("preview")
                         for event in env.status_events),
                 f"goals={goals_before_preview}->{goals_after_preview}")

    task_id = f"e2e-task-{uuid.uuid4().hex[:8]}"
    start = client.start(client_id, task_id, map_id, session_id, "bottle", 1)
    checks.check("A4 start 受理且状态 planning",
                 start and start.get("ok") and
                 (start.get("result", {}).get("task", {}).get("state") == "planning"),
                 json.dumps(start, ensure_ascii=False)[:300])
    accepted_task = (start.get("result", {}).get("task") or {})
    checks.check("A5 任务身份一致",
                 accepted_task.get("task_id") == task_id and
                 accepted_task.get("client_id") == client_id and
                 accepted_task.get("map_id") == map_id and
                 accepted_task.get("server_session_id") == session_id)

    event = wait_status_event(env, task_id, {"navigating"}, timeout=40.0)
    checks.check("A6 状态推进到 navigating(goal_accepted)",
                 event is not None and event.get("reason_code") == "goal_accepted",
                 json.dumps(event, ensure_ascii=False)[:200] if event else "timeout")

    status = client.status(client_id, task_id, map_id, session_id)
    status_task = (status.get("result", {}) or {}).get("task", {}) if status else {}
    checks.check("A7 主动 status 查询为 navigating 且 checked_at 新鲜",
                 status and status.get("ok") and status_task.get("state") == "navigating" and
                 abs(time.time() - status_task.get("checked_at", 0)) < 5.0,
                 json.dumps(status, ensure_ascii=False)[:300])

    cancel = client.cancel(client_id, task_id, map_id, session_id)
    checks.check("A8 cancel 返回 canceling",
                 cancel and cancel.get("ok") and
                 (cancel.get("result", {}).get("task", {}).get("state") == "canceling"),
                 json.dumps(cancel, ensure_ascii=False)[:300])
    terminal = wait_status_event(env, task_id, TERMINAL_STATES, timeout=20.0)
    checks.check("A9 取消终态 canceled/nav2_canceled",
                 terminal is not None and terminal.get("state") == "canceled" and
                 terminal.get("reason_code") == "nav2_canceled",
                 json.dumps(terminal, ensure_ascii=False)[:200] if terminal else "timeout")
    with env.nav_goal_lock:
        goals = list(env.nav_goals)
    checks.check("A10 假Nav2第1个目标以CANCELED终态确认",
                 len(goals) == 1 and goals[0]["terminal"] == "CANCELED",
                 json.dumps(goals))
    return task_id


def phase_b(checks, env, report_dir, domain, map_id, expected_object_id):
    """Real DeepSeek LLM drives one navigation through the ROSA agent."""
    with env.nav_goal_lock:
        before = len(env.nav_goals)
    rc, stdout, stderr, elapsed = run_cli(
        env, ["--once", "请导航到离机器人最近的瓶子"], timeout=240.0,
        log_path=report_dir / "phase_b_cli.log")
    checks.check("B1 ROSA CLI 退出码 0", rc == 0, f"rc={rc} stderr={stderr[-200:]}")
    checks.check("B2 CLI 报告 Nav2 任务成功",
                 "Nav2报告任务成功" in stdout, stdout[-300:])
    deadline = time.monotonic() + 15.0
    succeeded_event = None
    while time.monotonic() < deadline and succeeded_event is None:
        events = [event for event in env.status_events
                  if event.get("state") == "succeeded" and
                  event.get("reason_code") == "nav2_succeeded"]
        succeeded_event = events[-1] if events else None
        time.sleep(0.2)
    checks.check("B3 结构化 succeeded(nav2_succeeded) 终态广播",
                 succeeded_event is not None,
                 json.dumps(env.status_events[-5:], ensure_ascii=False)[:400] if not succeeded_event else "")
    with env.nav_goal_lock:
        goals = list(env.nav_goals)
    checks.check("B4 假Nav2第2个目标以SUCCEEDED终态确认",
                 len(goals) == before + 1 and goals[-1]["terminal"] == "SUCCEEDED",
                 json.dumps(goals))
    start_requests = [payload for payload in env.request_payloads if payload.get("op") == "start"]
    checks.check("B4a LLM依据relative最近关系选择合成瓶子",
                 bool(start_requests) and start_requests[-1].get("object_id") == expected_object_id,
                 json.dumps(start_requests[-1] if start_requests else None))
    if succeeded_event is not None:
        checks.check("B5 终态身份与地图一致",
                     succeeded_event.get("map_id") == map_id and
                     bool(succeeded_event.get("task_id")) and
                     isinstance(succeeded_event.get("seq"), int),
                     json.dumps(succeeded_event, ensure_ascii=False)[:300])
    checks.check("B6 LLM导航耗时合理(<240s)", elapsed < 240.0, f"{elapsed:.1f}s")


def phase_c(checks, env, report_dir, domain, expected_synthetic_id):
    """Three read-only LLM questions must never start a navigation goal."""
    with env.nav_goal_lock:
        before_goals = len(env.nav_goals)
    queries = [
        ("nearest", "最近的瓶子在哪里？请给出清单里的目标编号和相对距离。"),
        ("robot", "机器人现在在地图什么位置、朝向哪里？"),
        ("approach", "能否靠近最近的瓶子？请先只做只读预检。"),
    ]
    outputs = []
    query_traces = []
    for label, query in queries:
        # Capture request-list offsets, not the number of start operations:
        # previews and list/context reads also occupy entries in this trace.
        request_start = len(env.request_payloads)
        rc, stdout, stderr, _ = run_cli(
            env, ["--once", query], timeout=180.0,
            log_path=report_dir / f"phase_c_{label}.log")
        request_end = len(env.request_payloads)
        trace = env.request_payloads[request_start:request_end]
        query_traces.append({
            "label": label, "query": query, "rc": rc,
            "output": stdout,
            "requests": trace,
        })
        outputs.append(stdout)
        checks.check(f"C1-{label} 只读问题CLI退出码0", rc == 0,
                     f"rc={rc} stderr={stderr[-200:]}")
    c_trace = [request for item in query_traces for request in item["requests"]]
    starts_after = sum(1 for request in c_trace if request.get("op") == "start")
    with env.nav_goal_lock:
        goals_after = len(env.nav_goals)
    checks.check("C2 只读问题未触发任何 start 请求", starts_after == 0,
                 f"phase-C start count={starts_after}")
    checks.check("C3 只读问题未产生新的导航目标", goals_after == before_goals,
                 f"before={before_goals} after={goals_after}")
    nearest_objects = []
    for item in query_traces[0]["requests"]:
        if item.get("op") == "list_objects":
            response = env.response_payloads.get(item.get("request_id"))
            result = (response or {}).get("result") or {}
            nearest_objects.extend(result.get("objects") or [])
    nearest_bottles = [item for item in nearest_objects
                       if item.get("class_label") == "bottle" and
                       isinstance(item.get("relative"), dict) and
                       item["relative"].get("available") is True and
                       isinstance(item["relative"].get("distance_m"), (int, float))]
    nearest_bottle = min(nearest_bottles,
                         key=lambda item: item["relative"]["distance_m"],
                         default=None)
    checks.check("C4 最近问题的响应与synthetic最近瓶子ID一致",
                 nearest_bottle is not None and
                 nearest_bottle.get("object_id") == expected_synthetic_id and
                 str(expected_synthetic_id) in outputs[0] and
                 ("瓶子" in outputs[0] or "bottle" in outputs[0].lower()),
                 json.dumps({"nearest_from_relative": nearest_bottle,
                             "output": outputs[0][-500:]}, ensure_ascii=False))
    checks.check("C5 机器人位置问题包含位置信息",
                 any(token in outputs[1] for token in ("map", "地图", "米", "位置")),
                 outputs[1][-300:])
    approach_requests = query_traces[2]["requests"]
    preview_requests = [request for request in approach_requests
                        if request.get("op") == "preview_navigation"]
    checks.check("C6 只读靠近问题使用preview且不start",
                 bool(preview_requests) and
                 not any(request.get("op") == "start" for request in approach_requests),
                 json.dumps(approach_requests, ensure_ascii=False))
    checks.check("C7 preview使用清单中的synthetic最近瓶子ID",
                 bool(preview_requests) and
                 preview_requests[-1].get("object_id") == expected_synthetic_id,
                 json.dumps(preview_requests[-1] if preview_requests else None))
    (report_dir / "phase_c_trace.json").write_text(
        json.dumps(query_traces, ensure_ascii=False, indent=2), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, default=78,
                        help="隔离的 ROS_DOMAIN_ID，默认 78")
    parser.add_argument("--maps-root", default=str(WORKSPACE / "maps"))
    parser.add_argument("--report-root", default=str(WORKSPACE / "output"))
    parser.add_argument("--skip-llm", action="store_true",
                        help="只跑阶段A协议验证，不调用真实LLM")
    args = parser.parse_args()

    maps_root = Path(args.maps_root).expanduser()
    bundle = maps_root / "latest"
    identity = json.loads((bundle / "run_identity.json").read_text(encoding="utf-8"))
    map_id = identity["map_id"]

    report_dir = Path(args.report_root) / f"e2e_semantic_rosa_{time.strftime('%Y%m%d_%H%M%S')}"
    report_dir.mkdir(parents=True, exist_ok=True)
    run_dir = Path(tempfile.mkdtemp(prefix="e2e_nav_"))
    semantic_copy = run_dir / "semantic.sqlite3"
    shutil.copy2(bundle / "semantic.sqlite3", semantic_copy)
    with sqlite3.connect(semantic_copy) as db:
        meta = dict(db.execute("SELECT key,value FROM metadata"))
        rows = db.execute("SELECT object_id,class_id,x,y,z,count FROM objects").fetchall()
        synthetic_object_id = max(row[0] for row in rows) + 1
        first_stamp = max(row[-1] for row in db.execute(
            "SELECT last_stamp_ns FROM objects").fetchall()) + 1
        # Synthetic-only test fixture: this close bottle makes relative-distance
        # reasoning measurable without changing the real map bundle.
        db.execute("INSERT INTO objects(object_id,class_id,x,y,z,count,first_stamp_ns,last_stamp_ns) "
                   "VALUES(?,39,0.01,0.0,0.5,5,?,?)",
                   (synthetic_object_id, first_stamp, first_stamp))
        rows = db.execute("SELECT object_id,class_id,x,y,z,count FROM objects").fetchall()
    os.chmod(semantic_copy, 0o444)
    assert meta.get("map_id") == map_id and rows, "semantic DB copy is invalid"
    log(f"map_id={map_id} objects={rows}; synthetic nearest bottle id={synthetic_object_id}")

    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init()
    checks = Checks()
    report = {
        "map_id": map_id, "domain": args.domain, "semantic_copy": str(semantic_copy),
        "objects": rows, "synthetic_nearest_bottle_id": synthetic_object_id,
        "phases": {}, "started_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }

    env = FakeNavEnvironment()
    client = RawProtocolClient(env.node)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(env.node)
    threading.Thread(target=executor.spin, daemon=True).start()

    navigator_cmd = [
        str(WORKSPACE / "install/semantic_mapping/lib/semantic_mapping/semantic_navigator"),
        "--ros-args", "-p", f"semantic_db:={semantic_copy}",
        "-p", f"map_id:={map_id}",
        "-p", "enable_chassis_control:=true",
        "-p", "navigation_timeout_s:=60.0",
        "-p", "standoff_min:=0.9", "-p", "standoff_max:=2.5",
        "-p", "camera_margin:=0.10",
    ]
    navigator_log = open(report_dir / "navigator.log", "w", encoding="utf-8")
    navigator = subprocess.Popen(navigator_cmd, stdout=navigator_log,
                                 stderr=subprocess.STDOUT,
                                 env={**os.environ})
    log(f"navigator pid={navigator.pid}")

    try:
        ready = wait_for_navigator(env.node)
        checks.check("E1 navigator 启动并可发现", ready and navigator.poll() is None,
                     f"ready={ready} returncode={navigator.poll()}")
        if not ready:
            raise SystemExit(1)
        time.sleep(1.0)

        log("=== 阶段A：裸协议 查询/启动/状态/取消（无LLM） ===")
        task_id = phase_a(env, client, checks, map_id)
        report["phases"]["A"] = {"task_id": task_id}

        if not args.skip_llm:
            log("=== 阶段B：真实LLM 自然语言导航 ===")
            phase_b(checks, env, report_dir, args.domain, map_id, synthetic_object_id)
            log("=== 阶段C：真实LLM 只读问题不得触发导航 ===")
            phase_c(checks, env, report_dir, args.domain, synthetic_object_id)

        with env.nav_goal_lock:
            report["nav_goals"] = list(env.nav_goals)
        report["request_ops"] = env.requests_seen
        report["status_events"] = env.status_events
        report["cmd_vel_publishers_at_end"] = env.node.count_publishers("/cmd_vel")
        checks.check("E2 全程无 /cmd_vel 发布者（底盘关机且不可被驱动）",
                     report["cmd_vel_publishers_at_end"] == 0,
                     str(report["cmd_vel_publishers_at_end"]))
    finally:
        if navigator.poll() is None:
            navigator.send_signal(signal.SIGINT)
            try:
                navigator.wait(timeout=15)
            except subprocess.TimeoutExpired:
                navigator.kill()
        env.shutdown()
        navigator_log.close()
        report["navigator_returncode"] = navigator.returncode
        report["finished_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        report["checks"] = checks.results
        report["all_passed"] = checks.passed
        (report_dir / "report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
        shutil.rmtree(run_dir, ignore_errors=True)
        rclpy.shutdown()

    log(f"结果: {'全部通过' if checks.passed else '存在失败'}；证据目录 {report_dir}")
    return 0 if checks.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
