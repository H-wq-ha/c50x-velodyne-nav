#!/usr/bin/env python3
"""Isolated ROS 2 integration smoke test for the semantic map node.

Run from the workspace root after building the package and capturing an RGB-D
reference frame with the camera. All reference paths are explicit command-line
arguments so this smoke test does not depend on private /tmp files.

  source /opt/ros/humble/setup.bash
  source install/setup.bash
  export ROS_DOMAIN_ID=71 ROS_LOCALHOST_ONLY=1
  export PYTHONPATH="$PWD/yolov8_d435i/.venv/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
  /usr/bin/python3.10 src/semantic_mapping/test/integration_smoke.py \
    --ref-image output/.../reference_color.png \
    --ref-depth output/.../reference_depth.npy \
    --ref-json output/.../reference.json --expected-class-id 39 --confidence 0.70

The test uses a captured RGB-D reference frame, stamps its ROS messages with
the current ROS clock, and runs the installed node as a separate ROS process.
All test publishers and TF broadcasters are restricted to ROS domain 71.
"""
from __future__ import annotations

import json
import argparse
import os
from pathlib import Path
import shlex
import sqlite3
import subprocess
import sys
import tempfile
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
import tf2_ros


ROOT = Path(__file__).resolve().parents[3]
PKG = ROOT / "src/semantic_mapping"
MODEL = ROOT / "yolov8_d435i/models/yolov8n_256.onnx"
DOMAIN = "71"
COLOR_TOPIC = "/test/laptop/color"
DEPTH_TOPIC = "/test/laptop/depth"
INFO_TOPIC = "/test/laptop/camera_info"
FRAME = "camera_color_optical_frame"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def db_snapshot(path: Path) -> tuple[list[tuple], list[tuple]]:
    with sqlite3.connect(path, timeout=2.0) as db:
        observations = db.execute(
            "SELECT stamp_ns, class_id, x, y, z, payload FROM observations ORDER BY id"
        ).fetchall()
        objects = db.execute(
            "SELECT object_id, class_id, x, y, z, count FROM objects ORDER BY object_id"
        ).fetchall()
    return observations, objects


class Smoke:
    def __init__(self, db_path: Path, log_file, reference_paths, expected_class_id,
                 confidence_threshold):
        self.db_path = db_path
        self.log_file = log_file
        self.process = None
        self.ref_image_path, self.ref_depth_path, self.ref_json_path = reference_paths
        self.expected_class_id = expected_class_id
        self.confidence_threshold = confidence_threshold
        self.node = rclpy.create_node("semantic_mapping_integration_smoke")
        self.bridge = CvBridge()
        self.reference = json.loads(self.ref_json_path.read_text())
        self.image = cv2.imread(str(self.ref_image_path), cv2.IMREAD_COLOR)
        self.depth = np.load(self.ref_depth_path)
        require(self.image is not None, f"could not read {self.ref_image_path}")
        require(self.depth.shape == self.image.shape[:2], "captured color/depth dimensions differ")
        self.height, self.width = self.image.shape[:2]
        # Exercise the same best-effort profile as the RealSense RGB-D topics.
        self.input_qos = qos_profile_sensor_data
        self.color_pub = self.node.create_publisher(Image, COLOR_TOPIC, self.input_qos)
        self.depth_pub = self.node.create_publisher(Image, DEPTH_TOPIC, self.input_qos)
        self.info_pub = self.node.create_publisher(CameraInfo, INFO_TOPIC, self.input_qos)
        self.input_seen = {COLOR_TOPIC: 0, DEPTH_TOPIC: 0, INFO_TOPIC: 0}
        self.node.create_subscription(Image, COLOR_TOPIC,
            lambda _msg: self._count_input(COLOR_TOPIC), self.input_qos)
        self.node.create_subscription(Image, DEPTH_TOPIC,
            lambda _msg: self._count_input(DEPTH_TOPIC), self.input_qos)
        self.node.create_subscription(CameraInfo, INFO_TOPIC,
            lambda _msg: self._count_input(INFO_TOPIC), self.input_qos)
        self.tf_pub = tf2_ros.TransformBroadcaster(self.node)
        self.static_tf_pub = tf2_ros.StaticTransformBroadcaster(self.node)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node, spin_thread=False)
        self.observations: list[dict] = []
        # Match the SUT's transient-local map publisher so late discovery retains the current map.
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, "/semantic/laptop/observations",
                                      self._on_observation, 10)
        self.node.create_subscription(String, "/semantic/laptop/objects",
                                      lambda _msg: None, map_qos)
        self.last_stamp = 0
        self._publish_static_camera_transform()

    def _count_input(self, topic: str) -> None:
        self.input_seen[topic] += 1

    def _on_observation(self, msg: String) -> None:
        try:
            self.observations.append(json.loads(msg.data))
        except (TypeError, json.JSONDecodeError):
            pass

    def start_node(self) -> None:
        args = [
            "ros2", "run", "semantic_mapping", "semantic_mapping",
            "--ros-args",
        ]
        params = {
            "color_topic": COLOR_TOPIC,
            "depth_topic": DEPTH_TOPIC,
            "camera_info_topic": INFO_TOPIC,
            "model_path": str(MODEL),
            "target_class_ids": [self.expected_class_id],
            "map_frame": "map",
            "odom_frame": "odom",
            "map_id": "integration_smoke",
            "db_path": str(self.db_path),
            "inference_hz": 10.0,
            "confidence_threshold": self.confidence_threshold,
            "depth_window_px": 5,
            "min_valid_depth_samples": 5,
            "depth_min_valid_ratio": 0.4,
            "confirmations": 3,
            "association_gate_m": 0.35,
            "sync_slop_s": 0.02,
            "max_frame_age_s": 0.75,
            "ort_threads": 1,
            "map_jump_translation_m": 0.75,
            "map_jump_rotation_deg": 25.0,
        }
        for key, value in params.items():
            args.extend(["-p", f"{key}:={value}"])
        sourced = (
            "source /opt/ros/humble/setup.bash && "
            f"source {shlex.quote(str(ROOT / 'install/setup.bash'))} && "
            f"export ROS_DOMAIN_ID={DOMAIN} ROS_LOCALHOST_ONLY=1 && "
            f"export PYTHONPATH={shlex.quote(str(ROOT / 'yolov8_d435i/.venv/lib/python3.10/site-packages'))}:\"${{PYTHONPATH:-}}\" && "
            "exec " + shlex.join(args)
        )
        env = os.environ.copy()
        env.update(ROS_DOMAIN_ID=DOMAIN, ROS_LOCALHOST_ONLY="1", ROS2CLI_DISABLE_DAEMON="1")
        self.process = subprocess.Popen(
            ["bash", "-lc", sourced], cwd=ROOT, env=env,
            stdout=self.log_file, stderr=subprocess.STDOUT,
        )
        self.spin_until(
            lambda: (self.node.count_publishers("/semantic/laptop/observations") > 0 and
                     self.node.count_subscribers(COLOR_TOPIC) >= 2 and
                     self.node.count_subscribers(DEPTH_TOPIC) >= 2 and
                     self.node.count_subscribers(INFO_TOPIC) >= 2),
            35.0, "semantic node publishers/subscribers did not become discoverable",
        )
        for topic in (COLOR_TOPIC, DEPTH_TOPIC, INFO_TOPIC):
            endpoints = self.node.get_subscriptions_info_by_topic(topic)
            print(f"ROS endpoint {topic}: " + ", ".join(
                f"{ep.node_name} reliability={ep.qos_profile.reliability}" for ep in endpoints))

    def _publish_static_camera_transform(self) -> None:
        msg = TransformStamped()
        msg.header.frame_id = "odom"
        msg.child_frame_id = FRAME
        msg.transform.rotation.w = 1.0
        self.static_tf_pub.sendTransform(msg)

    def _publish_map_odom(self, stamp_ns: int, x: float = 1.0, y: float = 2.0,
                          z: float = 0.0) -> None:
        msg = TransformStamped()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        msg.header.frame_id = "map"
        msg.child_frame_id = "odom"
        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.translation.z = z
        msg.transform.rotation.w = 1.0
        self.tf_pub.sendTransform(msg)

    def _new_stamp(self) -> int:
        now = self.node.get_clock().now().nanoseconds
        self.last_stamp = max(now, self.last_stamp + 10_000_000)
        return self.last_stamp

    def publish_frame(self, *, stamp_ns: int | None = None, depth=None, image=None,
                      map_tf: bool = True, map_x: float = 1.0) -> int:
        stamp_ns = self._new_stamp() if stamp_ns is None else stamp_ns
        seen_before = self.input_seen.copy()
        if map_tf:
            self._publish_map_odom(stamp_ns, map_x, 2.0, 0.0)
            # Confirm the synthetic transform is on the ROS graph at the exact image stamp.
            # The SUT is already discovered and receives the same TF topic independently.
            t = Time(nanoseconds=stamp_ns)
            self.spin_until(
                lambda: (self.tf_buffer.can_transform("map", "odom", t) and
                         self.tf_buffer.can_transform("map", FRAME, t)),
                2.0, f"synthetic TF did not become available at stamp {stamp_ns}",
            )
            self.spin_for(0.12)
        color = self.bridge.cv2_to_imgmsg(self.image if image is None else image, encoding="bgr8")
        color.header.frame_id = FRAME
        color.header.stamp.sec, color.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        depth_array = self.depth if depth is None else depth
        depth_msg = self.bridge.cv2_to_imgmsg(depth_array, encoding="passthrough")
        depth_msg.header = color.header
        info = CameraInfo()
        info.header = color.header
        info.width, info.height = self.width, self.height
        info.distortion_model = "plumb_bob"
        info.k = self.reference["K"]
        info.d = self.reference["D"]
        # Repeat the same capture stamp to make the RGB-D graph robust to transient
        # best-effort packet loss. The SUT must count this stamp once at most.
        for _ in range(3):
            self.info_pub.publish(info)
            self.color_pub.publish(color)
            self.depth_pub.publish(depth_msg)
            self.spin_for(0.03)
        self.spin_until(
            lambda: all(self.input_seen[t] > seen_before[t]
                        for t in (COLOR_TOPIC, DEPTH_TOPIC, INFO_TOPIC)),
            2.0, f"test frame did not reach local ROS subscriptions: before={seen_before}, after={self.input_seen}",
        )
        return stamp_ns

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=min(0.03, max(0.0, deadline-time.monotonic())))

    def spin_until(self, condition, timeout: float, message: str) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.process is not None and self.process.poll() is not None:
                self.log_file.flush()
                raise RuntimeError(f"semantic node exited ({self.process.returncode}); see {self.log_file.name}")
            if condition():
                return
            rclpy.spin_once(self.node, timeout_sec=0.04)
        raise TimeoutError(message)

    def wait_observation(self, stamp_ns: int, timeout: float = 6.0) -> dict:
        self.spin_until(lambda: any(x.get("stamp_ns") == stamp_ns and
                                    x.get("status") != "tf_pending"
                                    for x in self.observations),
                        timeout, f"no node observation for stamp {stamp_ns}")
        return next(x for x in reversed(self.observations)
                    if x.get("stamp_ns") == stamp_ns and x.get("status") != "tf_pending")

    def wait_db_observations(self, n: int, timeout: float = 6.0) -> tuple[list[tuple], list[tuple]]:
        snapshot = ([], [])
        def ready():
            nonlocal snapshot
            try:
                snapshot = db_snapshot(self.db_path)
            except sqlite3.OperationalError:
                return False
            return len(snapshot[0]) == n
        self.spin_until(ready, timeout, f"database did not reach {n} unique observations")
        return snapshot

    def close(self) -> None:
        if self.process is not None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill(); self.process.wait(timeout=2)
        self.node.destroy_node()


def run() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ref-image", type=Path, required=True)
    parser.add_argument("--ref-depth", type=Path, required=True)
    parser.add_argument("--ref-json", type=Path, required=True)
    parser.add_argument("--expected-class-id", type=int, default=63)
    parser.add_argument("--confidence", type=float, default=0.35)
    parser.add_argument(
        "--artifacts-dir", type=Path,
        help="Optional directory for retained SQLite database and semantic node log.",
    )
    args = parser.parse_args()
    reference_paths = tuple(path.resolve() for path in
                            (args.ref_image, args.ref_depth, args.ref_json))
    require(os.environ.get("ROS_DOMAIN_ID") == DOMAIN,
            f"safety stop: run with ROS_DOMAIN_ID={DOMAIN} (got {os.environ.get('ROS_DOMAIN_ID')!r})")
    require(os.environ.get("ROS_LOCALHOST_ONLY", "1") == "1",
            "safety stop: ROS_LOCALHOST_ONLY must be 1")
    for path in (*reference_paths, MODEL):
        require(path.is_file(), f"missing integration input: {path}")
    require(args.expected_class_id in (63, 39, 41),
            f"unsupported expected semantic class ID: {args.expected_class_id}")

    rclpy.init()
    temp = None
    if args.artifacts_dir is None:
        temp = tempfile.TemporaryDirectory(prefix="semantic-mapping-smoke-")
        artifact_dir = Path(temp.name)
    else:
        artifact_dir = args.artifacts_dir.resolve()
        artifact_dir.mkdir(parents=True, exist_ok=True)
    db_path = artifact_dir / "semantic.sqlite3"
    require(not db_path.exists(), f"refusing to overwrite existing smoke database: {db_path}")
    if args.artifacts_dir is None:
        log_file = tempfile.NamedTemporaryFile(prefix="semantic-mapping-node-", suffix=".log", delete=False)
    else:
        log_file = (artifact_dir / "semantic_node.log").open("w", encoding="utf-8")
    smoke = None
    try:
        smoke = Smoke(db_path, log_file, reference_paths, args.expected_class_id,
                      args.confidence)
        smoke.start_node()

        # First observed RGB-D frame anchors map->odom and creates exactly one unconfirmed object.
        stamp1 = smoke.publish_frame()
        smoke.spin_until(lambda: all(smoke.input_seen[t] > 0 for t in (COLOR_TOPIC, DEPTH_TOPIC, INFO_TOPIC)),
                         2.0, f"test publisher messages not delivered: {smoke.input_seen}")
        first = smoke.wait_observation(stamp1)
        require(first.get("status") == "ok", f"first valid frame status was {first.get('status')!r}")
        require(first.get("class_id") == args.expected_class_id and
                first.get("class_ids") == [args.expected_class_id],
                f"single-class observation metadata was inaccurate: {first}")
        require(len(first.get("detections", [])) == 1,
                f"expected one class {args.expected_class_id} detection, got {first.get('detections')}")
        det1 = first["detections"][0]
        require(det1.get("class_id") == args.expected_class_id and det1.get("status") == "mapped",
                f"expected mapped COCO class {args.expected_class_id}, got {det1}")
        camera_xyz = np.asarray(det1["camera_xyz"], dtype=float)
        map_xyz = np.asarray(det1["map_xyz"], dtype=float)
        require(np.allclose(map_xyz, camera_xyz + [1.0, 2.0, 0.0], atol=0.015),
                f"camera XYZ + known map transform != map XYZ: {camera_xyz} -> {map_xyz}")
        reference_detection = next((d for d in smoke.reference["detections"]
                                    if d["class_id"] == args.expected_class_id and
                                    d["score"] >= args.confidence), None)
        require(reference_detection is not None,
                f"reference frame has no class {args.expected_class_id} depth estimate")
        expected_camera = np.asarray(reference_detection["camera_xyz"])
        require(np.allclose(camera_xyz, expected_camera, atol=0.12),
                f"real RGB-D inference XYZ deviated from reference: {camera_xyz} vs {expected_camera}")
        rows, objects = smoke.wait_db_observations(1)
        require(len(objects) == 1 and objects[0][1] == args.expected_class_id and objects[0][5] == 1,
                f"first observation class/count was unexpected: {objects}")
        print("PASS real RGB-D detector, aligned depth, and camera-to-map numeric transform")

        # Replayed messages with the same sensor timestamp cannot add confirmations.
        replay_stamp = stamp1
        for _ in range(3):
            smoke.publish_frame(stamp_ns=replay_stamp)
        smoke.spin_for(0.6)
        rows, objects = db_snapshot(db_path)
        require(len(rows) == 1 and len(objects) == 1 and objects[0][5] == 1,
                f"same-stamp replay incorrectly accumulated confirmations: observations={len(rows)}, objects={objects}")
        print("PASS repeated timestamp cannot count as multiple confirmations")

        stamp2 = smoke.publish_frame()
        second = smoke.wait_observation(stamp2)
        require(second.get("status") == "ok", f"second valid frame status was {second.get('status')!r}")
        rows, objects = smoke.wait_db_observations(2)
        require(objects[0][5] == 2, f"two distinct stamps should remain unconfirmed: {objects}")
        stamp3 = smoke.publish_frame()
        third = smoke.wait_observation(stamp3)
        require(third.get("status") == "ok", f"third valid frame status was {third.get('status')!r}")
        rows, objects = smoke.wait_db_observations(3)
        require(objects[0][5] == 3, f"three new stamps did not confirm one laptop: {objects}")
        print(f"PASS three distinct RGB-D timestamps confirm class {args.expected_class_id}")

        zeros = np.zeros_like(smoke.depth)
        bad_depth_stamp = smoke.publish_frame(depth=zeros)
        bad_depth = smoke.wait_observation(bad_depth_stamp)
        require(bad_depth["detections"][0].get("status") == "invalid_depth_map_not_stored",
                f"zero depth was not rejected: {bad_depth}")
        require(len(db_snapshot(db_path)[0]) == 3, "zero-depth detection was written to SQLite")
        print("PASS zero depth is reported and excluded from SQLite")

        missing_tf_stamp = smoke.publish_frame(map_tf=False)
        missing_tf = smoke.wait_observation(missing_tf_stamp)
        require(missing_tf.get("status") == "tf_expired",
                f"frame without map->odom TF did not expire without mapping: {missing_tf}")
        require(len(db_snapshot(db_path)[0]) == 3, "missing-TF detection was written to SQLite")
        print("PASS missing map transform expires without adding a map row")

        blank = np.zeros_like(smoke.image)
        blank_stamp = smoke.publish_frame(image=blank)
        no_detection = smoke.wait_observation(blank_stamp)
        require(no_detection.get("detections") == [], f"blank non-laptop frame produced detections: {no_detection}")
        require(len(db_snapshot(db_path)[0]) == 3, "non-laptop frame changed the semantic map")
        print("PASS blank frame produces no semantic map observation")

        jump_stamp = smoke.publish_frame(map_x=2.0)
        jump = smoke.wait_observation(jump_stamp)
        require(jump.get("status") == "map_jump_pause", f"map->odom jump was not paused: {jump}")
        require(jump["detections"][0].get("status") == "map_jump_pause_map_not_stored",
                f"map-jump detection was stored: {jump}")
        require(len(db_snapshot(db_path)[0]) == 3, "map-jump detection was written to SQLite")
        print("PASS map->odom jump pauses map writes")
        print("RESULT: all isolated ROS integration smoke assertions passed (ROS_DOMAIN_ID=71)")
    finally:
        if smoke is not None:
            smoke.close()
        log_file.flush()
        log_file.close()
        rclpy.shutdown()
        if temp is not None:
            temp.cleanup()


if __name__ == "__main__":
    run()
