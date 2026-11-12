import math
from pathlib import Path

import pytest

from tools.analyze_nav_tracking_bag import (
    _linear_fit,
    _rebase_samples,
    detect_stationary_start,
    find_db3,
    header_time_diagnostics,
    pose_difference_samples,
    summarize_samples,
)


def test_stationary_start_requires_a_sustained_joint_chassis_and_gyro_stop():
    chassis = []
    gyro = []
    for i in range(80):
        t = i * 0.1
        stopped = t >= 4.0
        chassis.append({"t": t, "vx": 0.0 if stopped else 0.1,
                        "vy": 0.0, "wz": 0.0 if stopped else 0.2})
        gyro.append({"t": t, "wz": 0.0 if stopped else 0.3})

    inferred = detect_stationary_start(chassis, gyro, 0.0, 7.9)

    assert inferred is not None
    assert 3.9 <= inferred <= 4.1


def test_stationary_start_does_not_treat_a_short_zero_gap_as_stop():
    chassis = []
    gyro = []
    for i in range(60):
        t = i * 0.1
        stopped = 2.0 <= t < 2.5 or t >= 4.0
        chassis.append({"t": t, "vx": 0.0 if stopped else 0.1,
                        "vy": 0.0, "wz": 0.0 if stopped else 0.2})
        gyro.append({"t": t, "wz": 0.0 if stopped else 0.3})

    inferred = detect_stationary_start(chassis, gyro, 0.0, 5.9)

    assert inferred is not None
    assert inferred >= 3.9


def test_pose_difference_uses_header_time_despite_receive_jitter_and_unwraps_yaw():
    poses = [
        {"t": 0.0, "header_ns": 1_000_000_000, "x": 0.0, "y": 0.0,
         "yaw": math.pi - 0.1},
        {"t": 1.45, "header_ns": 2_000_000_000, "x": 1.0, "y": 0.0,
         "yaw": -math.pi + 0.1},
    ]

    result = pose_difference_samples(poses)

    assert len(result) == 1
    assert result[0]["vx"] == pytest.approx(-1.0)
    assert result[0]["vy"] == pytest.approx(0.0, abs=1e-9)
    assert result[0]["wz"] == pytest.approx(0.2)
    assert result[0]["t"] == pytest.approx(0.725)


def test_pose_derivative_is_constant_under_receive_timestamp_jitter():
    poses = [
        {"t": 0.0, "header_ns": 1_000_000_000, "x": 0.0, "y": 0.0, "yaw": 0.0},
        {"t": 0.15, "header_ns": 2_000_000_000, "x": 0.0, "y": 0.0, "yaw": 0.2},
        {"t": 1.9, "header_ns": 3_000_000_000, "x": 0.0, "y": 0.0, "yaw": 0.4},
    ]

    result = pose_difference_samples(poses)

    assert [sample["wz"] for sample in result] == pytest.approx([0.2, 0.2])
    assert [sample["header_t"] for sample in result] == pytest.approx([1.5, 2.5])


def test_invalid_and_duplicate_odom_header_stamps_are_dropped():
    poses = [
        {"t": 0.0, "header_ns": 1_000_000_000, "x": 0.0, "y": 0.0, "yaw": 0.0},
        {"t": 0.1, "header_ns": 1_000_000_000, "x": 1.0, "y": 0.0, "yaw": 0.1},
        {"t": 0.2, "header_ns": 0, "x": 2.0, "y": 0.0, "yaw": 0.2},
        {"t": 1.0, "header_ns": 2_000_000_000, "x": 1.0, "y": 0.0, "yaw": 0.2},
    ]

    diagnostics = header_time_diagnostics(poses)
    result = pose_difference_samples(poses)

    assert diagnostics["invalid_or_zero_header_stamps_dropped"] == 1
    assert diagnostics["duplicate_header_stamps_dropped"] == 1
    assert diagnostics["valid_unique_header_samples"] == 2
    assert len(result) == 1


def test_reference_fit_recovers_lateral_velocity_slope():
    fit = _linear_fit([0.1, 0.2, 0.3, 0.4], [-0.071, -0.142, -0.213, -0.284])

    assert fit["slope"] == pytest.approx(-0.71)
    assert fit["intercept"] == pytest.approx(0.0)
    assert fit["correlation"] == pytest.approx(-1.0)


def test_rebase_updates_pose_and_lateral_velocity_for_body_x_offset():
    original = [{"t": 0.0, "x": 1.0, "y": 2.0, "yaw": math.pi / 2,
                 "vx": 0.3, "vy": -0.2, "wz": 0.5}]

    shifted = _rebase_samples(original, 0.6)[0]

    assert shifted["x"] == pytest.approx(1.0)
    assert shifted["y"] == pytest.approx(2.6)
    assert shifted["vx"] == pytest.approx(0.3)
    assert shifted["vy"] == pytest.approx(0.1)
    assert shifted["wz"] == pytest.approx(0.5)


def test_spin_action_and_bt_events_are_reported_separately_from_recoveries():
    samples = {
        "/navigate_to_pose/_action/feedback": [
            {"t": 1.0, "distance_remaining": 1.0, "recoveries": 1},
            {"t": 2.0, "distance_remaining": 0.9, "recoveries": 1},
        ],
        "/cmd_vel": [{"t": 1.0, "vx": 0.1, "vy": 0.0, "wz": 0.2}],
        "/chassis_vel": [
            {"t": 1.0, "vx": 0.1, "vy": 0.0, "wz": 0.2},
            {"t": 2.0, "vx": 0.1, "vy": 0.0, "wz": 0.2},
        ],
        "/imu/data": [{"t": 1.0, "wz": 0.2}, {"t": 2.0, "wz": 0.2}],
        "/odom": [
            {"t": 1.0, "x": 0.0, "y": 0.0, "yaw": 0.0,
             "vx": 0.1, "vy": 0.0, "wz": 0.2},
            {"t": 2.0, "x": 0.1, "y": 0.0, "yaw": 0.2,
             "vx": 0.1, "vy": 0.0, "wz": 0.2},
        ],
        "/plan": [{"t": 1.0, "poses": [{"x": 0.0, "y": 0.0},
                                           {"x": 1.0, "y": 0.0}]}],
        "/behavior_tree_log": [{"t": 1.5, "events": [
            {"node_name": "Spin", "current_status": "RUNNING"}]}],
        "/spin/_action/status": [],
        "/spin/_action/feedback": [],
    }

    summary = summarize_samples(samples, bag_end=2.0)

    assert summary["recovery"]["navigate_to_pose_reported_recoveries_max"] == 1
    assert summary["recovery"]["spin_action_status_and_feedback_messages"] == 0
    assert summary["recovery"]["bt_spin_running_events"] == 1


def test_summarizer_skips_initial_command_before_first_measured_motion():
    samples = {
        "/navigate_to_pose/_action/feedback": [
            {"t": 0.0, "distance_remaining": 1.0, "recoveries": 0}],
        "/cmd_vel": [
            {"t": 0.0, "vx": 0.1, "vy": 0.0, "wz": 0.2},
            {"t": 1.0, "vx": 0.1, "vy": 0.0, "wz": 0.2}],
        "/chassis_vel": [
            {"t": round(i * 0.25, 2),
             "vx": 0.1 if 1.0 <= i * 0.25 < 3.0 else 0.0,
             "vy": 0.0, "wz": 0.2 if 1.0 <= i * 0.25 < 3.0 else 0.0}
            for i in range(21)],
        "/imu/data": [
            {"t": round(i * 0.25, 2),
             "wz": 0.2 if 1.0 <= i * 0.25 < 3.0 else 0.0}
            for i in range(21)],
        "/odom": [],
        "/plan": [],
    }

    summary = summarize_samples(samples, bag_end=5.0)

    assert summary["motion_start_time"] == 1.0
    assert summary["inferred_stationary_start"] >= 3.0
    assert summary["task_window_seconds"] > summary["motion_window_seconds"]
    assert summary["task_window_seconds"] == pytest.approx(
        summary["motion_window_seconds"] + summary["motion_start_time"]
    )


def test_find_db3_refuses_to_silently_analyze_only_one_split(tmp_path: Path):
    (tmp_path / "bag_0.db3").touch()
    (tmp_path / "bag_1.db3").touch()

    with pytest.raises(ValueError, match="split SQLite files"):
        find_db3(tmp_path)
