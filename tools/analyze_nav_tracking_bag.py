#!/usr/bin/env python3
"""Offline summary of Nav2 tracking and chassis motion in a rosbag2 SQLite bag.

The analyzer reads only a small allowlist of topics from one rosbag2 SQLite
file. A bag directory must contain exactly one .db3; for a split bag, pass a
specific .db3 path explicitly. It never creates ROS nodes or deserializes point
clouds. A stationary interval is an inferred candidate for the operator stop;
rosbag data cannot identify an emergency-stop action.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import sqlite3
import statistics
import sys
from pathlib import Path
from typing import Any, Iterable


TOPICS = (
    "/navigate_to_pose/_action/feedback",
    "/cmd_vel_nav",
    "/cmd_vel",
    "/chassis_vel",
    "/imu/data",
    "/imu_raw",
    "/odom",
    "/plan",
    "/behavior_tree_log",
    "/spin/_action/status",
    "/spin/_action/feedback",
    "/backup/_action/status",
    "/backup/_action/feedback",
    "/wait/_action/status",
    "/wait/_action/feedback",
)


def _median(values: Iterable[float]) -> float | None:
    values = list(values)
    return float(statistics.median(values)) if values else None


def _mean(values: Iterable[float]) -> float | None:
    values = list(values)
    return float(statistics.mean(values)) if values else None


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def _linear_fit(x: list[float], y: list[float]) -> dict[str, float | int | None]:
    """Return y = slope*x + intercept and Pearson correlation."""
    n = min(len(x), len(y))
    if n < 2:
        return {"count": n, "slope": None, "intercept": None, "correlation": None}
    x, y = x[:n], y[:n]
    mx, my = statistics.mean(x), statistics.mean(y)
    xx = sum((value - mx) ** 2 for value in x)
    yy = sum((value - my) ** 2 for value in y)
    xy = sum((a - mx) * (b - my) for a, b in zip(x, y))
    if xx <= 1e-12:
        return {"count": n, "slope": None, "intercept": my, "correlation": None}
    return {
        "count": n,
        "slope": xy / xx,
        "intercept": my - (xy / xx) * mx,
        "correlation": xy / math.sqrt(xx * yy) if yy > 1e-12 else None,
    }


def _nearest(samples: list[dict[str, Any]], timestamps: list[float], t: float,
             max_distance: float) -> dict[str, Any] | None:
    if not samples:
        return None
    i = bisect.bisect_left(timestamps, t)
    candidates = []
    if i < len(samples):
        candidates.append(samples[i])
    if i > 0:
        candidates.append(samples[i - 1])
    sample = min(candidates, key=lambda item: abs(item["t"] - t))
    return sample if abs(sample["t"] - t) <= max_distance else None


def detect_stationary_start(
    chassis: list[dict[str, Any]],
    gyro: list[dict[str, Any]],
    search_start: float,
    bag_end: float,
    *,
    hold_seconds: float = 1.5,
    sample_period: float = 0.25,
    max_sample_age: float = 0.35,
    linear_threshold: float = 0.015,
    chassis_yaw_threshold: float = 0.02,
    gyro_threshold: float = 0.03,
) -> float | None:
    """Infer first sustained stationary bin from chassis and IMU feedback.

    Both sources must remain fresh and below thresholds for ``hold_seconds``.
    This deliberately labels a signal-based estimate, not an estop state.
    """
    if not chassis or not gyro or bag_end <= search_start:
        return None
    chassis_ts = [sample["t"] for sample in chassis]
    gyro_ts = [sample["t"] for sample in gyro]
    run_start = None
    count = math.ceil((bag_end - search_start) / sample_period)
    for i in range(count):
        t = search_start + i * sample_period
        c = _nearest(chassis, chassis_ts, t, max_sample_age)
        g = _nearest(gyro, gyro_ts, t, max_sample_age)
        stationary = bool(
            c and g
            and abs(c.get("vx", 0.0)) <= linear_threshold
            and abs(c.get("vy", 0.0)) <= linear_threshold
            and abs(c.get("wz", 0.0)) <= chassis_yaw_threshold
            and abs(g.get("wz", 0.0)) <= gyro_threshold
        )
        if stationary:
            if run_start is None:
                run_start = t
            if t - run_start + sample_period >= hold_seconds:
                return run_start
        else:
            run_start = None
    return None


def _yaw(q: dict[str, float]) -> float:
    return math.atan2(
        2.0 * (q["w"] * q["z"] + q["x"] * q["y"]),
        1.0 - 2.0 * (q["y"] ** 2 + q["z"] ** 2),
    )


def _validated_header_samples(
    odom: list[dict[str, Any]],
) -> tuple[list[dict[str, Any]], dict[str, int]]:
    """Drop invalid/duplicate odom header stamps, then sort by header time."""
    received_order = sorted(odom, key=lambda item: item["t"])
    valid = []
    seen_header_ns = set()
    invalid = 0
    duplicate = 0
    out_of_order = 0
    previous_header_ns = None
    for item in received_order:
        header_ns = item.get("header_ns")
        if not isinstance(header_ns, int) or header_ns <= 0:
            invalid += 1
            continue
        if header_ns in seen_header_ns:
            duplicate += 1
            continue
        seen_header_ns.add(header_ns)
        if previous_header_ns is not None and header_ns < previous_header_ns:
            out_of_order += 1
        previous_header_ns = header_ns
        valid.append(item)
    valid.sort(key=lambda item: item["header_ns"])
    return valid, {
        "input_samples": len(odom),
        "valid_unique_header_samples": len(valid),
        "invalid_or_zero_header_stamps_dropped": invalid,
        "duplicate_header_stamps_dropped": duplicate,
        "receive_order_header_regressions_sorted": out_of_order,
    }


def header_time_diagnostics(odom: list[dict[str, Any]]) -> dict[str, int]:
    return _validated_header_samples(odom)[1]


def pose_difference_samples(odom: list[dict[str, Any]]) -> list[dict[str, float]]:
    """Differentiate odom poses by header time; keep receive time for pairing."""
    ordered, _ = _validated_header_samples(odom)
    result = []
    for a, b in zip(ordered, ordered[1:]):
        dt = (b["header_ns"] - a["header_ns"]) / 1_000_000_000.0
        if dt <= 1e-3 or dt > 1.5:
            continue
        dx = (b["x"] - a["x"]) / dt
        dy = (b["y"] - a["y"]) / dt
        dyaw = (b["yaw"] - a["yaw"] + math.pi) % (2 * math.pi) - math.pi
        yaw_mid = a["yaw"] + 0.5 * dyaw
        result.append({
            # Receive time aligns the derived velocity with /odom.twist and
            # other bag topics; the derivative itself uses header_ns above.
            "t": 0.5 * (a["t"] + b["t"]),
            "header_t": 0.5 * (a["header_ns"] + b["header_ns"]) / 1e9,
            "vx": math.cos(yaw_mid) * dx + math.sin(yaw_mid) * dy,
            "vy": -math.sin(yaw_mid) * dx + math.cos(yaw_mid) * dy,
            "wz": dyaw / dt,
        })
    return sorted(result, key=lambda item: item["t"])


def _topic_stats(samples: list[dict[str, Any]], keys: tuple[str, ...]) -> dict[str, Any]:
    result: dict[str, Any] = {"count": len(samples)}
    for key in keys:
        values = [float(sample[key]) for sample in samples]
        result[key] = {
            "mean": _mean(values),
            "median": _median(values),
            "p05": _percentile(values, 0.05),
            "p95": _percentile(values, 0.95),
        }
    return result


def _correlation_at_lag(
    command: list[dict[str, Any]], chassis: list[dict[str, Any]], gyro: list[dict[str, Any]],
    lag: float, max_distance: float = 0.08,
) -> dict[str, Any]:
    cts = [item["t"] for item in chassis]
    gts = [item["t"] for item in gyro]
    cmd_values, chassis_values, gyro_values = [], [], []
    for item in command:
        c = _nearest(chassis, cts, item["t"] + lag, max_distance)
        g = _nearest(gyro, gts, item["t"] + lag, max_distance)
        if c and g:
            cmd_values.append(float(item["wz"]))
            chassis_values.append(float(c["wz"]))
            gyro_values.append(float(g["wz"]))
    chassis_fit = _linear_fit(cmd_values, chassis_values)
    gyro_fit = _linear_fit(cmd_values, gyro_values)
    return {
        "lag_seconds": lag,
        "sample_count": len(cmd_values),
        "cmd_to_chassis_yaw": chassis_fit,
        "cmd_to_gyro_yaw": gyro_fit,
    }


def _pose_twist_consistency(
    poses: list[dict[str, Any]], odom: list[dict[str, Any]], max_distance: float = 0.7,
) -> dict[str, Any]:
    derived = pose_difference_samples(poses)
    odom_ts = [item["t"] for item in odom]
    residuals = {"vx": [], "vy": [], "wz": []}
    paired = 0
    for item in derived:
        other = _nearest(odom, odom_ts, item["t"], max_distance)
        if other is None:
            continue
        paired += 1
        for key in residuals:
            residuals[key].append(item[key] - float(other[key]))
    return {
        "pose_difference_count": len(derived),
        "paired_count": paired,
        "body_velocity_from_pose": _topic_stats(derived, ("vx", "vy", "wz")),
        "median_abs_pose_minus_twist": {
            key: _median(abs(value) for value in values)
            for key, values in residuals.items()
        },
    }


def _reference_offset_fit(
    odom: list[dict[str, Any]], chassis: list[dict[str, Any]], max_distance: float = 0.25,
) -> dict[str, Any]:
    """Fit odom vy - chassis vy against odom wz as a reference-point clue."""
    cts = [item["t"] for item in chassis]
    feedback_y, yaw_rate = [], []
    for item in odom:
        if abs(float(item["wz"])) < 0.05:
            continue
        c = _nearest(chassis, cts, item["t"], max_distance)
        if c is None:
            continue
        feedback_y.append(float(item["vy"]) - float(c["vy"]))
        yaw_rate.append(float(item["wz"]))
    fit = _linear_fit(yaw_rate, feedback_y)
    fit["interpretation"] = (
        "slope is an inferred planar reference-point offset only if both velocity "
        "streams use comparable frames and the chassis feedback is at the rotation center"
    )
    return fit


def _lateral_residual_to_chassis(
    odom: list[dict[str, Any]], chassis: list[dict[str, Any]], max_distance: float = 0.25,
) -> dict[str, float | int | None]:
    cts = [item["t"] for item in chassis]
    residuals = []
    for item in odom:
        c = _nearest(chassis, cts, item["t"], max_distance)
        if c is not None:
            residuals.append(float(item["vy"]) - float(c["vy"]))
    return {
        "paired_count": len(residuals),
        "median_abs_m_s": _median(abs(value) for value in residuals),
        "mean_abs_m_s": _mean(abs(value) for value in residuals),
        "median_signed_m_s": _median(residuals),
    }


def _rebase_samples(odom: list[dict[str, Any]], shift_x: float) -> list[dict[str, Any]]:
    """Move the pose/twist reference point forward along each body x axis."""
    shifted = []
    for item in odom:
        q = dict(item)
        q["x"] = float(item["x"]) + shift_x * math.cos(float(item["yaw"]))
        q["y"] = float(item["y"]) + shift_x * math.sin(float(item["yaw"]))
        q["vy"] = float(item["vy"]) + shift_x * float(item["wz"])
        shifted.append(q)
    return shifted


def summarize_samples(samples: dict[str, list[dict[str, Any]]], bag_end: float,
                      reference_shift_x: float | None = None) -> dict[str, Any]:
    nav_feedback = samples.get("/navigate_to_pose/_action/feedback", [])
    cmd = samples.get("/cmd_vel", [])
    cmd_nav = samples.get("/cmd_vel_nav", [])
    chassis = samples.get("/chassis_vel", [])
    gyro = samples.get("/imu/data", []) or samples.get("/imu_raw", [])
    odom = samples.get("/odom", [])
    plans = samples.get("/plan", [])
    all_times = [item["t"] for topic_samples in samples.values() for item in topic_samples]

    task_start = nav_feedback[0]["t"] if nav_feedback else min(
        (item["t"] for item in cmd), default=bag_end
    )
    measured_activity = [
        item["t"] for item in chassis if item["t"] >= task_start
        and (abs(float(item["vx"])) > 0.02
             or abs(float(item["vy"])) > 0.02
             or abs(float(item["wz"])) > 0.03)
    ]
    measured_activity.extend(
        item["t"] for item in gyro
        if item["t"] >= task_start and abs(float(item["wz"])) > 0.03
    )
    command_activity = []
    for stream in (cmd, cmd_nav):
        command_activity.extend(
            item["t"] for item in stream if item["t"] >= task_start
            and (abs(float(item["vx"])) > 0.02
                 or abs(float(item["vy"])) > 0.02
                 or abs(float(item["wz"])) > 0.03)
        )
    motion_start = min(measured_activity or command_activity, default=task_start)
    stop_start = detect_stationary_start(chassis, gyro, motion_start, bag_end) \
        if measured_activity else None
    motion_end = stop_start if stop_start is not None else bag_end

    def within(items: list[dict[str, Any]]) -> list[dict[str, Any]]:
        return [item for item in items if task_start <= item["t"] < motion_end]

    cmd_motion, cmd_nav_motion = within(cmd), within(cmd_nav)
    chassis_motion, gyro_motion = within(chassis), within(gyro)
    odom_motion, feedback_motion = within(odom), within(nav_feedback)
    plan_motion = within(plans)
    lags = [_correlation_at_lag(cmd_motion, chassis_motion, gyro_motion, lag)
            for lag in [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]]
    best_gyro_lag = max(
        lags,
        key=lambda entry: (abs(entry["cmd_to_gyro_yaw"]["correlation"] or 0.0),
                           -entry["lag_seconds"]),
    ) if lags else None

    goal_points = []
    for plan in plan_motion:
        poses = plan.get("poses", [])
        if poses:
            goal_points.append(poses[-1])
    goal_x = _median([point["x"] for point in goal_points])
    goal_y = _median([point["y"] for point in goal_points])
    goal_spread = None
    if goal_x is not None and goal_y is not None:
        goal_spread = _median([math.hypot(point["x"] - goal_x, point["y"] - goal_y)
                               for point in goal_points])
    distance_values = [float(item["distance_remaining"]) for item in feedback_motion]
    segment = max(1, len(distance_values) // 4)

    spin_action_count = sum(len(samples.get(topic, [])) for topic in (
        "/spin/_action/status", "/spin/_action/feedback"
    ))
    bt_spin_running_events = sum(
        1 for item in samples.get("/behavior_tree_log", [])
        for event in item.get("events", [])
        if "spin" in event.get("node_name", "").lower()
        and event.get("current_status", "").upper() == "RUNNING"
    )

    summary = {
        "time_unit": "Unix epoch seconds",
        "time_bases": {
            "task_window_and_cross_topic_alignment": "SQLite receive timestamp",
            "odom_pose_differentiation": "nav_msgs/Odometry header stamp (unique, positive stamps only)",
        },
        "analysis_data_start_time": min(all_times, default=None),
        "analysis_data_end_time": max(all_times, default=bag_end),
        "bag_end_time": bag_end,
        "task_start_time": task_start,
        "motion_start_time": motion_start,
        "motion_end_time": motion_end,
        "statistical_window_start_time": task_start,
        "statistical_window_end_time": motion_end,
        "inferred_stationary_start": stop_start,
        "stationary_start_is_inferred_not_estop_state": True,
        "task_window_seconds": max(0.0, motion_end - task_start),
        "motion_window_seconds": max(0.0, motion_end - motion_start),
        "topics": {
            "/cmd_vel_nav": _topic_stats(cmd_nav_motion, ("vx", "vy", "wz")),
            "/cmd_vel": _topic_stats(cmd_motion, ("vx", "vy", "wz")),
            "/chassis_vel": _topic_stats(chassis_motion, ("vx", "vy", "wz")),
            "/imu": _topic_stats(gyro_motion, ("wz",)),
            "/odom_twist": _topic_stats(odom_motion, ("vx", "vy", "wz")),
        },
        "yaw_command_response": {
            "lag_scan": lags,
            "best_gyro_lag": best_gyro_lag,
            "gain_is_linear_fit_slope": True,
        },
        "odom_pose_twist_consistency": _pose_twist_consistency(odom_motion, odom_motion),
        "odom_header_time_quality": header_time_diagnostics(odom_motion),
        "odom_lateral_reference_offset_fit": _reference_offset_fit(odom_motion, chassis_motion),
        "plan": {
            "count": len(plan_motion),
            "goal_endpoint_median_xy": [goal_x, goal_y] if goal_x is not None else None,
            "goal_endpoint_median_spread_m": goal_spread,
            "distance_remaining_first_quartile_median": _median(distance_values[:segment]),
            "distance_remaining_last_quartile_median": _median(distance_values[-segment:]),
            "distance_remaining_samples": len(distance_values),
        },
        "recovery": {
            "spin_action_status_and_feedback_messages": spin_action_count,
            "bt_spin_running_events": bt_spin_running_events,
            "navigate_to_pose_reported_recoveries_max": max(
                (int(item.get("recoveries", 0)) for item in feedback_motion), default=0
            ),
        },
    }
    if reference_shift_x is not None:
        rebased_odom = _rebase_samples(odom_motion, reference_shift_x)
        rebased_pose_twist = _pose_twist_consistency(rebased_odom, rebased_odom)
        summary["reference_point_counterfactual"] = {
            "body_x_shift_m": reference_shift_x,
            "formula": "T_new = T_old * Trans(dx,0,0); vy_new = vy_old + wz*dx; vx and wz unchanged",
            "lateral_velocity_residual_before": _lateral_residual_to_chassis(
                odom_motion, chassis_motion),
            "lateral_velocity_residual_after": _lateral_residual_to_chassis(
                rebased_odom, chassis_motion),
            "pose_difference_twist_median_abs_residual_before":
                summary["odom_pose_twist_consistency"]["median_abs_pose_minus_twist"],
            "pose_difference_twist_median_abs_residual_after":
                rebased_pose_twist["median_abs_pose_minus_twist"],
            "pose_difference_lateral_vs_chassis_before": _lateral_residual_to_chassis(
                pose_difference_samples(odom_motion), chassis_motion),
            "pose_difference_lateral_vs_chassis_after": _lateral_residual_to_chassis(
                pose_difference_samples(rebased_odom), chassis_motion),
            "note": "offline rigid reference-point calculation; not a live TF or hardware validation",
        }
    return summary


def _deserialize_bag(db_path: Path) -> tuple[dict[str, list[dict[str, Any]]], float]:
    """Read only topic rows needed by this analyzer from SQLite rosbag2."""
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Python messages are unavailable; use /usr/bin/python3 with "
            "source /opt/ros/humble/setup.bash and source install/setup.bash"
        ) from exc

    conn = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    try:
        topic_info = {
            name: (topic_id, type_name)
            for topic_id, name, type_name in conn.execute("SELECT id,name,type FROM topics")
            if name in TOPICS
        }
        samples: dict[str, list[dict[str, Any]]] = {name: [] for name in topic_info}
        max_timestamp = 0
        type_cache = {}
        for topic, (topic_id, type_name) in topic_info.items():
            if type_name not in type_cache:
                type_cache[type_name] = get_message(type_name)
            message_type = type_cache[type_name]
            for timestamp, raw in conn.execute(
                "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
                (topic_id,),
            ):
                max_timestamp = max(max_timestamp, timestamp)
                msg = deserialize_message(raw, message_type)
                t = timestamp / 1_000_000_000.0
                if topic in ("/cmd_vel", "/cmd_vel_nav", "/chassis_vel"):
                    sample = {"t": t, "vx": msg.linear.x, "vy": msg.linear.y,
                              "wz": msg.angular.z}
                elif topic in ("/imu/data", "/imu_raw"):
                    sample = {"t": t, "wz": msg.angular_velocity.z}
                elif topic == "/odom":
                    pos = msg.pose.pose.position
                    q = msg.pose.pose.orientation
                    twist = msg.twist.twist
                    header_ns = (int(msg.header.stamp.sec) * 1_000_000_000
                                 + int(msg.header.stamp.nanosec))
                    sample = {
                        "t": t, "x": pos.x, "y": pos.y,
                        "header_ns": header_ns,
                        "header_t": header_ns / 1e9 if header_ns > 0 else None,
                        "yaw": _yaw({"x": q.x, "y": q.y, "z": q.z, "w": q.w}),
                        "vx": twist.linear.x, "vy": twist.linear.y, "wz": twist.angular.z,
                    }
                elif topic == "/navigate_to_pose/_action/feedback":
                    f = msg.feedback
                    sample = {"t": t, "distance_remaining": f.distance_remaining,
                              "recoveries": f.number_of_recoveries}
                elif topic == "/plan":
                    sample = {"t": t, "poses": [
                        {"x": pose.pose.position.x, "y": pose.pose.position.y}
                        for pose in msg.poses
                    ]}
                elif topic == "/behavior_tree_log":
                    sample = {"t": t, "events": [
                        {"node_name": event.node_name, "current_status": event.current_status}
                        for event in msg.event_log
                    ]}
                else:
                    sample = {"t": t}
                samples[topic].append(sample)
        return samples, max_timestamp / 1_000_000_000.0
    finally:
        conn.close()


def _format_num(value: Any, digits: int = 3) -> str:
    return "n/a" if value is None else f"{value:.{digits}f}"


def print_summary(summary: dict[str, Any], bag: Path) -> None:
    print(f"bag: {bag}")
    if summary["inferred_stationary_start"] is None:
        stop_text = "未检出"
    else:
        stop_text = (f"{summary['inferred_stationary_start']:.3f} s (Unix时间；"
                     "仅由持续静止信号推断，不是急停状态)")
    print(f"任务统计窗: {summary['task_window_seconds']:.1f} s；"
          f"实测运动窗: {summary['motion_window_seconds']:.1f} s；"
          f"停止候选: {stop_text}")
    print("运动窗速度统计 (median):")
    for topic, label in (("/cmd_vel_nav", "cmd_vel_nav"), ("/cmd_vel", "cmd_vel"),
                         ("/chassis_vel", "chassis_vel"), ("/imu", "gyro_z"),
                         ("/odom_twist", "odom twist")):
        stats = summary["topics"][topic]
        if topic == "/imu":
            print(f"  {label}: n={stats['count']} wz={_format_num(stats['wz']['median'])}")
        else:
            print(f"  {label}: n={stats['count']} vx/vy/wz="
                  f"{_format_num(stats['vx']['median'])}/"
                  f"{_format_num(stats['vy']['median'])}/"
                  f"{_format_num(stats['wz']['median'])}")
    best = summary["yaw_command_response"]["best_gyro_lag"]
    if best:
        fit = best["cmd_to_gyro_yaw"]
        print("角速度响应: "
              f"gyro-vs-cmd相关={_format_num(fit['correlation'])}, "
              f"线性斜率={_format_num(fit['slope'])}, "
              f"扫描lag={best['lag_seconds']:.2f}s")
    offset = summary["odom_lateral_reference_offset_fit"]
    print("odom lateral拟合: vy_odom - vy_chassis = "
          f"{_format_num(offset['slope'])} * wz + {_format_num(offset['intercept'])}; "
          f"r={_format_num(offset['correlation'])} (参考点线索，非frame结论)")
    plan = summary["plan"]
    print("路径: "
          f"n={plan['count']}, goal中位={plan['goal_endpoint_median_xy']}, "
          f"distance remaining首/末四分位中位="
          f"{_format_num(plan['distance_remaining_first_quartile_median'])}/"
          f"{_format_num(plan['distance_remaining_last_quartile_median'])} m")
    recovery = summary["recovery"]
    print("恢复: "
          f"Spin action消息={recovery['spin_action_status_and_feedback_messages']}, "
          f"BT Spin RUNNING事件={recovery['bt_spin_running_events']}, "
          f"NavigateToPose reported recoveries max="
          f"{recovery['navigate_to_pose_reported_recoveries_max']} "
          "(恢复次数不单独证明Spin执行)")


def find_db3(path: Path) -> Path:
    if path.is_file():
        return path
    if not path.is_dir():
        raise FileNotFoundError(path)
    candidates = sorted(path.glob("*.db3"))
    if not candidates:
        raise FileNotFoundError(f"no rosbag2 SQLite .db3 file in {path}")
    if len(candidates) > 1:
        raise ValueError(
            f"bag directory contains {len(candidates)} split SQLite files; "
            "pass the intended .db3 file explicitly to avoid analyzing only part of the bag"
        )
    return candidates[0]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag2 directory or SQLite .db3")
    parser.add_argument("--json-out", type=Path,
                        help="also write the machine-readable summary to this JSON file")
    parser.add_argument("--reference-point-shift-x", type=float,
                        help="optional offline body-x reference-point shift in meters")
    args = parser.parse_args(argv)
    try:
        db_path = find_db3(args.bag)
        samples, bag_end = _deserialize_bag(db_path)
        summary = summarize_samples(samples, bag_end, args.reference_point_shift_x)
    except (OSError, sqlite3.Error, RuntimeError, ValueError) as exc:
        print(f"分析失败: {exc}", file=sys.stderr)
        return 2
    print_summary(summary, args.bag)
    if "reference_point_counterfactual" in summary:
        counterfactual = summary["reference_point_counterfactual"]
        before = counterfactual["lateral_velocity_residual_before"]
        after = counterfactual["lateral_velocity_residual_after"]
        pose_before = counterfactual["pose_difference_twist_median_abs_residual_before"]
        pose_after = counterfactual["pose_difference_twist_median_abs_residual_after"]
        print("参考点反事实: "
              f"dx={counterfactual['body_x_shift_m']:.4f}m, "
              f"|vy_odom-vy_chassis| median "
              f"{_format_num(before['median_abs_m_s'])}→"
              f"{_format_num(after['median_abs_m_s'])} m/s; "
              f"pose-diff vs twist median abs residual vy "
              f"{_format_num(pose_before['vy'])}→{_format_num(pose_after['vy'])} m/s "
              "(仅离线重参考计算)")
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
                                 encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
