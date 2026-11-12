"""Small pure helpers for RealSense launch parameter normalization."""

import json


def normalize_camera_serial(value):
    serial = str(value).strip().strip("'\"").strip()
    if not serial:
        raise ValueError('serial_no must name a connected RealSense camera')
    return serial


def camera_serial_yaml_literal(value):
    """Return an explicit YAML string scalar for launch_ros parameter parsing."""
    return json.dumps(normalize_camera_serial(value))
