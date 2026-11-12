#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Helpers for camera intrinsics and hand-eye extrinsics."""

import os

import numpy as np
import yaml


def resolve_config_file(name, explicit='', extra_candidates=None):
    """Find a config file in the workspace/config directory.

    Search order:
      1. explicit path (if it exists)
      2. $PIPER_VISION_CONFIG_DIR/name
      3. <cwd>/src/config/name
      4. <cwd>/config/name
      5. package source: <this package>/../../config/name
      6. workspace root derived from the installed package share:
         <share>/../../../../src/config/name
      7. package share config/name
    """
    if explicit:
        if os.path.isfile(explicit):
            return os.path.abspath(explicit)
        # Keep the requested path so the error message is useful, but
        # still try the search paths below.

    candidates = []
    env_dir = os.environ.get('PIPER_VISION_CONFIG_DIR')
    if env_dir:
        candidates.append(os.path.join(env_dir, name))
    candidates.extend([
        os.path.join(os.getcwd(), 'src', 'config', name),
        os.path.join(os.getcwd(), 'config', name),
        os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', '..', 'config', name)),
    ])

    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('piper_vision')
        candidates.extend([
            os.path.abspath(
                os.path.join(share, '..', '..', '..', '..',
                             'src', 'config', name)),
            os.path.join(share, 'config', name),
        ])
    except Exception:
        pass

    if extra_candidates:
        candidates.extend(extra_candidates)

    for path in candidates:
        if os.path.isfile(path):
            return os.path.abspath(path)

    return os.path.abspath(explicit) if explicit else os.path.abspath(candidates[0])


def load_intrinsics(path):
    """Load camera_matrix, dist_coeff and image_size from calibration.yaml."""
    with open(path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    if 'camera_matrix' not in cfg:
        raise KeyError(f'{path}: missing camera_matrix')
    K = np.asarray(cfg['camera_matrix'], dtype=np.float64)
    if K.shape != (3, 3):
        raise ValueError(f'{path}: camera_matrix must be 3x3, got {K.shape}')

    D = np.asarray(
        cfg.get('dist_coeff', [0.0, 0.0, 0.0, 0.0, 0.0]),
        dtype=np.float64).reshape(-1)

    image_size = tuple(cfg.get('image_size', [0, 0]))
    return K, D, image_size, cfg


def load_extrinsics(path):
    """Load T_cam2base (camera optical -> robot base) from YAML."""
    with open(path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    if 'T_cam2base' in cfg:
        T = np.asarray(cfg['T_cam2base'], dtype=np.float64)
    elif 'R_cam2base' in cfg and 't_cam2base' in cfg:
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = np.asarray(cfg['R_cam2base'], dtype=np.float64)
        T[:3, 3] = np.asarray(cfg['t_cam2base'], dtype=np.float64).reshape(3)
    else:
        raise KeyError(f'{path}: missing T_cam2base or R_cam2base/t_cam2base')

    if T.shape != (4, 4):
        raise ValueError(f'{path}: T_cam2base must be 4x4, got {T.shape}')
    return T, cfg


def invert_transform(T):
    """Invert a rigid 4x4 transform."""
    T = np.asarray(T, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def transform_point(T, point):
    """Apply a 4x4 rigid transform to a 3D point."""
    p = np.asarray(point, dtype=np.float64).reshape(3)
    return (T[:3, :3] @ p + T[:3, 3]).tolist()


def quat_to_matrix(x, y, z, w):
    """Quaternion (x, y, z, w) to 3x3 rotation matrix."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ], dtype=np.float64)


def matrix_to_quat(R):
    """3x3 rotation matrix to quaternion (x, y, z, w)."""
    R = np.asarray(R, dtype=np.float64)
    tr = float(np.trace(R))
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    return q / np.linalg.norm(q)


def transform_to_matrix(transform_msg):
    """geometry_msgs/TransformStamped -> 4x4 numpy matrix."""
    t = transform_msg.transform.translation
    q = transform_msg.transform.rotation
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.x, t.y, t.z]
    return T
