#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Matplotlib 3D view of base_link, camera and detected objects.

Run this node with PYTHONNOUSERSITE=1 if the user-site matplotlib is a
different version than the system mpl_toolkits (typical on Ubuntu).
The launch file sets that environment variable for this node.
"""

import os
import sys
import threading
import time

# On this machine the user-site matplotlib (3.10.x) is newer than the
# system mpl_toolkits (3.5.x).  Re-exec without user site-packages so
# matplotlib and mpl_toolkits come from the same system installation.
if not os.environ.get('PIPER_PLOT_REEXEC') and \
        not os.environ.get('PYTHONNOUSERSITE'):
    os.environ['PYTHONNOUSERSITE'] = '1'
    os.environ['PIPER_PLOT_REEXEC'] = '1'
    os.execv(sys.executable, [sys.executable] + sys.argv)

import numpy as np
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray

from piper_vision.calibration import (
    load_extrinsics,
    resolve_config_file,
)

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt  # noqa: E402
from mpl_toolkits.mplot3d import Axes3D  # noqa: E402,F401


class VisionTfPlot(Node):

    def __init__(self):
        super().__init__('vision_tf_plot')

        self.declare_parameter('extrinsics_file', '')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('plot_rate', 2.0)

        self.base_frame = self.get_parameter(
            'base_frame').get_parameter_value().string_value
        self.plot_rate = max(
            1.0, self.get_parameter(
                'plot_rate').get_parameter_value().double_value)

        path = resolve_config_file(
            'left_hand.yaml',
            self.get_parameter(
                'extrinsics_file').get_parameter_value().string_value)
        self.T_cam2base, self.ext_cfg = load_extrinsics(path)
        self.camera_origin = self.T_cam2base[:3, 3].copy()
        self.camera_rotation = self.T_cam2base[:3, :3].copy()

        self._latest = None
        self._lock = threading.Lock()
        self._last_draw = 0.0
        self._last_signature = None
        self._has_drawn = False
        self._view_set = False
        self._elev = 25.0
        self._azim = -60.0

        self.create_subscription(
            Detection3DArray,
            '/vision/detections_3d_base',
            self.detections_callback,
            10)

        self.fig = plt.figure(figsize=(9, 7))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.fig.canvas.manager.set_window_title(
            'PiPER eye-to-hand: base / camera / objects')

        self.get_logger().info(
            f'TF plot ready: extrinsics={path}, base_frame={self.base_frame}, '
            f'camera_origin={self.camera_origin.tolist()}')

    def detections_callback(self, msg):
        with self._lock:
            self._latest = msg

    def draw(self, force=False):
        now = time.monotonic()
        if not force and now - self._last_draw < 1.0 / self.plot_rate:
            return

        with self._lock:
            msg = self._latest

        # The scene is static unless the object set/positions change by
        # more than ~1 cm.  Avoid a full 3D redraw at every frame; it is
        # expensive with Matplotlib/Tk.
        signature = self._scene_signature(msg)
        if not force and self._has_drawn and signature == self._last_signature:
            return
        self._last_draw = now
        self._last_signature = signature

        if self._view_set:
            self._elev = self.ax.elev
            self._azim = self.ax.azim

        self.ax.cla()
        self.ax.grid(True)

        # Base frame
        self._draw_frame(
            np.zeros(3), np.eye(3), 0.15, 'base', 'k', 's')

        # Camera optical frame in base coordinates
        self._draw_frame(
            self.camera_origin, self.camera_rotation, 0.10,
            'camera', 'c', 'o')

        points = [np.zeros(3), self.camera_origin]
        obj_count = 0

        if msg is not None:
            for det in msg.detections:
                if not det.results:
                    continue
                label = det.results[0].hypothesis.class_id
                p = det.bbox.center.position
                point = np.array([p.x, p.y, p.z], dtype=float)
                points.append(point)
                obj_count += 1

                self.ax.scatter(
                    [point[0]], [point[1]], [point[2]],
                    c='r', marker='*', s=90,
                    label='objects' if obj_count == 1 else None)
                self.ax.text(
                    point[0], point[1], point[2],
                    f'{label}\n({point[0]:.2f},{point[1]:.2f},'
                    f'{point[2]:.2f})',
                    color='r', fontsize=8)

                # base -> object and camera -> object sight lines
                self.ax.plot(
                    [0.0, point[0]], [0.0, point[1]], [0.0, point[2]],
                    'r--', linewidth=0.8, alpha=0.5)
                self.ax.plot(
                    [self.camera_origin[0], point[0]],
                    [self.camera_origin[1], point[1]],
                    [self.camera_origin[2], point[2]],
                    'c:', linewidth=0.8, alpha=0.6)

        pts = np.asarray(points, dtype=float)
        mins = pts.min(axis=0)
        maxs = pts.max(axis=0)
        center = (mins + maxs) / 2.0
        radius = max(float(np.max(maxs - mins)) * 0.65, 0.35)
        self.ax.set_xlim(center[0] - radius, center[0] + radius)
        self.ax.set_ylim(center[1] - radius, center[1] + radius)
        self.ax.set_zlim(center[2] - radius, center[2] + radius)
        try:
            self.ax.set_box_aspect((1, 1, 1))
        except Exception:
            pass

        self.ax.set_xlabel('X base (m)')
        self.ax.set_ylabel('Y base (m)')
        self.ax.set_zlabel('Z base (m)')
        self.ax.set_title(
            f'PiPER eye-to-hand TF    objects={obj_count}')
        self.ax.legend(loc='upper left')

        if self._view_set:
            self.ax.view_init(self._elev, self._azim)
        else:
            self.ax.view_init(25, -60)
            self._view_set = True

        self.fig.canvas.draw_idle()
        self._has_drawn = True

    @staticmethod
    def _scene_signature(msg):
        if msg is None:
            return ()
        items = []
        for det in msg.detections:
            if not det.results:
                continue
            label = det.results[0].hypothesis.class_id
            p = det.bbox.center.position
            # 1 cm quantisation: ignore sub-centimetre jitter.
            items.append((label, round(p.x, 2), round(p.y, 2),
                          round(p.z, 2)))
        return tuple(sorted(items))

    def _draw_frame(self, origin, rotation, length, name, color, marker):
        self.ax.scatter(
            [origin[0]], [origin[1]], [origin[2]],
            c=color, marker=marker, s=45, label=name)

        axis_colors = [(1.0, 0.0, 0.0), (0.0, 0.8, 0.0), (0.0, 0.2, 1.0)]
        axis_names = ['X', 'Y', 'Z']
        for i, c in enumerate(axis_colors):
            v = rotation[:, i] * length
            self.ax.quiver(
                origin[0], origin[1], origin[2],
                v[0], v[1], v[2],
                color=c, arrow_length_ratio=0.18, linewidth=2.0)
            self.ax.text(
                origin[0] + v[0] * 1.15,
                origin[1] + v[1] * 1.15,
                origin[2] + v[2] * 1.15,
                f'{name}_{axis_names[i]}',
                color=c, fontsize=8)


def main(args=None):
    rclpy.init(args=args)
    node = VisionTfPlot()
    plt.ion()
    node.fig.show()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.draw()
            # flush_events + sleep is much cheaper than plt.pause(),
            # which busy-spins the Tk event loop on this machine.
            node.fig.canvas.flush_events()
            time.sleep(0.05)
    except (KeyboardInterrupt, Exception) as exc:  # noqa: BLE001
        if rclpy.ok():
            node.get_logger().info(f'plot loop ended: {exc}')
    finally:
        try:
            plt.close('all')
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
