#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YOLOv8n ONNX + Intel RealSense D435i depth -> 3D object coordinates.

Subscribes:
  /camera/camera/color/image_raw                 (sensor_msgs/Image)
  /camera/camera/aligned_depth_to_color/image_raw (sensor_msgs/Image, 16UC1 mm)
  /camera/camera/color/camera_info               (sensor_msgs/CameraInfo)

Publishes:
  /vision/annotated_image  (sensor_msgs/Image)
  /vision/detections_3d    (vision_msgs/Detection3DArray)
  /vision/detections_json  (std_msgs/String)
  /vision/markers          (visualization_msgs/MarkerArray)

3D coordinates are expressed in the color optical frame
(+X right, +Y down, +Z forward), in metres.
"""

import json
import os
import time

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, TransformStamped, Vector3
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, String
from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformBroadcaster,
    TransformListener,
)
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray

from piper_vision.calibration import (
    invert_transform,
    load_extrinsics,
    load_intrinsics,
    matrix_to_quat,
    resolve_config_file,
    transform_point,
    transform_to_matrix,
)


COCO_CLASSES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
    'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag',
    'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite',
    'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
    'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon',
    'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
    'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant',
    'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote',
    'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
    'hair drier', 'toothbrush',
]


class YoloDepthNode(Node):

    def __init__(self):
        super().__init__('yolo_depth_node')

        default_model = self._default_model_path()

        self.model_path = self._param('model_path', default_model)
        self.color_topic = self._param(
            'color_topic', '/camera/camera/color/image_raw')
        self.depth_topic = self._param(
            'depth_topic',
            '/camera/camera/aligned_depth_to_color/image_raw')
        self.camera_info_topic = self._param(
            'camera_info_topic', '/camera/camera/color/camera_info')

        self.intrinsics_file = resolve_config_file(
            'calibration.yaml',
            self._param('intrinsics_file', ''))
        self.extrinsics_file = resolve_config_file(
            'left_hand.yaml',
            self._param('extrinsics_file', ''))
        self.base_frame = self._param('base_frame', 'base_link')
        self.camera_link_frame = self._param('camera_link_frame', 'camera_link')
        self.publish_base_tf = bool(self._param('publish_base_tf', True))
        self.publish_object_tf = bool(self._param('publish_object_tf', True))

        self.input_size = int(self._param('input_size', 640))
        # 1 = infer every frame, 2 = every other frame, 3 = infer once
        # every three frames (skip two frames between inferences).
        self.infer_every_n_frames = max(
            1, int(self._param('infer_every_n_frames', 3)))
        self.conf_threshold = float(self._param('conf_threshold', 0.25))
        self.iou_threshold = float(self._param('iou_threshold', 0.45))
        self.depth_scale = float(self._param('depth_scale', 0.001))
        self.max_depth = float(self._param('max_depth', 8.0))
        self.depth_patch_ratio = float(self._param('depth_patch_ratio', 0.20))
        self.depth_patch_min = int(self._param('depth_patch_min', 5))
        self.publish_annotated = bool(self._param('publish_annotated', True))
        self.publish_markers = bool(self._param('publish_markers', True))
        self.num_threads = int(self._param('num_threads', 4))
        self.log_interval = float(self._param('log_interval', 2.0))

        self.bridge = CvBridge()
        self.camera_info = None
        self._last_log = 0.0
        self._last_warn = 0.0
        self._frames = 0
        self._frame_index = 0
        self._has_inferred = False
        self._last_raw_detections = []
        self._last_infer_ms = 0.0
        self._active_k = None
        self._active_size = None
        self._camera_frame = 'camera_color_optical_frame'

        # TF: base_link -> camera_link (static) and base_link -> object_i.
        self._tf_attempt_start = time.monotonic()
        self._base_tf_published = False
        self._tf_warned = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.object_tf_broadcaster = TransformBroadcaster(self)

        self._load_calibration()
        self._load_model()
        self._setup_publishers()
        self._setup_subscribers()

        self.get_logger().info(
            f'YOLO depth node ready: model={self.model_path}, '
            f'color={self.color_topic}, depth={self.depth_topic}')
        self.get_logger().info(
            f'intrinsics={self.intrinsics_file}, '
            f'extrinsics={self.extrinsics_file}, '
            f'base_frame={self.base_frame}')
        self.get_logger().info(
            '3D coordinates are in the color optical frame '
            '(+X right, +Y down, +Z forward), unit: metre')

    # ------------------------------------------------------------------ #
    # parameters / model
    # ------------------------------------------------------------------ #
    def _param(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    @staticmethod
    def _default_model_path():
        try:
            share = get_package_share_directory('piper_vision')
            path = os.path.join(share, 'models', 'yolov8n.onnx')
            if os.path.isfile(path):
                return path
        except Exception:
            pass

        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        candidates = [
            os.path.join(here, 'models', 'yolov8n.onnx'),
            os.path.abspath('models/yolov8n.onnx'),
            os.path.abspath('src/piper_vision/models/yolov8n.onnx'),
        ]
        for path in candidates:
            if os.path.isfile(path):
                return path
        return candidates[0]

    def _load_model(self):
        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(
                f'ONNX model not found: {self.model_path}. '
                'Put yolov8n.onnx in piper_vision/models or set model_path.')

        opts = ort.SessionOptions()
        opts.intra_op_num_threads = max(1, self.num_threads)
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        self.session = ort.InferenceSession(
            self.model_path,
            sess_options=opts,
            providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name

        in_shape = self.session.get_inputs()[0].shape
        self.input_h = in_shape[2] if isinstance(in_shape[2], int) else self.input_size
        self.input_w = in_shape[3] if isinstance(in_shape[3], int) else self.input_size

        out_shape = self.session.get_outputs()[0].shape
        self.get_logger().info(
            f'ONNX loaded: input={in_shape}, output={out_shape}')

    def _load_calibration(self):
        self.K, self.D, self.calib_image_size, self.calib_cfg = \
            load_intrinsics(self.intrinsics_file)
        self.T_cam2base, self.ext_cfg = load_extrinsics(
            self.extrinsics_file)
        self.R_cam2base = self.T_cam2base[:3, :3]
        self.t_cam2base = self.T_cam2base[:3, 3]

        self.get_logger().info(
            f'intrinsics: fx={self.K[0, 0]:.3f}, fy={self.K[1, 1]:.3f}, '
            f'cx={self.K[0, 2]:.3f}, cy={self.K[1, 2]:.3f}, '
            f'image_size={self.calib_image_size}, '
            f'dist_coeff={self.D.tolist()}')
        self.get_logger().info(
            'extrinsics T_cam2base: '
            f't=({self.t_cam2base[0]:.4f}, '
            f'{self.t_cam2base[1]:.4f}, {self.t_cam2base[2]:.4f}) m')
        if self.ext_cfg.get('physical_acceptance') is False:
            self.get_logger().warn(
                'handeye config says physical_acceptance=false; '
                'please verify the camera-to-base extrinsic.')

    def _get_k_for_size(self, width, height):
        """Return intrinsics scaled to the actual image size."""
        size = (int(width), int(height))
        if self._active_size == size:
            return self._active_k

        k = self.K.copy()
        cw, ch = self.calib_image_size
        if cw and ch and (cw, ch) != size:
            sx = float(width) / float(cw)
            sy = float(height) / float(ch)
            k[0, 0] *= sx
            k[0, 2] *= sx
            k[1, 1] *= sy
            k[1, 2] *= sy
            self.get_logger().warn(
                f'camera image size {size} differs from calibration '
                f'image size {(cw, ch)}; scaling intrinsics by '
                f'({sx:.4f}, {sy:.4f})')
        self._active_size = size
        self._active_k = k
        return k

    # ------------------------------------------------------------------ #
    # ROS interfaces
    # ------------------------------------------------------------------ #
    def _setup_publishers(self):
        # Use reliable QoS so rqt_image_view / RViz / normal subscribers
        # (which typically use the default reliable profile) can receive it.
        self.annotated_pub = self.create_publisher(
            Image, '/vision/annotated_image', 10)
        self.det3d_pub = self.create_publisher(
            Detection3DArray, '/vision/detections_3d', 10)
        self.det3d_base_pub = self.create_publisher(
            Detection3DArray, '/vision/detections_3d_base', 10)
        self.json_pub = self.create_publisher(
            String, '/vision/detections_json', 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/vision/markers', 10)
        self.marker_base_pub = self.create_publisher(
            MarkerArray, '/vision/markers_base', 10)

    def _setup_subscribers(self):
        self.color_sub = Subscriber(
            self, Image, self.color_topic,
            qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(
            self, Image, self.depth_topic,
            qos_profile=qos_profile_sensor_data)

        self.sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=5, slop=0.10)
        self.sync.registerCallback(self.image_callback)

        self.info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.info_callback,
            qos_profile_sensor_data)

    def info_callback(self, msg):
        if self.camera_info is None:
            self.get_logger().info(
                f'camera_info received: {msg.width}x{msg.height}, '
                f'frame={msg.header.frame_id}, '
                f'fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, '
                f'cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}')
        self.camera_info = msg

    # ------------------------------------------------------------------ #
    # main callback
    # ------------------------------------------------------------------ #
    def image_callback(self, color_msg, depth_msg):
        if self.camera_info is None:
            self._warn_throttle('waiting for camera_info ...')
            return

        t_start = time.monotonic()
        try:
            color = self.bridge.imgmsg_to_cv2(
                color_msg, desired_encoding='bgr8')
            depth_raw = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'cv_bridge conversion failed: {exc}')
            return

        depth_m = self._to_depth_meters(depth_raw)
        if depth_m.shape[:2] != color.shape[:2]:
            depth_m = cv2.resize(
                depth_m,
                (color.shape[1], color.shape[0]),
                interpolation=cv2.INTER_NEAREST)

        do_infer = (
            not self._has_inferred
            or self._frame_index % self.infer_every_n_frames == 0)
        if do_infer:
            raw_detections, infer_ms = self._infer(color)
            self._last_raw_detections = self._clone_detections(raw_detections)
            self._last_infer_ms = infer_ms
            self._has_inferred = True
        else:
            raw_detections = self._clone_detections(self._last_raw_detections)
            infer_ms = self._last_infer_ms
        self._frame_index += 1
        detections = raw_detections

        header = Header()
        header.stamp = color_msg.header.stamp
        header.frame_id = (
            color_msg.header.frame_id or 'camera_color_optical_frame')
        self._camera_frame = header.frame_id

        for idx, det in enumerate(detections):
            det['index'] = idx
            x1, y1, x2, y2 = det['bbox']
            z, center = self._sample_depth(depth_m, x1, y1, x2, y2)
            det['depth_m'] = z
            det['center_uv'] = center
            if z is not None:
                xyz_cam = self._pixel_to_3d(
                    center[0], center[1], z,
                    color.shape[1], color.shape[0])
                det['xyz_camera_m'] = xyz_cam
                det['xyz_base_m'] = transform_point(
                    self.T_cam2base, xyz_cam)
                # Backward-compatible alias: xyz_m was camera-frame before.
                det['xyz_m'] = xyz_cam
            else:
                det['xyz_camera_m'] = None
                det['xyz_base_m'] = None
                det['xyz_m'] = None

        annotated = color.copy()
        self._draw_boxes(annotated, detections)

        if self.publish_annotated:
            try:
                out_msg = self.bridge.cv2_to_imgmsg(
                    annotated, encoding='bgr8')
                out_msg.header = color_msg.header
                self.annotated_pub.publish(out_msg)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'annotated image publish failed: {exc}')

        if self.publish_base_tf:
            self._publish_base_camera_tf()
        if self.publish_object_tf:
            self._publish_object_tfs(header, detections)

        self._publish_detections(header, detections)
        self._publish_json(
            header, detections, color.shape, infer_ms, do_infer)

        if self.publish_markers:
            self._publish_markers(header, detections)

        self._frames += 1
        now = time.monotonic()
        if now - self._last_log >= self.log_interval:
            n_depth = sum(1 for d in detections if d['xyz_m'] is not None)
            mode = 'infer' if do_infer else 'cached'
            self.get_logger().info(
                f'frame={self._frames}, detections={len(detections)}, '
                f'with_depth={n_depth}, mode={mode}, '
                f'infer={infer_ms:.1f} ms, '
                f'total={(now - t_start) * 1000.0:.1f} ms')
            self._last_log = now

    # ------------------------------------------------------------------ #
    # YOLOv8 ONNX inference
    # ------------------------------------------------------------------ #
    def _infer(self, image):
        img, ratio, pad_x, pad_y = self._letterbox(
            image, (self.input_w, self.input_h))
        blob = cv2.dnn.blobFromImage(
            img,
            scalefactor=1.0 / 255.0,
            size=(self.input_w, self.input_h),
            mean=(0, 0, 0),
            swapRB=True,
            crop=False)

        t0 = time.monotonic()
        output = self.session.run(None, {self.input_name: blob})[0]
        infer_ms = (time.monotonic() - t0) * 1000.0

        pred = output
        if pred.ndim == 3:
            pred = pred[0]
        if pred.ndim != 2:
            self.get_logger().error(f'unexpected ONNX output shape: {output.shape}')
            return [], infer_ms

        n_cls = len(COCO_CLASSES)
        if pred.shape[0] == 4 + n_cls and pred.shape[1] != 4 + n_cls:
            pred = pred.T
        if pred.shape[1] < 4 + n_cls:
            self.get_logger().error(
                f'ONNX output has {pred.shape[1]} columns, expected {4 + n_cls}')
            return [], infer_ms

        boxes = pred[:, :4]
        class_scores = pred[:, 4:4 + n_cls]
        class_ids = np.argmax(class_scores, axis=1)
        scores = class_scores[np.arange(class_scores.shape[0]), class_ids]

        keep = scores >= self.conf_threshold
        if not np.any(keep):
            return [], infer_ms

        boxes = boxes[keep]
        scores = scores[keep]
        class_ids = class_ids[keep]

        cx = boxes[:, 0]
        cy = boxes[:, 1]
        bw = boxes[:, 2]
        bh = boxes[:, 3]
        x1 = (cx - bw / 2.0 - pad_x) / ratio
        y1 = (cy - bh / 2.0 - pad_y) / ratio
        x2 = (cx + bw / 2.0 - pad_x) / ratio
        y2 = (cy + bh / 2.0 - pad_y) / ratio

        h, w = image.shape[:2]
        x1 = np.clip(x1, 0, w - 1)
        y1 = np.clip(y1, 0, h - 1)
        x2 = np.clip(x2, 0, w - 1)
        y2 = np.clip(y2, 0, h - 1)

        nms_boxes = np.stack(
            [x1, y1, np.maximum(0.0, x2 - x1), np.maximum(0.0, y2 - y1)],
            axis=1).tolist()
        indices = cv2.dnn.NMSBoxes(
            nms_boxes,
            scores.tolist(),
            self.conf_threshold,
            self.iou_threshold)
        if indices is None or len(indices) == 0:
            return [], infer_ms
        if isinstance(indices, tuple):
            indices = indices[0]
        indices = np.asarray(indices).reshape(-1)

        detections = []
        for i in indices:
            cls_id = int(class_ids[i])
            detections.append({
                'label': COCO_CLASSES[cls_id],
                'class_id': cls_id,
                'score': float(scores[i]),
                'bbox': [float(x1[i]), float(y1[i]),
                         float(x2[i]), float(y2[i])],
            })
        return detections, infer_ms

    @staticmethod
    def _clone_detections(detections):
        return [{
            'label': d['label'],
            'class_id': d['class_id'],
            'score': d['score'],
            'bbox': list(d['bbox']),
        } for d in detections]

    @staticmethod
    def _letterbox(image, new_shape):
        shape = image.shape[:2]
        ratio = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        new_unpad = (int(round(shape[1] * ratio)),
                     int(round(shape[0] * ratio)))
        dw = (new_shape[1] - new_unpad[0]) / 2.0
        dh = (new_shape[0] - new_unpad[1]) / 2.0

        if (shape[1], shape[0]) != new_unpad:
            image = cv2.resize(
                image, new_unpad, interpolation=cv2.INTER_LINEAR)
        top = int(round(dh - 0.1))
        bottom = int(round(dh + 0.1))
        left = int(round(dw - 0.1))
        right = int(round(dw + 0.1))
        image = cv2.copyMakeBorder(
            image, top, bottom, left, right,
            cv2.BORDER_CONSTANT, value=(114, 114, 114))
        return image, ratio, left, top

    # ------------------------------------------------------------------ #
    # depth -> 3D
    # ------------------------------------------------------------------ #
    def _to_depth_meters(self, depth_raw):
        if depth_raw.dtype == np.uint16:
            return depth_raw.astype(np.float32) * self.depth_scale
        if depth_raw.dtype in (np.float32, np.float64):
            return depth_raw.astype(np.float32)
        return depth_raw.astype(np.float32) * self.depth_scale

    def _sample_depth(self, depth_m, x1, y1, x2, y2):
        h, w = depth_m.shape[:2]
        xi1 = max(0, min(w - 1, int(round(x1))))
        yi1 = max(0, min(h - 1, int(round(y1))))
        xi2 = max(0, min(w - 1, int(round(x2))))
        yi2 = max(0, min(h - 1, int(round(y2))))
        if xi2 <= xi1 or yi2 <= yi1:
            return None, None

        cx = (xi1 + xi2) // 2
        cy = (yi1 + yi2) // 2
        bw = xi2 - xi1
        bh = yi2 - yi1
        pw = max(self.depth_patch_min, int(round(bw * self.depth_patch_ratio)))
        ph = max(self.depth_patch_min, int(round(bh * self.depth_patch_ratio)))

        px1 = max(xi1, cx - pw // 2)
        px2 = min(xi2 + 1, cx + pw // 2 + 1)
        py1 = max(yi1, cy - ph // 2)
        py2 = min(yi2 + 1, cy + ph // 2 + 1)

        patch = depth_m[py1:py2, px1:px2]
        valid = patch[np.isfinite(patch)]
        valid = valid[(valid > 0.05) & (valid < self.max_depth)]
        if valid.size >= 3:
            return float(np.median(valid)), (float(cx), float(cy))

        roi = depth_m[yi1:yi2 + 1, xi1:xi2 + 1]
        valid = roi[np.isfinite(roi)]
        valid = valid[(valid > 0.05) & (valid < self.max_depth)]
        if valid.size >= 3:
            return float(np.median(valid)), (float(cx), float(cy))

        return None, None

    def _pixel_to_3d(self, u, v, z, width, height):
        """Back-project a pixel with the calibrated intrinsics.

        cv2.undistortPoints returns normalized ideal-pinhole coordinates,
        so distortion in calibration.yaml is applied here.
        """
        k = self._get_k_for_size(width, height)
        pts = np.array([[[float(u), float(v)]]], dtype=np.float64)
        undistorted = cv2.undistortPoints(pts, k, self.D)
        xn, yn = undistorted[0, 0]
        return (float(xn * z), float(yn * z), float(z))

    # ------------------------------------------------------------------ #
    # publications
    # ------------------------------------------------------------------ #
    def _publish_base_camera_tf(self):
        """Publish static base_link -> camera_link from T_cam2base.

        The hand-eye result is camera optical frame -> base.  The
        realsense driver already publishes camera_link ->
        camera_color_optical_frame, so we compose the inverse transforms
        to obtain base_link -> camera_link without creating a second
        parent for the optical frame.
        """
        if self._base_tf_published:
            return

        camera_frame = self._camera_frame
        T_link_optical = None
        source = ''
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.camera_link_frame,
                camera_frame,
                Time(),
                timeout=Duration(seconds=0.02))
            T_link_optical = transform_to_matrix(tf_msg)
            source = 'tf_static'
        except Exception as exc:  # noqa: BLE001
            elapsed = time.monotonic() - self._tf_attempt_start
            if elapsed < 5.0:
                return
            # Fallback: standard ROS camera_link -> optical rotation.
            T_link_optical = np.eye(4, dtype=np.float64)
            T_link_optical[:3, :3] = np.array([
                [0.0, 0.0, 1.0],
                [-1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
            ])
            if not self._tf_warned:
                self.get_logger().warn(
                    f'Could not look up {self.camera_link_frame} -> '
                    f'{camera_frame}: {exc}; using standard optical '
                    'rotation without color-sensor offset')
                self._tf_warned = True
            source = 'fallback'

        T_optical_link = invert_transform(T_link_optical)
        # T_cam2base already maps camera optical -> robot base.
        T_base_link = self.T_cam2base @ T_optical_link

        q = matrix_to_quat(T_base_link[:3, :3])
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.child_frame_id = self.camera_link_frame
        msg.transform.translation.x = float(T_base_link[0, 3])
        msg.transform.translation.y = float(T_base_link[1, 3])
        msg.transform.translation.z = float(T_base_link[2, 3])
        msg.transform.rotation.x = float(q[0])
        msg.transform.rotation.y = float(q[1])
        msg.transform.rotation.z = float(q[2])
        msg.transform.rotation.w = float(q[3])
        self.static_tf_broadcaster.sendTransform(msg)
        self._base_tf_published = True
        self.get_logger().info(
            f'published static TF {self.base_frame} -> '
            f'{self.camera_link_frame} '
            f'({source}, t=({T_base_link[0, 3]:.4f}, '
            f'{T_base_link[1, 3]:.4f}, {T_base_link[2, 3]:.4f}))')

    def _publish_object_tfs(self, header, detections):
        transforms = []
        for det in detections:
            xyz = det.get('xyz_base_m')
            if xyz is None:
                continue
            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame
            msg.child_frame_id = f'object_{det["index"]}'
            msg.transform.translation.x = float(xyz[0])
            msg.transform.translation.y = float(xyz[1])
            msg.transform.translation.z = float(xyz[2])
            msg.transform.rotation.x = 0.0
            msg.transform.rotation.y = 0.0
            msg.transform.rotation.z = 0.0
            msg.transform.rotation.w = 1.0
            transforms.append(msg)

        if transforms:
            self.object_tf_broadcaster.sendTransform(transforms)

    def _publish_detections(self, header, detections):
        k = self._active_k if self._active_k is not None else self.K
        fx, fy = k[0, 0], k[1, 1]

        cam_header = Header()
        cam_header.stamp = header.stamp
        cam_header.frame_id = header.frame_id

        base_header = Header()
        base_header.stamp = header.stamp
        base_header.frame_id = self.base_frame

        self.det3d_pub.publish(self._make_det3d_array(
            cam_header, detections, 'xyz_camera_m', fx, fy))
        self.det3d_base_pub.publish(self._make_det3d_array(
            base_header, detections, 'xyz_base_m', fx, fy))

    @staticmethod
    def _make_det3d_array(header, detections, coord_key, fx, fy):
        arr = Detection3DArray()
        arr.header = header

        for det in detections:
            xyz = det.get(coord_key)
            if xyz is None:
                continue
            x1, y1, x2, y2 = det['bbox']
            z = det['depth_m']

            pose = Pose()
            pose.position.x = xyz[0]
            pose.position.y = xyz[1]
            pose.position.z = xyz[2]
            pose.orientation.w = 1.0

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = det['label']
            hyp.hypothesis.score = float(det['score'])
            hyp.pose.pose = pose

            bbox = BoundingBox3D()
            bbox.center = pose
            bbox.size = Vector3(
                x=max(0.01, (x2 - x1) * z / fx),
                y=max(0.01, (y2 - y1) * z / fy),
                z=0.10)

            d = Detection3D()
            d.header = header
            d.id = str(det['index'])
            d.results = [hyp]
            d.bbox = bbox
            arr.detections.append(d)

        return arr

    def _publish_json(self, header, detections, image_shape, infer_ms, inferred):
        items = []
        for det in detections:
            items.append({
                'label': det['label'],
                'score': round(det['score'], 4),
                'bbox_xyxy': [round(v, 1) for v in det['bbox']],
                'center_uv': (
                    None if det['center_uv'] is None
                    else [round(v, 1) for v in det['center_uv']]),
                'depth_m': (
                    None if det['depth_m'] is None
                    else round(det['depth_m'], 4)),
                # xyz_m / distance_m are kept as camera-frame aliases
                # for compatibility with the previous version.
                'xyz_m': (
                    None if det['xyz_camera_m'] is None
                    else [round(v, 4) for v in det['xyz_camera_m']]),
                'distance_m': (
                    None if det['xyz_camera_m'] is None
                    else round(
                        float(np.linalg.norm(det['xyz_camera_m'])), 4)),
                'xyz_camera_m': (
                    None if det['xyz_camera_m'] is None
                    else [round(v, 4) for v in det['xyz_camera_m']]),
                'xyz_base_m': (
                    None if det['xyz_base_m'] is None
                    else [round(v, 4) for v in det['xyz_base_m']]),
                'distance_camera_m': (
                    None if det['xyz_camera_m'] is None
                    else round(
                        float(np.linalg.norm(det['xyz_camera_m'])), 4)),
                'distance_base_m': (
                    None if det['xyz_base_m'] is None
                    else round(
                        float(np.linalg.norm(det['xyz_base_m'])), 4)),
            })

        payload = {
            'stamp': {
                'sec': header.stamp.sec,
                'nanosec': header.stamp.nanosec,
            },
            'frame_id': header.frame_id,
            'base_frame': self.base_frame,
            'image': {
                'width': image_shape[1],
                'height': image_shape[0],
            },
            'inferred': inferred,
            'inference_ms': round(infer_ms, 2),
            'detections': items,
        }
        self.json_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))

    def _publish_markers(self, header, detections):
        self.marker_pub.publish(self._make_markers(
            header, detections, 'xyz_camera_m', offset_y=-0.05))

        base_header = Header()
        base_header.stamp = header.stamp
        base_header.frame_id = self.base_frame
        self.marker_base_pub.publish(self._make_markers(
            base_header, detections, 'xyz_base_m', offset_z=0.05))

    @staticmethod
    def _make_markers(header, detections, coord_key, offset_y=0.0, offset_z=0.0):
        markers = MarkerArray()
        delete = Marker()
        delete.header = header
        delete.ns = 'yolo'
        delete.id = 0
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)

        palette = [
            (1.0, 0.2, 0.2), (0.2, 1.0, 0.2), (0.2, 0.4, 1.0),
            (1.0, 0.8, 0.1), (1.0, 0.2, 1.0), (0.1, 1.0, 1.0),
        ]

        marker_id = 1
        for det in detections:
            xyz = det.get(coord_key)
            if xyz is None:
                continue
            color = palette[det['class_id'] % len(palette)]

            sphere = Marker()
            sphere.header = header
            sphere.ns = 'yolo'
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = xyz[0]
            sphere.pose.position.y = xyz[1]
            sphere.pose.position.z = xyz[2]
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.04
            sphere.scale.y = 0.04
            sphere.scale.z = 0.04
            sphere.color.r = color[0]
            sphere.color.g = color[1]
            sphere.color.b = color[2]
            sphere.color.a = 0.9
            markers.markers.append(sphere)

            label = Marker()
            label.header = header
            label.ns = 'yolo'
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = xyz[0]
            label.pose.position.y = xyz[1] + offset_y
            label.pose.position.z = xyz[2] + offset_z
            label.pose.orientation.w = 1.0
            label.scale.z = 0.05
            label.color.r = color[0]
            label.color.g = color[1]
            label.color.b = color[2]
            label.color.a = 1.0
            label.text = f"{det['label']} {det['score']:.2f} Z={xyz[2]:.2f}m"
            markers.markers.append(label)

        return markers

    # ------------------------------------------------------------------ #
    # drawing
    # ------------------------------------------------------------------ #
    def _draw_boxes(self, image, detections):
        palette = [
            (0, 0, 255), (0, 255, 0), (255, 0, 0),
            (0, 255, 255), (255, 0, 255), (255, 255, 0),
        ]
        for det in detections:
            x1, y1, x2, y2 = [int(round(v)) for v in det['bbox']]
            color = palette[det['class_id'] % len(palette)]
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

            if det['center_uv'] is not None:
                cu, cv = det['center_uv']
                cv2.circle(
                    image, (int(round(cu)), int(round(cv))),
                    5, color, -1)

            xyz_cam = det.get('xyz_camera_m')
            xyz_base = det.get('xyz_base_m')
            if xyz_cam is not None and xyz_base is not None:
                lines = [
                    f"{det['label']} {det['score']:.2f}",
                    f"cam=({xyz_cam[0]:+.2f},{xyz_cam[1]:+.2f},"
                    f"{xyz_cam[2]:+.2f})m",
                    f"base=({xyz_base[0]:+.2f},{xyz_base[1]:+.2f},"
                    f"{xyz_base[2]:+.2f})m",
                ]
            else:
                lines = [
                    f"{det['label']} {det['score']:.2f}",
                    "depth:N/A",
                ]

            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.40
            line_h = 14
            block_h = line_h * len(lines)
            text_w = max(
                cv2.getTextSize(line, font, font_scale, 1)[0][0]
                for line in lines)
            tx = int(min(max(0, x1), max(0, image.shape[1] - text_w - 2)))
            if y1 - block_h - 4 >= 0:
                ty = int(y1 - block_h - 2)
            else:
                ty = int(min(image.shape[0] - 4, y1 + 4))

            for i, line in enumerate(lines):
                ly = ty + i * line_h
                cv2.putText(
                    image, line, (tx, ly),
                    font, font_scale,
                    (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(
                    image, line, (tx, ly),
                    font, font_scale,
                    color, 1, cv2.LINE_AA)

    def _warn_throttle(self, text):
        now = time.monotonic()
        if now - self._last_warn >= 5.0:
            self.get_logger().warn(text)
            self._last_warn = now


def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
