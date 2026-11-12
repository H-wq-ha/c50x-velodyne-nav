"""ROS 2 RGB-D detector for COCO laptop, bottle, and cup objects."""
from __future__ import annotations

import json
import os
import time
from collections import deque

import numpy as np

from .core import (CLASS_NAMES, LAPTOP_CLASS_ID, SEMANTIC_CLASS_IDS,
                   SemanticMapStore, backproject_pixel,
                   letterbox_image, postprocess_yolov8, sample_depth_m,
                   transform_point, normalize_class_ids,
                   observation_class_fields)


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _matrix_from_transform(t):
    q = t.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]], dtype=float)
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = [t.translation.x,t.translation.y,t.translation.z]
    return T


def select_latest_ready_frame(frames, now_ns, max_age_s, is_tf_ready):
    """Return the newest non-expired frame index whose stamped TF is ready."""
    max_age_ns = int(float(max_age_s) * 1e9)
    selected = None
    for i, frame in enumerate(frames):
        stamp_ns = int(frame[0])
        if stamp_ns <= 0 or stamp_ns > now_ns or now_ns - stamp_ns > max_age_ns:
            continue
        if is_tf_ready(stamp_ns, frame):
            selected = i
    return selected


def main(args=None):
    import cv2
    cv2.setNumThreads(1)
    import onnxruntime as ort
    import rclpy
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                           qos_profile_sensor_data)
    from rclpy.time import Time
    from cv_bridge import CvBridge
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray
    import message_filters
    import tf2_ros
    from threading import Lock
    from collections import Counter

    class SemanticMappingNode(Node):
        def __init__(self):
            super().__init__('semantic_mapping')
            defaults = {
                'color_topic': '/camera/camera/color/image_raw',
                'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
                'model_path': '/home/y/arm_car/lidar/yolov8_d435i/models/yolov8n_256.onnx',
                'target_class_ids': list(SEMANTIC_CLASS_IDS),
                'map_frame': 'map', 'odom_frame': 'odom', 'map_id': 'default',
                'db_path': '~/.ros/semantic_map.sqlite3',
                'inference_hz': 2.0, 'confidence_threshold': .35,
                'iou_threshold': .45, 'depth_min_m': .15, 'depth_max_m': 6.0,
                'depth_window_px': 5, 'min_valid_depth_samples': 5,
                'depth_min_valid_ratio': .4, 'confirmations': 3,
                'association_gate_m': .35, 'sync_slop_s': .02,
                'max_frame_age_s': 2.0, 'pending_queue_size': 60, 'ort_threads': 2,
                'map_jump_translation_m': .75, 'map_jump_rotation_deg': 25.0,
            }
            for name, value in defaults.items(): self.declare_parameter(name, value)
            self.p = {k: self.get_parameter(k).value for k in defaults}
            self.target_class_ids = normalize_class_ids(self.p['target_class_ids'])
            self.class_fields = observation_class_fields(self.target_class_ids)
            if (self.p['inference_hz'] <= 0 or self.p['sync_slop_s'] <= 0 or
                    self.p['max_frame_age_s'] <= 0 or self.p['pending_queue_size'] < 1):
                raise ValueError('inference_hz, sync_slop_s, max_frame_age_s, and pending_queue_size must be positive')
            if self.p['ort_threads'] < 1: raise ValueError('ort_threads must be positive')
            model_path = os.path.expanduser(self.p['model_path'])
            if not os.path.isfile(model_path): raise FileNotFoundError(model_path)
            options = ort.SessionOptions()
            options.intra_op_num_threads = int(self.p['ort_threads'])
            options.inter_op_num_threads = 1
            self.session = ort.InferenceSession(model_path, sess_options=options,
                                                providers=['CPUExecutionProvider'])
            inp = self.session.get_inputs()[0]
            shape = inp.shape
            if len(shape) != 4 or shape[1] != 3:
                raise ValueError(f'expected NCHW RGB ONNX input with 3 channels, got {shape}')
            self.input_name = inp.name
            self.input_h = int(shape[2]) if isinstance(shape[2], int) else 256
            self.input_w = int(shape[3]) if isinstance(shape[3], int) else 256
            outshape = self.session.get_outputs()[0].shape
            if len(outshape) != 3 or 84 not in outshape[1:]:
                raise ValueError(f'expected COCO YOLO output with 84 channels, got {outshape}')
            self.output_name = self.session.get_outputs()[0].name
            self.store = SemanticMapStore(os.path.expanduser(self.p['db_path']), self.p['map_id'],
                int(self.p['confirmations']), float(self.p['association_gate_m']), self.p['map_frame'],
                active_class_ids=self.target_class_ids)
            self.bridge = CvBridge(); self.info = None; self.pending_pairs = deque()
            self.pending_limit = int(self.p['pending_queue_size'])
            self.buffer_lock = Lock(); self.last_processed_stamp_ns = 0
            self.pending_reported = set()
            self.first_info_logged = False; self.first_pair_logged = False
            self.last_map_odom_anchor = None; self.map_paused = False
            self.stats = Counter(); self.last_stats_log = time.monotonic()
            self.processing_group = MutuallyExclusiveCallbackGroup()
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            latched_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.observation_pub = self.create_publisher(String, '/semantic/laptop/observations', 10)
            self.objects_pub = self.create_publisher(String, '/semantic/laptop/objects', latched_qos)
            self.marker_pub = self.create_publisher(MarkerArray, '/semantic/laptop/markers', latched_qos)
            self.image_pub = self.create_publisher(Image, '/semantic/laptop/image', 2)
            self.info_sub = self.create_subscription(CameraInfo, self.p['camera_info_topic'],
                self._on_info, qos_profile_sensor_data)
            color = message_filters.Subscriber(self, Image, self.p['color_topic'],
                                               qos_profile=qos_profile_sensor_data)
            depth = message_filters.Subscriber(self, Image, self.p['depth_topic'],
                                               qos_profile=qos_profile_sensor_data)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [color, depth], 5, float(self.p['sync_slop_s']))
            self.sync.registerCallback(self._cache_pair)
            self.timer = self.create_timer(1.0 / float(self.p['inference_hz']),
                self._process_latest, callback_group=self.processing_group)
            # Publish the persistent map periodically even if the camera is silent.
            self.publish_timer = self.create_timer(1.0, self._publish_persistent_map,
                callback_group=self.processing_group)
            classes = ', '.join(f'{CLASS_NAMES[i]}={i}' for i in self.target_class_ids)
            self.get_logger().info(f"Loaded ONNX model {model_path}; target classes: {classes}; fixed map={self.p['map_frame']} id={self.p['map_id']}")

        def _on_info(self, msg):
            with self.buffer_lock:
                if self.info is None:
                    self.pending_reported.clear()
                self.info = msg
                self.stats['camera_info_received'] += 1
                log_first = not self.first_info_logged
                self.first_info_logged = True
            if log_first:
                self.get_logger().info(f"Received CameraInfo {msg.width}x{msg.height} frame={msg.header.frame_id}")

        def _cache_pair(self, color_msg, depth_msg):
            # Synchronizer callback only appends; inference and TF waits never run here.
            stamp = _stamp_ns(color_msg.header.stamp)
            with self.buffer_lock:
                if stamp <= self.last_processed_stamp_ns:
                    return
                if any(_stamp_ns(old[0].header.stamp) == stamp for old in self.pending_pairs):
                    return
                if len(self.pending_pairs) >= self.pending_limit:
                    dropped = self.pending_pairs.popleft()
                    dropped_ns = _stamp_ns(dropped[0].header.stamp)
                    self.pending_reported.discard(dropped_ns)
                    self.stats['pending_queue_overflow'] += 1
                self.pending_pairs.append((color_msg, depth_msg))
                self.stats['pairs_cached'] += 1
                log_first = not self.first_pair_logged
                self.first_pair_logged = True
            if log_first:
                self.get_logger().info(f"Cached first synchronized RGB-D pair stamp={stamp} frame={color_msg.header.frame_id}")

        def _publish_persistent_map(self):
            now_ns = self.get_clock().now().nanoseconds
            self._publish_objects(now_ns)
            self._publish_markers(now_ns)
            if time.monotonic() - self.last_stats_log >= 5.0:
                self.get_logger().info('semantic_stats ' + json.dumps(dict(self.stats), sort_keys=True))
                self.last_stats_log = time.monotonic()

        def _process_latest(self):
            now_ns = self.get_clock().now().nanoseconds
            with self.buffer_lock:
                info = self.info
                snapshot = list(self.pending_pairs)
                fresh = []
                expired_pairs = []
                for pair in snapshot:
                    stamp = _stamp_ns(pair[0].header.stamp)
                    if stamp <= 0:
                        expired_pairs.append((pair, 'zero_sensor_stamp'))
                    elif now_ns - stamp > float(self.p['max_frame_age_s']) * 1e9:
                        expired_pairs.append((pair, 'tf_expired'))
                    else:
                        fresh.append(pair)
                if expired_pairs:
                    expired_stamps = {_stamp_ns(p[0].header.stamp) for p, _ in expired_pairs}
                    self.pending_pairs = deque((p for p in self.pending_pairs
                                                if _stamp_ns(p[0].header.stamp) not in expired_stamps))
                    for stamp in expired_stamps: self.pending_reported.discard(stamp)
            ready_idx = None
            if info is not None:
                frames = [(_stamp_ns(p[0].header.stamp), p) for p in fresh]
                def tf_ready(stamp, frame_pair):
                    frame_id = frame_pair[1][0].header.frame_id
                    t = Time(nanoseconds=int(stamp))
                    return (self.tf_buffer.can_transform(self.p['map_frame'], frame_id, t) and
                            self.tf_buffer.can_transform(self.p['map_frame'], self.p['odom_frame'], t))
                ready_idx = select_latest_ready_frame(frames, now_ns,
                    float(self.p['max_frame_age_s']), tf_ready)
            pair = None
            pending_record = None
            if ready_idx is not None:
                pair = fresh[ready_idx]
                stamp = _stamp_ns(pair[0].header.stamp)
                # Discard superseded frames and consume the selected TF-ready pair.
                with self.buffer_lock:
                    current_stamps = [_stamp_ns(p[0].header.stamp) for p in self.pending_pairs]
                    try: current_idx = current_stamps.index(stamp)
                    except ValueError: current_idx = -1
                    if current_idx < 0:
                        pair = None
                    for _ in range(max(0, current_idx + 1)):
                        removed = self.pending_pairs.popleft()
                        self.pending_reported.discard(_stamp_ns(removed[0].header.stamp))
                    if pair is not None:
                        self.last_processed_stamp_ns = max(self.last_processed_stamp_ns, stamp)
            elif fresh:
                newest = fresh[-1]
                stamp = _stamp_ns(newest[0].header.stamp)
                with self.buffer_lock:
                    if stamp not in self.pending_reported:
                        pending_record = {'stamp_ns':stamp, 'frame_id':newest[0].header.frame_id,
                            **self.class_fields,
                            'detections':[],
                            'status':'tf_pending' if info is not None else 'camera_info_pending',
                            'map_id':self.p['map_id']}
                        self.pending_reported.add(stamp)
                        self.stats['tf_pending' if info is not None else 'camera_info_pending'] += 1
            for expired_pair, reason in expired_pairs:
                expired_stamp = _stamp_ns(expired_pair[0].header.stamp)
                self.stats[reason] += 1
                self._publish_observation({'stamp_ns':expired_stamp,
                    'frame_id':expired_pair[0].header.frame_id,
                    **self.class_fields, 'detections':[],
                    'status':reason,'map_id':self.p['map_id']})
            if pending_record is not None: self._publish_observation(pending_record)
            if pair is None: return
            color_msg, depth_msg = pair
            stamp_ns = _stamp_ns(color_msg.header.stamp)
            frame = color_msg.header.frame_id
            record = {'stamp_ns': stamp_ns, 'frame_id': frame,
                      **self.class_fields,
                      'detections': [], 'status': 'camera_only', 'map_id': self.p['map_id']}
            try:
                depth_stamp_ns = _stamp_ns(depth_msg.header.stamp)
                info_stamp_ns = _stamp_ns(info.header.stamp) if info is not None else 0
                age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
                if age_s < 0 or age_s > float(self.p['max_frame_age_s']):
                    record['status'] = 'stale_or_future_frame'; record['age_s'] = age_s
                    self._reject(record['status'], record); return
                if depth_stamp_ns == 0 or info_stamp_ns == 0:
                    record['status'] = 'zero_sensor_stamp'; self._reject(record['status'], record); return
                if (not frame or depth_msg.header.frame_id != frame or info.header.frame_id != frame):
                    record['status'] = 'sensor_frame_mismatch'
                    record['frames'] = [frame, depth_msg.header.frame_id, info.header.frame_id]
                    self._reject(record['status'], record); return
                if abs(stamp_ns-depth_stamp_ns) / 1e9 > float(self.p['sync_slop_s']):
                    record['status'] = 'depth_stamp_mismatch'; self._reject(record['status'], record); return
                image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
                depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
                if info.width != image.shape[1] or info.height != image.shape[0] or depth.shape[:2] != image.shape[:2]:
                    record['status'] = 'image_size_mismatch'; self._reject(record['status'], record); return
                boxed, transform = letterbox_image(image, (self.input_w, self.input_h))
                tensor = cv2.cvtColor(boxed, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.
                tensor = np.transpose(tensor, (2,0,1))[None]
                output = self.session.run([self.output_name], {self.input_name: tensor})[0]
                detections = postprocess_yolov8(output, transform,
                    float(self.p['confidence_threshold']), float(self.p['iou_threshold']),
                    class_ids=self.target_class_ids)
                try:
                    tf_msg = self.tf_buffer.lookup_transform(self.p['map_frame'], frame,
                        Time(nanoseconds=stamp_ns), timeout=rclpy.duration.Duration(seconds=0.0))
                    T_map_camera = _matrix_from_transform(tf_msg.transform)
                    tf_ok = True
                except Exception as exc:
                    tf_ok = False; record['tf_error'] = str(exc); self.stats['tf_lookup_errors'] += 1
                if tf_ok:
                    try:
                        odom_msg = self.tf_buffer.lookup_transform(self.p['map_frame'], self.p['odom_frame'],
                            Time(nanoseconds=stamp_ns), timeout=rclpy.duration.Duration(seconds=0.0))
                        T_map_odom = _matrix_from_transform(odom_msg.transform)
                        if self.last_map_odom_anchor is None: self.last_map_odom_anchor = T_map_odom.copy()
                        if not self.map_paused:
                            delta_t = np.linalg.norm(T_map_odom[:3,3] - self.last_map_odom_anchor[:3,3])
                            R_delta = self.last_map_odom_anchor[:3,:3].T @ T_map_odom[:3,:3]
                            delta_angle = np.degrees(np.arccos(np.clip((np.trace(R_delta)-1)/2, -1, 1)))
                            if (delta_t > float(self.p['map_jump_translation_m']) or
                                delta_angle > float(self.p['map_jump_rotation_deg'])):
                                self.map_paused = True
                                record['map_pause_reason'] = {'reason':'map_to_odom_jump_from_session_anchor',
                                    'translation_m':float(delta_t),'rotation_deg':float(delta_angle)}
                    except Exception as exc:
                        record['map_pause_reason'] = {'reason':'map_to_odom_unavailable','detail':str(exc)}
                        tf_ok = False; self.stats['map_odom_lookup_errors'] += 1
                for det_index, detection in enumerate(detections):
                    x1,y1,x2,y2 = detection['box']; u=(x1+x2)*.5; v=(y1+y2)*.5
                    z, quality = sample_depth_m(depth, u, v, depth_msg.encoding,
                        int(self.p['depth_window_px']), float(self.p['depth_min_m']), float(self.p['depth_max_m']),
                        int(self.p['min_valid_depth_samples']), float(self.p['depth_min_valid_ratio']))
                    item = {**detection, 'pixel': [u,v], 'depth': quality}
                    if z is not None:
                        xyz = backproject_pixel(u,v,z,[info.k[0],info.k[4],info.k[2],info.k[5]],
                                                info.d,info.distortion_model)
                        item['camera_xyz'] = xyz.tolist()
                        if tf_ok and not self.map_paused:
                            map_xyz = transform_point(T_map_camera, xyz)
                            item['map_xyz'] = map_xyz.tolist(); item['status'] = 'mapped'
                            item['fusion'] = self.store.add(stamp_ns, self.p['map_frame'], map_xyz,
                                detection['score'], detection['class_id'],
                                {'bbox': detection['box'], 'pixel': [u,v], 'camera_xyz':xyz.tolist(),
                                 'source_frame':frame,'depth_quality':quality,'capture_stamp_ns':stamp_ns},
                                detection_index=det_index)
                        else:
                            item['status'] = ('map_jump_pause_map_not_stored' if self.map_paused
                                              else 'tf_unavailable_map_not_stored')
                            self.stats['map_write_rejected'] += 1
                    else:
                        item['status'] = 'invalid_depth_map_not_stored'
                        self.stats['invalid_depth'] += 1
                    record['detections'].append(item)
                record['status'] = ('map_jump_pause' if self.map_paused else
                                    ('ok' if tf_ok else 'tf_unavailable_camera_observations_only'))
                self.stats['frames_processed'] += 1
                self.stats['tf_ready_selected'] += 1
                self._publish_observation(record)
                self._publish_objects(stamp_ns); self._publish_markers(stamp_ns)
                overlay = image.copy()
                colors = {63: (0, 220, 0), 39: (0, 165, 255), 41: (255, 128, 0)}
                for d in detections:
                    color = colors[d['class_id']]
                    x1,y1,x2,y2 = map(int,d['box']); cv2.rectangle(overlay,(x1,y1),(x2,y2),color,2)
                    label = CLASS_NAMES[d['class_id']]
                    cv2.putText(overlay,f"{label} {d['score']:.2f}",(x1,max(15,y1-5)),cv2.FONT_HERSHEY_SIMPLEX,.5,color,1)
                out = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8'); out.header = color_msg.header
                self.image_pub.publish(out)
            except Exception as exc:
                # A SIGINT may shut down the ROS context while an inference
                # callback is finishing. Do not turn that normal exit race
                # into a second failed publish/unhandled callback exception.
                if not self.context.ok():
                    return
                record['status'] = 'processing_error'; record['error'] = str(exc)
                self.stats['processing_errors'] += 1
                self.get_logger().error(f"Frame processing failed: {exc}")
                self._publish_observation(record)

        def _reject(self, status, record):
            self.stats[status] += 1
            self._publish_observation(record)

        def _publish_observation(self, record):
            msg = String(); msg.data = json.dumps(record, separators=(',',':')); self.observation_pub.publish(msg)

        def _publish_objects(self, stamp_ns):
            msg = String(); msg.data = json.dumps({'map_id': self.p['map_id'], 'stamp_ns': int(stamp_ns),
                'objects': self.store.objects(),
                'visualization': {'cube_scale_m':[.35,.25,.08],
                                  'represents_measured_object_size':False}},
                separators=(',',':')); self.objects_pub.publish(msg)

        def _publish_markers(self, stamp_ns):
            array = MarkerArray()
            for obj in self.store.objects(confirmed_only=True):
                x,y,z = obj['position']
                point = Marker(); point.header.frame_id = self.p['map_frame']
                point.header.stamp.sec, point.header.stamp.nanosec = divmod(int(stamp_ns), 1_000_000_000)
                label = CLASS_NAMES[obj['class_id']]
                point.ns=f'{label}_grid'; point.id=int(obj['object_id'])*2; point.type=Marker.SPHERE
                point.action=Marker.ADD; point.pose.position.x=x; point.pose.position.y=y; point.pose.position.z=.05
                point.pose.orientation.w=1.; point.scale.x=point.scale.y=point.scale.z=.12
                point.color.r=.1; point.color.g=.9; point.color.b=.2; point.color.a=1.; array.markers.append(point)
                marker = Marker(); marker.header = point.header; marker.ns=f'{label}_3d'; marker.id=int(obj['object_id'])*2+1
                marker.type=Marker.CUBE; marker.action=Marker.ADD
                marker.pose.position.x=x; marker.pose.position.y=y; marker.pose.position.z=z
                # This cube is a visibility aid; dimensions are not estimated from the image.
                marker.pose.orientation.w=1.; marker.scale.x=.35; marker.scale.y=.25; marker.scale.z=.08
                marker.color.r=.15; marker.color.g=.75; marker.color.b=1.; marker.color.a=.45
                array.markers.append(marker)
                text = Marker(); text.header=point.header; text.ns=f'{label}_labels'; text.id=int(obj['object_id'])
                text.type=Marker.TEXT_VIEW_FACING; text.action=Marker.ADD
                text.pose.position.x=x; text.pose.position.y=y; text.pose.position.z=z+.18
                text.pose.orientation.w=1.; text.scale.z=.13; text.color.r=text.color.g=text.color.b=text.color.a=1.
                text.text=f"{label} {obj['object_id']} ({x:.2f}, {y:.2f}) m"; array.markers.append(text)
            self.marker_pub.publish(array)

    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = SemanticMappingNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        try:
            executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    finally:
        if executor is not None:
            try: executor.shutdown()
            except Exception: pass
        if node is not None:
            try: node.store.close()
            except Exception as exc: node.get_logger().error(f"SQLite cleanup failed: {exc}")
            try: node.destroy_node()
            except Exception: pass
        try: rclpy.shutdown()
        except Exception: pass
