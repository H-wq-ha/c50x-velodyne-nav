"""Navigate to a saved semantic object while keeping it inside the camera view."""

import json
import math
import os
import sqlite3
import threading
import time
from collections import OrderedDict
from lifecycle_msgs.srv import GetState

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from semantic_mapping.semantic_task_protocol import (
    CLASS_LABELS, ProtocolError, decode_request, encode_response,
    new_server_session_id, task_status,
)


ALIASES = {'bottle': 39, 'water bottle': 39, '瓶子': 39, '水瓶': 39, '矿泉水瓶': 39, 'cup': 41, '杯子': 41,
           'laptop': 63, '笔记本': 63}
CLASS_LABELS = {39: 'bottle', 41: 'cup', 63: 'laptop'}


class TargetReplaced(Exception):
    """Raised when a newer semantic target supersedes the active request."""


def relative_target(robot, target_xyz, *, frame='base_link'):
    """Express a map point in fresh planar robot coordinates (+X forward, +Y left)."""
    if (not robot or not robot.get('fresh') or
            any(not isinstance(robot.get(key), (int, float)) or
                not math.isfinite(robot[key]) for key in ('x', 'y', 'z', 'yaw')) or
            any(not isinstance(value, (int, float)) or not math.isfinite(value)
                for value in target_xyz)):
        return {'available': False, 'frame': frame, 'x': None, 'y': None, 'z': None,
                'distance_m': None, 'bearing_rad': None,
                'tf_stamp': (robot or {}).get('tf_stamp'),
                'age_s': (robot or {}).get('age_s'),
                'reason': (robot or {}).get('reason') or 'localization_tf_stale'}
    dx, dy = target_xyz[0]-robot['x'], target_xyz[1]-robot['y']
    c, s = math.cos(robot['yaw']), math.sin(robot['yaw'])
    rx, ry = c*dx+s*dy, -s*dx+c*dy
    return {'available': True, 'frame': frame, 'x': rx, 'y': ry,
            'z': target_xyz[2]-robot['z'], 'distance_m': math.hypot(rx, ry),
            'bearing_rad': math.atan2(ry, rx),
            'tf_stamp': robot['tf_stamp'], 'age_s': robot['age_s'], 'reason': None}


def _quat_matrix(q):
    x, y, z, w = q
    return (
        (1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)),
        (2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)),
        (2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)),
    )


def _mat_vec(m, v):
    return tuple(sum(m[i][j] * v[j] for j in range(3)) for i in range(3))


def target_in_camera(target_map, base_xy, yaw, base_to_camera):
    """Project a map point into the optical camera for a planar base pose."""
    tr, q = base_to_camera
    c, s = math.cos(yaw), math.sin(yaw)
    cam_xy = (base_xy[0] + c*tr[0] - s*tr[1],
              base_xy[1] + s*tr[0] + c*tr[1])
    cam_z = tr[2]
    r_bc = _quat_matrix(q)
    r_yaw = ((c, -s, 0), (s, c, 0), (0, 0, 1))
    r_mc = tuple(tuple(sum(r_yaw[i][k]*r_bc[k][j] for k in range(3))
                       for j in range(3)) for i in range(3))
    delta = (target_map[0] - cam_xy[0], target_map[1] - cam_xy[1],
             target_map[2] - cam_z)
    # R is orthonormal, so camera coordinates are R^T * (target - camera).
    pc = tuple(sum(r_mc[j][i] * delta[j] for j in range(3)) for i in range(3))
    return pc


def camera_view_score(pc, camera_info, margin=0.10):
    pixel = project_camera_point(pc, camera_info)
    if pixel is None:
        return None
    u, v = pixel
    width, height = camera_info.width, camera_info.height
    if not (margin*width <= u <= (1-margin)*width and
            margin*height <= v <= (1-margin)*height):
        return None
    return ((u-width/2)/(width/2))**2 + ((v-height/2)/(height/2))**2


def project_camera_point(pc, camera_info):
    if pc[2] <= 0.10:
        return None
    if camera_info is None or camera_info.width <= 0 or camera_info.height <= 0:
        return None
    x, y = pc[0]/pc[2], pc[1]/pc[2]
    coeffs = list(camera_info.d) + [0.0] * max(0, 5-len(camera_info.d))
    if camera_info.distortion_model == 'equidistant':
        radius = math.hypot(x, y)
        if radius > 1e-12:
            theta = math.atan(radius)
            theta2 = theta*theta
            theta_d = theta * (1 + coeffs[0]*theta2 + coeffs[1]*theta2**2 +
                               coeffs[2]*theta2**3 + coeffs[3]*theta2**4)
            x, y = x*theta_d/radius, y*theta_d/radius
    else:
        k1, k2, p1, p2, k3 = coeffs[:5]
        r2 = x*x + y*y
        radial = 1 + k1*r2 + k2*r2*r2 + k3*r2*r2*r2
        x, y = (x*radial + 2*p1*x*y + p2*(r2 + 2*x*x),
                y*radial + p1*(r2 + 2*y*y) + 2*p2*x*y)
    k = camera_info.k
    u, v = k[0]*x + k[1]*y + k[2], k[4]*y + k[5]
    return u, v


def resolve_target(rows, command):
    """Resolve one confirmed object row from an English or Chinese label."""
    command = command.strip()
    confirmed = [row for row in rows if row[5] >= 3]
    if ':' in command:
        name, raw_id = command.rsplit(':', 1)
        if not raw_id.isdigit():
            raise RuntimeError('target syntax is name or name:object_id')
        object_id = int(raw_id)
        class_id = ALIASES.get(name.strip().lower())
        choices = [row for row in confirmed if row[0] == object_id and row[1] == class_id]
    else:
        class_id = ALIASES.get(command.lower())
        if class_id is None:
            raise RuntimeError(f'unknown target {command!r}; use bottle, cup, laptop or Chinese aliases')
        choices = [row for row in confirmed if row[1] == class_id]
    if not choices:
        raise RuntimeError(f'no confirmed saved object matches {command!r}')
    if len(choices) != 1:
        ids = ', '.join(str(row[0]) for row in choices)
        raise RuntimeError(f'ambiguous {command!r}; specify object_id from [{ids}]')
    return choices[0]


class SemanticNavigator(Node):
    def __init__(self):
        super().__init__('semantic_navigator')
        self.declare_parameter('semantic_db', '')
        self.declare_parameter('map_id', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('static_map_topic', '/static_map')
        self.declare_parameter('standoff_min', 0.9)
        self.declare_parameter('standoff_max', 2.5)
        self.declare_parameter('camera_margin', 0.10)
        self.declare_parameter('enable_chassis_control', False)
        self.declare_parameter('validation_duration_s', 3.0)
        self.declare_parameter('navigation_timeout_s', 300.0)
        self.db_path = self.get_parameter('semantic_db').value
        self.map_id = self.get_parameter('map_id').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.static_map_topic = self.get_parameter('static_map_topic').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_msg = None
        self.camera_info = None
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, self.static_map_topic, self._map_cb, map_qos)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self._camera_info_cb,
            qos_profile_sensor_data)
        self.target_sub = self.create_subscription(String, '/semantic_nav/target', self._target_cb, 10)
        latched_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/semantic_nav/status', latched_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic_nav/objects', latched_qos)
        self.path_pub = self.create_publisher(Path, '/semantic_nav/planned_path', latched_qos)
        self.camera_view_pub = self.create_publisher(
            MarkerArray, '/semantic_nav/camera_view', latched_qos)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self._nonzero_cmd_count = 0
        self.plan_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.lifecycle_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in ('planner_server', 'controller_server', 'bt_navigator', 'velocity_smoother')}
        self._busy = False
        self._target_worker_active = False
        self._target_lock = threading.Lock()
        self._pending_target = None
        self._active_target_key = None
        self._pending_target_key = None
        self._target_cancel_event = threading.Event()
        self._replacement_safe = True
        self._active_nav_handle = None
        self._terminal_watch_handles = set()
        self._server_session_id = new_server_session_id()
        self._semantic_task_lock = threading.RLock()
        self._semantic_tasks = OrderedDict()
        self._semantic_request_cache = OrderedDict()
        self._semantic_request_fingerprints = OrderedDict()
        self._semantic_task_seq = 0
        self._structured_task = None
        self._preview_slots = threading.BoundedSemaphore(1)
        self._preview_busy = threading.Event()
        self._preview_cancel_event = None
        self.semantic_response_pub = self.create_publisher(String, '/semantic_nav/response', 20)
        self.semantic_task_status_pub = self.create_publisher(
            String, '/semantic_nav/task_status', latched_qos)
        self.semantic_request_sub = self.create_subscription(
            String, '/semantic_nav/request', self._semantic_request_cb, 20)
        self.semantic_heartbeat_timer = self.create_timer(2.0, self._semantic_heartbeat)
        self._publish_objects()
        self.marker_timer = self.create_timer(2.0, self._publish_objects)
        self.get_logger().info(
            'Ready: publish bottle/cup/laptop or 瓶子/杯子/笔记本 to /semantic_nav/target')

    def _map_cb(self, msg):
        self.map_msg = msg

    def _camera_info_cb(self, msg):
        self.camera_info = msg

    def _cmd_vel_cb(self, msg):
        if abs(msg.linear.x) > 1e-4 or abs(msg.linear.y) > 1e-4 or abs(msg.angular.z) > 1e-4:
            self._nonzero_cmd_count += 1

    def _say(self, text):
        if not rclpy.ok():
            return
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def _db_objects(self, *, include_timestamps=False):
        if not os.path.isfile(self.db_path):
            raise RuntimeError('semantic database is missing')
        uri = 'file:' + self.db_path + '?mode=ro&immutable=1'
        with sqlite3.connect(uri, uri=True) as db:
            meta = dict(db.execute('SELECT key,value FROM metadata'))
            if meta.get('map_id') != self.map_id:
                raise RuntimeError('semantic database map_id does not match selected map')
            columns = ('object_id,class_id,x,y,z,count,first_stamp_ns,last_stamp_ns'
                       if include_timestamps else 'object_id,class_id,x,y,z,count')
            return db.execute(
                f'SELECT {columns} FROM objects WHERE count >= 3 ORDER BY object_id'
            ).fetchall()

    def _robot_pose_context(self, *, max_age=1.0):
        checked_at = time.time()
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.0))
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            now_ros = self.get_clock().now().nanoseconds * 1e-9
            age = now_ros - stamp
            tr, q = tf.transform.translation, tf.transform.rotation
            q_norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            numeric = (tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w, q_norm)
            if (not all(math.isfinite(value) for value in numeric) or q_norm < 1e-8):
                return {"available": True, "frame": self.map_frame,
                        "child_frame": self.base_frame, "x": None, "y": None,
                        "z": None, "yaw": None, "tf_stamp": stamp if stamp > 0 else None,
                        "age_s": age if math.isfinite(age) else None, "fresh": False,
                        "reason": "localization_tf_invalid", "checked_at": checked_at}
            qx, qy, qz, qw = q.x/q_norm, q.y/q_norm, q.z/q_norm, q.w/q_norm
            yaw = math.atan2(2 * (qw*qz + qx*qy),
                             1 - 2 * (qy*qy + qz*qz))
            fresh = bool(math.isfinite(stamp) and stamp > 0 and math.isfinite(age)
                         and -1.0 <= age <= max_age)
            return {"available": True, "frame": self.map_frame,
                    "child_frame": self.base_frame,
                    "x": tr.x if fresh else None, "y": tr.y if fresh else None,
                    "z": tr.z if fresh else None, "yaw": yaw if fresh else None,
                    "tf_stamp": stamp if stamp > 0 else None,
                    "age_s": age if math.isfinite(age) else None, "fresh": fresh,
                    "reason": None if fresh else "localization_tf_stale",
                    "checked_at": checked_at}
        except Exception as exc:
            return {"available": False, "frame": self.map_frame,
                    "child_frame": self.base_frame, "x": None, "y": None,
                    "z": None, "yaw": None, "tf_stamp": None, "age_s": None,
                    "fresh": False, "reason": f"localization_tf_unavailable:{type(exc).__name__}",
                    "checked_at": checked_at}

    def _semantic_context(self, active=None):
        m = self.map_msg
        if m is None:
            static_map = {"received": False, "frame": None, "width": None, "height": None,
                          "resolution": None, "origin": None,
                          "reason": f"waiting_for:{self.static_map_topic}"}
        else:
            origin = m.info.origin
            q = origin.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                             1 - 2*(q.y*q.y + q.z*q.z))
            static_map = {"received": True, "frame": m.header.frame_id,
                          "width": m.info.width,
                          "height": m.info.height, "resolution": m.info.resolution,
                          "origin": {"x": origin.position.x, "y": origin.position.y,
                                     "yaw": yaw}, "reason": None}
        nav2 = {}
        for name, client in self.lifecycle_clients.items():
            ready = bool(client.service_is_ready())
            nav2[name] = {"available": ready, "state_service_discoverable": ready,
                          "active": None,
                          "state": "unknown" if ready else "unavailable"}
        nav2["compute_path_to_pose"] = {
            "available": bool(self.plan_client.server_is_ready()), "active": None}
        nav2["navigate_to_pose"] = {
            "available": bool(self.nav_client.server_is_ready()), "active": None}
        active_task = self._public_task(active)
        return {"map_id": self.map_id, "server_session_id": self._server_session_id,
                "map_frame": self.map_frame, "checked_at": time.time(),
                "enable_chassis_control": bool(
                    self.get_parameter('enable_chassis_control').value),
                "static_map": static_map, "robot": self._robot_pose_context(),
                "camera_info": {"available": self.camera_info is not None,
                                "calibrated": bool(
                                    self.camera_info is not None and
                                    self.camera_info.width > 0 and
                                    self.camera_info.height > 0 and
                                    any(self.camera_info.k)),
                                "frame": (self.camera_info.header.frame_id
                                          if self.camera_info else None),
                                "width": self.camera_info.width if self.camera_info else None,
                                "height": self.camera_info.height if self.camera_info else None},
                "nav2": nav2, "active_task": active_task,
                "object_attributes": {"color": None, "appearance_name": None,
                                      "room": None,
                                      "note": "目标数据库不记录这些属性；观测时间不代表当前仍在原位。"},
                "base_convention": {"x": "forward", "y": "left",
                                    "bearing": "atan2(relative_y,relative_x), positive counterclockwise"}}

    def _map_id(self):
        if not os.path.isfile(self.db_path):
            raise RuntimeError('semantic database is missing')
        uri = 'file:' + self.db_path + '?mode=ro&immutable=1'
        with sqlite3.connect(uri, uri=True) as db:
            meta = dict(db.execute('SELECT key,value FROM metadata'))
        if meta.get('map_id') != self.map_id:
            raise RuntimeError('semantic database map_id does not match selected map')
        return self.map_id

    def _emit_task_status(self, record, state=None, reason_code=None, message=None):
        if state is not None:
            record['state'] = state
        if reason_code is not None:
            record['reason_code'] = reason_code
        if message is not None:
            record['message'] = str(message)[:240]
        with self._semantic_task_lock:
            self._semantic_task_seq += 1
            record['seq'] = self._semantic_task_seq
            record['timestamp'] = time.time()
            self._semantic_tasks.move_to_end(record['task_id'])
            payload = task_status(
                client_id=record['client_id'], task_id=record['task_id'], map_id=record['map_id'],
                server_session_id=self._server_session_id, state=record['state'],
                reason_code=record.get('reason_code'), target=record['target'],
                seq=record['seq'], timestamp=record['timestamp'],
                message=record.get('message'))
        self.semantic_task_status_pub.publish(String(data=json.dumps(
            payload, ensure_ascii=False, allow_nan=False, separators=(',', ':'))))

    def _semantic_heartbeat(self):
        record = self._structured_task
        if record is not None and record['state'] in ('planning', 'navigating', 'canceling'):
            self._emit_task_status(record)

    def _cache_semantic_response(self, request_id, fingerprint, raw):
        with self._semantic_task_lock:
            self._semantic_request_cache[request_id] = (fingerprint, raw)
            self._semantic_request_cache.move_to_end(request_id)
            while len(self._semantic_request_cache) > 256:
                self._semantic_request_cache.popitem(last=False)
        self.semantic_response_pub.publish(String(data=raw))

    def _stamp_semantic_response(self, raw, request):
        response = json.loads(raw)
        response.update({
            'op': request['op'], 'server_session_id': self._server_session_id,
            'map_id': self.map_id, 'client_id': request.get('client_id'),
            'task_id': request.get('task_id'),
        })
        return json.dumps(response, ensure_ascii=False, allow_nan=False,
                          separators=(',', ':'))

    def _semantic_request_cb(self, message):
        try:
            request = decode_request(message.data)
        except ProtocolError as exc:
            # A malformed request may not contain a trustworthy correlation ID.
            self.semantic_response_pub.publish(String(data=encode_response(
                None, ok=False, reason_code=str(exc))))
            return
        request_id = request['request_id']
        fingerprint = json.dumps(request, sort_keys=True, separators=(',', ':'))
        with self._semantic_task_lock:
            cached = self._semantic_request_cache.get(request_id)
            prior_fingerprint = self._semantic_request_fingerprints.get(request_id)
        if cached is not None:
            previous_fingerprint, previous_response = cached
            if previous_fingerprint != fingerprint:
                previous_response = self._stamp_semantic_response(encode_response(
                    request_id, ok=False, reason_code='request_id_conflict'), request)
            self.semantic_response_pub.publish(String(data=previous_response))
            return
        if prior_fingerprint is not None:
            reason = ('request_id_conflict' if prior_fingerprint != fingerprint
                      else 'request_in_progress')
            self.semantic_response_pub.publish(String(data=self._stamp_semantic_response(
                encode_response(request_id, ok=False, reason_code=reason), request)))
            return
        with self._semantic_task_lock:
            self._semantic_request_fingerprints[request_id] = fingerprint
            self._semantic_request_fingerprints.move_to_end(request_id)
            while len(self._semantic_request_fingerprints) > 256:
                self._semantic_request_fingerprints.popitem(last=False)
        try:
            if request['op'] == 'list_objects':
                map_id = self._map_id()
                rows = [row for row in self._db_objects(include_timestamps=True)
                        if row[1] in CLASS_LABELS]
                with self._semantic_task_lock:
                    active = dict(self._structured_task) if self._structured_task else None
                limit = request.get('limit', 100)
                offset = request.get('offset', 0)
                page = rows[offset:offset + limit]
                robot = self._robot_pose_context()
                objects = []
                for row in page:
                    obj = {'class_label': CLASS_LABELS.get(row[1], str(row[1])),
                           'class_id': row[1], 'object_id': row[0], 'x': row[2],
                           'y': row[3], 'z': row[4], 'count': row[5],
                           'first_stamp_ns': row[6], 'last_stamp_ns': row[7]}
                    obj['relative'] = relative_target(
                        robot, (row[2], row[3], row[4]), frame=self.base_frame)
                    objects.append(obj)
                context = self._semantic_context(active=active)
                context['robot'] = robot
                result = {
                    'map_id': map_id, 'server_session_id': self._server_session_id,
                    'context': context, 'objects': objects,
                    'pagination': {'offset': offset, 'limit': limit,
                                   'returned': len(objects), 'total': len(rows),
                                   'truncated': offset + len(objects) < len(rows),
                                   'next_offset': (offset + len(objects)
                                                   if offset + len(objects) < len(rows)
                                                   else None)},
                    'active_task': self._public_task(active),
                }
                raw = encode_response(request_id, ok=True, result=result)
                # Keep each response comfortably below the transport's bounded
                # payload size; pagination reports every omitted object.
                while len(raw.encode('utf-8')) > 96 * 1024 and objects:
                    objects.pop()
                    result['pagination']['returned'] = len(objects)
                    result['pagination']['truncated'] = offset + len(objects) < len(rows)
                    result['pagination']['next_offset'] = (
                        offset + len(objects) if offset + len(objects) < len(rows) else None)
                    raw = encode_response(request_id, ok=True, result=result)
            elif request['op'] == 'preview_navigation':
                if (request['server_session_id'] != self._server_session_id or
                        request['map_id'] != self._map_id()):
                    raw = encode_response(request_id, ok=False,
                                          reason_code='map_or_session_mismatch')
                elif self._busy or self._structured_task is not None:
                    raw = encode_response(request_id, ok=True, result={
                        'status': 'unknown', 'reachable': None,
                        'reason_code': 'preview_unavailable_busy',
                        'message': '已有导航任务，预检未运行以避免占用规划器。',
                        'checked_at': time.time()})
                else:
                    self._start_semantic_preview(request)
                    return
            elif request['op'] == 'start':
                raw = self._semantic_start(request)
            elif request['op'] == 'status':
                raw = self._semantic_status(request)
            else:
                raw = self._semantic_cancel(request)
        except Exception as exc:
            raw = encode_response(request_id, ok=False,
                                  reason_code='server_error', message=exc)
        self._cache_semantic_response(
            request_id, fingerprint, self._stamp_semantic_response(raw, request))

    def _start_semantic_preview(self, request):
        if not self._preview_slots.acquire(blocking=False):
            raw = encode_response(request['request_id'], ok=True, result={
                'status': 'unknown', 'reachable': None,
                'reason_code': 'preview_busy',
                'message': '另一个预检正在运行。', 'checked_at': time.time()})
            self._cache_semantic_response(
                request['request_id'],
                json.dumps(request, sort_keys=True, separators=(',', ':')),
                self._stamp_semantic_response(raw, request))
            return
        self._preview_busy.set()
        cancel_event = threading.Event()
        self._preview_cancel_event = cancel_event
        hold_slot = [False]
        release_lock = threading.Lock()
        released = [False]

        def release_slot(*_args):
            with release_lock:
                if released[0]:
                    return
                released[0] = True
            self._preview_busy.clear()
            if self._preview_cancel_event is cancel_event:
                self._preview_cancel_event = None
            self._preview_slots.release()

        def run():
            try:
                result = self._preview_navigation(
                    request, cancel_event=cancel_event, hold_slot=hold_slot,
                    release_slot=release_slot)
                raw = encode_response(request['request_id'], ok=True, result=result)
                self._cache_semantic_response(
                    request['request_id'],
                    json.dumps(request, sort_keys=True, separators=(',', ':')),
                    self._stamp_semantic_response(raw, request))
            except Exception as exc:
                raw = encode_response(request['request_id'], ok=True, result={
                    'status': 'unknown', 'reachable': None,
                    'reason_code': 'preview_error', 'message': str(exc)[:240],
                    'checked_at': time.time()})
                self._cache_semantic_response(
                    request['request_id'],
                    json.dumps(request, sort_keys=True, separators=(',', ':')),
                    self._stamp_semantic_response(raw, request))
            finally:
                if not hold_slot[0]:
                    release_slot()

        threading.Thread(target=run, daemon=True, name='semantic-preview').start()

    def _preview_navigation(self, request, *, cancel_event, hold_slot, release_slot,
                            timeout_s=12.0):
        checked_at = time.time()
        deadline = time.monotonic() + timeout_s
        unknown = lambda reason, message, **extra: {
            'status': 'unknown', 'reachable': None, 'reason_code': reason,
            'message': str(message)[:240], 'checked_at': checked_at, **extra}
        if request['server_session_id'] != self._server_session_id:
            return unknown('server_session_mismatch', '导航服务会话已变化。')
        if request['map_id'] != self._map_id():
            return unknown('map_id_mismatch', '语义地图已变化。')
        if self._busy or self._structured_task is not None:
            return unknown('preview_unavailable_busy', '已有导航任务，预检未运行。')
        rows = self._db_objects(include_timestamps=True)
        row = next((r for r in rows if r[0] == request['object_id']
                    and r[1] == request['class_id']), None)
        if row is None:
            return unknown('target_not_in_catalog', '对象不在当前确认目标清单中。')
        target = {'object_id': row[0], 'class_id': row[1],
                  'class_label': CLASS_LABELS.get(row[1]),
                  'x': row[2], 'y': row[3], 'z': row[4],
                  'count': row[5], 'first_stamp_ns': row[6],
                  'last_stamp_ns': row[7]}
        if cancel_event.is_set() or time.monotonic() >= deadline:
            return unknown('preview_timeout', '预检超过时间上限。', target=target)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.0))
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if stamp <= 0 or not math.isfinite(age) or age < -1.0 or age > 1.0:
                return unknown('localization_tf_stale', '机器人定位TF缺失或已过期。',
                               target=target)
        except Exception:
            return unknown('localization_tf_unavailable', '机器人定位TF当前不可用。',
                           target=target)
        if self.map_msg is None:
            return unknown('static_map_unavailable', '尚未收到静态地图。', target=target)
        if self.camera_info is None:
            return unknown('camera_info_unavailable', '尚未收到相机内参。', target=target)
        if not self.plan_client.server_is_ready():
            return unknown('planner_unavailable', 'Nav2路径规划器当前不可用。', target=target)
        preview_stop = lambda: cancel_event.is_set() or self._busy or time.monotonic() >= deadline

        def cancel_late_goal(send_future):
            hold_slot[0] = True

            def release_on_terminal(future):
                try:
                    terminal = future.result()
                    if terminal is not None and terminal.status in (
                            GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                            GoalStatus.STATUS_ABORTED):
                        release_slot()
                except Exception:
                    pass

            def accepted(future):
                try:
                    handle = future.result()
                    if handle is None or not handle.accepted:
                        release_slot()
                        return
                    result_future = handle.get_result_async()
                    handle.cancel_goal_async()
                    result_future.add_done_callback(release_on_terminal)
                except Exception:
                    # Keep the planner reservation if goal state cannot be proven.
                    pass

            send_future.add_done_callback(accepted)

        def cancel_known_goal(handle, result_future):
            def hold_until_terminal():
                hold_slot[0] = True

                def release_on_terminal(future):
                    try:
                        terminal = future.result()
                        if terminal is not None and terminal.status in (
                                GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                                GoalStatus.STATUS_ABORTED):
                            release_slot()
                    except Exception:
                        pass

                result_future.add_done_callback(release_on_terminal)

            def is_terminal_future(future):
                if not future.done():
                    return False
                try:
                    result = future.result()
                    return result is not None and result.status in (
                        GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                        GoalStatus.STATUS_ABORTED)
                except Exception:
                    return False

            try:
                cancel_future = handle.cancel_goal_async()
                cancel_response = self._wait_future(cancel_future, 1.5,
                                                     honor_replacement=False)
                cancellation_accepted = bool(
                    cancel_response and getattr(cancel_response, 'goals_canceling', []))
                if not cancellation_accepted and not is_terminal_future(result_future):
                    hold_until_terminal()
                    return False
                if is_terminal_future(result_future):
                    return True
                terminal = self._wait_future(result_future, 1.5,
                                             honor_replacement=False)
                if terminal is not None and terminal.status in (
                        GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                        GoalStatus.STATUS_ABORTED):
                    return True
                hold_until_terminal()
                return False
            except Exception:
                hold_until_terminal()
                return False
        try:
            candidates, base_to_camera = self._candidate_poses(
                (row[2], row[3], row[4]), cancel_event=cancel_event,
                deadline=deadline, stop_check=preview_stop)
        except TargetReplaced:
            return unknown('preview_interrupted', '导航开始，预检已停止。', target=target)
        except TimeoutError:
            return unknown('preview_timeout', '生成候选停靠位置超时。', target=target)
        except Exception as exc:
            return unknown('candidate_generation_failed', str(exc), target=target)
        if preview_stop():
            return unknown('preview_interrupted' if self._busy else 'preview_timeout',
                           '导航开始或预检超过时间上限。', target=target)
        if not candidates:
            return {'status': 'no_path', 'reachable': False,
                    'reason_code': 'no_camera_facing_candidate',
                    'message': '没有符合地图安全区与相机视野条件的候选停靠位置。',
                    'checked_at': checked_at, 'target': target,
                    'candidate_pose': None, 'path_end_pose': None,
                    'standoff_m': None, 'path_length_m': None}
        margin = float(self.get_parameter('camera_margin').value)
        planner_uncertain = False
        no_path_count = 0
        for score, radius, bx, by, yaw, _pc in candidates:
            if preview_stop():
                return unknown('preview_interrupted' if self._busy else 'preview_timeout',
                               '导航开始或预检超过时间上限。', target=target)
            goal = PoseStamped()
            goal.header.frame_id = self.map_frame
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x, goal.pose.position.y = bx, by
            goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            req = ComputePathToPose.Goal()
            req.goal, req.use_start, req.planner_id = goal, False, ''
            try:
                send_future = self.plan_client.send_goal_async(req)
                handle = self._wait_future(send_future,
                    max(0.01, min(2.0, deadline-time.monotonic())),
                    honor_replacement=False, stop_check=preview_stop)
                if handle is None:
                    cancel_late_goal(send_future)
                    if preview_stop():
                        return unknown('preview_interrupted' if self._busy else 'preview_timeout',
                                       '导航开始或预检超过时间上限。', target=target)
                    return unknown('preview_timeout', '等待Nav2接受路径预检超时。',
                                   target=target)
                if not handle.accepted:
                    planner_uncertain = True
                    continue
                result_future = handle.get_result_async()
                response = self._wait_future(result_future,
                    max(0.01, min(2.0, deadline-time.monotonic())),
                    honor_replacement=False, stop_check=preview_stop)
                if response is None:
                    if preview_stop():
                        canceled = cancel_known_goal(handle, result_future)
                        return unknown('preview_interrupted' if self._busy else 'preview_timeout',
                                       ('导航开始或预检超过时间上限；' if canceled else
                                        '预检已请求取消，等待Nav2确认路径任务终止；') +
                                       '当前不代表导航结果。', target=target)
                    cancel_known_goal(handle, result_future)
                    return unknown('preview_timeout', 'Nav2路径预检等待超时。', target=target)
                if response.status == GoalStatus.STATUS_ABORTED:
                    no_path_count += 1
                    continue
                if response.status != GoalStatus.STATUS_SUCCEEDED:
                    return unknown('planner_result_unconfirmed',
                                   'Nav2未返回明确的成功或无路终态。', target=target)
                if not response.result.path.poses:
                    return unknown('planner_path_invalid', 'Nav2成功终态没有路径点。',
                                   target=target)
                path = response.result.path
                end = path.poses[-1].pose
                end_yaw = math.atan2(2*(end.orientation.w*end.orientation.z +
                                        end.orientation.x*end.orientation.y),
                                     1-2*(end.orientation.y**2 + end.orientation.z**2))
                final_pc = target_in_camera((row[2], row[3], row[4]),
                                            (end.position.x, end.position.y), end_yaw,
                                            base_to_camera)
                if (camera_view_score(final_pc, self.camera_info, margin=margin) is None or
                        not self._map_free(end.position.x, end.position.y)):
                    no_path_count += 1
                    continue
                length = sum(math.hypot(b.pose.position.x-a.pose.position.x,
                                        b.pose.position.y-a.pose.position.y)
                             for a, b in zip(path.poses, path.poses[1:]))
                return {'status': 'path_found', 'reachable': True,
                        'reason_code': 'compute_path_succeeded',
                        'message': 'Nav2在本次预检中为候选停靠点返回路径；实际导航会重新规划。',
                        'checked_at': checked_at, 'target': target,
                        'candidate_pose': {'frame': self.map_frame, 'x': bx, 'y': by,
                                           'yaw': yaw, 'standoff_m': radius},
                        'path_end_pose': {'frame': self.map_frame,
                                          'x': end.position.x, 'y': end.position.y,
                                          'yaw': end_yaw},
                        'standoff_m': math.hypot(end.position.x-row[2],
                                                 end.position.y-row[3]),
                        'path_length_m': length}
            except Exception as exc:
                planner_uncertain = True
                planner_error = str(exc)
                continue
        if time.monotonic() >= deadline:
            return unknown('preview_timeout', 'Nav2路径预检超过时间上限。', target=target)
        if planner_uncertain:
            return unknown('planner_result_unconfirmed',
                           locals().get('planner_error', '部分候选路径结果无法确认。'),
                           target=target)
        if no_path_count != len(candidates):
            return unknown('planner_result_incomplete',
                           '尚未获得覆盖全部候选点的明确路径结果。', target=target)
        return {'status': 'no_path', 'reachable': False,
                'reason_code': 'no_collision_free_path',
                'message': '已完成所有候选位置的路径检查，Nav2未返回符合条件的路径。',
                'checked_at': checked_at, 'target': target,
                'candidate_pose': candidates[0] and {
                    'frame': self.map_frame, 'x': candidates[0][2],
                    'y': candidates[0][3], 'yaw': candidates[0][4],
                    'standoff_m': candidates[0][1]},
                'path_end_pose': None, 'standoff_m': None, 'path_length_m': None}

    @staticmethod
    def _public_task(record):
        if record is None:
            return None
        return {key: record.get(key) for key in (
            'client_id', 'task_id', 'map_id', 'server_session_id', 'target', 'state',
            'reason_code', 'message', 'timestamp', 'seq')}

    def _semantic_start(self, request):
        if not self.get_parameter('enable_chassis_control').value:
            return encode_response(request['request_id'], ok=False,
                                  reason_code='navigation_disabled')
        if request['server_session_id'] != self._server_session_id:
            return encode_response(request['request_id'], ok=False,
                                  reason_code='server_session_mismatch')
        if request['map_id'] != self._map_id():
            return encode_response(request['request_id'], ok=False, reason_code='map_id_mismatch')
        if self._preview_busy.is_set():
            preview_event = self._preview_cancel_event
            if preview_event is not None:
                preview_event.set()
            return encode_response(request['request_id'], ok=False,
                                   reason_code='preview_canceling',
                                   message='Stopping the read-only path preview before navigation.')
        rows = self._db_objects()
        with self._semantic_task_lock:
            existing = self._semantic_tasks.get(request['task_id'])
            reused_request_id = next((record for record in self._semantic_tasks.values()
                                      if record.get('request_id') == request['request_id']), None)
            if reused_request_id and reused_request_id['task_id'] != request['task_id']:
                return encode_response(request['request_id'], ok=False,
                                       reason_code='request_id_conflict')
            if existing:
                if existing['client_id'] != request['client_id']:
                    return encode_response(request['request_id'], ok=False,
                                          reason_code='task_id_conflict')
                same = (existing['map_id'] == request['map_id'] and
                        existing['target']['class_id'] == request['class_id'] and
                        existing['target']['object_id'] == request['object_id'])
                if not same:
                    return encode_response(request['request_id'], ok=False,
                                          reason_code='task_parameters_conflict')
                return encode_response(request['request_id'], ok=True,
                                       result={'task': self._public_task(existing),
                                               'idempotent': True})
            matches = [row for row in rows if row[0] == request['object_id']
                       and row[1] == request['class_id']]
            if not matches:
                return encode_response(request['request_id'], ok=False,
                                      reason_code='object_not_confirmed')
            row = matches[0]
            if self._busy or self._structured_task is not None:
                return encode_response(request['request_id'], ok=False, reason_code='busy')
            preview_event = getattr(self, '_preview_cancel_event', None)
            if preview_event is not None:
                preview_event.set()
            if len(self._semantic_tasks) >= 128:
                return encode_response(request['request_id'], ok=False,
                                       reason_code='task_budget_exhausted')
            target = {'class_label': CLASS_LABELS[request['class_id']],
                      'class_id': request['class_id'], 'object_id': request['object_id'],
                      'x': row[2], 'y': row[3], 'z': row[4], 'count': row[5]}
            record = {
                'client_id': request['client_id'], 'task_id': request['task_id'],
                'map_id': request['map_id'], 'server_session_id': self._server_session_id,
                'target': target,
                'state': 'planning', 'reason_code': None, 'message': None,
                'request_id': request['request_id'],
            }
            self._structured_task = record
            self._semantic_tasks[request['task_id']] = record
            self._busy = True
            self._target_worker_active = True
            self._replacement_safe = True
            self._active_target_key = (request['class_id'], request['object_id'])
            self._target_cancel_event.clear()
        self._emit_task_status(record, 'planning', 'accepted')
        command = f"{CLASS_LABELS[request['class_id']]}:{request['object_id']}"
        threading.Thread(target=self._navigate, args=(command, record), daemon=True).start()
        return encode_response(request['request_id'], ok=True,
                               result={'task': self._public_task(record), 'idempotent': False})

    def _semantic_record_for_request(self, request):
        if request['server_session_id'] != self._server_session_id:
            return None, 'server_session_mismatch'
        if request['map_id'] != self.map_id:
            return None, 'map_id_mismatch'
        with self._semantic_task_lock:
            record = self._semantic_tasks.get(request['task_id'])
        if record is None:
            return None, 'task_not_found'
        if record['client_id'] != request['client_id']:
            return None, 'task_owner_mismatch'
        return record, None

    def _semantic_status(self, request):
        record, error = self._semantic_record_for_request(request)
        if error:
            return encode_response(request['request_id'], ok=False, reason_code=error,
                                   result={'task': {'task_id': request['task_id'],
                                                    'state': 'unknown'}})
        task = self._public_task(record)
        task['checked_at'] = time.time()
        return encode_response(request['request_id'], ok=True,
                               result={'task': task})

    def _semantic_cancel(self, request):
        record, error = self._semantic_record_for_request(request)
        if error:
            return encode_response(request['request_id'], ok=False, reason_code=error,
                                   result={'task': {'task_id': request['task_id'],
                                                    'state': 'unknown'}})
        if record['state'] in ('succeeded', 'failed', 'rejected', 'canceled'):
            task = self._public_task(record)
            task['checked_at'] = time.time()
            return encode_response(request['request_id'], ok=True,
                                   result={'task': task})
        if self._structured_task is not record:
            return encode_response(request['request_id'], ok=False, reason_code='task_not_active',
                                   result={'task': self._public_task(record)})
        self._pending_target = None
        self._pending_target_key = None
        self._target_cancel_event.set()
        self._emit_task_status(record, 'canceling', 'cancel_requested')
        task = self._public_task(record)
        task['checked_at'] = time.time()
        return encode_response(request['request_id'], ok=True,
                               result={'task': task})

    def _publish_objects(self):
        try:
            rows = self._db_objects()
        except Exception as exc:
            self._say(f'Cannot load saved semantic objects: {exc}')
            return
        array = MarkerArray()
        for oid, cid, x, y, z, count in rows:
            label = {39: 'bottle', 41: 'cup', 63: 'laptop'}.get(cid, str(cid))
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns, marker.id, marker.type, marker.action = 'semantic_objects', int(oid), Marker.TEXT_VIEW_FACING, Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z + 0.18
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.16
            marker.color = ColorRGBA(r=1.0, g=0.8, b=0.1, a=1.0)
            marker.text = f'{label}:{oid} ({x:.2f}, {y:.2f}, {z:.2f}) n={count}'
            array.markers.append(marker)
        self.marker_pub.publish(array)

    def _target_cb(self, msg):
        command = msg.data.strip()
        try:
            row = resolve_target(self._db_objects(), command)
        except Exception as exc:
            self._say(f'Target rejected: {exc}')
            return
        target_key = (row[1], row[0])
        command = f'{CLASS_LABELS[row[1]]}:{row[0]}'
        self._say(f'Received target: {command}')
        with self._target_lock:
            preview_busy = getattr(self, '_preview_busy', None)
            if preview_busy is not None and preview_busy.is_set():
                preview_event = getattr(self, '_preview_cancel_event', None)
                if preview_event is not None:
                    preview_event.set()
                self._say('Legacy target deferred while read-only path preview stops; resend it afterward')
                return
            if getattr(self, '_structured_task', None) is not None:
                self._say('Legacy target rejected while a structured task is active')
                return
            if self._busy:
                if (target_key == self._active_target_key and
                        self._pending_target is None and
                        not self._target_cancel_event.is_set()):
                    message = f'Already processing {command}; duplicate request merged'
                elif target_key == self._pending_target_key:
                    message = f'Already queued {command}; duplicate request merged'
                else:
                    replaced = self._pending_target is not None
                    self._pending_target = command
                    self._pending_target_key = target_key
                    self._target_cancel_event.set()
                    message = 'Queued target for after current navigation'
                    if replaced:
                        message = 'Replaced queued target with the latest command'
                    message += '; canceling current request safely'
            else:
                self._busy = True
                self._target_worker_active = True
                self._replacement_safe = True
                self._active_target_key = target_key
                self._target_cancel_event.clear()
                message = None
        if message:
            self._say(message)
        else:
            threading.Thread(target=self._navigate, args=(command,), daemon=True).start()

    def _finish_target(self, worker_finished=False):
        """Start the latest queued target, or mark the navigator idle."""
        with self._target_lock:
            if worker_finished:
                self._target_worker_active = False
            elif self._target_worker_active:
                return
            command = self._pending_target
            if not self._replacement_safe:
                self._say('Queued target is held until Nav2 goal cancellation is confirmed')
                return
            if command is None:
                self._busy = False
                self._active_target_key = None
                self._target_cancel_event.clear()
                self._structured_task = None
                return
            self._pending_target = None
            self._active_target_key = self._pending_target_key
            self._pending_target_key = None
            self._target_cancel_event.clear()
            self._target_worker_active = True
        self._say(f'Starting queued target: {command}')
        threading.Thread(target=self._navigate, args=(command,), daemon=True).start()

    def _map_free(self, x, y, radius=0.50):
        m = self.map_msg
        if m is None:
            return False
        info = m.info
        origin_q = info.origin.orientation
        origin_yaw = math.atan2(2*(origin_q.w*origin_q.z + origin_q.x*origin_q.y),
                                1 - 2*(origin_q.y*origin_q.y + origin_q.z*origin_q.z))
        dx, dy = x-info.origin.position.x, y-info.origin.position.y
        gx = (math.cos(origin_yaw)*dx + math.sin(origin_yaw)*dy)/info.resolution
        gy = (-math.sin(origin_yaw)*dx + math.cos(origin_yaw)*dy)/info.resolution
        c0, r0 = math.floor(gx), math.floor(gy)
        cells = math.ceil(radius/info.resolution)
        for dr in range(-cells, cells+1):
            for dc in range(-cells, cells+1):
                # Include cells whose square intersects the circumscribed footprint disk.
                closest_x = max(0.0, abs(c0+dc+0.5-gx)-0.5)*info.resolution
                closest_y = max(0.0, abs(r0+dr+0.5-gy)-0.5)*info.resolution
                if closest_x**2 + closest_y**2 > radius**2:
                    continue
                c, r = c0+dc, r0+dr
                if not (0 <= c < info.width and 0 <= r < info.height):
                    return False
                value = m.data[r*info.width+c]
                if value < 0 or value >= 50:
                    return False
        return True

    def _candidate_poses(self, target, *, cancel_event=None, deadline=None,
                         stop_check=None):
        def check_stop():
            if stop_check is not None and stop_check():
                if self._busy:
                    raise TargetReplaced()
                raise TimeoutError('candidate generation deadline expired')
            if deadline is not None and time.monotonic() >= deadline:
                raise TimeoutError('candidate generation deadline expired')
            event = cancel_event if cancel_event is not None else self._target_cancel_event
            if event.is_set():
                raise TargetReplaced()

        check_stop()
        tf = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame,
                                              rclpy.time.Time(), timeout=rclpy.duration.Duration(
                                                  seconds=min(2.0, max(0.0, deadline-time.monotonic())
                                                              if deadline is not None else 2.0)))
        tr = tf.transform.translation
        q = tf.transform.rotation
        base_to_camera = ((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
        tmin = float(self.get_parameter('standoff_min').value)
        tmax = float(self.get_parameter('standoff_max').value)
        margin = float(self.get_parameter('camera_margin').value)
        if self.camera_info is None:
            raise RuntimeError('waiting for calibrated color CameraInfo')
        candidates = []
        # Sample a safe ring. Candidate heading is tested using the measured camera extrinsic.
        for radius in (tmin, 1.1, 1.3, 1.6, 2.0, tmax):
            check_stop()
            if radius < tmin or radius > tmax:
                continue
            for i in range(72):
                check_stop()
                a = 2*math.pi*i/72
                bx, by = target[0] + radius*math.cos(a), target[1] + radius*math.sin(a)
                if not self._map_free(bx, by):
                    continue
                for j in range(72):
                    if j % 12 == 0:
                        check_stop()
                    yaw = -math.pi + 2*math.pi*j/72
                    pc = target_in_camera(target, (bx, by), yaw, base_to_camera)
                    score = camera_view_score(pc, self.camera_info, margin=margin)
                    if score is None:
                        continue
                    candidates.append((score, radius, bx, by, yaw, pc))
        diverse = []
        for candidate in sorted(candidates):
            check_stop()
            angle = math.atan2(candidate[3]-target[1], candidate[2]-target[0])
            separated = True
            for item in diverse:
                item_angle = math.atan2(item[3]-target[1], item[2]-target[0])
                delta = math.atan2(math.sin(angle-item_angle), math.cos(angle-item_angle))
                if abs(delta) <= math.radians(12):
                    separated = False
                    break
            if separated:
                diverse.append(candidate)
            if len(diverse) >= 24:
                break
        return diverse, base_to_camera

    def _wait_future(self, future, timeout, honor_replacement=True, stop_check=None):
        if honor_replacement and self._target_cancel_event.is_set():
            raise TargetReplaced()
        deadline = time.monotonic()+timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            if honor_replacement and self._target_cancel_event.is_set():
                raise TargetReplaced()
            if stop_check is not None and stop_check():
                if self._busy:
                    raise TargetReplaced()
                return None
            time.sleep(0.02)
        return future.result() if future.done() else None

    def _set_task_from_nav2_terminal(self, record, status):
        if record is None or status is None:
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._emit_task_status(record, 'succeeded', 'nav2_succeeded')
        elif status == GoalStatus.STATUS_CANCELED:
            self._emit_task_status(record, 'canceled', 'nav2_canceled')
        elif status == GoalStatus.STATUS_ABORTED:
            self._emit_task_status(record, 'failed', 'nav2_aborted')

    def _cancel_nav_handle(self, handle, structured_task=None, return_status=False):
        """Cancel an accepted Nav2 goal and require a confirmed terminal result."""
        result_future = None
        try:
            result_future = handle.get_result_async()
            self._wait_future(handle.cancel_goal_async(), 3.0,
                              honor_replacement=False)
            terminal = self._wait_future(result_future, 3.0, honor_replacement=False)
            if terminal is None:
                self._mark_replacement_unsafe()
                if structured_task is None:
                    self._watch_nav_terminal(handle, result_future)
                else:
                    self._watch_nav_terminal(handle, result_future, structured_task)
                return (False, None) if return_status else False
            # A terminal result proves Nav2 no longer owns an active motion goal.
            safe = terminal.status in (GoalStatus.STATUS_SUCCEEDED,
                                       GoalStatus.STATUS_CANCELED,
                                       GoalStatus.STATUS_ABORTED)
            if safe:
                if structured_task is None:
                    self._resolve_late_nav_send(safe=True)
                else:
                    self._resolve_late_nav_send(safe=True, structured_task=structured_task)
            else:
                self._mark_replacement_unsafe()
            return (safe, terminal.status) if return_status else safe
        except Exception as exc:
            self._say(f'Could not confirm Nav2 goal cancellation: {exc}')
            self._mark_replacement_unsafe()
            if result_future is not None and not result_future.done():
                if structured_task is None:
                    self._watch_nav_terminal(handle, result_future)
                else:
                    self._watch_nav_terminal(handle, result_future, structured_task)
            return (False, None) if return_status else False

    def _mark_replacement_unsafe(self):
        with self._target_lock:
            self._replacement_safe = False

    def _watch_nav_terminal(self, handle, result_future, structured_task=None):
        """Resolve an uncertain cancellation only after Nav2 reports a terminal state."""
        handle_key = id(handle)
        with self._target_lock:
            if handle_key in self._terminal_watch_handles:
                return
            self._terminal_watch_handles.add(handle_key)

        def resolve_terminal(future):
            try:
                terminal = future.result()
            except Exception as exc:
                self._say(f'Nav2 terminal result remains unconfirmed: {exc}')
                return
            finally:
                with self._target_lock:
                    self._terminal_watch_handles.discard(handle_key)
            if terminal is None:
                self._say('Nav2 terminal result is empty; keeping queued target held')
                return
            if terminal.status in (GoalStatus.STATUS_SUCCEEDED,
                                   GoalStatus.STATUS_CANCELED,
                                   GoalStatus.STATUS_ABORTED):
                self._say(f'Nav2 goal reached terminal status={terminal.status}; releasing queued target')
                if structured_task is not None:
                    if terminal.status == GoalStatus.STATUS_SUCCEEDED:
                        self._emit_task_status(structured_task, 'succeeded', 'nav2_succeeded')
                    elif terminal.status == GoalStatus.STATUS_CANCELED:
                        self._emit_task_status(structured_task, 'canceled', 'nav2_canceled')
                    else:
                        self._emit_task_status(structured_task, 'failed', 'nav2_aborted')
                if structured_task is None:
                    self._resolve_late_nav_send(safe=True)
                else:
                    self._resolve_late_nav_send(safe=True, structured_task=structured_task)
            else:
                self._say(f'Nav2 goal is not terminal yet (status={terminal.status}); keeping queued target held')

        try:
            result_future.add_done_callback(resolve_terminal)
        except Exception as exc:
            with self._target_lock:
                self._terminal_watch_handles.discard(handle_key)
            self._say(f'Could not watch Nav2 terminal result; queued target remains held: {exc}')

    def _watch_late_nav_acceptance(self, send_future, structured_task=None):
        """Cancel a goal if Nav2 accepts it after the request timed out."""
        def cancel_if_accepted(future):
            try:
                handle = future.result()
            except Exception as exc:
                self._say(f'Late Nav2 goal request failed: {exc}')
                if structured_task is not None:
                    self._emit_task_status(structured_task, 'failed', 'goal_request_failed', str(exc))
                    self._resolve_late_nav_send(safe=True, structured_task=structured_task)
                else:
                    self._resolve_late_nav_send(safe=True)
                return
            if handle is None or not handle.accepted:
                self._say('Late Nav2 goal request was rejected')
                if structured_task is not None:
                    self._emit_task_status(structured_task, 'failed', 'goal_rejected', 'Nav2 rejected the late goal request')
                    self._resolve_late_nav_send(safe=True, structured_task=structured_task)
                else:
                    self._resolve_late_nav_send(safe=True)
                return
            args = ((handle,) if structured_task is None
                    else (handle, structured_task))
            threading.Thread(target=self._cancel_late_nav_handle,
                             args=args, daemon=True).start()

        send_future.add_done_callback(cancel_if_accepted)

    def _cancel_late_nav_handle(self, handle, structured_task=None):
        if structured_task is None:
            safe = self._cancel_nav_handle(handle)
            terminal_status = None
        else:
            safe, terminal_status = self._cancel_nav_handle(
                handle, structured_task=structured_task, return_status=True)
        if safe:
            self._say('Late Nav2 goal acceptance was canceled and reached a terminal state')
            if structured_task is not None:
                if terminal_status == GoalStatus.STATUS_SUCCEEDED:
                    self._emit_task_status(structured_task, 'succeeded', 'nav2_succeeded')
                elif terminal_status == GoalStatus.STATUS_CANCELED:
                    self._emit_task_status(structured_task, 'canceled', 'nav2_canceled')
                else:
                    self._emit_task_status(structured_task, 'failed', 'nav2_aborted')
        elif structured_task is not None:
            self._emit_task_status(structured_task, 'unknown', 'cancel_unconfirmed')
        if structured_task is None:
            self._resolve_late_nav_send(safe=safe)
        else:
            self._resolve_late_nav_send(safe=safe, structured_task=structured_task)

    def _resolve_late_nav_send(self, safe, structured_task=None):
        # A delayed callback may only release the exact task whose goal it watched.
        if (structured_task is not None and
                getattr(self, '_structured_task', None) is not structured_task):
            return
        with self._target_lock:
            self._replacement_safe = safe
        if safe:
            self._finish_target()

    def _require_fresh_localization(self):
        deadline = time.monotonic() + 2.0
        age = float('nan')
        while rclpy.ok() and time.monotonic() <= deadline:
            if self._target_cancel_event.is_set():
                raise TargetReplaced()
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.2))
                stamp = transform.header.stamp.sec + transform.header.stamp.nanosec*1e-9
                now = self.get_clock().now().nanoseconds*1e-9
                age = now-stamp
                if (math.isfinite(stamp) and stamp > 0 and
                        math.isfinite(age) and age <= 1.0 and age >= -1.0):
                    return
            except Exception:
                # TF may be between updates during startup or a short transport delay.
                pass
            time.sleep(0.05)
        raise RuntimeError(
            f'RTAB-Map localization TF is stale or invalid after 2.0s refresh wait (age={age:.2f}s)')

    def _require_nav_active(self):
        for name, client in self.lifecycle_clients.items():
            if self._target_cancel_event.is_set():
                raise TargetReplaced()
            if not client.wait_for_service(timeout_sec=0.4):
                raise RuntimeError(f'Nav2 lifecycle state service unavailable: {name}')
            response = self._wait_future(client.call_async(GetState.Request()), 1.5)
            if response is None or response.current_state.id != 3:
                state = response.current_state.label if response else 'no response'
                raise RuntimeError(f'Nav2 server {name} is not active ({state})')

    def _navigate(self, command, structured_task=None):
        task_terminal = None
        task_reason = None
        task_message = None
        try:
            rows = self._db_objects()
            oid, cid, x, y, z, count = resolve_target(rows, command)
            self._require_fresh_localization()
            self._require_nav_active()
            if self.map_msg is None:
                raise RuntimeError(f'waiting for authoritative {self.static_map_topic}')
            if not self.tf_buffer.can_transform(self.map_frame, self.base_frame,
                                                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1)):
                raise RuntimeError('RTAB-Map localization has not produced map-to-base TF')
            self._say(f'Planning to {command} id={oid} at map ({x:.2f}, {y:.2f}, {z:.2f})')
            candidates, base_to_camera = self._candidate_poses((x, y, z))
            if not candidates:
                raise RuntimeError('no free, footprint-safe pose keeps the object inside the camera view')
            if not self.plan_client.wait_for_server(timeout_sec=5):
                raise RuntimeError('Nav2 ComputePathToPose action is unavailable')
            if not self.nav_client.wait_for_server(timeout_sec=5):
                raise RuntimeError('Nav2 NavigateToPose action is unavailable')
            chosen = None
            margin = float(self.get_parameter('camera_margin').value)
            for score, radius, bx, by, yaw, pc in candidates:
                goal = PoseStamped()
                goal.header.frame_id = self.map_frame
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x, goal.pose.position.y = bx, by
                goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                req = ComputePathToPose.Goal()
                req.goal = goal
                req.use_start = False
                req.planner_id = ''
                result = self._wait_future(self.plan_client.send_goal_async(req), 8)
                if result is None or not result.accepted:
                    continue
                res = self._wait_future(result.get_result_async(), 15)
                if res and res.status == GoalStatus.STATUS_SUCCEEDED and res.result.path.poses:
                    final_pose = res.result.path.poses[-1].pose
                    final_yaw = math.atan2(2*(final_pose.orientation.w*final_pose.orientation.z +
                                                final_pose.orientation.x*final_pose.orientation.y),
                                           1-2*(final_pose.orientation.y**2 + final_pose.orientation.z**2))
                    final_pc = target_in_camera((x, y, z),
                                                (final_pose.position.x, final_pose.position.y),
                                                final_yaw, base_to_camera)
                    final_pixel = project_camera_point(final_pc, self.camera_info)
                    if (camera_view_score(final_pc, self.camera_info, margin=margin) is not None and
                            self._map_free(final_pose.position.x, final_pose.position.y)):
                        chosen = (goal, res.result.path, final_pc, final_pixel)
                        break
            if chosen is None:
                raise RuntimeError('Nav2 found no collision-free path to any camera-facing candidate')
            if structured_task is not None:
                self._emit_task_status(structured_task, 'planning', 'path_validated')
            self._require_fresh_localization()
            self._require_nav_active()
            goal, path, final_pc, final_pixel = chosen
            self.path_pub.publish(path)
            # Visualize the calibrated camera origin at the planner's actual
            # terminal base pose, with an arrow toward the saved object.
            terminal_pose = path.poses[-1].pose
            terminal_yaw = math.atan2(
                2*(terminal_pose.orientation.w*terminal_pose.orientation.z +
                   terminal_pose.orientation.x*terminal_pose.orientation.y),
                1-2*(terminal_pose.orientation.y**2 + terminal_pose.orientation.z**2))
            camera_translation = base_to_camera[0]
            camera_origin = (
                terminal_pose.position.x + math.cos(terminal_yaw)*camera_translation[0] -
                math.sin(terminal_yaw)*camera_translation[1],
                terminal_pose.position.y + math.sin(terminal_yaw)*camera_translation[0] +
                math.cos(terminal_yaw)*camera_translation[1],
                camera_translation[2])
            sightline = Marker()
            sightline.header.frame_id = self.map_frame
            sightline.header.stamp = self.get_clock().now().to_msg()
            sightline.ns, sightline.id = 'camera_sightline', int(oid)
            sightline.type, sightline.action = Marker.ARROW, Marker.ADD
            sightline.pose.orientation.w = 1.0
            sightline.scale.x, sightline.scale.y, sightline.scale.z = 0.025, 0.075, 0.12
            sightline.color = ColorRGBA(r=0.0, g=0.75, b=1.0, a=1.0)
            from geometry_msgs.msg import Point
            sightline.points = [Point(x=camera_origin[0], y=camera_origin[1], z=camera_origin[2]),
                                Point(x=x, y=y, z=z)]
            self.camera_view_pub.publish(MarkerArray(markers=[sightline]))
            self._say(f'Planned {len(path.poses)} poses; goal=({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}), final camera point=({final_pc[0]:.2f},{final_pc[1]:.2f},{final_pc[2]:.2f}) m pixel=({final_pixel[0]:.0f},{final_pixel[1]:.0f})')
            if self._target_cancel_event.is_set():
                raise TargetReplaced()
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = goal
            send_future = self.nav_client.send_goal_async(nav_goal)
            try:
                handle = self._wait_future(send_future, 8)
            except TargetReplaced:
                handle = self._wait_future(send_future, 3, honor_replacement=False)
                if handle is None:
                    self._mark_replacement_unsafe()
                    self._watch_late_nav_acceptance(send_future, structured_task)
                elif handle.accepted:
                    self._active_nav_handle = handle
                    safe, terminal_status = self._cancel_nav_handle(
                        handle, structured_task, return_status=True)
                    if safe:
                        self._set_task_from_nav2_terminal(structured_task, terminal_status)
                    self._active_nav_handle = None
                raise
            if handle is None:
                # A timed-out send request can still be accepted later. Give it a
                # bounded chance to resolve and cancel any late accepted goal.
                handle = self._wait_future(send_future, 3, honor_replacement=False)
                if handle is None:
                    self._mark_replacement_unsafe()
                    self._watch_late_nav_acceptance(send_future, structured_task)
                    raise RuntimeError('Nav2 goal acceptance timed out; late acceptance cannot be ruled out')
                if handle.accepted:
                    self._active_nav_handle = handle
                    safe, terminal_status = self._cancel_nav_handle(
                        handle, structured_task, return_status=True)
                    if safe:
                        self._set_task_from_nav2_terminal(structured_task, terminal_status)
                    self._active_nav_handle = None
                raise RuntimeError('Nav2 goal acceptance timed out; any late goal was canceled')
            if handle is None or not handle.accepted:
                raise RuntimeError('Nav2 rejected NavigateToPose goal')
            self._active_nav_handle = handle
            if structured_task is not None:
                self._emit_task_status(structured_task, 'navigating', 'goal_accepted')
            if self._target_cancel_event.is_set():
                raise TargetReplaced()
            self._nonzero_cmd_count = 0
            if self.get_parameter('enable_chassis_control').value:
                self._say('Chassis control enabled; following NavigateToPose until arrival or timeout')
                terminal = self._wait_future(
                    handle.get_result_async(),
                    float(self.get_parameter('navigation_timeout_s').value))
                if terminal is None:
                    if not self._cancel_nav_handle(handle):
                        task_terminal, task_reason = 'canceling', 'cancel_unconfirmed'
                        raise RuntimeError('navigation timed out and Nav2 cancellation was not confirmed')
                    self._active_nav_handle = None
                    task_terminal, task_reason = 'failed', 'navigation_timeout'
                    self._say('Navigation timed out; Nav2 canceled the goal and the chassis will stop')
                elif terminal.status == GoalStatus.STATUS_SUCCEEDED:
                    self._active_nav_handle = None
                    task_terminal, task_reason = 'succeeded', 'nav2_succeeded'
                    self._say('Navigation complete: Nav2 reached the camera-facing standoff goal')
                elif terminal.status == GoalStatus.STATUS_CANCELED:
                    self._active_nav_handle = None
                    task_terminal, task_reason = 'canceled', 'nav2_canceled'
                    self._say('Navigation canceled by Nav2')
                else:
                    self._active_nav_handle = None
                    task_terminal, task_reason = 'failed', 'nav2_terminal_failure'
                    self._say(f'Navigation ended with Nav2 status={terminal.status}')
            else:
                self._say('Read-only control validation; observing Nav2 output for a bounded 3 seconds')
                end = time.monotonic()+float(self.get_parameter('validation_duration_s').value)
                while rclpy.ok() and time.monotonic() < end:
                    time.sleep(0.02)
                cancel_response = self._wait_future(handle.cancel_goal_async(), 3)
                terminal = self._wait_future(handle.get_result_async(), 3)
                if cancel_response is None or not cancel_response.goals_canceling:
                    raise RuntimeError('Nav2 did not confirm cancellation of the bounded navigation goal')
                if terminal is None or terminal.status != GoalStatus.STATUS_CANCELED:
                    raise RuntimeError('Nav2 goal did not report canceled after the bounded validation window')
                if self._nonzero_cmd_count == 0:
                    raise RuntimeError('Nav2 emitted no nonzero /cmd_vel during the bounded validation window')
                self._say(f'Validated {self._nonzero_cmd_count} nonzero /cmd_vel messages; goal canceled and base-control subscriber remains disabled')
        except TargetReplaced:
            self._say('Current request superseded; stopping before the latest target')
            if structured_task is not None:
                task_reason = 'cancel_requested'
        except Exception as exc:
            self._say(f'Target rejected: {exc}')
            task_message = str(exc)[:240]
            if structured_task is not None and not self._replacement_safe:
                task_terminal, task_reason = 'unknown', 'acceptance_or_cancel_unconfirmed'
            elif structured_task is not None and task_terminal != 'canceling':
                task_terminal, task_reason = 'failed', 'navigation_failed'
        finally:
            if self._active_nav_handle is not None:
                safe, terminal_status = self._cancel_nav_handle(
                    self._active_nav_handle, structured_task=structured_task,
                    return_status=True)
                if not safe:
                    self._mark_replacement_unsafe()
                    if structured_task is not None:
                        task_terminal, task_reason = 'canceling', 'cancel_unconfirmed'
                elif structured_task is not None:
                    self._set_task_from_nav2_terminal(structured_task, terminal_status)
                    if terminal_status == GoalStatus.STATUS_SUCCEEDED:
                        task_terminal, task_reason = 'succeeded', 'nav2_succeeded'
                    elif terminal_status == GoalStatus.STATUS_CANCELED:
                        task_terminal, task_reason = 'canceled', 'nav2_canceled'
                    else:
                        task_terminal, task_reason = 'failed', 'nav2_aborted'
                self._active_nav_handle = None
            elif (structured_task is not None and self._target_cancel_event.is_set()
                  and task_terminal is None):
                if self._replacement_safe:
                    task_terminal, task_reason = 'canceled', 'canceled_before_goal'
                else:
                    task_terminal, task_reason = 'unknown', 'cancel_unconfirmed'
            elif structured_task is not None and not self._replacement_safe:
                task_terminal, task_reason = 'unknown', 'acceptance_or_cancel_unconfirmed'
            if structured_task is not None and task_terminal is not None:
                if structured_task['state'] != task_terminal:
                    self._emit_task_status(
                        structured_task, task_terminal, task_reason, task_message)
            self._finish_target(worker_finished=True)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticNavigator()
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
