import json
import math
import sqlite3
import threading
from collections import OrderedDict
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped

import semantic_mapping.semantic_navigation as navigation
from semantic_mapping.semantic_navigation import SemanticNavigator, relative_target
from semantic_mapping.semantic_task_protocol import decode_request


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


class FakeThread:
    def __init__(self, *, target, args, daemon):
        self.target, self.args, self.daemon = target, args, daemon

    def start(self):
        pass


def _request(op="start", **extra):
    data = {
        "schema_version": 1, "request_id": "req-1", "op": op,
        "client_id": "client-1", "task_id": "task-1", "map_id": "map-1",
        "server_session_id": "session-1",
    }
    if op == "start":
        data.update(class_label="bottle", object_id=2)
    elif op == "preview_navigation":
        data.pop("task_id")
        data.update(class_label="bottle", object_id=2)
    data.update(extra)
    return decode_request(json.dumps(data))


def _node():
    node = SimpleNamespace(
        _semantic_task_lock=threading.RLock(), _semantic_tasks=OrderedDict(),
        _semantic_request_cache=OrderedDict(), _semantic_task_seq=0,
        _structured_task=None, _server_session_id="session-1", map_id="map-1",
        _busy=False, _target_worker_active=False, _replacement_safe=True,
        _preview_busy=threading.Event(), _preview_cancel_event=None,
        _active_target_key=None, _pending_target=None, _pending_target_key=None,
        _target_cancel_event=threading.Event(), semantic_task_status_pub=Publisher(),
        _navigate=lambda *_args: None,
        _db_objects=lambda: [(2, 39, 1.0, 2.0, 0.5, 4)],
        _map_id=lambda: "map-1", _say=lambda _message: None,
    )
    node.get_parameter = lambda _name: SimpleNamespace(value=True)
    node._emit_task_status = lambda record, state=None, reason_code=None: \
        SemanticNavigator._emit_task_status(node, record, state, reason_code)
    node._public_task = SemanticNavigator._public_task
    node._semantic_record_for_request = lambda request: \
        SemanticNavigator._semantic_record_for_request(node, request)
    return node


def test_structured_start_is_idempotent_and_busy_is_explicit(monkeypatch):
    monkeypatch.setattr(navigation.threading, "Thread", FakeThread)
    node = _node()
    first = json.loads(SemanticNavigator._semantic_start(node, _request()))
    assert first["ok"] is True
    assert first["result"]["idempotent"] is False
    assert first["result"]["task"]["state"] == "planning"

    duplicate = json.loads(SemanticNavigator._semantic_start(node, _request()))
    assert duplicate["ok"] is True
    assert duplicate["result"]["idempotent"] is True

    changed = json.loads(SemanticNavigator._semantic_start(
        node, _request(object_id=3)))
    assert changed["reason_code"] == "task_parameters_conflict"

    other = json.loads(SemanticNavigator._semantic_start(
        node, _request(task_id="task-2", request_id="req-2")))
    assert other["reason_code"] == "busy"


def test_structured_start_rejects_disabled_navigation_and_old_server_session():
    node = _node()
    node.get_parameter = lambda _name: SimpleNamespace(value=False)
    result = json.loads(SemanticNavigator._semantic_start(node, _request()))
    assert result["reason_code"] == "navigation_disabled"
    node.get_parameter = lambda _name: SimpleNamespace(value=True)
    result = json.loads(SemanticNavigator._semantic_start(
        node, _request(server_session_id="old-session")))
    assert result["reason_code"] == "server_session_mismatch"


def test_semantic_object_rows_include_saved_observation_timestamps(tmp_path):
    path = tmp_path / "semantic.db"
    with sqlite3.connect(path) as db:
        db.execute("CREATE TABLE metadata(key TEXT PRIMARY KEY, value TEXT)")
        db.execute("INSERT INTO metadata VALUES('map_id', 'map-1')")
        db.execute("CREATE TABLE objects(object_id INTEGER PRIMARY KEY, class_id INTEGER, "
                   "x REAL, y REAL, z REAL, count INTEGER, first_stamp_ns INTEGER, "
                   "last_stamp_ns INTEGER)")
        db.execute("INSERT INTO objects VALUES(1,39,2.0,3.0,0.5,8,100,900)")
    node = SimpleNamespace(db_path=str(path), map_id="map-1")
    rows = SemanticNavigator._db_objects(node, include_timestamps=True)
    assert rows == [(1, 39, 2.0, 3.0, 0.5, 8, 100, 900)]


def test_relative_target_uses_base_forward_left_and_refuses_stale_tf():
    robot = {"fresh": True, "x": 1.0, "y": 2.0, "z": 0.0,
             "yaw": math.pi / 2, "tf_stamp": 123.0, "age_s": 0.1}
    relative = relative_target(robot, (1.0, 3.0, 0.5))
    assert relative["available"] is True
    assert relative["frame"] == "base_link"
    assert relative["x"] == pytest.approx(1.0)
    assert relative["y"] == pytest.approx(0.0, abs=1e-8)
    assert relative["distance_m"] == pytest.approx(1.0)
    assert relative["bearing_rad"] == pytest.approx(0.0, abs=1e-8)
    stale = relative_target({**robot, "fresh": False, "reason": "localization_tf_stale"},
                            (9.0, 9.0, 0.0))
    assert stale["available"] is False
    assert stale["distance_m"] is None and stale["bearing_rad"] is None
    assert stale["reason"] == "localization_tf_stale"


def test_invalid_tf_numeric_values_become_unknown_context_not_nonfinite_json():
    stamp = SimpleNamespace(sec=123, nanosec=0)
    transform = SimpleNamespace(
        header=SimpleNamespace(stamp=stamp),
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=float("nan"), y=0.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
    node = SimpleNamespace(
        map_frame="map", base_frame="base_link",
        tf_buffer=SimpleNamespace(lookup_transform=lambda *_args, **_kwargs: transform),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=123_000_000_000)))
    pose = SemanticNavigator._robot_pose_context(node)
    assert pose["fresh"] is False
    assert pose["frame"] == "map" and pose["child_frame"] == "base_link"
    assert pose["x"] is None and pose["yaw"] is None
    assert pose["reason"] == "localization_tf_invalid"


def test_robot_pose_normalizes_finite_nonunit_tf_quaternion():
    stamp = SimpleNamespace(sec=123, nanosec=0)
    half = math.sqrt(0.5) * 2.0
    transform = SimpleNamespace(
        header=SimpleNamespace(stamp=stamp),
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=half, w=half)))
    node = SimpleNamespace(
        map_frame="map", base_frame="base_link",
        tf_buffer=SimpleNamespace(lookup_transform=lambda *_args, **_kwargs: transform),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=123_000_000_000)))
    pose = SemanticNavigator._robot_pose_context(node)
    assert pose["fresh"] is True
    assert pose["yaw"] == pytest.approx(math.pi / 2)


def test_structured_task_id_tombstones_are_bounded_without_replay_after_eviction(monkeypatch):
    monkeypatch.setattr(navigation.threading, "Thread", FakeThread)
    node = _node()
    node._semantic_tasks = OrderedDict((f"old-{i}", {}) for i in range(128))
    result = json.loads(SemanticNavigator._semantic_start(node, _request()))
    assert result["reason_code"] == "task_budget_exhausted"
    assert len(node._semantic_tasks) == 128


def test_start_stops_read_only_preview_before_creating_any_navigation_task():
    node = _node()
    node._preview_busy.set()
    node._preview_cancel_event = threading.Event()
    node._target_cancel_event.set()
    response = json.loads(SemanticNavigator._semantic_start(node, _request()))
    assert response["reason_code"] == "preview_canceling"
    assert node._preview_cancel_event.is_set()
    assert node._structured_task is None
    assert node._busy is False
    assert node._target_cancel_event.is_set()


def test_legacy_target_stops_preview_and_requires_resend_after_terminal():
    preview_cancel = threading.Event()
    messages = []
    node = SimpleNamespace(
        _target_lock=threading.RLock(), _structured_task=None, _busy=False,
        _preview_busy=threading.Event(), _preview_cancel_event=preview_cancel,
        _db_objects=lambda: [(2, 39, 1.0, 2.0, 0.5, 4)],
        _say=messages.append)
    node._preview_busy.set()
    SemanticNavigator._target_cb(node, SimpleNamespace(data="bottle"))
    assert preview_cancel.is_set()
    assert node._busy is False
    assert any("resend" in message for message in messages)


def _preview_fixture(monkeypatch):
    now_ns = 123_000_000_000
    transform = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=123, nanosec=0)),
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
    node = _node()
    node._db_objects = lambda **_kwargs: [(2, 39, 1.0, 2.0, 0.5, 4, 10, 20)]
    node._structured_task = None
    node.map_frame = "map"
    node.base_frame = "base_link"
    node.tf_buffer = SimpleNamespace(lookup_transform=lambda *_a, **_kw: transform)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=now_ns),
        )
    node.map_msg = object()
    node.camera_info = object()
    node.camera_margin = 0.1
    node.get_parameter = lambda _name: SimpleNamespace(value=0.1)
    node.plan_client = SimpleNamespace(server_is_ready=lambda: True)
    node._candidate_poses = lambda *_a, **_kw: ([], 0.0)
    return node


def test_preview_empty_candidate_set_is_explicit_no_path(monkeypatch):
    node = _preview_fixture(monkeypatch)
    result = SemanticNavigator._preview_navigation(
        node, _request("preview_navigation"), cancel_event=threading.Event(),
        hold_slot=[False], release_slot=lambda: None, timeout_s=2.0)
    assert result["status"] == "no_path"
    assert result["reachable"] is False
    assert result["reason_code"] == "no_camera_facing_candidate"
    assert result["target"]["first_stamp_ns"] == 10


def test_preview_expired_deadline_is_unknown_not_unreachable():
    node = _preview_fixture(None)
    result = SemanticNavigator._preview_navigation(
        node, _request("preview_navigation"), cancel_event=threading.Event(),
        hold_slot=[False], release_slot=lambda: None, timeout_s=0.0)
    assert result["status"] == "unknown"
    assert result["reachable"] is None
    assert result["reason_code"] == "preview_timeout"


class _ImmediateFuture:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _PendingFuture:
    def __init__(self):
        self._done = False
        self._result = None
        self._callbacks = []

    def done(self):
        return self._done

    def result(self):
        if not self._done:
            raise RuntimeError("future is pending")
        return self._result

    def add_done_callback(self, callback):
        if self._done:
            callback(self)
        else:
            self._callbacks.append(callback)

    def set_result(self, result):
        self._done = True
        self._result = result
        for callback in self._callbacks:
            callback(self)


class _FakePlanHandle:
    accepted = True

    def __init__(self, result_future, cancel_response=None):
        self.result_future = result_future
        self.cancel_response = cancel_response or SimpleNamespace(goals_canceling=[1])

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        return _ImmediateFuture(self.cancel_response)


def _preview_candidate_node(monkeypatch, result_future, *, cancel_response=None):
    node = _preview_fixture(monkeypatch)
    candidate = (1.0, 1.0, 2.0, 3.0, 0.0, (0.0, 0.0, 1.0))
    node._candidate_poses = lambda *_a, **_kw: ([candidate], (0.0, 0.0))
    now = SimpleNamespace(nanoseconds=123_000_000_000, to_msg=lambda: Time())
    node.get_clock = lambda: SimpleNamespace(now=lambda: now)
    monkeypatch.setattr(navigation, "target_in_camera", lambda *_a, **_kw: (0.0, 0.0, 1.0))
    monkeypatch.setattr(navigation, "camera_view_score", lambda *_a, **_kw: 1.0)
    end_pose = PoseStamped()
    end_pose.pose.position.x = 2.0
    end_pose.pose.position.y = 3.0
    end_pose.pose.orientation.w = 1.0
    path = SimpleNamespace(poses=[end_pose])
    planner_result = SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED,
                                     result=SimpleNamespace(path=path))
    handle = _FakePlanHandle(result_future, cancel_response=cancel_response)
    node.plan_client = SimpleNamespace(
        server_is_ready=lambda: True,
        send_goal_async=lambda _goal: _ImmediateFuture(handle))
    node._map_free = lambda *_a: True
    node._wait_future = lambda future, *_a, **_kw: (
        future.result() if future.done() else None)
    return node, result_future, planner_result


def test_preview_compute_path_success_returns_candidate_and_actual_path(monkeypatch):
    future = _ImmediateFuture(None)
    node, _, planner_result = _preview_candidate_node(monkeypatch, future)
    future._result = planner_result
    result = SemanticNavigator._preview_navigation(
        node, _request("preview_navigation"), cancel_event=threading.Event(),
        hold_slot=[False], release_slot=lambda: None, timeout_s=2.0)
    assert result["status"] == "path_found" and result["reachable"] is True, result
    assert result["candidate_pose"]["frame"] == "map"
    assert result["path_end_pose"]["x"] == pytest.approx(2.0)
    assert result["path_length_m"] == 0.0


def test_preview_aborted_compute_path_is_no_path(monkeypatch):
    future = _ImmediateFuture(None)
    node, _, planner_result = _preview_candidate_node(monkeypatch, future)
    planner_result.status = GoalStatus.STATUS_ABORTED
    future._result = planner_result
    result = SemanticNavigator._preview_navigation(
        node, _request("preview_navigation"), cancel_event=threading.Event(),
        hold_slot=[False], release_slot=lambda: None, timeout_s=2.0)
    assert result["status"] == "no_path" and result["reachable"] is False
    assert result["reason_code"] == "no_collision_free_path"


def test_preview_cancel_rejection_holds_planner_slot_until_terminal(monkeypatch):
    future = _PendingFuture()
    node, _, planner_result = _preview_candidate_node(
        monkeypatch, future,
        cancel_response=SimpleNamespace(goals_canceling=[]))
    hold_slot = [False]
    released = []
    node._wait_future = lambda candidate, *_a, **_kw: (
        None if candidate is future else candidate.result())
    result = SemanticNavigator._preview_navigation(
        node, _request("preview_navigation"), cancel_event=threading.Event(),
        hold_slot=hold_slot, release_slot=lambda: released.append(True), timeout_s=2.0)
    assert result["status"] == "unknown"
    assert result["reason_code"] == "preview_timeout"
    assert hold_slot[0] is True and released == []
    future.set_result(planner_result)
    assert released == [True]


def test_preview_late_accepted_goal_holds_slot_until_its_terminal(monkeypatch):
    result_future = _PendingFuture()
    node, _, planner_result = _preview_candidate_node(monkeypatch, result_future)
    send_future = _PendingFuture()
    node.plan_client.send_goal_async = lambda _goal: send_future
    node._wait_future = lambda future, *_a, **_kw: (
        None if future is send_future else future.result())
    hold_slot = [False]
    released = []
    result = SemanticNavigator._preview_navigation(
        node, _request("preview_navigation"), cancel_event=threading.Event(),
        hold_slot=hold_slot, release_slot=lambda: released.append(True), timeout_s=2.0)
    assert result["status"] == "unknown" and result["reason_code"] == "preview_timeout"
    assert hold_slot[0] is True and released == []
    handle = _FakePlanHandle(result_future)
    send_future.set_result(handle)
    assert hold_slot[0] is True and released == []
    result_future.set_result(planner_result)
    assert released == [True]


def test_cancel_is_scoped_to_client_session_and_keeps_canceling_until_resolution():
    node = _node()
    record = {
        "client_id": "client-1", "task_id": "task-1", "map_id": "map-1",
        "server_session_id": "session-1", "target": {"object_id": 2},
        "state": "navigating", "reason_code": None,
    }
    node._semantic_tasks["task-1"] = record
    node._structured_task = record
    node._busy = True
    node._semantic_record_for_request = lambda request: \
        SemanticNavigator._semantic_record_for_request(node, request)

    response = json.loads(SemanticNavigator._semantic_cancel(node, _request("cancel")))
    assert response["ok"] is True
    assert response["result"]["task"]["state"] == "canceling"
    assert node._target_cancel_event.is_set()

    wrong_owner = json.loads(SemanticNavigator._semantic_cancel(
        node, _request("cancel", client_id="other-client")))
    assert wrong_owner["reason_code"] == "task_owner_mismatch"


class Future:
    def __init__(self, result=None, done=True):
        self.value, self.is_done = result, done
        self.callbacks = []

    def done(self):
        return self.is_done

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        self.callbacks.append(callback)


class Handle:
    accepted = True

    def __init__(self, status):
        self.result_future = Future(SimpleNamespace(status=status))
        self.cancel_future = Future(SimpleNamespace(goals_canceling=[object()]))

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        return self.cancel_future


@pytest.mark.parametrize("status,expected", [
    (GoalStatus.STATUS_SUCCEEDED, True),
    (GoalStatus.STATUS_CANCELED, True),
    (GoalStatus.STATUS_ABORTED, True),
])
def test_cancel_nav_handle_requires_terminal_result_and_preserves_nav2_status(status, expected):
    node = SimpleNamespace(
        _wait_future=lambda future, _timeout, honor_replacement=True: future.result(),
        _resolve_late_nav_send=lambda safe: None,
        _mark_replacement_unsafe=lambda: None,
        _watch_nav_terminal=lambda *_args: None,
        _say=lambda _message: None,
    )
    safe, terminal_status = SemanticNavigator._cancel_nav_handle(
        node, Handle(status), return_status=True)
    assert safe is expected
    assert terminal_status == status


def test_cancel_nav_handle_unconfirmed_returns_false_and_registers_terminal_watch():
    watched = []
    handle = Handle(GoalStatus.STATUS_CANCELED)
    handle.result_future = Future(done=False)
    node = SimpleNamespace(
        _wait_future=lambda future, _timeout, honor_replacement=True: (
            future.result() if future.done() else None),
        _resolve_late_nav_send=lambda safe: None,
        _mark_replacement_unsafe=lambda: None,
        _watch_nav_terminal=lambda *args: watched.append(args),
        _say=lambda _message: None,
    )
    safe, terminal_status = SemanticNavigator._cancel_nav_handle(
        node, handle, return_status=True)
    assert safe is False
    assert terminal_status is None
    assert len(watched) == 1
