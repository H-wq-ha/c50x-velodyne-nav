import math
import threading
from types import SimpleNamespace

import pytest
from sensor_msgs.msg import CameraInfo

from semantic_mapping.semantic_navigation import (
    SemanticNavigator,
    camera_view_score,
    project_camera_point,
    resolve_target,
    target_in_camera,
)
from semantic_mapping.nav2_config import configure_semantic_navigation_params


def _camera_info():
    info = CameraInfo()
    info.width, info.height = 640, 480
    info.k = [604.0, 0.0, 321.8, 0.0, 604.0, 253.1, 0.0, 0.0, 1.0]
    info.d = [0.0] * 5
    info.distortion_model = 'plumb_bob'
    return info


def _map(width=31, height=31, resolution=0.1, yaw=math.pi/2):
    q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw/2), w=math.cos(yaw/2))
    origin = SimpleNamespace(position=SimpleNamespace(x=10.0, y=20.0), orientation=q)
    info = SimpleNamespace(width=width, height=height, resolution=resolution, origin=origin)
    return SimpleNamespace(info=info, data=[0] * (width*height))


def test_camera_projection_uses_calibrated_positive_optical_axes():
    pixel = project_camera_point((0.1, 0.05, 1.0), _camera_info())
    assert pixel == pytest.approx((382.2, 283.3))
    assert camera_view_score((0.1, 0.05, 1.0), _camera_info()) is not None
    assert project_camera_point((0.0, 0.0, -1.0), _camera_info()) is None


def test_target_projection_accounts_for_camera_offset_and_heading():
    target = (0.4, -0.2, 2.5)
    optical = target_in_camera(target, (0.0, 0.0), 0.0,
                               ((0.4, -0.2, 0.5), (0.0, 0.0, 0.0, 1.0)))
    assert optical == pytest.approx((0.0, 0.0, 2.0))
    off_axis = target_in_camera((0.6, -0.2, 2.5), (0.0, 0.0), 0.0,
                                ((0.4, -0.2, 0.5), (0.0, 0.0, 0.0, 1.0)))
    assert project_camera_point(off_axis, _camera_info())[0] > _camera_info().k[2]
    rotated = target_in_camera((0.0, 1.0, 1.5), (0.0, 0.0), math.pi/2,
                               ((1.0, 0.0, 0.5), (0.0, 0.0, 0.0, 1.0)))
    assert rotated == pytest.approx((0.0, 0.0, 1.0))


def test_footprint_checks_inverse_rotated_origin_and_unknown_cells():
    occupancy = _map()
    navigator = SimpleNamespace(map_msg=occupancy)
    # In the rotated map, local cell (15, 5) is world point (9.5, 21.5).
    assert SemanticNavigator._map_free(navigator, 9.5, 21.5)
    # A cell intersecting the 0.50 m circumscribed footprint blocks the candidate.
    occupancy.data[5*31 + 19] = 100
    assert not SemanticNavigator._map_free(navigator, 9.5, 21.5)
    occupancy.data[5*31 + 19] = -1
    assert not SemanticNavigator._map_free(navigator, 9.5, 21.5)
    # Flooring and inverse-origin rotation reject candidates outside the map.
    assert not SemanticNavigator._map_free(navigator, 9.8, 20.2)


def test_target_resolution_only_uses_confirmed_rows_and_rejects_unknown_or_ambiguous():
    rows = [
        (1, 39, -4.3, -0.7, 0.4, 114),
        (2, 39, -2.0, 1.0, 0.4, 1),
        (3, 39, -3.0, 2.0, 0.4, 20),
    ]
    assert resolve_target(rows, '瓶子:1') == rows[0]
    with pytest.raises(RuntimeError, match='ambiguous'):
        resolve_target(rows, 'bottle')
    with pytest.raises(RuntimeError, match='unknown target'):
        resolve_target(rows, 'chair')
    with pytest.raises(RuntimeError, match='confirmed'):
        resolve_target(rows, 'bottle:2')


def test_target_callback_queues_latest_command_while_navigation_is_busy(monkeypatch):
    import semantic_mapping.semantic_navigation as navigation

    started = []

    class FakeThread:
        def __init__(self, *, target, args, daemon):
            assert daemon
            self.call = (target, args)

        def start(self):
            started.append(self.call[1][0])

    monkeypatch.setattr(navigation.threading, 'Thread', FakeThread)
    messages = []
    node = SimpleNamespace(
        _busy=False, _pending_target=None, _active_target_key=None,
        _pending_target_key=None, _target_lock=threading.Lock(),
        _target_cancel_event=threading.Event(), _replacement_safe=True,
        _db_objects=lambda: [(1, 39, 0, 0, 0, 3), (2, 41, 0, 0, 0, 3),
                             (3, 63, 0, 0, 0, 3)],
        _say=messages.append, _navigate=lambda command: None,
    )

    SemanticNavigator._target_cb(node, SimpleNamespace(data=' bottle '))
    SemanticNavigator._target_cb(node, SimpleNamespace(data='bottle:1'))
    SemanticNavigator._target_cb(node, SimpleNamespace(data='cup'))
    SemanticNavigator._target_cb(node, SimpleNamespace(data='cup'))
    SemanticNavigator._target_cb(node, SimpleNamespace(data='laptop'))

    assert started == ['bottle:1']
    assert node._pending_target == 'laptop:3'
    assert node._target_cancel_event.is_set()
    assert any('Already processing bottle:1; duplicate request merged' in message
               for message in messages)
    assert any('Already queued cup:2; duplicate request merged' in message
               for message in messages)
    assert any('Replaced queued target with the latest command' in message
               for message in messages)

    SemanticNavigator._finish_target(node, worker_finished=True)
    assert started == ['bottle:1', 'laptop:3']
    assert node._busy

    SemanticNavigator._finish_target(node, worker_finished=True)
    assert not node._busy


def test_localization_freshness_waits_for_next_tf_update_with_one_second_limit(monkeypatch):
    import semantic_mapping.semantic_navigation as navigation

    stamps = iter((8, 9))

    class FakeBuffer:
        def lookup_transform(self, *_args, **_kwargs):
            return SimpleNamespace(header=SimpleNamespace(
                stamp=SimpleNamespace(sec=next(stamps), nanosec=0)))

    node = SimpleNamespace(
        map_frame='map', base_frame='base_link', tf_buffer=FakeBuffer(),
        _target_cancel_event=threading.Event(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=10_000_000_000)),
    )
    monkeypatch.setattr(navigation.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(navigation.time, 'sleep', lambda _seconds: None)

    SemanticNavigator._require_fresh_localization(node)


def test_new_target_interrupts_active_wait_and_unconfirmed_cancel_holds_queue(monkeypatch):
    import semantic_mapping.semantic_navigation as navigation

    started = []

    class FakeThread:
        def __init__(self, *, target, args, daemon):
            self.args = args

        def start(self):
            started.append(self.args[0])

    class Future:
        def __init__(self, done, value=None):
            self._done, self._value, self._callbacks = done, value, []

        def done(self):
            return self._done

        def result(self):
            return self._value

        def add_done_callback(self, callback):
            self._callbacks.append(callback)
            if self._done:
                callback(self)

    class Handle:
        def __init__(self, result_future):
            self.result_future = result_future
            self.cancel_calls = 0
            self.result_calls = 0

        def cancel_goal_async(self):
            self.cancel_calls += 1
            return Future(True, SimpleNamespace(goals_canceling=[object()]))

        def get_result_async(self):
            self.result_calls += 1
            return self.result_future

    monkeypatch.setattr(navigation.threading, 'Thread', FakeThread)
    monkeypatch.setattr(navigation.rclpy, 'ok', lambda: True)
    ticks = iter(range(2000))
    monkeypatch.setattr(navigation.time, 'monotonic', lambda: next(ticks)/100.0)
    monkeypatch.setattr(navigation.time, 'sleep', lambda _seconds: None)
    messages = []
    handle = Handle(Future(False))
    node = SimpleNamespace(
        _busy=True, _pending_target=None, _active_target_key=(39, 1),
        _pending_target_key=None, _target_lock=threading.Lock(),
        _target_worker_active=True, _terminal_watch_handles=set(),
        _target_cancel_event=threading.Event(), _replacement_safe=True,
        _db_objects=lambda: [(1, 39, 0, 0, 0, 3), (2, 41, 0, 0, 0, 3)],
        _active_nav_handle=handle, _say=messages.append,
    )
    node._wait_future = lambda future, timeout, honor_replacement=True: \
        SemanticNavigator._wait_future(node, future, timeout, honor_replacement)
    node._watch_nav_terminal = lambda action, future: \
        SemanticNavigator._watch_nav_terminal(node, action, future)
    node._mark_replacement_unsafe = lambda: \
        SemanticNavigator._mark_replacement_unsafe(node)
    node._resolve_late_nav_send = lambda safe: \
        SemanticNavigator._resolve_late_nav_send(node, safe)
    node._finish_target = lambda worker_finished=False: \
        SemanticNavigator._finish_target(node, worker_finished)

    SemanticNavigator._target_cb(node, SimpleNamespace(data='cup'))
    with pytest.raises(navigation.TargetReplaced):
        SemanticNavigator._wait_future(node, Future(False), 1)
    with pytest.raises(navigation.TargetReplaced):
        SemanticNavigator._wait_future(node, Future(True, 'already done'), 1)

    SemanticNavigator._cancel_nav_handle(node, handle)
    SemanticNavigator._finish_target(node, worker_finished=True)

    assert not node._replacement_safe
    assert handle.cancel_calls == 1
    assert handle.result_calls == 1
    assert node._busy
    assert node._pending_target == 'cup:2'
    assert started == []
    assert any('held until Nav2 goal cancellation is confirmed' in message
               for message in messages)

    node._pending_target = None
    node._pending_target_key = None
    confirmed = Handle(Future(True, SimpleNamespace(status=navigation.GoalStatus.STATUS_CANCELED)))
    assert SemanticNavigator._cancel_nav_handle(node, confirmed)
    assert confirmed.cancel_calls == 1
    assert confirmed.result_calls == 1


def test_active_target_repeated_after_different_target_restores_it_as_latest(monkeypatch):
    import semantic_mapping.semantic_navigation as navigation

    started = []

    class FakeThread:
        def __init__(self, *, target, args, daemon):
            self.args = args

        def start(self):
            started.append(self.args[0])

    monkeypatch.setattr(navigation.threading, 'Thread', FakeThread)
    rows = [(1, 39, 0, 0, 0, 3), (2, 41, 0, 0, 0, 3)]
    node = SimpleNamespace(
        _busy=False, _pending_target=None, _active_target_key=None,
        _pending_target_key=None, _target_lock=threading.Lock(),
        _target_cancel_event=threading.Event(), _replacement_safe=True,
        _db_objects=lambda: rows, _say=lambda _message: None,
        _navigate=lambda _command: None,
    )

    SemanticNavigator._target_cb(node, SimpleNamespace(data='bottle'))
    SemanticNavigator._target_cb(node, SimpleNamespace(data='cup'))
    SemanticNavigator._target_cb(node, SimpleNamespace(data='bottle'))

    assert node._pending_target == 'bottle:1'
    assert node._pending_target_key == (39, 1)
    assert node._target_cancel_event.is_set()
    SemanticNavigator._finish_target(node, worker_finished=True)
    assert started == ['bottle:1', 'bottle:1']


def test_late_nav_acceptance_is_canceled_then_pending_target_starts_once(monkeypatch):
    import semantic_mapping.semantic_navigation as navigation

    started = []

    class FakeThread:
        def __init__(self, *, target, args, daemon):
            self.target, self.args = target, args

        def start(self):
            if isinstance(self.args[0], LateHandle):
                self.target(*self.args)
            else:
                started.append(self.args[0])

    class Future:
        def __init__(self, done=False, value=None):
            self._done, self._value, self._callbacks = done, value, []

        def done(self):
            return self._done

        def result(self):
            return self._value

        def add_done_callback(self, callback):
            self._callbacks.append(callback)
            if self._done:
                callback(self)

        def complete(self, value):
            self._done, self._value = True, value
            for callback in self._callbacks:
                callback(self)

    class LateHandle:
        accepted = True

        def __init__(self):
            self.cancel_calls = 0

        def cancel_goal_async(self):
            self.cancel_calls += 1
            return Future(True, SimpleNamespace(goals_canceling=[object()]))

        def get_result_async(self):
            return Future(True, SimpleNamespace(status=navigation.GoalStatus.STATUS_CANCELED))

    monkeypatch.setattr(navigation.threading, 'Thread', FakeThread)
    monkeypatch.setattr(navigation.rclpy, 'ok', lambda: True)
    logs = []
    node = SimpleNamespace(
        _target_lock=threading.Lock(), _target_worker_active=False,
        _replacement_safe=False, _busy=True, _pending_target='cup:2',
        _active_target_key=(39, 1), _pending_target_key=(41, 2),
        _target_cancel_event=threading.Event(), _say=logs.append,
        _navigate=lambda _command: None,
    )
    node._wait_future = lambda future, timeout, honor_replacement=True: \
        SemanticNavigator._wait_future(node, future, timeout, honor_replacement)
    node._cancel_nav_handle = lambda handle: \
        SemanticNavigator._cancel_nav_handle(node, handle)
    node._cancel_late_nav_handle = lambda handle: \
        SemanticNavigator._cancel_late_nav_handle(node, handle)
    node._resolve_late_nav_send = lambda safe: \
        SemanticNavigator._resolve_late_nav_send(node, safe)
    node._finish_target = lambda worker_finished=False: \
        SemanticNavigator._finish_target(node, worker_finished)

    send_future = Future()
    SemanticNavigator._watch_late_nav_acceptance(node, send_future)
    handle = LateHandle()
    send_future.complete(handle)

    assert handle.cancel_calls == 1
    assert node._replacement_safe
    assert node._target_worker_active
    assert node._pending_target is None
    assert started == ['cup:2']
    assert any('Late Nav2 goal acceptance was canceled' in message for message in logs)

    # A late resolution before the worker's finally block must defer dispatch;
    # the worker finish then starts the queued command exactly once.
    node._replacement_safe = False
    node._pending_target = 'laptop:3'
    node._pending_target_key = (63, 3)
    second_send = Future()
    SemanticNavigator._watch_late_nav_acceptance(node, second_send)
    second_send.complete(LateHandle())
    assert started == ['cup:2']
    assert node._pending_target == 'laptop:3'
    SemanticNavigator._finish_target(node, worker_finished=True)
    assert started == ['cup:2', 'laptop:3']

    # A late rejection is also a terminal resolution and releases the latest
    # queued request once the active worker has already left.
    class Rejected:
        accepted = False

    node._busy = True
    node._target_worker_active = False
    node._replacement_safe = False
    node._pending_target = 'cup:2'
    node._pending_target_key = (41, 2)
    third_send = Future()
    SemanticNavigator._watch_late_nav_acceptance(node, third_send)
    third_send.complete(Rejected())
    assert node._replacement_safe
    assert node._target_worker_active
    assert started == ['cup:2', 'laptop:3', 'cup:2']


def test_late_terminal_result_releases_held_target(monkeypatch):
    import semantic_mapping.semantic_navigation as navigation

    started = []

    class Future:
        def __init__(self, done=False, value=None):
            self._done, self._value, self._callbacks = done, value, []

        def done(self):
            return self._done

        def result(self):
            return self._value

        def add_done_callback(self, callback):
            self._callbacks.append(callback)
            if self._done:
                callback(self)

        def complete(self, value):
            self._done, self._value = True, value
            for callback in self._callbacks:
                callback(self)

    class Handle:
        def __init__(self):
            self.result_future = Future()

        def cancel_goal_async(self):
            return Future(True, SimpleNamespace(goals_canceling=[]))

        def get_result_async(self):
            return self.result_future

    class FakeThread:
        def __init__(self, *, target, args, daemon):
            self.args = args

        def start(self):
            started.append(self.args[0])

    monkeypatch.setattr(navigation.threading, 'Thread', FakeThread)
    monkeypatch.setattr(navigation.rclpy, 'ok', lambda: True)
    ticks = iter(range(2000))
    monkeypatch.setattr(navigation.time, 'monotonic', lambda: next(ticks)/100.0)
    monkeypatch.setattr(navigation.time, 'sleep', lambda _seconds: None)
    messages = []
    handle = Handle()
    node = SimpleNamespace(
        _target_lock=threading.Lock(), _target_worker_active=False,
        _terminal_watch_handles=set(), _replacement_safe=True,
        _busy=True, _pending_target='cup:2', _pending_target_key=(41, 2),
        _active_target_key=(39, 1), _target_cancel_event=threading.Event(),
        _say=messages.append, _navigate=lambda _command: None,
    )
    node._wait_future = lambda future, timeout, honor_replacement=True: \
        SemanticNavigator._wait_future(node, future, timeout, honor_replacement)
    node._watch_nav_terminal = lambda action, future: \
        SemanticNavigator._watch_nav_terminal(node, action, future)
    node._mark_replacement_unsafe = lambda: \
        SemanticNavigator._mark_replacement_unsafe(node)
    node._resolve_late_nav_send = lambda safe: \
        SemanticNavigator._resolve_late_nav_send(node, safe)
    node._finish_target = lambda worker_finished=False: \
        SemanticNavigator._finish_target(node, worker_finished)

    assert not SemanticNavigator._cancel_nav_handle(node, handle)
    SemanticNavigator._finish_target(node)
    assert not node._replacement_safe
    assert node._busy
    assert started == []

    handle.result_future.complete(SimpleNamespace(
        status=navigation.GoalStatus.STATUS_CANCELED))

    assert node._replacement_safe
    assert node._target_worker_active
    assert node._pending_target is None
    assert node._busy
    assert started == ['cup:2']
    assert any('releasing queued target' in message for message in messages)


def test_invalid_target_does_not_cancel_current_navigation():
    messages = []
    node = SimpleNamespace(
        _busy=True, _pending_target=None, _active_target_key=(39, 1),
        _pending_target_key=None, _target_lock=threading.Lock(),
        _target_cancel_event=threading.Event(),
        _db_objects=lambda: [(1, 39, 0, 0, 0, 3)],
        _say=messages.append,
    )

    SemanticNavigator._target_cb(node, SimpleNamespace(data='chair'))

    assert not node._target_cancel_event.is_set()
    assert node._pending_target is None
    assert node._active_target_key == (39, 1)
    assert any('unknown target' in message for message in messages)


def test_semantic_nav_mppi_matches_control_period_and_preserves_speed_limits():
    params = {
        'bt_navigator': {'ros__parameters': {
            'default_server_timeout': 20,
            'bt_loop_duration': 10,
            'wait_for_service_timeout': 1000,
        }},
        'controller_server': {'ros__parameters': {
            'controller_frequency': 10.0,
            'controller_plugins': ['FollowPath'],
            'progress_checker': {
                'required_movement_radius': 0.2, 'movement_time_allowance': 15.0,
            },
            'FollowPath': {'plugin': 'dwb_core::DWBLocalPlanner', 'max_vel_x': 0.5},
        }},
        'velocity_smoother': {'ros__parameters': {
            'max_velocity': [0.5, 0.0, 0.5], 'min_velocity': [-0.5, 0.0, -0.5],
            'max_accel': [0.3, 0.0, 0.4], 'max_decel': [-0.3, 0.0, -0.4],
        }},
        'behavior_server': {'ros__parameters': {
            'max_rotational_vel': 0.5, 'min_rotational_vel': 0.05,
            'rotational_acc_lim': 0.4,
        }},
    }

    configure_semantic_navigation_params(params)

    controller = params['controller_server']['ros__parameters']
    follow_path = controller['FollowPath']
    bt = params['bt_navigator']['ros__parameters']
    assert bt['default_server_timeout'] == 1500
    assert bt['bt_loop_duration'] == 50
    assert bt['wait_for_service_timeout'] == 3000
    assert follow_path['model_dt'] == pytest.approx(1.0 / controller['controller_frequency'])
    assert controller['progress_checker']['required_movement_radius'] == pytest.approx(0.08)
    assert controller['progress_checker']['movement_time_allowance'] == pytest.approx(45.0)
    assert follow_path['plugin'] == 'nav2_mppi_controller::MPPIController'
    assert follow_path['motion_model'] == 'DiffDrive'
    assert follow_path['vx_max'] == params['velocity_smoother']['ros__parameters']['max_velocity'][0]
    assert follow_path['wz_max'] == params['velocity_smoother']['ros__parameters']['max_velocity'][2]
    assert follow_path['vx_min'] == 0.0
    assert follow_path['vy_max'] == 0.0
    assert 'ObstaclesCritic' in follow_path['critics']
    assert follow_path['ObstaclesCritic']['enabled'] is True
    assert follow_path['ObstaclesCritic']['consider_footprint'] is True
    assert 'max_vel_x' not in follow_path
    assert params['velocity_smoother']['ros__parameters']['max_velocity'] == [0.12, 0.0, 0.2]
    assert params['velocity_smoother']['ros__parameters']['min_velocity'] == [-0.12, 0.0, -0.2]
    assert params['behavior_server']['ros__parameters']['max_rotational_vel'] == pytest.approx(0.2)
