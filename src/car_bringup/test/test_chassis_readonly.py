import math

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter

import car_bringup.chassis_node as chassis_module
from car_bringup.protocol import RobotStatus, read_s16, GYRO_SCALE


class FakeSerial:
    def __init__(self):
        self.is_open = True
        self.writes = []

    def write(self, data):
        self.writes.append(bytes(data))
        return len(data)

    def close(self):
        self.is_open = False


@pytest.fixture
def ros_context():
    initialized_here = not rclpy.ok()
    if initialized_here:
        rclpy.init()
    yield
    if initialized_here and rclpy.ok():
        rclpy.shutdown()


def _make_node(monkeypatch, enable_control, chassis_yaw_offset=None):
    fake = FakeSerial()
    monkeypatch.setattr(chassis_module, 'find_wheeltec_port', lambda: '/dev/fake-chassis')
    def fake_open(port, baud, *, exclusive=False):
        fake.exclusive = exclusive
        return fake
    monkeypatch.setattr(
        chassis_module, 'open_serial', fake_open)
    overrides = [Parameter('enable_control', value=enable_control)]
    if chassis_yaw_offset is not None:
        overrides.append(Parameter('chassis_yaw_offset', value=chassis_yaw_offset))
    node = chassis_module.ChassisNode(parameter_overrides=overrides)
    return node, fake


def test_sensor_only_mode_never_subscribes_or_writes(ros_context, monkeypatch):
    node, fake = _make_node(monkeypatch, False)
    assert node._control_sub is None
    assert fake.exclusive is True
    msg = Twist()
    msg.linear.x = 0.5
    node._cmd_vel_cb(msg)
    node.destroy_node()
    assert fake.writes == []


def test_legacy_control_mode_still_sends_and_stops(ros_context, monkeypatch):
    node, fake = _make_node(monkeypatch, True)
    assert node._control_sub is not None
    assert fake.exclusive is False
    msg = Twist()
    msg.linear.x = 0.5
    node._cmd_vel_cb(msg)
    node.destroy_node()
    assert len(fake.writes) == 2
    assert fake.writes[0] != fake.writes[1]


def test_yaw_offset_rotates_commands_and_feedback_but_preserves_yaw_rate(ros_context, monkeypatch):
    node, fake = _make_node(monkeypatch, True, chassis_yaw_offset=3.141592653589793)
    msg = Twist()
    msg.linear.x = 0.1
    msg.linear.y = 0.0
    msg.angular.z = 0.12
    node._cmd_vel_cb(msg)

    packet = fake.writes[0]
    assert read_s16(packet[3], packet[4]) == -100
    assert read_s16(packet[5], packet[6]) == 0
    assert read_s16(packet[7], packet[8]) == 120

    class Capture:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    vel_pub, imu_pub, bat_pub = Capture(), Capture(), Capture()
    node._vel_pub, node._imu_pub, node._bat_pub = vel_pub, imu_pub, bat_pub
    node._publish_all(RobotStatus(vel_x=-0.1, vel_y=0.2, vel_z=0.3, gyro_z=100))
    feedback = vel_pub.messages[0]
    assert feedback.linear.x == pytest.approx(0.1)
    assert feedback.linear.y == pytest.approx(-0.2)
    assert feedback.angular.z == pytest.approx(0.3)
    assert imu_pub.messages[0].header.frame_id == 'imu_link'
    assert imu_pub.messages[0].angular_velocity.z == pytest.approx(100*GYRO_SCALE)
    node.destroy_node()


def test_default_zero_yaw_offset_preserves_legacy_command_and_feedback(ros_context, monkeypatch):
    node, fake = _make_node(monkeypatch, True)
    assert node.get_parameter('chassis_yaw_offset').value == 0.0
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.angular.z = 0.1, -0.2, 0.12
    node._cmd_vel_cb(msg)
    packet = fake.writes[0]
    assert read_s16(packet[3], packet[4]) == 100
    assert read_s16(packet[5], packet[6]) == -200
    assert read_s16(packet[7], packet[8]) == 120

    class Capture:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    node._vel_pub = Capture()
    node._imu_pub = Capture()
    node._bat_pub = Capture()
    node._publish_all(RobotStatus(vel_x=0.1, vel_y=-0.2, vel_z=0.3))
    feedback = node._vel_pub.messages[0]
    assert feedback.linear.x == pytest.approx(0.1)
    assert feedback.linear.y == pytest.approx(-0.2)
    assert feedback.angular.z == pytest.approx(0.3)
    node.destroy_node()


def test_quarter_turn_offset_matches_base_to_imu_tf_direction(ros_context, monkeypatch):
    node, fake = _make_node(monkeypatch, True, chassis_yaw_offset=math.pi/2)
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = 0.12
    node._cmd_vel_cb(msg)

    packet = fake.writes[0]
    assert read_s16(packet[3], packet[4]) == 0
    assert read_s16(packet[5], packet[6]) == -100
    assert read_s16(packet[7], packet[8]) == 120

    class Capture:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    node._vel_pub = Capture()
    node._imu_pub = Capture()
    node._bat_pub = Capture()
    node._publish_all(RobotStatus(vel_x=0.0, vel_y=-0.2, vel_z=0.3))
    feedback = node._vel_pub.messages[0]
    assert feedback.linear.x == pytest.approx(0.2)
    assert feedback.linear.y == pytest.approx(0.0, abs=1e-12)
    assert feedback.angular.z == pytest.approx(0.3)
    node.destroy_node()
