"""
WHEELTEC C50X 底盘 ROS2 驱动节点

订阅 /cmd_vel → 串口发控制帧
读取串口状态帧 → 发布 /chassis_vel /imu_raw /battery
"""
import time
import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from .protocol import (
    RobotStatus, parse_status_frame, send_speed, stop_robot,
    open_serial, find_wheeltec_port, ACCEL_SCALE, GYRO_SCALE,
    FRAME_HEADER, STATUS_FRAME_SIZE,
)


def rotate_planar_velocity(vx: float, vy: float, yaw: float):
    """Rotate a planar velocity vector by yaw radians."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c*vx - s*vy, s*vx + c*vy


class ChassisNode(Node):
    def __init__(self, *, parameter_overrides=None):
        super().__init__('chassis_node', parameter_overrides=parameter_overrides)

        # ---- 参数 ----
        self.declare_parameter('port', 'auto')
        self.declare_parameter('baudrate', 460800)
        # Legacy chassis launches keep control enabled. Sensor-only launches
        # can disable the /cmd_vel subscription and every serial write.
        self.declare_parameter('enable_control', True)
        self.declare_parameter('chassis_yaw_offset', 0.0)
        # This is the chassis/IMU child-frame yaw in logical base_link,
        # matching the base_link -> imu_link static transform.
        self._chassis_yaw_offset = float(self.get_parameter('chassis_yaw_offset').value)
        if not math.isfinite(self._chassis_yaw_offset):
            raise ValueError('chassis_yaw_offset must be finite')
        self._control_enabled = self.get_parameter('enable_control').value
        if not isinstance(self._control_enabled, bool):
            raise TypeError('enable_control must be a boolean')

        # ---- 串口 ----
        self._serial = None
        self._buf = bytearray()
        self._last_reconnect_attempt = 0.0

        # ---- 发布器 ----
        # /chassis_vel 使用 Twist 类型，仅供遥控/监控参考
        self._vel_pub = self.create_publisher(Twist, '/chassis_vel', 10)
        self._imu_pub = self.create_publisher(Imu, '/imu_raw', 200)
        self._bat_pub = self.create_publisher(Float32, '/battery', 10)

        # ---- 订阅器 ----
        self._control_sub = None
        if self._control_enabled:
            self._control_sub = self.create_subscription(
                Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # ---- 定时器: 200Hz 读取状态帧 ----
        self.create_timer(0.005, self._read_status)

        self._open_port()
        if self._control_enabled:
            self.get_logger().info('底盘节点已启动，等待 /cmd_vel 指令...')
        else:
            self.get_logger().info('只读 IMU 传感器节点已启动，串口仅接收状态帧；底盘控制已禁用。')

    # ==================== 串口管理 ====================

    def _open_port(self) -> bool:
        if self._serial and self._serial.is_open:
            return True

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        if port == 'auto':
            port = find_wheeltec_port()
            if not port:
                self.get_logger().error(
                    '未找到 WHEELTEC 串口，将继续重试')
                return False
            self.get_logger().info(f'自动检测到串口: {port}')
        try:
            self._serial = open_serial(
                port, baud, exclusive=not self._control_enabled)
            self.get_logger().info(f'串口已连接 {port} @ {baud} baud')
            return True
        except Exception as e:
            self._serial = None
            self.get_logger().error(f'打开串口失败 {port}: {e}，将继续重试')
            return False

    def _close_port(self):
        if self._serial:
            try:
                if self._serial.is_open:
                    self._serial.close()
            except Exception:
                pass
        self._serial = None

    def _try_reconnect(self):
        now = time.monotonic()
        if now - self._last_reconnect_attempt < 1.0:
            return
        self._last_reconnect_attempt = now
        self._open_port()

    # ==================== 指令回调 ====================

    def _cmd_vel_cb(self, msg: Twist):
        """收到 /cmd_vel → 发串口"""
        if not self._control_enabled:
            return
        if not self._serial or not self._serial.is_open:
            return
        try:
            # Child-frame coordinates are the inverse of the base->IMU TF rotation.
            vx, vy = rotate_planar_velocity(
                msg.linear.x, msg.linear.y, -self._chassis_yaw_offset)
            send_speed(self._serial, vx, vy, msg.angular.z)
        except Exception as e:
            self.get_logger().error(f'发送控制帧失败: {e}')

    # ==================== 状态读取 ====================

    def _read_status(self):
        """定时器回调: 读串口 → 解析 → 发布"""
        if not self._serial or not self._serial.is_open:
            self._try_reconnect()
            return
        try:
            n = self._serial.in_waiting
            if n > 0:
                self._buf.extend(self._serial.read(n))
                self._parse_and_publish()
        except Exception as e:
            self.get_logger().error(f'读取串口失败: {e}，关闭串口并重连')
            self._close_port()

    def _parse_and_publish(self):
        """从缓冲区提取帧并发布话题"""
        while len(self._buf) >= STATUS_FRAME_SIZE:
            # 找帧头
            idx = self._buf.find(bytes([FRAME_HEADER]))
            if idx < 0:
                self._buf.clear()
                return
            if idx > 0:
                self._buf = self._buf[idx:]
                continue
            if len(self._buf) < STATUS_FRAME_SIZE:
                return

            frame = bytes(self._buf[:STATUS_FRAME_SIZE])
            status = parse_status_frame(frame)
            if status:
                self._publish_all(status)
            self._buf = self._buf[STATUS_FRAME_SIZE:]

    def _publish_all(self, s: RobotStatus):
        """根据 RobotStatus 发布三个话题"""

        # ---- /chassis_vel (geometry_msgs/Twist) ----
        # Firmware reports child-frame velocity; rotate it into logical base_link.
        vel_x, vel_y = rotate_planar_velocity(
            s.vel_x, s.vel_y, self._chassis_yaw_offset)
        twist = Twist()
        twist.linear.x = vel_x
        twist.linear.y = vel_y
        twist.angular.z = s.vel_z
        self._vel_pub.publish(twist)

        # ---- /imu_raw ----
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'
        # 原始 accel → m/s²
        imu.linear_acceleration.x = s.accel_x * ACCEL_SCALE
        imu.linear_acceleration.y = s.accel_y * ACCEL_SCALE
        imu.linear_acceleration.z = s.accel_z * ACCEL_SCALE
        # 原始 gyro → rad/s
        imu.angular_velocity.x = s.gyro_x * GYRO_SCALE
        imu.angular_velocity.y = s.gyro_y * GYRO_SCALE
        imu.angular_velocity.z = s.gyro_z * GYRO_SCALE
        # 无磁力计 → orientation 不可用
        imu.orientation_covariance[0] = -1.0
        # 加速度计协方差（~0.06 m/s² 噪声水平，对角线占优）
        imu.linear_acceleration_covariance = [
            0.004, 0.0, 0.0,
            0.0, 0.004, 0.0,
            0.0, 0.0, 0.004,
        ]
        # 陀螺仪协方差（~0.1 rad/s 噪声水平，含振动/偏置漂移余量）
        # 社区标准值 0.01，参考 robot_localization 作者确认和 MPU6050 Allan方差分析
        imu.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]
        self._imu_pub.publish(imu)

        # ---- /battery ----
        bat = Float32()
        bat.data = s.voltage
        self._bat_pub.publish(bat)

    # ==================== 清理 ====================

    def destroy_node(self):
        if self._serial and self._serial.is_open:
            if self._control_enabled:
                try:
                    stop_robot(self._serial)
                except Exception:
                    pass
            self._serial.close()
            self.get_logger().info('串口已关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChassisNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
