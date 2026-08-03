"""
WHEELTEC C50X 键盘遥控节点

读取按键 → 发布 Twist 到 /cmd_vel 控制底盘

按键：
  W/S    前进/后退   速度用 Q/E 调节，初始 0.1 m/s
  A/D    左转/右转   固定 0.2 rad/s
  Q/E    减速/加速   步进 0.05 m/s
  Space  急停
  松开   自动归零
"""

import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # 速度参数
        self._linear_speed = 0.1       # m/s，W/S 的当前速度
        self._angular_speed = 0.2      # rad/s，A/D 固定值
        self._linear_step = 0.05       # Q/E 每次加减步进

        # 当前运动状态
        self._linear_x = 0.0
        self._angular_z = 0.0

        # 发布器
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 定时器：50Hz 持续发布，确保底盘不收不到指令时自动归零
        self._timer = self.create_timer(0.02, self._publish_cmd)

        self._print_help()
        self._print_status()

    def _print_help(self):
        print()
        print('┌──────────────────────────────────────────────┐')
        print('│          键盘遥控 — WHEELTEC C50X            │')
        print('├──────────────────────────────────────────────┤')
        print('│  W         前进                    Q  减速  │')
        print('│  S         后退                    E  加速  │')
        print('│  A         左转    Space  急停              │')
        print('│  D         右转    Ctrl+C 退出              │')
        print('├──────────────────────────────────────────────┤')
        print('│  松开按键自动停止                            │')
        print('└──────────────────────────────────────────────┘')

    def _print_status(self):
        print(f'\r  线速度: {self._linear_speed:5.2f} m/s  '
              f'角速度(A/D): {self._angular_speed:.1f} rad/s   '
              f'当前指令: v={self._linear_x:+.2f}  w={self._angular_z:+.2f}  ', end='', flush=True)

    def _publish_cmd(self):
        """发布当前指令到 /cmd_vel"""
        msg = Twist()
        msg.linear.x = self._linear_x
        msg.angular.z = self._angular_z
        self._pub.publish(msg)

    def run(self):
        """主循环：读取按键并发布速度指令"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                # 非阻塞读按键（100ms 超时）
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)

                    if key == '\x03':  # Ctrl+C
                        break

                    # ── 方向键 ──
                    if key == 'w' or key == 'W':
                        self._linear_x = self._linear_speed
                    elif key == 's' or key == 'S':
                        self._linear_x = -self._linear_speed
                    elif key == 'a' or key == 'A':
                        self._angular_z = self._angular_speed
                    elif key == 'd' or key == 'D':
                        self._angular_z = -self._angular_speed

                    # ── 加减速 ──
                    elif key == 'q' or key == 'Q':
                        self._linear_speed = max(0.05, self._linear_speed - self._linear_step)
                        if self._linear_x != 0.0:
                            self._linear_x = self._linear_speed if self._linear_x > 0 else -self._linear_speed
                    elif key == 'e' or key == 'E':
                        self._linear_speed = min(1.0, self._linear_speed + self._linear_step)
                        if self._linear_x != 0.0:
                            self._linear_x = self._linear_speed if self._linear_x > 0 else -self._linear_speed

                    # ── 急停 ──
                    elif key == ' ':
                        self._linear_x = 0.0
                        self._angular_z = 0.0
                        self._linear_speed = 0.1

                    # ── 松开按键 → 归零 ──
                    else:
                        self._linear_x = 0.0
                        self._angular_z = 0.0

                    self._publish_cmd()
                    self._print_status()

                else:
                    # 超时未按键 → 归零（防止程序切走时车还在跑）
                    if self._linear_x != 0.0 or self._angular_z != 0.0:
                        self._linear_x = 0.0
                        self._angular_z = 0.0
                        self._publish_cmd()
                        self._print_status()

                # spin_once 处理 ROS 内部事件
                rclpy.spin_once(self, timeout_sec=0)

        finally:
            # 退出前发送停止指令
            msg = Twist()
            self._pub.publish(msg)
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            print('\n已停止')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
