#!/usr/bin/env python3
"""
robot_v0 方向键遥控 → /cmd_vel

↑↓ 前进/后退，←→ 原地转弯，空格/s 急停，q 退出并刹车。
松键后 key_release_timeout 内自动发零速度（配合 cmd_vel_timeout 双保险）。
"""

from __future__ import annotations

import select
import sys
import termios
import tty
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

CMD_VEL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

HELP = """
========================================
  robot_v0 键盘遥控（方向键）
========================================
        ↑ 前进
   ← 左转   → 右转
        ↓ 后退
  空格 / s  立刻停止
  q         退出并刹车
========================================
松键约 0.2s 自动停；中继节点 0.5s 兜底刹车
请在本终端按键（不要切到别的窗口）
========================================
"""


def read_key(timeout_sec: float = 0.05) -> Optional[str]:
    """读取单键；方向键返回 UP/DOWN/LEFT/RIGHT。"""
    if not select.select([sys.stdin], [], [], timeout_sec)[0]:
        return None

    ch = sys.stdin.read(1)
    if ch != '\x1b':
        if ch in ('\x03', '\x04'):
            return 'QUIT'
        if ch in ('\r', '\n'):
            return None
        return ch.lower()

    if not select.select([sys.stdin], [], [], 0.01)[0]:
        return None
    ch2 = sys.stdin.read(1)
    if ch2 != '[' or not select.select([sys.stdin], [], [], 0.01)[0]:
        return None
    ch3 = sys.stdin.read(1)
    return {
        'A': 'UP',
        'B': 'DOWN',
        'C': 'RIGHT',
        'D': 'LEFT',
    }.get(ch3)


class KeyboardTeleop(Node):
    """方向键发布 /cmd_vel。"""

    def __init__(self) -> None:
        super().__init__('keyboard_teleop')

        # use_sim_time 由 launch 注入，勿在此 declare
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('key_release_timeout_sec', 0.2)

        topic = self.get_parameter('cmd_vel_topic').value
        self._linear = float(self.get_parameter('linear_speed').value)
        self._angular = float(self.get_parameter('angular_speed').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._release_timeout = float(self.get_parameter('key_release_timeout_sec').value)

        self._twist = Twist()
        self._moving = False
        self._last_key_time = self.get_clock().now()

        self._pub = self.create_publisher(Twist, topic, CMD_VEL_QOS)
        self.create_timer(1.0 / rate_hz, self._on_timer)

        if not sys.stdin.isatty():
            raise RuntimeError(
                'keyboard_teleop 必须在交互式终端运行。\n'
                '请用: bash ~/inspection-robot/scripts/run_gazebo_teleop.sh --build'
            )

        self._term_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        print(HELP, flush=True)

    def destroy_node(self) -> bool:
        self._publish_stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._term_settings)
        return super().destroy_node()

    def _publish_stop(self) -> None:
        self._twist = Twist()
        self._moving = False
        self._pub.publish(Twist())

    def _set_motion(self, linear_x: float, angular_z: float) -> None:
        self._twist.linear.x = linear_x
        self._twist.angular.z = angular_z
        self._moving = True
        self._last_key_time = self.get_clock().now()
        self._pub.publish(self._twist)

    def _on_timer(self) -> None:
        if not self._moving:
            return
        elapsed = (self.get_clock().now() - self._last_key_time).nanoseconds * 1e-9
        if elapsed > self._release_timeout:
            self._publish_stop()
            return
        self._pub.publish(self._twist)

    def spin_keyboard(self) -> None:
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            key = read_key(0.05)
            if key is None:
                continue
            if key in ('q', 'QUIT'):
                break
            if key in (' ', 's'):
                self._publish_stop()
                self.get_logger().info('急停')
                continue
            if key == 'UP':
                self._set_motion(self._linear, 0.0)
            elif key == 'DOWN':
                self._set_motion(-self._linear, 0.0)
            elif key == 'LEFT':
                self._set_motion(0.0, self._angular)
            elif key == 'RIGHT':
                self._set_motion(0.0, -self._angular)


def main() -> None:
    rclpy.init()
    node = KeyboardTeleop()
    try:
        node.spin_keyboard()
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
