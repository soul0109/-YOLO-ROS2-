#!/usr/bin/env python3
"""
robot_v0 键盘遥控 → /cmd_vel

按住 W/A/S/D 或方向键持续运动，可组合（如 W+A 前进左转）；松手后超时停止；空格急停。

按键状态：每个方向独立记「最后一次按下时间」。
Linux 终端同时只 auto-repeat 一个键 → W+A 时只有 A 在 repeat，必须用 A 的 repeat 给 W 续命。
"""

from __future__ import annotations

import select
import sys
import termios
import time
import tty
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

HELP = """
========================================
  robot_v0 键盘遥控（按住运动，松手停）
========================================
  W / S     前进 / 后退
  A / D     左转 / 右转
  组合键    如 W+A 前进左转、W+D 前进右转
  方向键    同上（VMware 建议仍用 WASD 更稳）
  空格      急停
  Q         退出
========================================
按住只打一次日志；松手约 0.8s 后自动停。
========================================
"""

_ARROW_TAIL: dict[str, str] = {
    'A': 'UP',
    'B': 'DOWN',
    'C': 'RIGHT',
    'D': 'LEFT',
}

_LETTER_KEYS: dict[str, str] = {
    'w': 'UP', 's': 'DOWN', 'a': 'LEFT', 'd': 'RIGHT',
    'i': 'UP', 'k': 'DOWN', 'j': 'LEFT', 'l': 'RIGHT',
}

_ACTION_TO_AXIS: dict[str, str] = {
    'UP': 'fwd',
    'DOWN': 'back',
    'LEFT': 'left',
    'RIGHT': 'right',
}

_AXIS_LABELS: dict[str, str] = {
    'fwd': '前进',
    'back': '后退',
    'left': '左转',
    'right': '右转',
}

# 同轴互斥：按 W 时清掉 S 的时间戳，反之亦然
_AXIS_OPPOSITE: dict[str, str] = {
    'fwd': 'back',
    'back': 'fwd',
    'left': 'right',
    'right': 'left',
}

_LINEAR_AXES = frozenset({'fwd', 'back'})
_ANGULAR_AXES = frozenset({'left', 'right'})


class KeyReader:
    """带缓冲的按键读取，避免方向键 ESC 序列被拆成单个字母（如 D→d=右转）。"""

    def __init__(self) -> None:
        self._buf = ''

    def poll(self, timeout_sec: float = 0.05) -> Optional[str]:
        if not self._buf:
            if not select.select([sys.stdin], [], [], timeout_sec)[0]:
                return None
            self._buf += sys.stdin.read(1)

        while select.select([sys.stdin], [], [], 0)[0]:
            self._buf += sys.stdin.read(1)

        return self._consume_one()

    def _consume_one(self) -> Optional[str]:
        while self._buf:
            ch = self._buf[0]

            if ch in ('\x03', '\x04'):
                self._buf = self._buf[1:]
                return 'QUIT'
            if ch in ('\r', '\n'):
                self._buf = self._buf[1:]
                continue

            if ch == '\x1b':
                action = self._parse_escape()
                if action is not None:
                    return action
                return None

            if ch == '[' or ch.isdigit() or ch in ';':
                self._buf = self._buf[1:]
                continue

            self._buf = self._buf[1:]
            return ch.lower()

        return None

    def _parse_escape(self) -> Optional[str]:
        """解析 \\x1b[? 或 \\x1bO? 方向键；没收齐则返回 None（保留 buf）。"""
        buf = self._buf
        if len(buf) < 2:
            return None

        if buf[1] == 'O':
            if len(buf) < 3:
                return None
            tail = buf[2]
            self._buf = buf[3:]
            return _ARROW_TAIL.get(tail)

        if buf[1] == '[':
            for i in range(2, len(buf)):
                if buf[i] in _ARROW_TAIL:
                    action = _ARROW_TAIL[buf[i]]
                    self._buf = buf[i + 1:]
                    return action
            if len(buf) > 12:
                self._buf = ''
            return None

        self._buf = buf[1:]
        return None


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__('keyboard_teleop')

        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('key_release_timeout_sec', 0.8)

        topic = self.get_parameter('cmd_vel_topic').value
        self._linear = float(self.get_parameter('linear_speed').value)
        self._angular = float(self.get_parameter('angular_speed').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._release_timeout = float(self.get_parameter('key_release_timeout_sec').value)

        self._twist = Twist()
        self._moving = False
        self._last_label: Optional[str] = None
        # 每个方向独立超时：W 与 A/D 互不影响
        self._axis_last_seen: dict[str, float] = {}
        self._keys = KeyReader()

        self._pub = self.create_publisher(Twist, topic, qos_profile_system_default)
        self.create_timer(1.0 / rate_hz, self._on_timer)

        if not sys.stdin.isatty():
            raise RuntimeError(
                'keyboard_teleop 必须在交互式终端运行。\n'
                '请用: bash ~/inspection-robot/scripts/run_gazebo_teleop.sh --build'
            )

        print(HELP, flush=True)
        self._term_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

    def destroy_node(self) -> bool:
        self._publish_stop(manual=False)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._term_settings)
        return super().destroy_node()

    def _active_axes(self) -> set[str]:
        """返回仍在按住窗口内的方向（超时未续则剔除）。"""
        now = time.monotonic()
        alive: set[str] = set()
        expired: list[str] = []
        for axis, ts in self._axis_last_seen.items():
            if now - ts <= self._release_timeout:
                alive.add(axis)
            else:
                expired.append(axis)
        for axis in expired:
            del self._axis_last_seen[axis]
        return alive

    @staticmethod
    def _format_label(alive: set[str]) -> str:
        parts: list[str] = []
        if 'fwd' in alive:
            parts.append(_AXIS_LABELS['fwd'])
        if 'back' in alive:
            parts.append(_AXIS_LABELS['back'])
        if 'left' in alive:
            parts.append(_AXIS_LABELS['left'])
        if 'right' in alive:
            parts.append(_AXIS_LABELS['right'])
        return '+'.join(parts)

    def _compute_twist(self, alive: set[str]) -> tuple[float, float]:
        lx = 0.0
        az = 0.0
        if 'fwd' in alive:
            lx += self._linear
        if 'back' in alive:
            lx -= self._linear
        if 'left' in alive:
            az += self._angular
        if 'right' in alive:
            az -= self._angular
        return lx, az

    def _sync_twist(self, log_stop: bool = True) -> None:
        alive = self._active_axes()
        lx, az = self._compute_twist(alive)

        if lx == 0.0 and az == 0.0:
            if self._moving and log_stop:
                self.get_logger().info('停止（松手）')
            self._twist = Twist()
            self._moving = False
            self._last_label = None
            self._pub.publish(Twist())
            return

        label = self._format_label(alive)
        if label != self._last_label:
            self.get_logger().info(label)
            self._last_label = label

        self._twist.linear.x = lx
        self._twist.angular.z = az
        self._moving = True
        self._pub.publish(self._twist)

    def _publish_stop(self, manual: bool = False) -> None:
        if self._moving and manual:
            self.get_logger().info('急停')
        self._axis_last_seen.clear()
        self._twist = Twist()
        self._moving = False
        self._last_label = None
        self._pub.publish(Twist())

    def _touch_axis(self, axis: str) -> None:
        now = time.monotonic()
        opposite = _AXIS_OPPOSITE.get(axis)
        if opposite is not None:
            self._axis_last_seen.pop(opposite, None)
        self._axis_last_seen[axis] = now

        # 关键：TTY 同时只 repeat 一个键。W+A 时通常只有 A/D 在 repeat，
        # W/S 收不到 repeat → 0.8s 后被误判松手（日志：前进+左转 → 左转 → 停）。
        # 任一通道收到 repeat 时，给另一通道里仍「按住」的轴续命。
        if axis in _ANGULAR_AXES:
            for lin in _LINEAR_AXES:
                if lin in self._axis_last_seen:
                    self._axis_last_seen[lin] = now
        elif axis in _LINEAR_AXES:
            for ang in _ANGULAR_AXES:
                if ang in self._axis_last_seen:
                    self._axis_last_seen[ang] = now

        self._sync_twist(log_stop=False)

    def _on_timer(self) -> None:
        self._sync_twist()
        if self._moving:
            self._pub.publish(self._twist)

    def spin_keyboard(self) -> None:
        # 先读键再 spin：避免 timer 里的超时逻辑抢在按键之前把 W/S 清掉
        while rclpy.ok():
            raw = self._keys.poll(0.05)
            if raw is not None:
                if raw in ('q', 'QUIT'):
                    break
                if raw == ' ':
                    self._publish_stop(manual=True)
                else:
                    action = raw if raw in _ACTION_TO_AXIS else _LETTER_KEYS.get(raw)
                    if action:
                        self._touch_axis(_ACTION_TO_AXIS[action])
            rclpy.spin_once(self, timeout_sec=0.0)


def main() -> None:
    rclpy.init()
    node = KeyboardTeleop()
    try:
        node.spin_keyboard()
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop(manual=False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
