#!/usr/bin/env python3
"""
robot_v0 键盘遥控 → /cmd_vel

优先 evdev（读真实 key-down/key-up，组合键 W+A 松 A 后 W 仍有效）。
无 evdev 权限时回退 TTY：W/S 点按锁定，A/D 按住转弯（TTY 无法可靠检测组合键松手）。
"""

from __future__ import annotations

import select
import sys
import termios
import time
import tty
from abc import ABC, abstractmethod
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

HELP_EVDEV = """
========================================
  robot_v0 键盘遥控 [evdev 模式]
========================================
  W / S     按住 前进 / 后退
  A / D     按住 左转 / 右转
  组合键    W+A / W+D 弧线；松 A/D 后 W 仍有效
  空格      急停    Q  退出
========================================
"""

HELP_TTY_TOGGLE = """
========================================
  robot_v0 键盘遥控 [TTY 回退模式]
========================================
  VMware/TTY 读不到 key-up，组合键松 A 后 W 会失效。
  本模式改为：
    W / S     点一下 开始前进/后退，再点一次或空格 停止
    A / D     按住 左转 / 右转（可与 W/S 叠加）
  推荐安装 evdev 恢复「按住 W」：
    sudo apt install python3-evdev
    sudo usermod -aG input $USER   # 然后注销重登
========================================
"""

_ARROW_TAIL: dict[str, str] = {
    'A': 'UP', 'B': 'DOWN', 'C': 'RIGHT', 'D': 'LEFT',
}

_LETTER_KEYS: dict[str, str] = {
    'w': 'UP', 's': 'DOWN', 'a': 'LEFT', 'd': 'RIGHT',
    'i': 'UP', 'k': 'DOWN', 'j': 'LEFT', 'l': 'RIGHT',
}

_ACTION_TO_AXIS: dict[str, str] = {
    'UP': 'fwd', 'DOWN': 'back', 'LEFT': 'left', 'RIGHT': 'right',
}

_AXIS_LABELS: dict[str, str] = {
    'fwd': '前进', 'back': '后退', 'left': '左转', 'right': '右转',
}

_AXIS_OPPOSITE: dict[str, str] = {
    'fwd': 'back', 'back': 'fwd', 'left': 'right', 'right': 'left',
}

_LINEAR_AXES = frozenset({'fwd', 'back'})
_ANGULAR_AXES = frozenset({'left', 'right'})


class InputBackend(ABC):
    @abstractmethod
    def poll(self, timeout_sec: float) -> None:
        """读输入，更新内部状态。"""

    @abstractmethod
    def active_axes(self) -> set[str]:
        """当前应参与合成的方向。"""

    @abstractmethod
    def consume_quit(self) -> bool:
        ...

    @abstractmethod
    def consume_stop(self) -> bool:
        ...

    @abstractmethod
    def clear_all(self) -> None:
        ...

    @abstractmethod
    def cleanup(self) -> None:
        ...


class EvdevBackend(InputBackend):
    """读 /dev/input/event* 的真实按键状态（不受 TTY repeat 限制）。"""

    def __init__(self, logger) -> None:
        from evdev import InputDevice, ecodes, list_devices

        self._ecodes = ecodes
        self._key_map = {
            ecodes.KEY_W: 'fwd',
            ecodes.KEY_S: 'back',
            ecodes.KEY_A: 'left',
            ecodes.KEY_D: 'right',
            ecodes.KEY_UP: 'fwd',
            ecodes.KEY_DOWN: 'back',
            ecodes.KEY_LEFT: 'left',
            ecodes.KEY_RIGHT: 'right',
        }
        self._device = self._open_keyboard(InputDevice, list_devices, ecodes)
        self._fd = self._device.fd
        self._pressed: set[str] = set()
        self._quit = False
        self._stop = False
        logger.info(f'evdev 键盘：{self._device.path} ({self._device.name})')

    @staticmethod
    def _open_keyboard(InputDevice, list_devices, ecodes):
        import glob

        paths = list_devices()
        if not paths:
            # 部分环境 list_devices() 为空但 /dev/input/event* 可读（需在 input 组）
            paths = sorted(glob.glob('/dev/input/event*'))
        candidates = []
        for path in paths:
            try:
                dev = InputDevice(path)
            except OSError:
                continue
            keys = dev.capabilities().get(ecodes.EV_KEY, [])
            if ecodes.KEY_W in keys and ecodes.KEY_A in keys:
                candidates.append(dev)
        if not candidates:
            raise RuntimeError('未找到带 WASD 的键盘设备（确认在 input 组：groups | grep input）')
        for dev in candidates:
            if 'keyboard' in dev.name.lower():
                return dev
        return candidates[0]

    def poll(self, timeout_sec: float) -> None:
        if not select.select([self._fd], [], [], timeout_sec)[0]:
            return
        for event in self._device.read():
            if event.type != self._ecodes.EV_KEY:
                continue
            if event.code == self._ecodes.KEY_Q and event.value == 1:
                self._quit = True
                continue
            if event.code == self._ecodes.KEY_SPACE and event.value == 1:
                self._stop = True
                self._pressed.clear()
                continue
            axis = self._key_map.get(event.code)
            if axis is None:
                continue
            if event.value == 0:
                self._pressed.discard(axis)
            elif event.value in (1, 2):
                opposite = _AXIS_OPPOSITE.get(axis)
                if opposite:
                    self._pressed.discard(opposite)
                self._pressed.add(axis)

    def active_axes(self) -> set[str]:
        return set(self._pressed)

    def consume_quit(self) -> bool:
        if self._quit:
            self._quit = False
            return True
        return False

    def consume_stop(self) -> bool:
        if self._stop:
            self._stop = False
            return True
        return False

    def clear_all(self) -> None:
        self._pressed.clear()

    def cleanup(self) -> None:
        self._device.close()


class TtyToggleBackend(InputBackend):
    """TTY 回退：W/S 点按锁定，A/D 超时按住。"""

    def __init__(self, release_timeout: float) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError('keyboard_teleop 必须在交互式终端运行')
        self._release_timeout = release_timeout
        self._linear_latched: Optional[str] = None
        self._angular_last_seen: dict[str, float] = {}
        self._buf = ''
        self._quit = False
        self._stop = False
        self._term_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

    def poll(self, timeout_sec: float) -> None:
        raw = self._read_one(timeout_sec)
        if raw is None:
            self._expire_angular()
            return
        if raw in ('q', 'QUIT'):
            self._quit = True
            return
        if raw == ' ':
            self._stop = True
            self.clear_all()
            return
        action = raw if raw in _ACTION_TO_AXIS else _LETTER_KEYS.get(raw)
        if not action:
            return
        axis = _ACTION_TO_AXIS[action]
        now = time.monotonic()
        if axis in _LINEAR_AXES:
            if self._linear_latched == axis:
                self._linear_latched = None
            else:
                self._linear_latched = axis
        elif axis in _ANGULAR_AXES:
            opposite = _AXIS_OPPOSITE[axis]
            self._angular_last_seen.pop(opposite, None)
            self._angular_last_seen[axis] = now

    def _expire_angular(self) -> None:
        now = time.monotonic()
        for axis in list(self._angular_last_seen):
            if now - self._angular_last_seen[axis] > self._release_timeout:
                del self._angular_last_seen[axis]

    def _read_one(self, timeout_sec: float) -> Optional[str]:
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

    def active_axes(self) -> set[str]:
        self._expire_angular()
        alive: set[str] = set()
        if self._linear_latched:
            alive.add(self._linear_latched)
        alive.update(self._angular_last_seen.keys())
        return alive

    def consume_quit(self) -> bool:
        if self._quit:
            self._quit = False
            return True
        return False

    def consume_stop(self) -> bool:
        if self._stop:
            self._stop = False
            return True
        return False

    def clear_all(self) -> None:
        self._linear_latched = None
        self._angular_last_seen.clear()

    def cleanup(self) -> None:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._term_settings)


def _create_backend(node: Node, prefer_evdev: bool) -> InputBackend:
    timeout = float(node.get_parameter('key_release_timeout_sec').value)
    if prefer_evdev:
        try:
            return EvdevBackend(node.get_logger())
        except Exception as exc:
            node.get_logger().warn(
                f'evdev 不可用（{exc}），回退 TTY 点按模式。'
                '修复：sudo apt install python3-evdev && sudo usermod -aG input $USER 后重登'
            )
    return TtyToggleBackend(timeout)


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__('keyboard_teleop')

        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('key_release_timeout_sec', 0.8)
        self.declare_parameter('prefer_evdev', True)

        topic = self.get_parameter('cmd_vel_topic').value
        self._linear = float(self.get_parameter('linear_speed').value)
        self._angular = float(self.get_parameter('angular_speed').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        prefer_evdev = bool(self.get_parameter('prefer_evdev').value)

        self._twist = Twist()
        self._moving = False
        self._last_label: Optional[str] = None
        self._backend = _create_backend(self, prefer_evdev)

        self._pub = self.create_publisher(Twist, topic, qos_profile_system_default)
        self.create_timer(1.0 / rate_hz, self._on_timer)

        help_text = HELP_EVDEV if isinstance(self._backend, EvdevBackend) else HELP_TTY_TOGGLE
        print(help_text, flush=True)

    def destroy_node(self) -> bool:
        self._publish_stop(manual=False)
        self._backend.cleanup()
        return super().destroy_node()

    @staticmethod
    def _format_label(alive: set[str]) -> str:
        parts: list[str] = []
        for axis in ('fwd', 'back', 'left', 'right'):
            if axis in alive:
                parts.append(_AXIS_LABELS[axis])
        return '+'.join(parts)

    def _compute_twist(self, alive: set[str]) -> tuple[float, float]:
        lx = az = 0.0
        if 'fwd' in alive:
            lx += self._linear
        if 'back' in alive:
            lx -= self._linear
        if 'left' in alive:
            az += self._angular
        if 'right' in alive:
            az -= self._angular
        return lx, az

    def _apply_axes(self, alive: set[str], log_stop: bool = True) -> None:
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
        self._backend.clear_all()
        self._twist = Twist()
        self._moving = False
        self._last_label = None
        self._pub.publish(Twist())

    def _on_timer(self) -> None:
        self._apply_axes(self._backend.active_axes())
        if self._moving:
            self._pub.publish(self._twist)

    def spin_keyboard(self) -> None:
        while rclpy.ok():
            self._backend.poll(0.05)
            if self._backend.consume_quit():
                break
            if self._backend.consume_stop():
                self._publish_stop(manual=True)
            else:
                self._apply_axes(self._backend.active_axes(), log_stop=False)
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
