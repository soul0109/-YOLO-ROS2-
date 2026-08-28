#!/usr/bin/env python3
"""测试 /cmd_vel → /cmd_vel_gazebo 中继（供 smoke_test 调用）。"""

from __future__ import annotations

import sys
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default


class RelayTester(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_relay_tester')
        self._pub = self.create_publisher(Twist, 'cmd_vel', qos_profile_system_default)
        self._last: float | None = None
        self.create_subscription(
            Twist, 'cmd_vel_gazebo', self._on_msg, qos_profile_system_default,
        )

    def _on_msg(self, msg: Twist) -> None:
        self._last = msg.linear.x


def main() -> int:
    mode = sys.argv[1] if len(sys.argv) > 1 else 'forward'
    rclpy.init()
    node = RelayTester()

    if mode == 'forward':
        target = 0.15
        for _ in range(60):
            node._pub.publish(Twist(linear=Vector3(x=target)))
            rclpy.spin_once(node, timeout_sec=0.05)
            if node._last == target:
                print(f'OK: got linear.x={node._last}')
                node.destroy_node()
                rclpy.shutdown()
                return 0
        print(f'FAIL: expected {target}, got {node._last}')
    elif mode == 'stop':
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node._last == 0.0:
                print('OK: brake linear.x=0.0')
                node.destroy_node()
                rclpy.shutdown()
                return 0
        print(f'FAIL: expected 0.0, got {node._last}')
    else:
        print(f'unknown mode: {mode}')
        node.destroy_node()
        rclpy.shutdown()
        return 2

    node.destroy_node()
    rclpy.shutdown()
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
