#!/usr/bin/env python3
"""
cmd_vel 超时刹车中继（阶段 3.3）

Gazebo 差速插件订阅 RELIABLE QoS 的 /cmd_vel_gazebo；
必须用相同 QoS 转发，并在超时后持续发零速度刹车。
"""

from __future__ import annotations

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


class CmdVelTimeout(Node):
    """带超时的 cmd_vel 转发器。"""

    def __init__(self) -> None:
        super().__init__('cmd_vel_timeout')

        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('input_topic', 'cmd_vel')
        self.declare_parameter('output_topic', 'cmd_vel_gazebo')
        self.declare_parameter('publish_rate_hz', 20.0)

        self._timeout = float(self.get_parameter('timeout_sec').value)
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self._last_cmd = Twist()
        self._last_cmd_time = self.get_clock().now()
        self._has_cmd = False

        self._pub = self.create_publisher(Twist, output_topic, CMD_VEL_QOS)
        self.create_subscription(Twist, input_topic, self._on_cmd_vel, CMD_VEL_QOS)
        self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(
            f'转发 {input_topic} → {output_topic}（RELIABLE），'
            f'超时 {self._timeout}s 自动刹车'
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()
        self._has_cmd = True
        self._pub.publish(msg)

    def _on_timer(self) -> None:
        if self._has_cmd:
            elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
            if elapsed <= self._timeout:
                self._pub.publish(self._last_cmd)
                return
        self._pub.publish(Twist())


def main() -> None:
    rclpy.init()
    node = CmdVelTimeout()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
