#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
巡检日志订阅节点（阶段 2 demo）

功能：
    订阅话题 /inspection/log，在终端打印收到的每一条日志。

学习要点：
    - 创建 Subscriber 并注册回调函数
    - 理解 pub/sub 是“发布者不知道谁在听”的广播模式

与 log_publisher 配合：
    两个节点可以运行在不同进程；只要话题名和消息类型一致就能通信。
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LogSubscriber(Node):
    """巡检日志的订阅者，收到消息后打印到本节点日志。"""

    def __init__(self) -> None:
        super().__init__('log_subscriber')

        # 订阅 /inspection/log；回调 _on_log_received 在收到消息时自动执行
        self.create_subscription(
            String,
            '/inspection/log',
            self._on_log_received,
            10,
        )

        self.get_logger().info('日志订阅节点已启动，等待 /inspection/log 消息...')

    def _on_log_received(self, msg: String) -> None:
        """
        订阅回调：msg 是发布者发来的 String 消息。

        注意：回调里不要做耗时操作，否则会阻塞其他回调。
        阶段 2 只打印一行，没问题；后续 YOLO 推理要放到独立线程。
        """
        self.get_logger().info(f'[订阅收到] {msg.data}')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LogSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到 Ctrl+C，日志订阅节点退出')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
