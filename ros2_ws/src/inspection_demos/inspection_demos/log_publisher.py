#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
巡检日志发布节点（阶段 2 demo）

功能：
    按固定时间间隔向话题 /inspection/log 发布模拟巡检日志（std_msgs/String）。

学习要点：
    - 创建 ROS2 节点（rclpy）
    - 声明并读取参数（log_interval_sec、robot_id）
    - 创建 Publisher 并定时 publish

后续阶段：
    真正的巡检任务节点会发布结构化日志或事件，话题名可沿用 /inspection/log。
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LogPublisher(Node):
    """模拟巡检日志的发布者。"""

    # 预置几条巡检日志，循环发送，便于在终端观察 topic echo 效果
    SAMPLE_LOGS = [
        '巡检启动：开始按预设路线检查厂区',
        '到达 1 号机房门口，准备进入',
        '1 号机房：温湿度正常，未发现异常',
        '前往 2 号走廊检查消防通道',
        '2 号走廊：通道畅通，无杂物堆放',
        '巡检任务阶段性完成，等待下一条指令',
    ]

    def __init__(self) -> None:
        super().__init__('log_publisher')

        # declare_parameter：声明参数并给默认值；实际值可由 launch 加载的 YAML 覆盖
        self.declare_parameter('log_interval_sec', 2.0)
        self.declare_parameter('robot_id', 'robot_01')

        self._interval = float(self.get_parameter('log_interval_sec').value)
        self._robot_id = str(self.get_parameter('robot_id').value)
        self._log_index = 0

        # 创建发布者：话题名 /inspection/log，队列深度 10（缓存最近 10 条）
        self._publisher = self.create_publisher(String, '/inspection/log', 10)

        # 定时器：每隔 _interval 秒调用一次 _publish_log
        self._timer = self.create_timer(self._interval, self._publish_log)

        self.get_logger().info(
            f'日志发布节点已启动 | robot_id={self._robot_id} | 间隔={self._interval}s'
        )

    def _publish_log(self) -> None:
        """构造一条带机器人 ID 前缀的日志并发布。"""
        raw_text = self.SAMPLE_LOGS[self._log_index % len(self.SAMPLE_LOGS)]
        self._log_index += 1

        msg = String()
        msg.data = f'[{self._robot_id}] {raw_text}'

        self._publisher.publish(msg)
        self.get_logger().info(f'已发布: {msg.data}')


def main(args: list[str] | None = None) -> None:
    """节点入口：初始化 rclpy → 运行节点 → 退出时清理资源。"""
    rclpy.init(args=args)
    node = LogPublisher()
    try:
        # spin：进入事件循环，处理定时器与回调
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到 Ctrl+C，日志发布节点退出')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
