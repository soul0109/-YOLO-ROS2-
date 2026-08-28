#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人状态查询服务节点（阶段 2 demo）

功能：
    提供 service /get_robot_status（类型 inspection_interfaces/srv/RobotStatus）。
    客户端调用时返回模拟的机器人 ID、状态、电量和位姿。

学习要点：
    - Service 是“请求-响应”模式：一问一答，不像 topic 持续广播
    - 使用自定义 srv 类型（定义在 inspection_interfaces 包）

后续阶段：
    真正的状态可能来自底盘驱动、电池 BMS 或任务管理节点；
    接口名和字段可先保持不变，只换数据来源。
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from rclpy.node import Node

from inspection_interfaces.srv import RobotStatus


class RobotStatusServer(Node):
    """RobotStatus 服务的 Server 端。"""

    def __init__(self) -> None:
        super().__init__('robot_status_server')

        self.declare_parameter('robot_id', 'robot_01')
        self.declare_parameter('default_status', 'idle')
        self.declare_parameter('battery_percent', 85.0)
        # 模拟机器人在地图原点附近的位置（单位：米）
        self.declare_parameter('pose_x', 0.0)
        self.declare_parameter('pose_y', 0.0)
        self.declare_parameter('pose_z', 0.0)

        self._robot_id = str(self.get_parameter('robot_id').value)
        self._default_status = str(self.get_parameter('default_status').value)
        self._battery = float(self.get_parameter('battery_percent').value)
        self._pose_x = float(self.get_parameter('pose_x').value)
        self._pose_y = float(self.get_parameter('pose_y').value)
        self._pose_z = float(self.get_parameter('pose_z').value)

        # 创建服务：服务名 /get_robot_status，回调 _handle_request
        self.create_service(
            RobotStatus,
            '/get_robot_status',
            self._handle_request,
        )

        self.get_logger().info('状态查询服务已启动 | 服务名: /get_robot_status')

    def _handle_request(
        self,
        request: RobotStatus.Request,
        response: RobotStatus.Response,
    ) -> RobotStatus.Response:
        """
        服务回调：request 是客户端请求（本 demo 无字段），response 需填充后返回。

        ROS2 Python 中也可 return response，两种写法等价。
        """
        # request 在阶段 2 为空，这里用 _ 表示故意不使用
        _ = request

        response.robot_id = self._robot_id
        response.status = self._default_status
        response.battery_percent = self._battery

        # 构造 geometry_msgs/Pose：position + orientation（无旋转时 w=1）
        response.pose = Pose()
        response.pose.position = Point(
            x=self._pose_x,
            y=self._pose_y,
            z=self._pose_z,
        )
        response.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.get_logger().info(
            f'收到状态查询 | 返回 status={response.status}, '
            f'battery={response.battery_percent}%'
        )
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RobotStatusServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到 Ctrl+C，状态服务节点退出')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
