#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
静态 TF 广播节点（阶段 2 demo）

功能：
    广播 base_link → camera_link 的固定坐标变换（相机相对机器人主体的安装位置）。

学习要点：
    - TF2 维护一棵坐标系树，描述各部件相对关系
    - StaticTransformBroadcaster 用于不会变的变换（相机支架、雷达安装位）
    - 动态变换（机器人在地图中移动）由 odom/base_link 等节点发布，阶段 3 再学

后续阶段：
    YOLO 检测在 camera_link 坐标系；要映射到 map 需沿 TF 树变换。
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class StaticTfBroadcaster(Node):
    """发布 base_link 到 camera_link 的静态变换。"""

    def __init__(self) -> None:
        super().__init__('static_tf_broadcaster')

        # 相机在 base_link 坐标系下的位置（米）与绕 Z 轴旋转（弧度）
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'camera_link')
        self.declare_parameter('camera_x', 0.10)
        self.declare_parameter('camera_y', 0.0)
        self.declare_parameter('camera_z', 0.30)
        self.declare_parameter('camera_yaw_rad', 0.0)

        self._parent = str(self.get_parameter('parent_frame').value)
        self._child = str(self.get_parameter('child_frame').value)
        x = float(self.get_parameter('camera_x').value)
        y = float(self.get_parameter('camera_y').value)
        z = float(self.get_parameter('camera_z').value)
        yaw = float(self.get_parameter('camera_yaw_rad').value)

        self._broadcaster = StaticTransformBroadcaster(self)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._parent
        transform.child_frame_id = self._child

        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z

        # 将绕 Z 轴旋转角 yaw 转为四元数（roll=pitch=0 的简化情况）
        half_yaw = yaw * 0.5
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(half_yaw)
        transform.transform.rotation.w = math.cos(half_yaw)

        # sendTransform 只需调用一次；StaticTransformBroadcaster 会持续生效
        self._broadcaster.sendTransform(transform)

        self.get_logger().info(
            f'已发布静态 TF: {self._parent} → {self._child} | '
            f'平移=({x:.2f}, {y:.2f}, {z:.2f}) m, yaw={yaw:.2f} rad'
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StaticTfBroadcaster()
    try:
        # 静态 TF 发布后即可退出；但保持 spin 以便节点存活、方便 launch 管理
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到 Ctrl+C，静态 TF 节点退出')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
