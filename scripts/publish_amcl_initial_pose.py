#!/usr/bin/env python3
"""
向 AMCL 发布一次性 /initialpose（map 系）。

用法：
  python3 scripts/publish_amcl_initial_pose.py
  python3 scripts/publish_amcl_initial_pose.py --world-x 0.9 --world-y 3.0 --yaw 0
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import qos_profile_system_default


def load_map_origin(map_yaml: Path) -> tuple[float, float, float]:
    data = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
    origin = data.get('origin', [0.0, 0.0, 0.0])
    return float(origin[0]), float(origin[1]), float(origin[2])


def world_to_map_xy(world_x: float, world_y: float, origin_x: float, origin_y: float) -> tuple[float, float]:
    """ROS map 元数据：world = origin + map_xy。"""
    return world_x - origin_x, world_y - origin_y


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def main() -> int:
    parser = argparse.ArgumentParser(description='发布 AMCL 初始位姿到 /initialpose')
    parser.add_argument(
        '--map-yaml',
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / 'ros2_ws/src/navigation_config/maps/test_room.yaml',
        help='地图 yaml（读 origin）',
    )
    parser.add_argument('--world-x', type=float, default=0.9)
    parser.add_argument('--world-y', type=float, default=3.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--delay', type=float, default=0.0, help='发布前等待秒数')
    args = parser.parse_args()

    if not args.map_yaml.is_file():
        print(f'错误: 找不到地图 yaml: {args.map_yaml}', file=sys.stderr)
        return 1

    ox, oy, oyaw = load_map_origin(args.map_yaml)
    mx, my = world_to_map_xy(args.world_x, args.world_y, ox, oy)
    qx, qy, qz, qw = yaw_to_quat(args.yaw - oyaw)

    rclpy.init()
    node = rclpy.create_node('publish_amcl_initial_pose')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos_profile_system_default)

    if args.delay > 0:
        time.sleep(args.delay)

    msg = PoseWithCovarianceStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = mx
    msg.pose.pose.position.y = my
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    # x, y, yaw 方差
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.06853891909122467

    for _ in range(3):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)

    print(
        f'OK: /initialpose  map=({mx:.3f}, {my:.3f}, yaw={args.yaw:.2f})  '
        f'world=({args.world_x}, {args.world_y})  origin=({ox}, {oy})'
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
