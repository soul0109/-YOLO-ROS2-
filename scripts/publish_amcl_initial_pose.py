#!/usr/bin/env python3
"""
向 AMCL 发布一次性 /initialpose（map 系）。

用法：
  python3 scripts/publish_amcl_initial_pose.py
  python3 scripts/publish_amcl_initial_pose.py --world-x 0.9 --world-y 3.0 --yaw 0
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from coords import load_map_origin, world_to_map_xy, yaw_to_quat

# nav2_amcl /initialpose 订阅：BEST_EFFORT + VOLATILE（与 system_default RELIABLE 不匹配）
_INITIALPOSE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


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
    node = rclpy.create_node(
        'publish_amcl_initial_pose',
        parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
    )
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', _INITIALPOSE_QOS)

    # 等仿真时钟
    clock_end = time.time() + 20.0
    while time.time() < clock_end and node.get_clock().now().nanoseconds == 0:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.get_clock().now().nanoseconds == 0:
        print('警告: /clock 未就绪，仍用 stamp=0 发布', file=sys.stderr)

    # 等 AMCL 订阅匹配（DDS discovery）
    match_end = time.time() + 20.0
    while time.time() < match_end and pub.get_subscription_count() < 1:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() < 1:
        print('错误: /initialpose 无订阅者（AMCL 未就绪）', file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if args.delay > 0:
        time.sleep(args.delay)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = mx
    msg.pose.pose.position.y = my
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.06853891909122467

    for _ in range(5):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)

    print(
        f'OK: /initialpose  map=({mx:.3f}, {my:.3f}, yaw={args.yaw:.2f})  '
        f'world=({args.world_x}, {args.world_y})  origin=({ox}, {oy})  '
        f'subs={pub.get_subscription_count()}'
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
