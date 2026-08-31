#!/usr/bin/env python3
"""
4.4：对比 AMCL /amcl_pose 与 Gazebo /ground_truth，量化定位误差。

用法（AMCL + Gazebo 运行中，另开终端）：
  source /opt/ros/humble/setup.bash
  source ~/inspection-robot/ros2_ws/install/setup.bash
  python3 scripts/check_amcl_accuracy.py --duration 30
  python3 scripts/check_amcl_accuracy.py --duration 30 --static-hint

闸门默认 0.10 m（巡检场景规格 §5）。
静止验收：车停稳 + 已发 initialpose + 本脚本；边走边测误差会变大属正常。
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
from nav_msgs.msg import Odometry
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)


def load_map_origin(map_yaml: Path) -> tuple[float, float, float]:
    data = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
    origin = data.get('origin', [0.0, 0.0, 0.0])
    return float(origin[0]), float(origin[1]), float(origin[2])


def yaw_from_quat(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


# AMCL 发布 /amcl_pose 使用 TRANSIENT_LOCAL，订阅必须用兼容 QoS
AMCL_POSE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


class Checker:
    def __init__(self, origin_x: float, origin_y: float, origin_yaw: float):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.origin_yaw = origin_yaw
        self.amcl: PoseWithCovarianceStamped | None = None
        self.gt: Odometry | None = None
        self.amcl_count = 0
        self.gt_count = 0
        self.errors: list[tuple[float, float, float]] = []

    def on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl = msg
        self.amcl_count += 1

    def on_gt(self, msg: Odometry) -> None:
        self.gt = msg
        self.gt_count += 1

    def sample(self) -> None:
        if self.amcl is None or self.gt is None:
            return
        gx = self.gt.pose.pose.position.x - self.origin_x
        gy = self.gt.pose.pose.position.y - self.origin_y
        gq = self.gt.pose.pose.orientation
        gyaw = yaw_from_quat(gq.x, gq.y, gq.z, gq.w) - self.origin_yaw

        ax = self.amcl.pose.pose.position.x
        ay = self.amcl.pose.pose.position.y
        aq = self.amcl.pose.pose.orientation
        ayaw = yaw_from_quat(aq.x, aq.y, aq.z, aq.w)

        pos_err = math.hypot(ax - gx, ay - gy)
        yaw_err = abs(normalize_angle(ayaw - gyaw))
        self.errors.append((pos_err, math.degrees(yaw_err), time.time()))


def wait_for_topics(node: rclpy.node.Node, checker: Checker, timeout: float) -> bool:
    print(f'>>> 等待 /amcl_pose + /ground_truth（最多 {timeout:.0f}s）...')
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if checker.amcl_count > 0 and checker.gt_count > 0:
            print(f'    OK: amcl_pose×{checker.amcl_count}  ground_truth×{checker.gt_count}')
            return True
    print(
        f'    FAIL: amcl_pose={checker.amcl_count}  ground_truth={checker.gt_count}',
        file=sys.stderr,
    )
    if checker.amcl_count == 0:
        print(
            '    提示: 确认 run_amcl_localization.sh 仍在跑，且已发 /initialpose',
            file=sys.stderr,
        )
        print(
            '    可执行: python3 ~/inspection-robot/scripts/publish_amcl_initial_pose.py',
            file=sys.stderr,
        )
    if checker.gt_count == 0:
        print('    提示: Gazebo 未跑或 p3d 未发布 /ground_truth', file=sys.stderr)
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description='AMCL 定位精度检查')
    parser.add_argument('--duration', type=float, default=30.0, help='采样秒数')
    parser.add_argument('--gate', type=float, default=0.10, help='位置误差闸门 (m)')
    parser.add_argument('--wait', type=float, default=15.0, help='等待话题秒数')
    parser.add_argument(
        '--static-hint',
        action='store_true',
        help='打印静止验收提示（车停稳、勿按 WASD）',
    )
    parser.add_argument(
        '--map-yaml',
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / 'ros2_ws/src/navigation_config/maps/test_room.yaml',
    )
    args = parser.parse_args()

    if not args.map_yaml.is_file():
        print(f'错误: 找不到 {args.map_yaml}', file=sys.stderr)
        return 1

    ox, oy, oyaw = load_map_origin(args.map_yaml)

    rclpy.init()
    node = rclpy.create_node('check_amcl_accuracy')
    checker = Checker(ox, oy, oyaw)
    node.create_subscription(
        PoseWithCovarianceStamped, '/amcl_pose', checker.on_amcl, AMCL_POSE_QOS,
    )
    node.create_subscription(
        Odometry, '/ground_truth', checker.on_gt, qos_profile_sensor_data,
    )

    if args.static_hint:
        print('>>> 【静止验收】车停稳、粒子聚束后采样；采样期间不要按 WASD')
    if not wait_for_topics(node, checker, args.wait):
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print(f'>>> 采样 {args.duration:.0f}s ...')
    end = time.time() + args.duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        checker.sample()

    if len(checker.errors) < 5:
        print(
            f'FAIL: 有效样本过少 ({len(checker.errors)})。amcl 累计 {checker.amcl_count} 条',
            file=sys.stderr,
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    pos = [e[0] for e in checker.errors]
    yaw = [e[1] for e in checker.errors]
    max_pos = max(pos)
    mean_pos = sum(pos) / len(pos)
    max_yaw = max(yaw)
    p95 = sorted(pos)[max(0, int(0.95 * len(pos)) - 1)]

    print('========================================')
    print(f'  样本数: {len(pos)}  (amcl_pose 消息: {checker.amcl_count})')
    print(f'  位置误差  mean={mean_pos:.3f} m  max={max_pos:.3f} m  p95={p95:.3f} m')
    print(f'  航向误差  max={max_yaw:.1f} deg')
    print(f'  闸门: 位置 max ≤ {args.gate:.2f} m（静止验收）')
    if max_pos <= args.gate:
        print('  结论: ✅ 通过 AMCL 定位闸门')
    else:
        print('  结论: ❌ 未过闸门')
        if mean_pos > args.gate:
            print('  解读: mean 也大 → 重设 2D Pose 或检查地图/initialpose')
        else:
            print('  解读: mean 小 max 大 → 多为转弯/长按 W 时尖峰；静止重测')
    print('========================================')

    node.destroy_node()
    rclpy.shutdown()
    return 0 if max_pos <= args.gate else 1


if __name__ == '__main__':
    sys.exit(main())
