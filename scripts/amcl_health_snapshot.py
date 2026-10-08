#!/usr/bin/env python3
"""分段 AMCL 健康快照（诊断门禁，非正式产品阈值）。

记录：amcl / GT / AMCL↔GT / cov / particle spread / map→odom TF。
质量恶化时 exit 4（LOC_FAIL），供探针截断下一 goal。

  python3 scripts/amcl_health_snapshot.py --label after_B_clear
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from tf2_ros import Buffer, TransformException, TransformListener

AMCL_POSE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

# 诊断门禁（报警/截断实验用，非最终产品阈值）
DEFAULT_MAX_XY_VAR = 0.25
DEFAULT_MAX_YAW_VAR = 0.20
DEFAULT_MAX_SPREAD = 0.50


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def fnum(v: float, fmt: str = '{:.3f}') -> str:
    return fmt.format(v) if math.isfinite(v) else 'n/a'


class AmclHealthSnapshot(Node):
    def __init__(self, label: str, wait_sec: float) -> None:
        super().__init__(
            'amcl_health_snapshot',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.label = label
        self.wait_sec = wait_sec
        self.amcl: PoseWithCovarianceStamped | None = None
        self.gt: Odometry | None = None
        self.cloud: ParticleCloud | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        clock_deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=15.0)
        while self.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now() > clock_deadline:
                break

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, AMCL_POSE_QOS,
        )
        self.create_subscription(Odometry, '/ground_truth', self._on_gt, qos_profile_sensor_data)
        self.create_subscription(
            ParticleCloud, '/particle_cloud', self._on_cloud, qos_profile_sensor_data,
        )

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl = msg

    def _on_gt(self, msg: Odometry) -> None:
        self.gt = msg

    def _on_cloud(self, msg: ParticleCloud) -> None:
        self.cloud = msg

    def collect(self) -> None:
        t_end = time.monotonic() + self.wait_sec
        while time.monotonic() < t_end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def particle_spread_xy(self) -> tuple[int, float]:
        if self.cloud is None or not self.cloud.particles:
            return 0, float('nan')
        parts = self.cloud.particles
        wsum = sum(max(p.weight, 0.0) for p in parts)
        if wsum <= 0.0:
            weights = [1.0] * len(parts)
            wsum = float(len(parts))
        else:
            weights = [max(p.weight, 0.0) for p in parts]
        mx = sum(w * p.pose.position.x for w, p in zip(weights, parts)) / wsum
        my = sum(w * p.pose.position.y for w, p in zip(weights, parts)) / wsum
        var = sum(
            w * ((p.pose.position.x - mx) ** 2 + (p.pose.position.y - my) ** 2)
            for w, p in zip(weights, parts)
        ) / wsum
        return len(parts), math.sqrt(var)

    def lookup_map_odom(self) -> TransformStamped | None:
        try:
            return self.tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
        except TransformException as exc:
            self.get_logger().warn(f'map→odom TF: {exc}')
            return None

    def evaluate(
        self,
        max_xy_var: float,
        max_yaw_var: float,
        max_spread: float,
        map_origin_xy: tuple[float, float],
    ) -> int:
        ox, oy = map_origin_xy
        n, spread = self.particle_spread_xy()
        tf_mo = self.lookup_map_odom()

        print(f'[LOC_SNAP] label={self.label}')

        if self.amcl is None:
            print('[LOC_SNAP] amcl_pose=MISSING')
            print('LOC:FAIL:reason=no_amcl')
            return 4

        p = self.amcl.pose.pose.position
        q = self.amcl.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        cov = self.amcl.pose.covariance
        c0, c7, c35 = float(cov[0]), float(cov[7]), float(cov[35])
        # amcl 在 map；打印 world≈ map+origin 便于对照
        print(
            f'[LOC_SNAP] amcl_map=({p.x:.3f},{p.y:.3f}) yaw={yaw:.4f} '
            f'amcl_world≈({p.x + ox:.3f},{p.y + oy:.3f})'
        )
        print(f'[LOC_SNAP] cov[0]={c0:.3f} cov[7]={c7:.3f} cov[35]={c35:.3f}')

        amcl_gt_xy = float('nan')
        amcl_gt_yaw = float('nan')
        if self.gt is not None:
            gx = self.gt.pose.pose.position.x
            gy = self.gt.pose.pose.position.y
            gq = self.gt.pose.pose.orientation
            gyaw = yaw_from_quat(gq.x, gq.y, gq.z, gq.w)
            print(f'[LOC_SNAP] ground_truth_world=({gx:.3f},{gy:.3f}) yaw={gyaw:.4f}')
            gt_map_x, gt_map_y = gx - ox, gy - oy
            amcl_gt_xy = math.hypot(p.x - gt_map_x, p.y - gt_map_y)
            amcl_gt_yaw = abs(math.degrees(normalize_angle(yaw - gyaw)))
            print(
                f'[LOC_SNAP] AMCL↔GT xy={amcl_gt_xy:.3f} m  yaw={amcl_gt_yaw:.1f} deg'
            )
        else:
            print('[LOC_SNAP] ground_truth=MISSING')

        print(f'[LOC_SNAP] particle_n={n} spread_xy={fnum(spread)} m')

        if tf_mo is not None:
            t = tf_mo.transform.translation
            r = tf_mo.transform.rotation
            tyaw = yaw_from_quat(r.x, r.y, r.z, r.w)
            print(
                f'[LOC_SNAP] map→odom t=({t.x:.3f},{t.y:.3f},{t.z:.3f}) '
                f'yaw={tyaw:.4f}'
            )
        else:
            print('[LOC_SNAP] map→odom=MISSING')

        reasons: list[str] = []
        if not math.isfinite(c0) or c0 > max_xy_var:
            reasons.append(f'cov0={c0:.3f}>{max_xy_var}')
        if not math.isfinite(c7) or c7 > max_xy_var:
            reasons.append(f'cov7={c7:.3f}>{max_xy_var}')
        if not math.isfinite(c35) or c35 > max_yaw_var:
            reasons.append(f'cov35={c35:.3f}>{max_yaw_var}')
        if math.isfinite(spread) and spread > max_spread:
            reasons.append(f'spread={spread:.3f}>{max_spread}')

        if reasons:
            print(f'[LOC_SNAP] DIAG_GATE FAIL reasons={";".join(reasons)}')
            print(f'LOC:{self.label}:FAIL:{"|".join(reasons)}')
            return 4

        print('[LOC_SNAP] DIAG_GATE PASS (diagnostic thresholds)')
        print(f'LOC:{self.label}:PASS')
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(description='AMCL 分段健康快照（诊断门禁）')
    parser.add_argument('--label', type=str, required=True)
    parser.add_argument('--wait', type=float, default=1.0, help='采集等待秒')
    parser.add_argument('--max-xy-var', type=float, default=DEFAULT_MAX_XY_VAR)
    parser.add_argument('--max-yaw-var', type=float, default=DEFAULT_MAX_YAW_VAR)
    parser.add_argument('--max-spread', type=float, default=DEFAULT_MAX_SPREAD)
    parser.add_argument(
        '--map-origin-x', type=float, default=-0.0134,
        help='test_room origin x（world = map + origin）',
    )
    parser.add_argument('--map-origin-y', type=float, default=0.0519)
    args = parser.parse_args()

    # 优先从 yaml 读 origin
    map_yaml = (
        __import__('pathlib').Path(__file__).resolve().parents[1]
        / 'ros2_ws/src/navigation_config/maps/test_room.yaml'
    )
    ox, oy = args.map_origin_x, args.map_origin_y
    if map_yaml.is_file():
        try:
            import yaml
            origin = yaml.safe_load(map_yaml.read_text(encoding='utf-8')).get(
                'origin', [ox, oy, 0.0],
            )
            ox, oy = float(origin[0]), float(origin[1])
        except Exception:  # noqa: BLE001
            pass

    rclpy.init()
    node = AmclHealthSnapshot(label=args.label, wait_sec=args.wait)
    node.collect()
    code = node.evaluate(
        max_xy_var=args.max_xy_var,
        max_yaw_var=args.max_yaw_var,
        max_spread=args.max_spread,
        map_origin_xy=(ox, oy),
    )
    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
