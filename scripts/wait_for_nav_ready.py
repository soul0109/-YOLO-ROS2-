#!/usr/bin/env python3
"""Wait for a comparable AMCL/Nav2 starting condition.

Simulation test precondition only. Separates INIT_FAIL from route failures.

Stable-window rules (review-aligned):
  - Require continuous GT + odom updates (sensor freshness).
  - Require TF lookup success; allow AMCL future stamps within transform_tolerance.
  - Require sim clock to advance.
  - Require a valid AMCL pose (error/cov/finite) once present.
  - Do NOT require N new /amcl_pose messages while stationary
    (Humble AMCL is gated by update_min_d / update_min_a).
  - topic_counts report true message counts; "polls" are not counted as samples.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from tf2_ros import Buffer, TransformException, TransformListener
from rclpy.time import Time

from coords import load_map_origin


AMCL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_error(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


class ReadinessProbe:
    def __init__(self, node, origin, tf_buffer: Buffer, transform_tolerance: float) -> None:
        self.node = node
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        self.tf_buffer = tf_buffer
        self.transform_tolerance = transform_tolerance
        self.gt = None
        self.amcl = None
        self.odom = None
        self.gt_count = 0
        self.amcl_count = 0
        self.odom_count = 0
        self.gt_recv_mono = None
        self.odom_recv_mono = None
        self.amcl_recv_mono = None
        self.last_clock_ns = None
        self.clock_advances = 0

        node.create_subscription(
            Odometry, '/ground_truth', self._on_gt, qos_profile_sensor_data,
        )
        node.create_subscription(
            Odometry, '/odom', self._on_odom, qos_profile_sensor_data,
        )
        node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, AMCL_QOS,
        )

    def _on_gt(self, msg: Odometry) -> None:
        self.gt = msg
        self.gt_count += 1
        self.gt_recv_mono = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg
        self.odom_count += 1
        self.odom_recv_mono = time.monotonic()

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl = msg
        self.amcl_count += 1
        self.amcl_recv_mono = time.monotonic()

    def note_clock(self) -> float | None:
        ns = self.node.get_clock().now().nanoseconds
        if ns <= 0:
            return None
        if self.last_clock_ns is not None and ns > self.last_clock_ns:
            self.clock_advances += 1
        self.last_clock_ns = ns
        return ns * 1e-9

    def snapshot(self) -> dict:
        result = {
            'tf_ready': False,
            'tf_detail': None,
            'clock_sec': None,
            'clock_advancing': self.clock_advances > 0,
            'ground_truth': None,
            'amcl': None,
            'amcl_error_m': None,
            'amcl_yaw_error_deg': None,
            'covariance': None,
            'odom_speed': None,
            'sensor_fresh': {
                'ground_truth': False,
                'odom': False,
                # Presence of a valid pose, not "new message this poll".
                'amcl_pose_present': False,
            },
            'ages_sec': {
                'ground_truth': None,
                'odom': None,
                'amcl_pose': None,
            },
        }
        clock = self.note_clock()
        result['clock_sec'] = None if clock is None else round(clock, 3)
        result['clock_advancing'] = self.clock_advances > 0

        now = time.monotonic()
        if self.gt_recv_mono is not None:
            result['ages_sec']['ground_truth'] = round(now - self.gt_recv_mono, 3)
            result['sensor_fresh']['ground_truth'] = (now - self.gt_recv_mono) < 0.5
        if self.odom_recv_mono is not None:
            result['ages_sec']['odom'] = round(now - self.odom_recv_mono, 3)
            result['sensor_fresh']['odom'] = (now - self.odom_recv_mono) < 0.5
        if self.amcl_recv_mono is not None:
            result['ages_sec']['amcl_pose'] = round(now - self.amcl_recv_mono, 3)
            result['sensor_fresh']['amcl_pose_present'] = True

        if self.gt is None or self.amcl is None or self.odom is None:
            return result

        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(),
                timeout=Duration(seconds=0.02),
            )
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            delta = None if clock is None else stamp - clock
            # Future stamps within transform_tolerance are normal for AMCL.
            future_ok = delta is None or delta <= self.transform_tolerance + 0.05
            past_ok = delta is None or delta >= -1.0
            result['tf_detail'] = {
                'stamp_sec': round(stamp, 3),
                'clock_sec': result['clock_sec'],
                'stamp_minus_clock_sec': None if delta is None else round(delta, 3),
                'within_transform_tolerance': future_ok,
            }
            result['tf_ready'] = bool(future_ok and past_ok)
        except TransformException as exc:
            result['tf_detail'] = {'ok': False, 'reason': str(exc)}

        gt_pose = self.gt.pose.pose
        amcl_pose = self.amcl.pose.pose
        gt_yaw = yaw_from_quaternion(gt_pose.orientation)
        amcl_yaw = yaw_from_quaternion(amcl_pose.orientation)
        gt_map_x = float(gt_pose.position.x) - self.origin_x
        gt_map_y = float(gt_pose.position.y) - self.origin_y
        result['ground_truth'] = {
            'x': round(float(gt_pose.position.x), 4),
            'y': round(float(gt_pose.position.y), 4),
            'yaw': round(gt_yaw, 4),
        }
        result['amcl'] = {
            'x': round(float(amcl_pose.position.x), 4),
            'y': round(float(amcl_pose.position.y), 4),
            'yaw': round(amcl_yaw, 4),
        }
        result['amcl_error_m'] = round(
            math.hypot(float(amcl_pose.position.x) - gt_map_x,
                       float(amcl_pose.position.y) - gt_map_y),
            4,
        )
        result['amcl_yaw_error_deg'] = round(
            math.degrees(angle_error(amcl_yaw, gt_yaw)), 2,
        )
        result['covariance'] = [
            round(float(self.amcl.pose.covariance[index]), 4)
            for index in (0, 7, 35)
        ]
        result['odom_speed'] = {
            'linear_x': round(float(self.odom.twist.twist.linear.x), 4),
            'angular_z': round(float(self.odom.twist.twist.angular.z), 4),
        }
        return result


def pose_preconditions(snapshot: dict, args) -> tuple[bool, str | None]:
    """Pose/TF/cov/stop checks without requiring AMCL message growth."""
    if snapshot.get('clock_sec') is None:
        return False, 'sim_clock_zero'
    if not snapshot['sensor_fresh']['ground_truth']:
        return False, 'gt_not_fresh'
    if not snapshot['sensor_fresh']['odom']:
        return False, 'odom_not_fresh'
    if not snapshot['sensor_fresh']['amcl_pose_present']:
        return False, 'amcl_missing'
    if not snapshot['tf_ready']:
        return False, 'tf_not_ready'
    error = snapshot['amcl_error_m']
    covariance = snapshot['covariance']
    speed = snapshot['odom_speed']
    if error is None or covariance is None or speed is None:
        return False, 'pose_not_ready'
    if not math.isfinite(error) or error > args.amcl_error_gate:
        return False, 'amcl_error'
    if any(not math.isfinite(value) for value in covariance):
        return False, 'covariance_invalid'
    if covariance[0] > args.max_cov_xy or covariance[1] > args.max_cov_xy:
        return False, 'covariance'
    if covariance[2] > args.max_cov_yaw:
        return False, 'covariance'
    if args.require_stopped and (
        abs(speed['linear_x']) >= args.max_linear_speed
        or abs(speed['angular_z']) >= args.max_angular_speed
    ):
        return False, 'robot_not_stopped'
    return True, None


def validity(snapshot: dict, args, gt_delta: int, odom_delta: int) -> tuple[bool, str | None]:
    """Stable-window check: pose preconditions + continuing GT/odom stream."""
    ok, reason = pose_preconditions(snapshot, args)
    if not ok:
        return False, reason
    # Stable window must observe continuing GT/odom traffic. AMCL may stay quiet.
    if gt_delta < args.min_gt_updates or odom_delta < args.min_odom_updates:
        return False, 'sensors_not_streaming'
    return True, None


def write_result(path: Path | None, result: dict) -> None:
    text = json.dumps(result, indent=2) + '\n'
    if path:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    print(f'NAV_READY_RESULT: {json.dumps(result)}', flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description='等待可比的 AMCL/Nav2 初始状态')
    parser.add_argument('--map-yaml', type=Path, default=Path(__file__).resolve().parents[1]
                        / 'ros2_ws/src/navigation_config/maps/test_room.yaml')
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument('--stable-sec', type=float, default=1.5)
    parser.add_argument('--amcl-error-gate', type=float, default=0.10)
    parser.add_argument('--max-cov-xy', type=float, default=0.25)
    parser.add_argument('--max-cov-yaw', type=float, default=0.20)
    parser.add_argument('--max-linear-speed', type=float, default=0.02)
    parser.add_argument('--max-angular-speed', type=float, default=0.05)
    parser.add_argument('--transform-tolerance', type=float, default=0.5,
                        help='AMCL transform_tolerance; future TF stamps within this are OK')
    parser.add_argument('--min-gt-updates', type=int, default=5,
                        help='Minimum new /ground_truth messages during stable window')
    parser.add_argument('--min-odom-updates', type=int, default=5,
                        help='Minimum new /odom messages during stable window')
    parser.add_argument('--require-stopped', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()

    if not args.map_yaml.is_file():
        print(f'找不到地图 yaml: {args.map_yaml}', file=sys.stderr)
        return 2

    origin = load_map_origin(args.map_yaml)
    rclpy.init()
    node = rclpy.create_node(
        'wait_for_nav_ready',
        parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
    )
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    _ = tf_listener
    probe = ReadinessProbe(node, origin, tf_buffer, args.transform_tolerance)
    start = time.monotonic()
    valid_since = None
    counts_at_valid_start = None
    latest = None
    last_fail_reason = 'timeout'
    polls = 0

    try:
        while time.monotonic() - start < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            polls += 1
            latest = probe.snapshot()

            if valid_since is None:
                # Enter stable window once pose/TF/cov look good. Streaming
                # deltas are enforced only inside the window (AMCL may stay quiet).
                pose_ok, pose_reason = pose_preconditions(latest, args)
                if pose_ok:
                    valid_since = time.monotonic()
                    counts_at_valid_start = {
                        'ground_truth': probe.gt_count,
                        'odom': probe.odom_count,
                        'amcl_pose': probe.amcl_count,
                        'clock_advances': probe.clock_advances,
                    }
                    last_fail_reason = None
                else:
                    last_fail_reason = pose_reason
                continue

            gt_delta = probe.gt_count - counts_at_valid_start['ground_truth']
            odom_delta = probe.odom_count - counts_at_valid_start['odom']
            amcl_delta = probe.amcl_count - counts_at_valid_start['amcl_pose']
            clock_delta = probe.clock_advances - counts_at_valid_start['clock_advances']
            pose_ok, pose_reason = pose_preconditions(latest, args)
            if not pose_ok:
                # Pose/TF/cov broke → restart window.
                valid_since = None
                counts_at_valid_start = None
                last_fail_reason = pose_reason
                continue
            if clock_delta <= 0 and time.monotonic() - valid_since > 0.5:
                # Sim clock must keep advancing during the stable window.
                valid_since = None
                counts_at_valid_start = None
                last_fail_reason = 'sim_clock_frozen'
                continue
            # Streaming deltas may still be accumulating; do not reset the window.
            if time.monotonic() - valid_since < args.stable_sec:
                continue
            if gt_delta < args.min_gt_updates or odom_delta < args.min_odom_updates:
                last_fail_reason = 'sensors_not_streaming'
                # Keep waiting until timeout; pose still OK so hold the window
                # only if sensors remain fresh. If they stall, ages will fail pose.
                continue
            result = {
                'ready': True,
                'reason': 'ready',
                'elapsed_sec': round(time.monotonic() - start, 2),
                'stable_sec': args.stable_sec,
                'polls': polls,
                'topic_counts': {
                    'ground_truth': probe.gt_count,
                    'amcl_pose': probe.amcl_count,
                    'odom': probe.odom_count,
                },
                'stable_window_deltas': {
                    'ground_truth': gt_delta,
                    'odom': odom_delta,
                    # Informational only: may be 0 while stationary.
                    'amcl_pose': amcl_delta,
                    'clock_advances': clock_delta,
                },
                'protocol_notes': {
                    'amcl_new_messages_required': False,
                    'amcl_update_min_d': 0.05,
                    'amcl_update_min_a': 0.10,
                    'transform_tolerance_sec': args.transform_tolerance,
                    'gt_reseed': False,
                },
                'thresholds': {
                    'amcl_error_m': args.amcl_error_gate,
                    'cov_xy': args.max_cov_xy,
                    'cov_yaw': args.max_cov_yaw,
                    'min_gt_updates': args.min_gt_updates,
                    'min_odom_updates': args.min_odom_updates,
                },
                'latest': latest,
            }
            write_result(args.output, result)
            return 0

        result = {
            'ready': False,
            'reason': last_fail_reason or 'timeout',
            'elapsed_sec': round(time.monotonic() - start, 2),
            'polls': polls,
            'topic_counts': {
                'ground_truth': probe.gt_count,
                'amcl_pose': probe.amcl_count,
                'odom': probe.odom_count,
            },
            'protocol_notes': {
                'amcl_new_messages_required': False,
                'transform_tolerance_sec': args.transform_tolerance,
                'gt_reseed': False,
            },
            'latest': latest,
        }
        write_result(args.output, result)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
