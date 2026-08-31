#!/usr/bin/env python3
"""
4.5a：NavigateToPose 三指标验收（到位 / 航向 / 停稳 1s）。

用法（nav2_test_room + initialpose 已运行）：
  python3 scripts/navigate_to_pose_gate.py --world-x 2.0 --world-y 3.0 --yaw 0
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from coords import load_map_origin, world_to_map_xy, yaw_to_quat

AMCL_POSE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

DEFAULT_MAP_YAML = (
    Path(__file__).resolve().parents[1] / 'ros2_ws/src/navigation_config/maps/test_room.yaml'
)


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def status_label(status: int) -> str:
    labels = {
        GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
        GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
        GoalStatus.STATUS_EXECUTING: 'EXECUTING',
        GoalStatus.STATUS_CANCELING: 'CANCELING',
        GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
        GoalStatus.STATUS_CANCELED: 'CANCELED',
        GoalStatus.STATUS_ABORTED: 'ABORTED',
    }
    return labels.get(status, str(status))


class NavigateToPoseGate(Node):
    STOPPED_WINDOW_SEC = 1.0

    def __init__(
        self,
        world_x: float,
        world_y: float,
        world_yaw: float,
        timeout_sec: float,
        map_yaml: Path,
        pos_gate: float,
        yaw_gate_deg: float,
        vel_gate: float,
        omega_gate: float,
    ):
        super().__init__(
            'navigate_to_pose_gate',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.world_x = world_x
        self.world_y = world_y
        self.world_yaw = world_yaw
        self.timeout_sec = timeout_sec
        self.pos_gate = pos_gate
        self.yaw_gate_deg = yaw_gate_deg
        self.vel_gate = vel_gate
        self.omega_gate = omega_gate

        ox, oy, oyaw = load_map_origin(map_yaml)
        self.origin_x = ox
        self.origin_y = oy
        mx, my = world_to_map_xy(world_x, world_y, ox, oy)
        qx, qy, qz, qw = yaw_to_quat(world_yaw - oyaw)
        self.goal_map_x = mx
        self.goal_map_y = my
        self.goal_quat = (qx, qy, qz, qw)

        self.gt: Odometry | None = None
        self.amcl: PoseWithCovarianceStamped | None = None
        self.goal_handle = None
        self.stopped_sampling = False
        self.odom_twist_samples: list[tuple[float, float, float]] = []  # (t_sec, v, w)
        self._stopped_window_start = None
        self._stopped_timer = None
        self.done = False
        self.exit_code = 1
        self.fail_reason = ''

        self.pos_err = float('inf')
        self.yaw_err_deg = float('inf')
        self.amcl_gt_diag = float('nan')
        self.stopped_pass = False
        self.recoveries = 0
        self.stopped_max_v = 0.0
        self.stopped_max_w = 0.0
        self.stopped_time_to_ok_ms: float | None = None
        self.stopped_violations: list[tuple[float, float, float]] = []
        self.stopped_settle_sec: float | None = None
        self.stopped_re_violation = False

        # 等 /clock，避免 timer/stamp 卡在 0
        clock_deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=20.0)
        while self.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now() > clock_deadline:
                break

        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.create_subscription(Odometry, '/ground_truth', self._on_gt, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, AMCL_POSE_QOS,
        )
        self._timeout_timer = self.create_timer(timeout_sec, self._on_timeout)

    def _on_gt(self, msg: Odometry) -> None:
        self.gt = msg

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl = msg

    def _on_odom(self, msg: Odometry) -> None:
        if not self.stopped_sampling or self._stopped_window_start is None:
            return
        t_sec = (self.get_clock().now() - self._stopped_window_start).nanoseconds / 1e9
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.odom_twist_samples.append((t_sec, v, w))

    def start_navigation(self) -> None:
        if not self._action_client.wait_for_server(timeout_sec=15.0):
            self._fail('Action server /navigate_to_pose 不可用')
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal_map_x
        goal.pose.pose.position.y = self.goal_map_y
        goal.pose.pose.orientation.x = self.goal_quat[0]
        goal.pose.pose.orientation.y = self.goal_quat[1]
        goal.pose.pose.orientation.z = self.goal_quat[2]
        goal.pose.pose.orientation.w = self.goal_quat[3]

        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback,
        )
        send_future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        if hasattr(feedback, 'number_of_recoveries'):
            self.recoveries = feedback.number_of_recoveries

    def _on_goal_response(self, future) -> None:
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self._fail('Goal rejected by bt_navigator')
            return
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        if self.done:
            return
        self._timeout_timer.cancel()
        wrapped = future.result()
        status = wrapped.status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._fail(f'NavigateToPose {status_label(status)}')
            return

        self._capture_goal_errors()
        self.stopped_sampling = True
        self.odom_twist_samples = []
        self._stopped_window_start = self.get_clock().now()
        self._stopped_timer = self.create_timer(self.STOPPED_WINDOW_SEC, self._finish_stopped_check)

    def _analyze_stopped_window(self) -> None:
        """停稳判定对齐规格 §5：到位后 1.0 s 内沉降，沉降后无再次超标。"""
        self.stopped_max_v = 0.0
        self.stopped_max_w = 0.0
        self.stopped_time_to_ok_ms = None
        self.stopped_violations = []
        self.stopped_settle_sec: float | None = None
        self.stopped_re_violation = False

        if not self.odom_twist_samples:
            self.stopped_pass = False
            return

        for t_sec, v, w in self.odom_twist_samples:
            self.stopped_max_v = max(self.stopped_max_v, abs(v))
            self.stopped_max_w = max(self.stopped_max_w, abs(w))
            if abs(v) >= self.vel_gate or abs(w) >= self.omega_gate:
                self.stopped_violations.append((t_sec, v, w))

        settle_sec: float | None = None
        for t_sec, v, w in self.odom_twist_samples:
            if abs(v) < self.vel_gate and abs(w) < self.omega_gate:
                settle_sec = t_sec
                self.stopped_time_to_ok_ms = t_sec * 1000.0
                break

        self.stopped_settle_sec = settle_sec
        if settle_sec is None:
            self.stopped_pass = False
            return

        self.stopped_re_violation = any(
            t_sec > settle_sec
            and (abs(v) >= self.vel_gate or abs(w) >= self.omega_gate)
            for t_sec, v, w in self.odom_twist_samples
        )
        self.stopped_pass = (
            settle_sec <= self.STOPPED_WINDOW_SEC and not self.stopped_re_violation
        )

    def _capture_goal_errors(self) -> None:
        if self.gt is None:
            self.pos_err = float('inf')
            self.yaw_err_deg = float('inf')
            self.amcl_gt_diag = float('nan')
            return

        gt_x = self.gt.pose.pose.position.x
        gt_y = self.gt.pose.pose.position.y
        gq = self.gt.pose.pose.orientation
        gt_yaw = yaw_from_quat(gq.x, gq.y, gq.z, gq.w)

        self.pos_err = math.hypot(gt_x - self.world_x, gt_y - self.world_y)
        self.yaw_err_deg = abs(math.degrees(normalize_angle(gt_yaw - self.world_yaw)))

        if self.amcl is not None:
            ax = self.amcl.pose.pose.position.x
            ay = self.amcl.pose.pose.position.y
            gt_map_x = gt_x - self.origin_x
            gt_map_y = gt_y - self.origin_y
            self.amcl_gt_diag = math.hypot(ax - gt_map_x, ay - gt_map_y)
        else:
            self.amcl_gt_diag = float('nan')

    def _finish_stopped_check(self) -> None:
        if self.done:
            return
        if self._stopped_timer is not None:
            self._stopped_timer.cancel()
            self._stopped_timer = None
        self.stopped_sampling = False
        self._analyze_stopped_window()
        self._print_report()
        self.done = True

    def _on_timeout(self) -> None:
        if self.done:
            return
        if self.goal_handle is not None and self.goal_handle.accepted:
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda _f: self._fail(
                f'Timeout after {self.timeout_sec:.0f}s waiting for result',
            ))
        else:
            self._fail(f'Timeout after {self.timeout_sec:.0f}s waiting for result')

    def _fail(self, reason: str) -> None:
        if self.done:
            return
        self._timeout_timer.cancel()
        self.fail_reason = reason
        self.done = True
        self.exit_code = 1
        print(f'Goal (x, y, yaw): ({self.world_x}, {self.world_y}, {self.world_yaw})')
        print(f'FAIL: {reason}')
        print('FINAL: FAIL')

    def _print_report(self) -> None:
        pos_pass = self.pos_err <= self.pos_gate
        yaw_pass = self.yaw_err_deg <= self.yaw_gate_deg
        final_pass = pos_pass and yaw_pass and self.stopped_pass

        print(f'Goal (x, y, yaw): ({self.world_x}, {self.world_y}, {self.world_yaw})')
        print(f'Position Error: {self.pos_err:.3f} m  {"PASS" if pos_pass else "FAIL"}')
        print(f'Yaw Error:      {self.yaw_err_deg:.1f} deg  {"PASS" if yaw_pass else "FAIL"}')
        if self.stopped_settle_sec is not None:
            settle_label = f'Stopped (settle {self.stopped_settle_sec:.2f} s)'
        else:
            settle_label = 'Stopped (settle n/a)'
        print(f'{settle_label}: {"PASS" if self.stopped_pass else "FAIL"}')
        n = len(self.odom_twist_samples)
        print(
            f'[stopped] samples={n}  max|v|={self.stopped_max_v:.4f} m/s  '
            f'max|ω|={self.stopped_max_w:.4f} rad/s'
        )
        if self.stopped_time_to_ok_ms is not None:
            print(f'[stopped] time_to_ok={self.stopped_time_to_ok_ms:.0f} ms (from window start)')
        elif n > 0:
            print('[stopped] time_to_ok=n/a (never within threshold in window)')
        if self.stopped_re_violation:
            print('[stopped] re-violation after settle: YES')
        if not self.stopped_pass and self.stopped_violations:
            print('[stopped] pre-settle violations (up to 5):')
            limit = self.stopped_settle_sec if self.stopped_settle_sec is not None else float('inf')
            pre = [(t, v, w) for t, v, w in self.stopped_violations if t <= limit]
            post = [(t, v, w) for t, v, w in self.stopped_violations if t > limit]
            for t_sec, v, w in pre[:5]:
                print(f'  t={t_sec * 1000:.0f} ms  v={v:+.4f} m/s  ω={w:+.4f} rad/s')
            if post:
                print('[stopped] post-settle violations (up to 5):')
                for t_sec, v, w in post[:5]:
                    print(f'  t={t_sec * 1000:.0f} ms  v={v:+.4f} m/s  ω={w:+.4f} rad/s')
        if math.isnan(self.amcl_gt_diag):
            print('[diag] AMCL vs GT at goal: n/a')
        else:
            print(f'[diag] AMCL vs GT at goal: {self.amcl_gt_diag:.3f} m')
        if self.recoveries:
            print(f'[diag] recoveries during nav: {self.recoveries}')
        print(f'FINAL: {"PASS" if final_pass else "FAIL"}')
        self.exit_code = 0 if final_pass else 1


def main() -> int:
    parser = argparse.ArgumentParser(description='NavigateToPose 三指标验收')
    parser.add_argument('--world-x', type=float, required=True)
    parser.add_argument('--world-y', type=float, required=True)
    parser.add_argument('--yaw', type=float, required=True, help='目标航向 (rad)')
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--map-yaml', type=Path, default=DEFAULT_MAP_YAML)
    parser.add_argument('--pos-gate', type=float, default=0.20)
    parser.add_argument('--yaw-gate', type=float, default=10.0, help='度')
    parser.add_argument('--vel-gate', type=float, default=0.02)
    parser.add_argument('--omega-gate', type=float, default=0.05)
    args = parser.parse_args()

    if not args.map_yaml.is_file():
        print(f'错误: 找不到地图 yaml: {args.map_yaml}', file=sys.stderr)
        return 1

    rclpy.init()
    node = NavigateToPoseGate(
        world_x=args.world_x,
        world_y=args.world_y,
        world_yaw=args.yaw,
        timeout_sec=args.timeout,
        map_yaml=args.map_yaml,
        pos_gate=args.pos_gate,
        yaw_gate_deg=args.yaw_gate,
        vel_gate=args.vel_gate,
        omega_gate=args.omega_gate,
    )
    node.start_navigation()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)

    code = node.exit_code
    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
