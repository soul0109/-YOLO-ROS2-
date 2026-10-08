#!/usr/bin/env python3
"""
4.5a：NavigateToPose 三指标验收（到位 / 航向 / 停稳 1s）。

诊断：分别在 action result 瞬间与停稳窗口结束后采样 GT/AMCL，
闸门仍用停稳后误差（0.20 m / 10°），同时打印 result_time 对照。

用法（nav2_test_room + initialpose 已运行）：
  python3 scripts/navigate_to_pose_gate.py --world-x 2.0 --world-y 3.0 --yaw 0
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import ParticleCloud
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


@dataclass
class PoseSnapshot:
    """某一时刻的 GT/AMCL 对照快照。"""
    label: str
    gt_pos_err: float = float('inf')
    gt_yaw_err_deg: float = float('inf')
    amcl_vs_gt_xy: float = float('nan')
    amcl_vs_gt_yaw_deg: float = float('nan')
    cov0: float = float('nan')
    cov7: float = float('nan')
    cov35: float = float('nan')
    particle_n: int = 0
    particle_spread_xy: float = float('nan')  # 加权 xy RMS
    notes: str = ''


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
        self.particle_cloud: ParticleCloud | None = None
        self.goal_handle = None
        self.stopped_sampling = False
        self.odom_twist_samples: list[tuple[float, float, float]] = []  # (t_sec, v, w)
        self._stopped_window_start = None
        self._stopped_timer = None
        self.done = False
        self.exit_code = 1
        self.fail_reason = ''

        self.snap_result = PoseSnapshot('result_time')
        self.snap_settled = PoseSnapshot('settled_time')
        # 闸门用停稳后快照
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
        self.create_subscription(
            ParticleCloud, '/particle_cloud', self._on_particles, qos_profile_sensor_data,
        )
        self._timeout_timer = self.create_timer(timeout_sec, self._on_timeout)

    def _on_gt(self, msg: Odometry) -> None:
        self.gt = msg

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl = msg

    def _on_particles(self, msg: ParticleCloud) -> None:
        self.particle_cloud = msg

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

        # result_time：Nav2 宣布 SUCCEEDED / Reached 的瞬间（对照用，不单独作最终闸）
        self.snap_result = self._capture_snapshot('result_time')
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

    def _particle_spread_xy(self) -> tuple[int, float]:
        cloud = self.particle_cloud
        if cloud is None or not cloud.particles:
            return 0, float('nan')
        parts = cloud.particles
        wsum = sum(max(p.weight, 0.0) for p in parts)
        if wsum <= 0.0:
            wsum = float(len(parts))
            weights = [1.0] * len(parts)
        else:
            weights = [max(p.weight, 0.0) for p in parts]
        mx = sum(w * p.pose.position.x for w, p in zip(weights, parts)) / wsum
        my = sum(w * p.pose.position.y for w, p in zip(weights, parts)) / wsum
        var = sum(
            w * ((p.pose.position.x - mx) ** 2 + (p.pose.position.y - my) ** 2)
            for w, p in zip(weights, parts)
        ) / wsum
        return len(parts), math.sqrt(var)

    def _capture_snapshot(self, label: str) -> PoseSnapshot:
        snap = PoseSnapshot(label=label)
        n, spread = self._particle_spread_xy()
        snap.particle_n = n
        snap.particle_spread_xy = spread

        if self.gt is None:
            snap.notes = 'no_gt'
            return snap

        gt_x = self.gt.pose.pose.position.x
        gt_y = self.gt.pose.pose.position.y
        gq = self.gt.pose.pose.orientation
        gt_yaw = yaw_from_quat(gq.x, gq.y, gq.z, gq.w)
        snap.gt_pos_err = math.hypot(gt_x - self.world_x, gt_y - self.world_y)
        snap.gt_yaw_err_deg = abs(math.degrees(normalize_angle(gt_yaw - self.world_yaw)))

        if self.amcl is None:
            snap.notes = 'no_amcl'
            return snap

        ax = self.amcl.pose.pose.position.x
        ay = self.amcl.pose.pose.position.y
        aq = self.amcl.pose.pose.orientation
        amcl_yaw = yaw_from_quat(aq.x, aq.y, aq.z, aq.w)
        gt_map_x = gt_x - self.origin_x
        gt_map_y = gt_y - self.origin_y
        snap.amcl_vs_gt_xy = math.hypot(ax - gt_map_x, ay - gt_map_y)
        snap.amcl_vs_gt_yaw_deg = abs(math.degrees(normalize_angle(amcl_yaw - gt_yaw)))
        cov = self.amcl.pose.covariance
        snap.cov0 = float(cov[0])
        snap.cov7 = float(cov[7])
        snap.cov35 = float(cov[35])
        return snap

    def _finish_stopped_check(self) -> None:
        if self.done:
            return
        if self._stopped_timer is not None:
            self._stopped_timer.cancel()
            self._stopped_timer = None
        self.stopped_sampling = False
        self._analyze_stopped_window()
        # settled_time：停稳窗口结束后重新采样（最终闸门依据）
        self.snap_settled = self._capture_snapshot('settled_time')
        self.pos_err = self.snap_settled.gt_pos_err
        self.yaw_err_deg = self.snap_settled.gt_yaw_err_deg
        self.amcl_gt_diag = self.snap_settled.amcl_vs_gt_xy
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

    def _fmt_snap(self, snap: PoseSnapshot) -> None:
        def fnum(v: float, fmt: str) -> str:
            return fmt.format(v) if math.isfinite(v) else 'n/a'

        print(f'[snap:{snap.label}] GT pos_err={fnum(snap.gt_pos_err, "{:.3f}")} m  '
              f'yaw_err={fnum(snap.gt_yaw_err_deg, "{:.1f}")} deg')
        print(
            f'[snap:{snap.label}] AMCL vs GT xy={fnum(snap.amcl_vs_gt_xy, "{:.3f}")} m  '
            f'yaw={fnum(snap.amcl_vs_gt_yaw_deg, "{:.1f}")} deg'
        )
        print(
            f'[snap:{snap.label}] cov[0]={fnum(snap.cov0, "{:.3f}")} '
            f'cov[7]={fnum(snap.cov7, "{:.3f}")} cov[35]={fnum(snap.cov35, "{:.3f}")}'
        )
        print(
            f'[snap:{snap.label}] particle_n={snap.particle_n} '
            f'spread_xy={fnum(snap.particle_spread_xy, "{:.3f}")} m'
            + (f' notes={snap.notes}' if snap.notes else '')
        )

    def _print_report(self) -> None:
        # 最终闸门：停稳后 GT（阈值不变 0.20 m / 10°）
        pos_pass = self.pos_err <= self.pos_gate
        yaw_pass = self.yaw_err_deg <= self.yaw_gate_deg
        final_pass = pos_pass and yaw_pass and self.stopped_pass

        print(f'Goal (x, y, yaw): ({self.world_x}, {self.world_y}, {self.world_yaw})')
        print('--- sampling (result_time = Nav2 SUCCEEDED 瞬间; settled_time = 停稳窗结束后) ---')
        self._fmt_snap(self.snap_result)
        self._fmt_snap(self.snap_settled)
        print('--- gate uses settled_time ---')
        print(
            f'Position Error (settled): {self.pos_err:.3f} m  '
            f'{"PASS" if pos_pass else "FAIL"} (gate {self.pos_gate:.2f} m)'
        )
        print(
            f'Yaw Error (settled):      {self.yaw_err_deg:.1f} deg  '
            f'{"PASS" if yaw_pass else "FAIL"} (gate {self.yaw_gate_deg:.0f}°)'
        )
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
        if self.recoveries:
            print(f'[diag] recoveries during nav: {self.recoveries}')
        # 粗分类提示（人工确认）
        r, s = self.snap_result, self.snap_settled
        if (
            math.isfinite(s.gt_pos_err) and s.gt_pos_err <= self.pos_gate
            and math.isfinite(s.gt_yaw_err_deg) and s.gt_yaw_err_deg <= self.yaw_gate_deg
        ):
            hint = 'settled_PASS → 更像 result_time 取样时机问题'
        elif (
            math.isfinite(s.amcl_vs_gt_xy) and s.amcl_vs_gt_xy > 0.15
        ) or (
            math.isfinite(s.particle_spread_xy) and math.isfinite(r.particle_spread_xy)
            and s.particle_spread_xy > r.particle_spread_xy * 1.3
        ):
            hint = 'AMCL↔GT 大 / particle spread 升 → 优先怀疑 AMCL 漂移'
        elif (
            math.isfinite(s.amcl_vs_gt_xy) and s.amcl_vs_gt_xy < 0.10
            and math.isfinite(s.gt_pos_err) and s.gt_pos_err > self.pos_gate
        ):
            hint = 'AMCL≈GT 但 GT 仍超闸 → 局部控制或目标姿态'
        else:
            hint = '需对照 bag /cmd_vel /local_plan 再分'
        print(f'[classify_hint] {hint}')
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
