#!/usr/bin/env python3
"""显式原地转向闸门：经 Nav2 /spin action（不用直接发 /cmd_vel）。

Accept 超时（默认 10s）可销毁 client 后重试 1 次；已 accepted 则只等 result。

用法（Nav2 已运行）：
  python3 scripts/spin_gate.py --name B_face_door --target-yaw 3.14159 \\
      --expect-yaw 1.5708 --timeout 60
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from nav2_msgs.action import Spin
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

ACCEPT_TIMEOUT_SEC = 10.0
ACCEPT_RETRY_SLEEP_SEC = 1.0
POST_SUCCESS_SLEEP_SEC = 0.8
MAX_ACCEPT_RETRIES = 1  # 共最多 2 次发送（retry_index 0 与 1）


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
        GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
        GoalStatus.STATUS_ABORTED: 'ABORTED',
        GoalStatus.STATUS_CANCELED: 'CANCELED',
        GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    }
    return labels.get(status, str(status))


class SpinGate(Node):
    STOPPED_WINDOW_SEC = 1.0

    def __init__(
        self,
        name: str,
        target_yaw: float,
        expect_yaw: float | None,
        timeout_sec: float,
        yaw_gate_deg: float,
        vel_gate: float,
        omega_gate: float,
        accept_timeout_sec: float,
    ) -> None:
        super().__init__(
            'spin_gate',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.name = name
        self.target_yaw = target_yaw
        self.expect_yaw = expect_yaw
        self.timeout_sec = timeout_sec
        self.yaw_gate_deg = yaw_gate_deg
        self.vel_gate = vel_gate
        self.omega_gate = omega_gate
        self.accept_timeout_sec = accept_timeout_sec

        self.gt: Odometry | None = None
        self.odom: Odometry | None = None
        self.goal_handle = None
        self.exit_code = 1
        self.result_status = 'UNKNOWN'
        self.yaw_err_deg = float('inf')
        self.gt_yaw = float('nan')
        self.stopped_pass = False
        self.odom_twist_samples: list[tuple[float, float, float]] = []
        self.angular_traveled = 0.0

        # 诊断字段
        self.server_ready = 0
        self.goal_sent = 0
        self.goal_accepted = 0
        self.goal_accept_timeout = 0
        self.retry_index = 0
        self.spin_result = 'NONE'
        self.transport_retry = 0

        clock_deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=20.0)
        while self.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.get_clock().now() > clock_deadline:
                break

        self._client: ActionClient | None = None
        self.create_subscription(Odometry, '/ground_truth', self._on_gt, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)

    def _on_gt(self, msg: Odometry) -> None:
        self.gt = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _log_diag(self, event: str, **extra: object) -> None:
        bits = ' '.join(f'{k}={v}' for k, v in extra.items())
        print(
            f'[SPIN_DIAG] name={self.name} event={event} '
            f'retry_index={self.retry_index} '
            f'server_ready={self.server_ready} goal_sent={self.goal_sent} '
            f'goal_accepted={self.goal_accepted} '
            f'goal_accept_timeout={self.goal_accept_timeout} '
            f'spin_result={self.spin_result}'
            + (f' {bits}' if bits else '')
        )

    def _make_client(self) -> ActionClient:
        if self._client is not None:
            try:
                self._client.destroy()
            except Exception:  # noqa: BLE001
                pass
            self._client = None
        self._client = ActionClient(self, Spin, '/spin')
        return self._client

    def _wait_server(self, timeout_sec: float = 15.0) -> bool:
        assert self._client is not None
        ok = self._client.wait_for_server(timeout_sec=timeout_sec)
        self.server_ready = 1 if ok else 0
        self._log_diag('server_ready_check', ok=int(ok))
        return ok

    def _send_and_wait_accept(self) -> object | None:
        """发送 goal，最多 wait accept_timeout；返回 goal_handle 或 None(ACCEPT_TIMEOUT)。"""
        assert self._client is not None
        goal = Spin.Goal()
        goal.target_yaw = float(self.target_yaw)
        allow = max(5, int(self.timeout_sec))
        goal.time_allowance = Duration(sec=allow, nanosec=0)

        self.goal_sent = 1
        self._log_diag('goal_sent')
        send_fut = self._client.send_goal_async(goal)

        t0 = time.monotonic()
        while rclpy.ok() and (time.monotonic() - t0) < self.accept_timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if send_fut.done():
                break
        else:
            # accept/response 未在时限内就绪
            self.goal_accept_timeout = 1
            self._log_diag('ACCEPT_TIMEOUT')
            print(f'[SPIN] ACCEPT_TIMEOUT after {self.accept_timeout_sec:.0f}s')
            return None

        try:
            handle = send_fut.result()
        except Exception as exc:  # noqa: BLE001
            self.goal_accept_timeout = 1
            self._log_diag('ACCEPT_TIMEOUT', error=str(exc))
            print(f'[SPIN] ACCEPT_TIMEOUT (send_goal exception: {exc})')
            return None

        if handle is None or not handle.accepted:
            self.goal_accepted = 0
            self._log_diag('goal_rejected')
            print('[SPIN] goal rejected (not accepted)')
            return None

        self.goal_accepted = 1
        self._log_diag('goal_accepted')
        return handle

    def _wait_result(self, handle: object) -> int:
        """已 accepted：只等 result，不重发。返回 GoalStatus。"""
        result_fut = handle.get_result_async()  # type: ignore[attr-defined]
        t0 = time.monotonic()
        while rclpy.ok() and (time.monotonic() - t0) < self.timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            if result_fut.done():
                break
        else:
            try:
                handle.cancel_goal_async()  # type: ignore[attr-defined]
            except Exception:  # noqa: BLE001
                pass
            self.spin_result = 'TIMEOUT'
            self._log_diag('result_timeout')
            return GoalStatus.STATUS_UNKNOWN

        wrapped = result_fut.result()
        status = wrapped.status
        if hasattr(wrapped, 'result') and hasattr(wrapped.result, 'total_elapsed_time'):
            pass
        # feedback 未必有；angular 从结果侧不可得时保持 0
        self.spin_result = status_label(status)
        self._log_diag('spin_result')
        return status

    def _sample_stopped(self) -> None:
        self.odom_twist_samples = []
        t0 = self.get_clock().now()
        deadline = time.monotonic() + self.STOPPED_WINDOW_SEC
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is None:
                continue
            t_sec = (self.get_clock().now() - t0).nanoseconds / 1e9
            v = self.odom.twist.twist.linear.x
            w = self.odom.twist.twist.angular.z
            self.odom_twist_samples.append((t_sec, v, w))

        max_v = 0.0
        max_w = 0.0
        settle = None
        for t_sec, v, w in self.odom_twist_samples:
            max_v = max(max_v, abs(v))
            max_w = max(max_w, abs(w))
            if settle is None and abs(v) < self.vel_gate and abs(w) < self.omega_gate:
                settle = t_sec
        re_vio = False
        if settle is not None:
            re_vio = any(
                t > settle and (abs(v) >= self.vel_gate or abs(w) >= self.omega_gate)
                for t, v, w in self.odom_twist_samples
            )
        self.stopped_pass = settle is not None and not re_vio
        self._stopped_max_v = max_v
        self._stopped_max_w = max_w

    def _capture_yaw(self) -> None:
        if self.gt is None:
            self.gt_yaw = float('nan')
            self.yaw_err_deg = float('inf')
            return
        q = self.gt.pose.pose.orientation
        self.gt_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        if self.expect_yaw is None:
            self.yaw_err_deg = float('nan')
            return
        self.yaw_err_deg = abs(math.degrees(normalize_angle(self.gt_yaw - self.expect_yaw)))

    def run(self) -> int:
        print(
            f'[SPIN] name={self.name} target_yaw={self.target_yaw:.4f} rad '
            f'expect_yaw={self.expect_yaw if self.expect_yaw is not None else "n/a"} '
            f'accept_timeout={self.accept_timeout_sec:.0f}s'
        )

        handle = None
        for attempt in range(MAX_ACCEPT_RETRIES + 1):
            self.retry_index = attempt
            self.goal_sent = 0
            self.goal_accepted = 0
            self.goal_accept_timeout = 0
            self.spin_result = 'NONE'

            self._make_client()
            if not self._wait_server(15.0):
                self._fail_summary('Action server /spin 不可用')
                return self.exit_code

            handle = self._send_and_wait_accept()
            if handle is not None:
                break

            # accept 超时或 reject：仅在未 accepted 时允许重试
            if attempt < MAX_ACCEPT_RETRIES:
                self.transport_retry = 1
                print(
                    f'[SPIN] recreate ActionClient after ACCEPT_TIMEOUT; '
                    f'sleep {ACCEPT_RETRY_SLEEP_SEC:.0f}s then retry '
                    f'(retry_index will be {attempt + 1})'
                )
                try:
                    if self._client is not None:
                        self._client.destroy()
                        self._client = None
                except Exception:  # noqa: BLE001
                    pass
                t_end = time.monotonic() + ACCEPT_RETRY_SLEEP_SEC
                while time.monotonic() < t_end and rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.05)
            else:
                self._fail_summary('ACCEPT_TIMEOUT after retry')
                return self.exit_code

        assert handle is not None
        # 已 accepted：禁止重发，只等 result
        status = self._wait_result(handle)
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.result_status = self.spin_result
            self._fail_summary(f'Spin {self.spin_result}')
            return self.exit_code

        self.result_status = 'SUCCEEDED'
        self._capture_yaw()
        self._sample_stopped()

        yaw_ok = True
        if self.expect_yaw is not None and math.isfinite(self.yaw_err_deg):
            yaw_ok = self.yaw_err_deg <= self.yaw_gate_deg
        final_pass = yaw_ok and self.stopped_pass

        # 收尾等待，避免下一 action 踩未清理状态
        t_end = time.monotonic() + POST_SUCCESS_SLEEP_SEC
        while time.monotonic() < t_end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

        self._print_pass_summary(final_pass, yaw_ok)
        self.exit_code = 0 if final_pass else 1
        return self.exit_code

    def _summary_line(self, outcome: str) -> str:
        return (
            f'SPIN:{self.name}:{outcome}:'
            f'accepted={self.goal_accepted}:retry={self.retry_index}'
        )

    def _print_pass_summary(self, final_pass: bool, yaw_ok: bool) -> None:
        outcome = 'PASS' if final_pass else 'FAIL'
        print(self._summary_line(outcome))
        print(f'target_yaw: {self.target_yaw:.4f} rad')
        print(f'result status: {self.result_status}')
        print(
            f'transport: server_ready={self.server_ready} goal_sent={self.goal_sent} '
            f'goal_accepted={self.goal_accepted} goal_accept_timeout={self.goal_accept_timeout} '
            f'retry_index={self.retry_index} transport_retry={self.transport_retry} '
            f'spin_result={self.spin_result}'
        )
        if math.isfinite(self.gt_yaw):
            print(f'ground_truth yaw: {self.gt_yaw:.4f} rad')
        else:
            print('ground_truth yaw: n/a')
        if self.expect_yaw is not None:
            print(
                f'Yaw Error: {self.yaw_err_deg:.1f} deg  '
                f'{"PASS" if yaw_ok else "FAIL"} (gate {self.yaw_gate_deg:.0f}°)'
            )
        print(
            f'Stopped: {"PASS" if self.stopped_pass else "FAIL"} '
            f'(samples={len(self.odom_twist_samples)} '
            f'max|v|={self._stopped_max_v:.4f} max|ω|={self._stopped_max_w:.4f})'
        )
        print(f'FINAL: {"PASS" if final_pass else "FAIL"}')

    def _fail_summary(self, reason: str) -> None:
        self.exit_code = 1
        print(self._summary_line('FAIL'))
        print(f'target_yaw: {self.target_yaw:.4f} rad')
        print(f'result status: FAIL ({reason})')
        print(
            f'transport: server_ready={self.server_ready} goal_sent={self.goal_sent} '
            f'goal_accepted={self.goal_accepted} goal_accept_timeout={self.goal_accept_timeout} '
            f'retry_index={self.retry_index} transport_retry={self.transport_retry} '
            f'spin_result={self.spin_result}'
        )
        print('FINAL: FAIL')


def main() -> int:
    parser = argparse.ArgumentParser(description='Nav2 /spin 闸门（accept 超时可重试 1 次）')
    parser.add_argument('--name', type=str, default='spin')
    parser.add_argument('--target-yaw', type=float, required=True, help='相对旋转量 (rad)')
    parser.add_argument(
        '--expect-yaw', type=float, default=None,
        help='期望最终绝对航向 (rad, world/gt)；省略则不过 yaw 闸',
    )
    parser.add_argument('--timeout', type=float, default=60.0, help='已 accepted 后等 result')
    parser.add_argument(
        '--accept-timeout', type=float, default=ACCEPT_TIMEOUT_SEC,
        help='等 goal accepted 的上限（秒）',
    )
    parser.add_argument('--yaw-gate', type=float, default=10.0, help='度')
    parser.add_argument('--vel-gate', type=float, default=0.02)
    parser.add_argument('--omega-gate', type=float, default=0.08,
                        help='Spin 后停稳 ω 闸（机动段略宽于巡检 0.05）')
    args = parser.parse_args()

    rclpy.init()
    node = SpinGate(
        name=args.name,
        target_yaw=args.target_yaw,
        expect_yaw=args.expect_yaw,
        timeout_sec=args.timeout,
        yaw_gate_deg=args.yaw_gate,
        vel_gate=args.vel_gate,
        omega_gate=args.omega_gate,
        accept_timeout_sec=args.accept_timeout,
    )
    t0 = time.time()
    while time.time() - t0 < 2.0 and node.gt is None and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
    code = node.run()
    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
