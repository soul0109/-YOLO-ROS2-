#!/usr/bin/env python3
"""4.5b：巡检单圈（语义 A→B→C）。

WAYPOINT VERSION = BC_SPLIT_V3_4
  A → B → B_clear(nav) → B_arc_east(nav) → B_arc_north(nav) → B_corridor_in(nav)
    → B_corridor_turn_spin(+π/2) → C_approach(nav) → C
  - navigate：NavigateToPose（world→map 一次）
  - spin：仅走廊同点换向仍用 /spin（房内已不用纯 +π Spin）
  - 房内掉头：两段路径切向短弧（各约 90°），替代单点 180° / 纯 Spin

禁止改本文件以外的 Nav2/AMCL/地图（Phase1 边界）。

  ros2 run inspection_mission patrol_mission_node
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

WAYPOINT_VERSION = 'BC_SPLIT_V3_4'

# (name, action, x, y, yaw_or_delta, kind)
#   navigate: x,y,yaw 为 world 绝对位姿
#   spin:     x,y 忽略；yaw_or_delta = /spin 的相对 target_yaw
WAYPOINTS = (
    ('A', 'navigate', 3.15, 1.75, -math.pi / 2.0, 'inspect'),
    ('B', 'navigate', 8.45, 1.75, -math.pi / 2.0, 'inspect'),
    ('B_clear', 'navigate', 8.45, 1.35, -math.pi / 2.0, 'maneuver'),
    # 路径切向短弧：东移 +90° → 北移 +90°（替代单点 180° / 纯 Spin）
    ('B_arc_east', 'navigate', 8.75, 1.35, 0.0, 'maneuver'),
    ('B_arc_north', 'navigate', 8.75, 1.75, math.pi / 2.0, 'maneuver'),
    ('B_corridor_in', 'navigate', 8.45, 3.00, math.pi / 2.0, 'maneuver'),
    # B_corridor_in≈+π/2 → 朝西 π：相对 +π/2（走廊段仍观察）
    ('B_corridor_turn_spin', 'spin', 0.0, 0.0, math.pi / 2.0, 'maneuver'),
    ('C_approach', 'navigate', 5.25, 3.00, math.pi / 2.0, 'maneuver'),
    ('C', 'navigate', 5.25, 4.25, math.pi / 2.0, 'inspect'),
)

GOAL_TIMEOUT_SEC = 300.0
SPIN_TIMEOUT_SEC = 60.0
SPIN_ACCEPT_TIMEOUT_SEC = 10.0
SPIN_ACCEPT_RETRY_SLEEP_SEC = 1.0
SPIN_POST_SUCCESS_SLEEP_SEC = 0.8
SPIN_MAX_ACCEPT_RETRIES = 1  # 共最多 2 次发送
INTER_GOAL_DWELL_SEC = 2.0
BEFORE_SPIN_EXTRA_DWELL_SEC = 1.5  # 进入 spin 段前额外 dwell（走廊第二次等）
BEFORE_INSPECT_C_DWELL_SEC = 3.5
RETRY_DWELL_SEC = 2.5
MAX_ATTEMPTS_PER_WP = 3  # 仅 navigate；spin 失败不包装成 NTP
CLEAR_TIMEOUT_SEC = 3.0
ENABLE_SOFT_REANCHOR = False
SOFT_REANCHOR_MAX_XY_VAR = 0.5
SOFT_REANCHOR_MAX_YAW_VAR = 0.5

_AMCL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def load_map_origin(map_yaml: Path) -> tuple[float, float, float]:
    data = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
    origin = data.get('origin', [0.0, 0.0, 0.0])
    return float(origin[0]), float(origin[1]), float(origin[2])


def world_to_map_xy(
    world_x: float, world_y: float, origin_x: float, origin_y: float,
) -> tuple[float, float]:
    return world_x - origin_x, world_y - origin_y


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def status_label(status: int) -> str:
    labels = {
        GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
        GoalStatus.STATUS_ABORTED: 'ABORTED',
        GoalStatus.STATUS_CANCELED: 'CANCELED',
    }
    return labels.get(status, str(status))


class PatrolMissionNode(Node):
    def __init__(self) -> None:
        super().__init__('patrol_mission_node')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        share = Path(get_package_share_directory('navigation_config'))
        map_yaml = share / 'maps' / 'test_room.yaml'
        self._ox, self._oy, _ = load_map_origin(map_yaml)
        self._goals = list(WAYPOINTS)

        self._idx = 0
        self._attempt = 0
        self._done = False
        self._exit_code = 1
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._spin_client = ActionClient(self, Spin, 'spin')
        self._clear_global = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap',
        )
        self._clear_local = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap',
        )
        self._amcl: PoseWithCovarianceStamped | None = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, _AMCL_QOS,
        )
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10,
        )
        self._timeout_timer = None
        self._dwell_timer = None
        self._goal_handle = None
        self._current_name = ''
        self._current_action = ''
        self._current_kind = ''
        self._recoveries = 0
        self._seg_t0 = 0.0
        self._seg_log: list[str] = []
        self._started = False

        clock_deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=20.0)
        while rclpy.ok() and self.get_clock().now() < clock_deadline:
            if self.get_clock().now().nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        names = ' → '.join(g[0] for g in self._goals)
        print(f'WAYPOINT VERSION = {WAYPOINT_VERSION}')
        print(
            'B_arc_east=(8.75,1.35,0)  B_arc_north=(8.75,1.75,+π/2)  '
            'B_corridor_turn_spin=+π/2  '
            f'origin=({self._ox:.4f},{self._oy:.4f})'
        )
        print(f'PATROL: route {names}')
        self.get_logger().info(
            f'{WAYPOINT_VERSION} ready ({len(self._goals)} goals, map={map_yaml})'
        )
        self.create_timer(0.5, self._kickoff_once)

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._amcl = msg

    def _kickoff_once(self) -> None:
        if self._started or self._done:
            return
        self._started = True
        self._prepare_and_send(reanchor=False)

    def _clear_costmaps_sync(self) -> None:
        req = ClearEntireCostmap.Request()
        futures: list[tuple[str, object]] = []
        for name, client in (
            ('global', self._clear_global),
            ('local', self._clear_local),
        ):
            if client.wait_for_service(timeout_sec=1.0):
                futures.append((name, client.call_async(req)))
            else:
                self.get_logger().warn(f'clear {name} costmap service not ready')

        deadline = time.monotonic() + CLEAR_TIMEOUT_SEC
        while futures and time.monotonic() < deadline and rclpy.ok():
            pending: list[tuple[str, object]] = []
            for name, fut in futures:
                if fut.done():  # type: ignore[attr-defined]
                    try:
                        fut.result()  # type: ignore[attr-defined]
                    except Exception as exc:  # noqa: BLE001
                        self.get_logger().warn(f'clear {name} failed: {exc}')
                else:
                    pending.append((name, fut))
            futures = pending
            if futures:
                rclpy.spin_once(self, timeout_sec=0.05)

        if futures:
            self.get_logger().warn(
                f'clear costmap timeout ({CLEAR_TIMEOUT_SEC:.1f}s), continue anyway'
            )

    def _soft_reanchor(self) -> None:
        if not ENABLE_SOFT_REANCHOR:
            return
        if self._amcl is None:
            self.get_logger().warn('soft reanchor skipped: no /amcl_pose yet')
            return

        p = self._amcl.pose.pose.position
        q = self._amcl.pose.pose.orientation
        cov = list(self._amcl.pose.covariance)
        vals = (p.x, p.y, p.z, q.x, q.y, q.z, q.w, cov[0], cov[7], cov[35])
        if not all(math.isfinite(v) for v in vals):
            self.get_logger().error('soft reanchor BLOCKED: non-finite amcl_pose')
            print('PATROL: soft reanchor BLOCKED (non-finite)')
            return

        qn = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if qn < 1e-6 or abs(qn - 1.0) > 0.25:
            self.get_logger().error(f'soft reanchor BLOCKED: bad quat norm={qn:.4f}')
            print('PATROL: soft reanchor BLOCKED (quat)')
            return

        if cov[0] > SOFT_REANCHOR_MAX_XY_VAR or cov[7] > SOFT_REANCHOR_MAX_XY_VAR:
            self.get_logger().warn(
                f'soft reanchor skipped: xy var too large ({cov[0]:.3f},{cov[7]:.3f})'
            )
            print('PATROL: soft reanchor skipped (cov xy)')
            return
        if cov[35] > SOFT_REANCHOR_MAX_YAW_VAR:
            self.get_logger().warn(
                f'soft reanchor skipped: yaw var too large ({cov[35]:.3f})'
            )
            print('PATROL: soft reanchor skipped (cov yaw)')
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = p.x
        msg.pose.pose.position.y = p.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = q.x / qn
        msg.pose.pose.orientation.y = q.y / qn
        msg.pose.pose.orientation.z = q.z / qn
        msg.pose.pose.orientation.w = q.w / qn
        msg.pose.covariance = cov
        self._initialpose_pub.publish(msg)
        print('PATROL: soft reanchor from /amcl_pose')
        end = time.monotonic() + 0.8
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def _prepare_and_send(self, reanchor: bool = False) -> None:
        if self._idx >= len(self._goals):
            self._print_seg_summary()
            print(f'PATROL: FINAL PASS ({WAYPOINT_VERSION})')
            self._exit_code = 0
            self._done = True
            return

        self._clear_costmaps_sync()
        if reanchor:
            self._soft_reanchor()
        self._send_segment()

    def _send_segment(self) -> None:
        name, action, wx, wy, yaw_or_delta, kind = self._goals[self._idx]
        self._current_name = name
        self._current_action = action
        self._current_kind = kind
        self._recoveries = 0
        self._seg_t0 = time.monotonic()
        self._attempt += 1

        if action == 'spin':
            self._send_spin(name, yaw_or_delta, kind)
        elif action == 'navigate':
            self._send_navigate(name, wx, wy, yaw_or_delta, kind)
        else:
            self._fail(f'unknown action={action} for {name}')

    def _send_navigate(
        self, name: str, wx: float, wy: float, yaw: float, kind: str,
    ) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=15.0):
            self._fail('Action server /navigate_to_pose 不可用')
            return

        mx, my = world_to_map_xy(wx, wy, self._ox, self._oy)
        quat = yaw_to_quat(yaw)
        print(
            f'[WAYPOINT] name={name} action=navigate kind={kind} '
            f'world=({wx:.3f},{wy:.3f}) map=({mx:.3f},{my:.3f}) yaw={yaw:.4f}'
        )
        print(
            f'PATROL: sending {name} navigate '
            f'attempt={self._attempt}/{MAX_ATTEMPTS_PER_WP}'
        )

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = mx
        goal.pose.pose.position.y = my
        goal.pose.pose.orientation.x = quat[0]
        goal.pose.pose.orientation.y = quat[1]
        goal.pose.pose.orientation.z = quat[2]
        goal.pose.pose.orientation.w = quat[3]

        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
        self._timeout_timer = self.create_timer(GOAL_TIMEOUT_SEC, self._on_timeout)
        fut = self._nav_client.send_goal_async(goal, feedback_callback=self._on_nav_feedback)
        fut.add_done_callback(self._on_goal_response)

    def _recreate_spin_client(self) -> None:
        try:
            self._spin_client.destroy()
        except Exception:  # noqa: BLE001
            pass
        self._spin_client = ActionClient(self, Spin, 'spin')

    def _send_spin(self, name: str, delta_yaw: float, kind: str) -> None:
        """显式 /spin：accept≤10s，超时销毁 client 后只重试 1 次；已 accepted 只等 result。"""
        print(
            f'[WAYPOINT] name={name} action=spin kind={kind} '
            f'target_yaw={delta_yaw:.4f} (relative)'
        )
        print(f'PATROL: sending {name} spin (accept-retry; no NavigateToPose fallback)')

        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None

        handle = None
        retry_index = 0
        for attempt in range(SPIN_MAX_ACCEPT_RETRIES + 1):
            retry_index = attempt
            if attempt > 0:
                print(
                    f'[SPIN_DIAG] name={name} ACCEPT_TIMEOUT → recreate client; '
                    f'sleep {SPIN_ACCEPT_RETRY_SLEEP_SEC:.0f}s retry_index={attempt}'
                )
                self._recreate_spin_client()
                end = time.monotonic() + SPIN_ACCEPT_RETRY_SLEEP_SEC
                while time.monotonic() < end and rclpy.ok() and not self._done:
                    rclpy.spin_once(self, timeout_sec=0.05)

            if not self._spin_client.wait_for_server(timeout_sec=15.0):
                self._fail(f'{name} Action server /spin 不可用')
                return
            print(f'[SPIN_DIAG] name={name} server_ready=1 retry_index={retry_index}')

            goal = Spin.Goal()
            goal.target_yaw = float(delta_yaw)
            goal.time_allowance = Duration(sec=int(SPIN_TIMEOUT_SEC), nanosec=0)
            send_fut = self._spin_client.send_goal_async(goal)
            print(f'[SPIN_DIAG] name={name} goal_sent=1 retry_index={retry_index}')

            t0 = time.monotonic()
            while rclpy.ok() and not self._done and (time.monotonic() - t0) < SPIN_ACCEPT_TIMEOUT_SEC:
                rclpy.spin_once(self, timeout_sec=0.05)
                if send_fut.done():
                    break
            else:
                print(
                    f'[SPIN_DIAG] name={name} goal_accept_timeout=1 '
                    f'retry_index={retry_index}'
                )
                if attempt >= SPIN_MAX_ACCEPT_RETRIES:
                    self._log_segment('ACCEPT_TIMEOUT')
                    self._print_seg_summary()
                    self._fail(f'{name} Spin ACCEPT_TIMEOUT after retry')
                    return
                continue

            try:
                handle = send_fut.result()
            except Exception as exc:  # noqa: BLE001
                print(f'[SPIN_DIAG] name={name} accept exception={exc}')
                if attempt >= SPIN_MAX_ACCEPT_RETRIES:
                    self._fail(f'{name} Spin accept failed: {exc}')
                    return
                continue

            if handle is None or not handle.accepted:
                print(f'[SPIN_DIAG] name={name} goal_accepted=0 retry_index={retry_index}')
                if attempt >= SPIN_MAX_ACCEPT_RETRIES:
                    self._fail(f'{name} Spin goal rejected')
                    return
                continue

            print(
                f'[SPIN_DIAG] name={name} goal_accepted=1 retry_index={retry_index} '
                f'(wait result only; no resend)'
            )
            break

        if handle is None:
            self._fail(f'{name} Spin no accepted goal')
            return

        self._goal_handle = handle
        self._timeout_timer = self.create_timer(SPIN_TIMEOUT_SEC + 5.0, self._on_timeout)
        result_fut = handle.get_result_async()
        result_fut.add_done_callback(self._on_result)

    def _on_nav_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        if hasattr(fb, 'number_of_recoveries'):
            self._recoveries = fb.number_of_recoveries

    def _on_goal_response(self, future) -> None:
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self._fail(f'Goal {self._current_name} rejected')
            return
        result_fut = self._goal_handle.get_result_async()
        result_fut.add_done_callback(self._on_result)

    def _log_segment(self, status: str) -> None:
        dt = time.monotonic() - self._seg_t0
        line = (
            f'PATROL_SEG: [{self._current_name}] {status} '
            f'action={self._current_action} kind={self._current_kind} '
            f'recoveries={self._recoveries} duration={dt:.1f}s'
        )
        print(line)
        self._seg_log.append(line)

    def _print_seg_summary(self) -> None:
        print('PATROL_SEG_SUMMARY:')
        for line in self._seg_log:
            print(f'  {line}')

    def _on_result(self, future) -> None:
        if self._done:
            return
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None
        wrapped = future.result()
        status = wrapped.status
        name = self._current_name
        action = self._current_action

        if status != GoalStatus.STATUS_SUCCEEDED:
            self._log_segment(status_label(status))
            # spin：失败立即停，不包装成 NavigateToPose 重试
            if action == 'spin':
                self._print_seg_summary()
                self._fail(f'{name} Spin {status_label(status)}')
                return
            if self._attempt < MAX_ATTEMPTS_PER_WP:
                print(
                    f'PATROL: {name} {status_label(status)} — '
                    f'retry after clear (no soft-reanchor) '
                    f'(attempt {self._attempt}/{MAX_ATTEMPTS_PER_WP})'
                )
                if self._dwell_timer is not None:
                    self._dwell_timer.cancel()
                self._dwell_timer = self.create_timer(
                    RETRY_DWELL_SEC, self._after_retry_dwell,
                )
                return
            self._print_seg_summary()
            self._fail(f'{name} NavigateToPose {status_label(status)}')
            return

        self._log_segment('SUCCESS')
        print(f'PATROL: {name} SUCCEEDED (action={action} recoveries={self._recoveries})')
        if action == 'spin':
            # 收尾等待，避免下一 action 踩未清理状态
            end = time.monotonic() + SPIN_POST_SUCCESS_SLEEP_SEC
            while time.monotonic() < end and rclpy.ok() and not self._done:
                rclpy.spin_once(self, timeout_sec=0.05)
        self._idx += 1
        self._attempt = 0
        dwell = INTER_GOAL_DWELL_SEC
        if self._idx < len(self._goals):
            nxt = self._goals[self._idx]
            if nxt[0] == 'C':
                dwell = BEFORE_INSPECT_C_DWELL_SEC
            elif nxt[1] == 'spin':
                dwell = max(dwell, BEFORE_SPIN_EXTRA_DWELL_SEC)
        if self._dwell_timer is not None:
            self._dwell_timer.cancel()
        self._dwell_timer = self.create_timer(dwell, self._after_dwell)

    def _after_retry_dwell(self) -> None:
        if self._dwell_timer is not None:
            self._dwell_timer.cancel()
            self._dwell_timer = None
        if self._done:
            return
        self._prepare_and_send(reanchor=False)

    def _after_dwell(self) -> None:
        if self._dwell_timer is not None:
            self._dwell_timer.cancel()
            self._dwell_timer = None
        if self._done:
            return
        self._prepare_and_send(reanchor=ENABLE_SOFT_REANCHOR)

    def _on_timeout(self) -> None:
        if self._done:
            return
        self._log_segment('TIMEOUT')
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._print_seg_summary()
        self._fail(f'{self._current_name} timeout')

    def _fail(self, msg: str) -> None:
        print(f'PATROL: FAIL — {msg}')
        self.get_logger().error(msg)
        self._exit_code = 1
        self._done = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolMissionNode()
    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('PATROL: interrupted')
        node._exit_code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(node._exit_code)


if __name__ == '__main__':
    main()
