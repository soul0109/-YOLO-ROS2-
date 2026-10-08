#!/usr/bin/env python3
"""4.5b：三航点单圈 NavigateToPose（A→B→C，无人工干预）。

顺序对齐《巡检场景规格》充电→A→B→C。
航点间 dwell + 同步清 costmap；失败同点最多 3 次；
发下一航点（及重试）前用当前 /amcl_pose 软重锚定，压低 B→C 走廊粒子发散。

前置：nav2_test_room.launch + publish_amcl_initial_pose 已跑通。

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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# 冻结 world 航点（docs/阶段4.5-Nav2说明.md §4.4）；顺序 A→B→C（规格）
WAYPOINTS = (
    ('A', 3.15, 1.75, -math.pi / 2.0),
    ('B', 8.45, 1.75, -math.pi / 2.0),
    ('C', 5.25, 4.25, math.pi / 2.0),
)

GOAL_TIMEOUT_SEC = 300.0
INTER_GOAL_DWELL_SEC = 2.0
# B→C 是压力测试唯一易挂腿：多等一会儿再发 C
BEFORE_C_DWELL_SEC = 3.5
RETRY_DWELL_SEC = 2.5
MAX_ATTEMPTS_PER_WP = 3  # 首次 + 2 次重试
CLEAR_TIMEOUT_SEC = 3.0
# 2026-10-08：stress run5 在 C ABORT 后 soft reanchor，随后观测到 map→odom TF NaN。
# 默认关闭；仅成功航点切换且 pose 有限、协方差未爆时才允许（见 _soft_reanchor）。
ENABLE_SOFT_REANCHOR = False
# 软锚定协方差上限（超过则跳过，避免把「不确定/坏估计」钉死）
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
        ox, oy, _ = load_map_origin(map_yaml)

        self._goals: list[tuple[str, float, float, tuple[float, float, float, float]]] = []
        for name, wx, wy, yaw in WAYPOINTS:
            mx, my = world_to_map_xy(wx, wy, ox, oy)
            self._goals.append((name, mx, my, yaw_to_quat(yaw)))

        self._idx = 0
        self._attempt = 0
        self._done = False
        self._exit_code = 1
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
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
        self._recoveries = 0
        self._started = False

        clock_deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=20.0)
        while rclpy.ok() and self.get_clock().now() < clock_deadline:
            if self.get_clock().now().nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(
            f'Patrol A→B→C ready ({len(self._goals)} goals, map={map_yaml})'
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
        """等清图完成再发 goal（异步 fire-and-forget 是时序坑）。"""
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
        """用当前 AMCL 估计重发 /initialpose（默认关闭；禁止在 ABORT 后调用）。"""
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
        # 归一化四元数，不人为压小协方差（避免虚假自信）
        msg.pose.pose.orientation.x = q.x / qn
        msg.pose.pose.orientation.y = q.y / qn
        msg.pose.pose.orientation.z = q.z / qn
        msg.pose.pose.orientation.w = q.w / qn
        msg.pose.covariance = cov
        self._initialpose_pub.publish(msg)
        print('PATROL: soft reanchor from /amcl_pose')
        self.get_logger().info('Soft reanchor published to /initialpose')
        end = time.monotonic() + 0.8
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def _prepare_and_send(self, reanchor: bool = False) -> None:
        if self._idx >= len(self._goals):
            print('PATROL: FINAL PASS (A→B→C)')
            self._exit_code = 0
            self._done = True
            return

        self._clear_costmaps_sync()
        if reanchor:
            self._soft_reanchor()
        self._send_goal()

    def _send_goal(self) -> None:
        if not self._client.wait_for_server(timeout_sec=15.0):
            self._fail('Action server /navigate_to_pose 不可用')
            return

        name, mx, my, quat = self._goals[self._idx]
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

        self._attempt += 1
        print(
            f'PATROL: sending goal {name} '
            f'map=({mx:.3f}, {my:.3f}) attempt={self._attempt}/{MAX_ATTEMPTS_PER_WP}'
        )
        self.get_logger().info(f'Sending waypoint {name} attempt={self._attempt}')
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
        self._timeout_timer = self.create_timer(GOAL_TIMEOUT_SEC, self._on_timeout)
        fut = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        fut.add_done_callback(self._on_goal_response)
        self._current_name = name
        self._recoveries = 0

    def _on_feedback(self, feedback_msg) -> None:
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

    def _on_result(self, future) -> None:
        if self._done:
            return
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None
        wrapped = future.result()
        status = wrapped.status
        name = self._current_name
        if status != GoalStatus.STATUS_SUCCEEDED:
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
            self._fail(f'{name} NavigateToPose {status_label(status)}')
            return

        print(f'PATROL: {name} SUCCEEDED (recoveries={self._recoveries})')
        self._idx += 1
        self._attempt = 0
        dwell = INTER_GOAL_DWELL_SEC
        # 下一航点是 C 时加长沉降
        if self._idx < len(self._goals) and self._goals[self._idx][0] == 'C':
            dwell = BEFORE_C_DWELL_SEC
        if self._dwell_timer is not None:
            self._dwell_timer.cancel()
        self._dwell_timer = self.create_timer(dwell, self._after_dwell)

    def _after_retry_dwell(self) -> None:
        if self._dwell_timer is not None:
            self._dwell_timer.cancel()
            self._dwell_timer = None
        if self._done:
            return
        # ABORT 后禁止 soft reanchor：坏估计写回可能放大成 map→odom NaN
        self._prepare_and_send(reanchor=False)

    def _after_dwell(self) -> None:
        if self._dwell_timer is not None:
            self._dwell_timer.cancel()
            self._dwell_timer = None
        if self._done:
            return
        # 仅成功切换航点时可选 soft reanchor（默认 ENABLE_SOFT_REANCHOR=False）
        self._prepare_and_send(reanchor=ENABLE_SOFT_REANCHOR)

    def _on_timeout(self) -> None:
        if self._done:
            return
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._fail(f'{self._current_name} timeout after {GOAL_TIMEOUT_SEC:.0f}s')

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
