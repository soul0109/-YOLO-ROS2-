#!/usr/bin/env python3
"""4.5b：三航点单圈 NavigateToPose（A→B→C，无人工干预）。

顺序对齐《巡检场景规格》充电→A→B→C。航点间 2 s dwell + clear costmap + 同点 1 次重试。

前置：nav2_test_room.launch + publish_amcl_initial_pose 已跑通。

  ros2 run inspection_mission patrol_mission_node
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

# 冻结 world 航点（docs/阶段4.5-Nav2说明.md §4.4）；顺序 A→B→C（规格）
WAYPOINTS = (
    ('A', 3.15, 1.75, -math.pi / 2.0),
    ('B', 8.45, 1.75, -math.pi / 2.0),
    ('C', 5.25, 4.25, math.pi / 2.0),
)

GOAL_TIMEOUT_SEC = 300.0
INTER_GOAL_DWELL_SEC = 2.0
MAX_ATTEMPTS_PER_WP = 2  # 首次 + 1 次重试


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

    def _kickoff_once(self) -> None:
        if self._started or self._done:
            return
        self._started = True
        self._prepare_and_send()

    def _clear_costmaps(self) -> None:
        req = ClearEntireCostmap.Request()
        for name, client in (
            ('global', self._clear_global),
            ('local', self._clear_local),
        ):
            if client.service_is_ready():
                client.call_async(req)
            else:
                self.get_logger().warn(f'clear {name} costmap service not ready')

    def _prepare_and_send(self) -> None:
        if self._idx >= len(self._goals):
            print('PATROL: FINAL PASS (A→B→C)')
            self._exit_code = 0
            self._done = True
            return

        self._clear_costmaps()
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
                    f'retry after clear (attempt {self._attempt}/{MAX_ATTEMPTS_PER_WP})'
                )
                self._prepare_and_send()
                return
            self._fail(f'{name} NavigateToPose {status_label(status)}')
            return

        print(f'PATROL: {name} SUCCEEDED (recoveries={self._recoveries})')
        self._idx += 1
        self._attempt = 0
        if self._dwell_timer is not None:
            self._dwell_timer.cancel()
        self._dwell_timer = self.create_timer(INTER_GOAL_DWELL_SEC, self._after_dwell)

    def _after_dwell(self) -> None:
        if self._dwell_timer is not None:
            self._dwell_timer.cancel()
            self._dwell_timer = None
        if self._done:
            return
        self._prepare_and_send()

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
