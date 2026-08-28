#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
巡检任务 Action 服务节点（阶段 2 demo）

功能：
    提供 action /patrol_task（类型 inspection_interfaces/action/PatrolTask）。
    收到航点列表后，模拟“逐个前往航点”的过程，期间持续发送 feedback，
    全部完成后返回 result。

学习要点：
    - Action = Goal（目标）+ Feedback（进度）+ Result（结果）
    - 适合耗时任务：导航、巡检、机械臂运动等
    - 客户端可在执行过程中取消任务

与后续阶段关系：
    阶段 6 的 patrol_mission_node 会调用 Nav2 真导航，
    但仍使用同一个 PatrolTask.action 接口。
"""

from __future__ import annotations

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from inspection_interfaces.action import PatrolTask


class PatrolActionServer(Node):
    """PatrolTask Action 的 Server 端（模拟版）。"""

    def __init__(self) -> None:
        super().__init__('patrol_action_server')

        # 模拟到达每个航点所需时间（秒）
        self.declare_parameter('step_duration_sec', 2.0)
        self._step_duration = float(self.get_parameter('step_duration_sec').value)

        self._action_server = ActionServer(
            self,
            PatrolTask,
            '/patrol_task',
            self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self.get_logger().info('巡检 Action 服务已启动 | action 名: /patrol_task')

    def _goal_callback(self, goal_request: PatrolTask.Goal) -> GoalResponse:
        """收到新目标时调用；可在此拒绝非法目标。demo 一律接受。"""
        waypoint_count = len(goal_request.waypoints)
        self.get_logger().info(
            f'收到巡检目标 | task_label={goal_request.task_label!r} | '
            f'航点数={waypoint_count} | snapshot_on_arrive={goal_request.snapshot_on_arrive}'
        )
        if waypoint_count == 0:
            self.get_logger().warn('航点列表为空，仍接受目标（将立即完成）')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        """客户端请求取消时调用。"""
        self.get_logger().info('收到取消巡检请求')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        """
        执行巡检逻辑（模拟）。

        流程：遍历航点 → 每个航点 sleep 一段时间 → 发 feedback → 最后 succeed + 返回 result。
        """
        goal = goal_handle.request
        waypoints = goal.waypoints
        total = max(len(waypoints), 1)  # 避免除零；0 航点时 total=1 用于进度计算

        feedback = PatrolTask.Feedback()
        visited = 0

        for index, waypoint in enumerate(waypoints):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('巡检已取消')
                result = PatrolTask.Result()
                result.success = False
                result.visited_count = visited
                result.anomaly_count = 0
                result.report_path = ''
                result.message = '任务被用户取消'
                return result

            label = waypoint.header.frame_id or f'waypoint_{index + 1}'
            self.get_logger().info(f'正在前往航点 {index + 1}/{len(waypoints)}: {label}')

            # 模拟导航阶段
            feedback.current_index = index
            feedback.total_waypoints = len(waypoints)
            feedback.phase = 'navigating'
            feedback.progress = float(index) / float(total)
            feedback.current_label = label
            goal_handle.publish_feedback(feedback)

            time.sleep(self._step_duration)

            visited += 1

            # 若目标要求到达后拍照，模拟 snapshot 阶段
            if goal.snapshot_on_arrive:
                feedback.phase = 'snapshot'
                feedback.progress = (float(index) + 0.5) / float(total)
                goal_handle.publish_feedback(feedback)
                self.get_logger().info(f'航点 {label}：模拟拍照/检测')
                time.sleep(self._step_duration * 0.5)

        # 全部航点走完
        feedback.current_index = len(waypoints)
        feedback.total_waypoints = len(waypoints)
        feedback.phase = 'idle'
        feedback.progress = 1.0
        feedback.current_label = 'done'
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()

        result = PatrolTask.Result()
        result.success = True
        result.visited_count = visited
        result.anomaly_count = 0
        result.report_path = '/tmp/inspection_demo_report.json'
        result.message = f'模拟巡检完成，共访问 {visited} 个航点'

        self.get_logger().info(result.message)
        return result


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PatrolActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到 Ctrl+C，Action 服务节点退出')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
