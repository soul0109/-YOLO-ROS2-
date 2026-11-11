"""NavigateToPose / NavigateThroughPoses ActionClient wrapper for Mission Runtime."""

from __future__ import annotations

from typing import Callable, Optional, Protocol, Sequence

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node


class NavAdapterProtocol(Protocol):
    """Testable surface used by MissionRuntime / MissionRunner."""

    def wait_for_server(
        self, timeout_sec: float, *, through_poses: bool = False,
    ) -> bool: ...

    def send_goal_async(
        self,
        pose: PoseStamped,
        feedback_callback: Optional[Callable] = None,
    ): ...

    def send_through_poses_async(
        self,
        poses: Sequence[PoseStamped],
        feedback_callback: Optional[Callable] = None,
    ): ...

    def cancel_goal_async(self, goal_handle: ClientGoalHandle): ...

    def get_result_async(self, goal_handle: ClientGoalHandle): ...


class NavAdapter:
    """Facade over NavigateToPose and NavigateThroughPoses (patrol baseline)."""

    def __init__(
        self,
        node: Node,
        *,
        to_pose_action: str = 'navigate_to_pose',
        through_poses_action: str = 'navigate_through_poses',
    ) -> None:
        self._node = node
        self._to_pose = ActionClient(node, NavigateToPose, to_pose_action)
        self._through = ActionClient(node, NavigateThroughPoses, through_poses_action)

    def wait_for_server(
        self, timeout_sec: float, *, through_poses: bool = False,
    ) -> bool:
        client = self._through if through_poses else self._to_pose
        return bool(client.wait_for_server(timeout_sec=timeout_sec))

    def send_goal_async(
        self,
        pose: PoseStamped,
        feedback_callback: Optional[Callable] = None,
    ):
        goal = NavigateToPose.Goal()
        goal.pose = pose
        return self._to_pose.send_goal_async(
            goal, feedback_callback=feedback_callback,
        )

    def send_through_poses_async(
        self,
        poses: Sequence[PoseStamped],
        feedback_callback: Optional[Callable] = None,
    ):
        if not poses:
            raise ValueError('NavigateThroughPoses requires a non-empty pose list')
        goal = NavigateThroughPoses.Goal()
        goal.poses = list(poses)
        return self._through.send_goal_async(
            goal, feedback_callback=feedback_callback,
        )

    def cancel_goal_async(self, goal_handle: ClientGoalHandle):
        return goal_handle.cancel_goal_async()

    def get_result_async(self, goal_handle: ClientGoalHandle):
        return goal_handle.get_result_async()
