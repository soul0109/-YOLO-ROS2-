#!/usr/bin/env python3
"""Run inspection stops, delegating continuous transfers to Nav2.

Simulation diagnostics never feed ground truth back into localization/control.
An action success and a station acceptance are reported separately.
"""

import json
import math
from pathlib import Path
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import yaml

from inspection_mission.route_diagnostics import RouteDiagnostics


class PatrolMissionNode(Node):
    def __init__(self):
        super().__init__('patrol_mission_node', parameter_overrides=[
            Parameter('use_sim_time', value=True),
        ])
        share = Path(get_package_share_directory('inspection_mission'))
        nav = Path(get_package_share_directory('navigation_config'))
        self.declare_parameter('route_file', str(share / 'config/patrol_route.yaml'))
        # Keep acceptance measurements on by default; they never feed back
        # into Nav2 and do not stop a route on covariance warnings.
        self.declare_parameter('diagnostic_mode', True)
        self.declare_parameter('output_dir', '')
        self.declare_parameter('goal_timeout_sec', 300.0)
        self.route = yaml.safe_load(Path(self.get_parameter('route_file').value).read_text())
        self.stations = self.route['stations']
        self.order = self.route['inspection_order']
        if not self.order or any(name not in self.stations for name in self.order):
            raise ValueError('inspection_order must reference configured stations')
        for pose in list(self.stations.values()) + [
            p for route in self.route['transfers'].values() for p in route['via']
        ]:
            if len(pose) != 3 or not all(math.isfinite(float(v)) for v in pose):
                raise ValueError(f'invalid route pose: {pose}')
        # Preserve the existing test_room calibration; origin is not a general
        # SLAM map-to-Gazebo transform. Recalibrate when replacing the map.
        self.origin = yaml.safe_load((nav / 'maps/test_room.yaml').read_text())['origin']
        self.diagnostic_mode = self.get_parameter('diagnostic_mode').value
        self.output = self.get_parameter('output_dir').value
        self.timeout = float(self.get_parameter('goal_timeout_sec').value)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.through_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.amcl = None
        self.health_last = 0.0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, qos)
        self.diagnostics = RouteDiagnostics(self, self.origin, self.output) if self.diagnostic_mode else None
        self.goal_handle = None
        self.pending_goal = None
        self.records = []
        self.current = ''
        self.recoveries = 0
        self.feedback_key = None

    def on_amcl(self, msg):
        self.amcl = msg
        cov = [float(msg.pose.covariance[i]) for i in (0, 7, 35)]
        valid = all(math.isfinite(v) and v >= 0 for v in cov)
        if not valid or cov[0] > 0.25 or cov[1] > 0.25 or cov[2] > 0.20:
            if time.monotonic() - self.health_last > 2.0:
                self.get_logger().warn(f'PATROL_LOC: {self.current} WARN covariance={cov}')
                self.health_last = time.monotonic()

    def pose(self, values):
        x, y, yaw = values
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x - self.origin[0])
        msg.pose.position.y = float(y - self.origin[1])
        msg.pose.orientation.z = math.sin((yaw - self.origin[2]) / 2)
        msg.pose.orientation.w = math.cos((yaw - self.origin[2]) / 2)
        return msg

    def feedback(self, msg):
        fb = msg.feedback
        self.recoveries = int(fb.number_of_recoveries)
        key = (getattr(fb, 'number_of_poses_remaining', 1), self.recoveries)
        if key != self.feedback_key:
            print(f'PATROL_FEEDBACK: {self.current} remaining={key[0]} recoveries={key[1]}', flush=True)
            self.feedback_key = key

    def wait(self, future, seconds, monitor=False):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if monitor and self.diagnostics:
                # fault() hard-stops only on control dependency failures.
                # Acceptance gaps / localization warnings are recorded, not raised.
                fault = self.diagnostics.fault()
                if fault:
                    raise RuntimeError(fault)
        if not future.done():
            raise TimeoutError(f'{self.current}: response timeout ({seconds}s wall clock)')
        return future.result()

    def cancel(self):
        # A delayed acceptance can arrive after a timeout. Drain it before
        # destroying the client, then wait for cancellation acknowledgement.
        if self.goal_handle is None and self.pending_goal is not None:
            try:
                self.goal_handle = self.wait(self.pending_goal, 10)
            except Exception as exc:
                self.get_logger().error(f'Cannot confirm goal acceptance: {exc}')
        if self.goal_handle is not None and self.goal_handle.accepted:
            try:
                self.wait(self.goal_handle.cancel_goal_async(), 5)
                self.wait(self.goal_handle.get_result_async(), 5)
            except Exception as exc:
                self.get_logger().error(f'Cannot confirm action cancellation: {exc}')
        self.goal_handle = None
        self.pending_goal = None

    def navigate(self, name, via):
        self.current = name
        self.recoveries = 0
        self.feedback_key = None
        self.goal_handle = None
        self.pending_goal = None
        self.records.append({'station': name, 'action_success': False})
        record = self.records[-1]
        if via:
            client = self.through_client
            goal = NavigateThroughPoses.Goal()
            goal.poses = [self.pose(p) for p in via + [self.stations[name]]]
            action = 'NavigateThroughPoses'
        else:
            client = self.client
            goal = NavigateToPose.Goal()
            goal.pose = self.pose(self.stations[name])
            action = 'NavigateToPose'
        record['action'] = action
        if not client.wait_for_server(timeout_sec=15):
            raise RuntimeError(f'{action} server unavailable')
        print(f'PATROL: sending {name} action={action} via={len(via)}', flush=True)
        start = time.monotonic()
        self.pending_goal = client.send_goal_async(goal, feedback_callback=self.feedback)
        self.goal_handle = self.wait(self.pending_goal, 15)
        if not self.goal_handle.accepted:
            raise RuntimeError(f'{name} rejected')
        wrapped = self.wait(self.goal_handle.get_result_async(), self.timeout, monitor=True)
        self.goal_handle = None
        self.pending_goal = None
        record.update(status=int(wrapped.status), recoveries=self.recoveries,
                      duration_sec=round(time.monotonic() - start, 2))
        print(f'PATROL_SEG: [{name}] status={wrapped.status} recoveries={self.recoveries}', flush=True)
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'{name} action failed, status={wrapped.status}')
        record['action_success'] = True
        if self.diagnostics:
            self.diagnostics.begin_stop_window()
            record['gate'] = self.diagnostics.station_gate(self.stations[name])
            print(f'PATROL_GATE: {name} {json.dumps(record["gate"])}', flush=True)
            # Failed station accuracy remains a failed trial, but diagnostic
            # mode continues to observe downstream behavior when it is safe.
        else:
            end = time.monotonic() + 1.0
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        error = ''
        try:
            deadline = time.monotonic() + 30
            while rclpy.ok() and (self.amcl is None or self.get_clock().now().nanoseconds == 0):
                if time.monotonic() > deadline:
                    raise TimeoutError('No simulation clock or AMCL; publish initialpose first')
                rclpy.spin_once(self, timeout_sec=0.1)
            if self.diagnostics:
                self.diagnostics.wait_ready()
            previous = None
            for name in self.order:
                transfer = self.route['transfers'].get(f'{previous}_to_{name}', {})
                self.navigate(name, transfer.get('via', []))
                previous = name
        except (Exception, KeyboardInterrupt) as exc:
            error = str(exc) or 'interrupted'
            self.cancel()
            self.get_logger().error(error)
        actions_ok = len(self.records) == len(self.order) and all(
            r['action_success'] for r in self.records) and not error
        acceptance_blocked = bool(
            self.diagnostics and self.diagnostics.acceptance_blocked
        )
        accepted = (
            actions_ok
            and self.diagnostic_mode
            and not acceptance_blocked
            and all(r.get('gate', {}).get('pass', False) for r in self.records)
        )
        summary = dict(
            version=self.route['version'],
            action_success=actions_ok,
            acceptance=bool(accepted) if self.diagnostic_mode else None,
            acceptance_blocked=acceptance_blocked if self.diagnostic_mode else None,
            acceptance_block_reason=(
                self.diagnostics.acceptance_block_reason if self.diagnostics else None
            ),
            error=error,
            initial_diagnostics=(
                self.diagnostics.ready_state if self.diagnostics else None
            ),
            health_events=(
                len(self.diagnostics.health_log) if self.diagnostics else 0
            ),
            stations=self.records,
        )
        if self.output:
            Path(self.output).mkdir(parents=True, exist_ok=True)
            (Path(self.output) / 'result.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(f'PATROL_RESULT: {json.dumps(summary)}', flush=True)
        return 0 if (accepted if self.diagnostic_mode else actions_ok) else 1

    def destroy_node(self):
        if self.diagnostics:
            self.diagnostics.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PatrolMissionNode()
        return node.run()
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
