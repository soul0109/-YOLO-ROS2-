#!/usr/bin/env python3
"""Confirm Nav2 goals are cancelled, then quiet cmd_vel before reset.

Publishing zero velocity alone does not prove the controller stopped.
This script:
  1) cancels NavigateToPose / NavigateThroughPoses (all goals)
  2) requires action status to have been observed, then no ACTIVE goals
  3) publishes zero /cmd_vel and requires real receipts on gated layers

It records evidence; it does not reseed AMCL from ground truth.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from unique_identifier_msgs.msg import UUID


ACTIVE_STATUSES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}

# action_msgs/srv/CancelGoal return codes
CANCEL_ERROR_NONE = 0
CANCEL_ERROR_REJECTED = 1
CANCEL_ERROR_UNKNOWN_GOAL_ID = 2
CANCEL_ERROR_GOAL_TERMINATED = 3
CANCEL_OK_CODES = {CANCEL_ERROR_NONE, CANCEL_ERROR_GOAL_TERMINATED}


def twist_norm(msg: Twist | None) -> tuple[float, float]:
    if msg is None:
        return 0.0, 0.0
    return abs(float(msg.linear.x)), abs(float(msg.angular.z))


class ClearNavControl(Node):
    def __init__(self, quiet_linear: float, quiet_angular: float) -> None:
        super().__init__(
            'clear_nav_control',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.quiet_linear = quiet_linear
        self.quiet_angular = quiet_angular
        self.status: dict[str, GoalStatusArray | None] = {
            'navigate_to_pose': None,
            'navigate_through_poses': None,
        }
        self.status_seen = {name: False for name in self.status}
        self.cmd: dict[str, Twist | None] = {
            '/cmd_vel': None,
            '/cmd_vel_gazebo': None,
            '/cmd_vel_nav': None,
        }
        self.cmd_counts = {name: 0 for name in self.cmd}

        for action in self.status:
            self.create_subscription(
                GoalStatusArray,
                f'/{action}/_action/status',
                lambda msg, key=action: self._on_status(key, msg),
                10,
            )
        for topic in self.cmd:
            self.create_subscription(
                Twist, topic, lambda msg, key=topic: self._on_cmd(key, msg),
                qos_profile_system_default if topic == '/cmd_vel' else qos_profile_sensor_data,
            )
        self.zero_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile_system_default)

    def _on_status(self, key: str, msg: GoalStatusArray) -> None:
        self.status[key] = msg
        self.status_seen[key] = True

    def _on_cmd(self, key: str, msg: Twist) -> None:
        self.cmd[key] = msg
        self.cmd_counts[key] += 1

    def active_goals(self) -> dict[str, list[dict]]:
        result: dict[str, list[dict]] = {}
        for name, array in self.status.items():
            items = []
            if array is not None:
                for entry in array.status_list:
                    st = int(entry.status)
                    if st in ACTIVE_STATUSES:
                        items.append({
                            'status': st,
                            'goal_id': list(entry.goal_info.goal_id.uuid),
                        })
            result[name] = items
        return result

    def cancel_all(self, action: str, timeout_sec: float) -> dict:
        service = f'/{action}/_action/cancel_goal'
        client = self.create_client(CancelGoal, service)
        waited = time.monotonic()
        while time.monotonic() - waited < timeout_sec and not client.wait_for_service(timeout_sec=0.2):
            rclpy.spin_once(self, timeout_sec=0.05)
        if not client.service_is_ready():
            return {
                'action': action,
                'service': service,
                'ok': False,
                'reason': 'cancel_service_unavailable',
            }

        request = CancelGoal.Request()
        # Zero UUID + stamp 0 → cancel all goals for this action server.
        request.goal_info.goal_id = UUID(uuid=[0] * 16)
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return {
                'action': action,
                'service': service,
                'ok': False,
                'reason': 'cancel_call_timeout',
            }
        response = future.result()
        code = int(response.return_code)
        ok = code in CANCEL_OK_CODES
        return {
            'action': action,
            'service': service,
            'ok': ok,
            'return_code': code,
            'goals_canceling': len(response.goals_canceling),
            'reason': None if ok else f'cancel_return_code={code}',
        }

    def action_servers_up(self, timeout_sec: float = 3.0) -> dict:
        """Evidence that Nav2 action servers exist (independent of status pubs)."""
        clients = {
            'navigate_to_pose': ActionClient(self, NavigateToPose, 'navigate_to_pose'),
            'navigate_through_poses': ActionClient(
                self, NavigateThroughPoses, 'navigate_through_poses',
            ),
        }
        result = {}
        for name, client in clients.items():
            ok = client.wait_for_server(timeout_sec=timeout_sec)
            result[name] = bool(ok)
            client.destroy()
        result['ok'] = all(result.values())
        return result

    def wait_inactive(self, timeout_sec: float) -> dict:
        """Prefer status-observed idle; fall back if servers are up but idle and mute.

        Humble action servers often publish no status until the first goal.
        That must NOT be reported as cancel_confirmed, but may allow proceed_ok
        when cancel calls succeeded and servers are reachable.
        """
        deadline = time.monotonic() + timeout_sec
        last = self.active_goals()
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            last = self.active_goals()
            seen = dict(self.status_seen)
            if all(seen.values()) and not any(last.values()):
                return {
                    'ok': True,
                    'mode': 'status_observed_idle',
                    'active': last,
                    'status_seen': seen,
                    'elapsed_sec': round(timeout_sec - (deadline - time.monotonic()), 2),
                }
            if all(seen.values()) and any(last.values()):
                # keep waiting for cancel to finish
                continue
        seen = dict(self.status_seen)
        if all(seen.values()) and any(last.values()):
            return {
                'ok': False,
                'mode': 'status_observed_active',
                'active': last,
                'status_seen': seen,
                'elapsed_sec': timeout_sec,
                'reason': 'goals_still_active',
            }
        servers = self.action_servers_up(timeout_sec=2.0)
        if not any(seen.values()) and servers.get('ok') and not any(last.values()):
            return {
                'ok': True,
                'mode': 'idle_status_unobserved',
                'active': last,
                'status_seen': seen,
                'action_servers': servers,
                'elapsed_sec': timeout_sec,
                'note': (
                    'No /_action/status messages received; Nav2 idle servers often '
                    'do not publish status until first goal. cancel_confirmed stays false.'
                ),
            }
        return {
            'ok': False,
            'mode': 'status_missing',
            'active': last,
            'status_seen': seen,
            'action_servers': servers,
            'elapsed_sec': timeout_sec,
            'reason': 'action_status_not_seen',
        }

    def quiet_cmd_vel(self, hold_sec: float, timeout_sec: float,
                      required_topics: list[str]) -> dict:
        """Zero /cmd_vel and require real message receipts; never treat missing as zero."""
        deadline = time.monotonic() + timeout_sec
        zero = Twist()
        samples = []
        quiet_since = None
        counts_at_start = dict(self.cmd_counts)
        while time.monotonic() < deadline:
            self.zero_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.05)
            snapshot = {}
            for topic, msg in self.cmd.items():
                received = self.cmd_counts[topic] > counts_at_start[topic]
                if msg is None or not received:
                    snapshot[topic] = {
                        'received': False,
                        'linear_x': None,
                        'angular_z': None,
                        'count': self.cmd_counts[topic],
                        'delta': self.cmd_counts[topic] - counts_at_start[topic],
                    }
                else:
                    lin, ang = twist_norm(msg)
                    snapshot[topic] = {
                        'received': True,
                        'linear_x': round(lin, 4),
                        'angular_z': round(ang, 4),
                        'count': self.cmd_counts[topic],
                        'delta': self.cmd_counts[topic] - counts_at_start[topic],
                    }
            samples.append(snapshot)

            missing = [
                topic for topic in required_topics
                if not snapshot.get(topic, {}).get('received')
            ]
            if missing:
                quiet_since = None
                continue

            quiet = all(
                snapshot[topic]['linear_x'] < self.quiet_linear
                and snapshot[topic]['angular_z'] < self.quiet_angular
                for topic in required_topics
            )
            if quiet:
                quiet_since = quiet_since or time.monotonic()
                if time.monotonic() - quiet_since >= hold_sec:
                    return {
                        'ok': True,
                        'hold_sec': hold_sec,
                        'elapsed_sec': round(timeout_sec - (deadline - time.monotonic()), 2),
                        'required_topics': required_topics,
                        'latest': snapshot,
                        'sample_n': len(samples),
                    }
            else:
                quiet_since = None
        latest = samples[-1] if samples else None
        missing = []
        if latest:
            missing = [
                topic for topic in required_topics
                if not latest.get(topic, {}).get('received')
            ]
        return {
            'ok': False,
            'reason': 'cmd_layer_not_received' if missing else 'cmd_vel_not_quiet',
            'missing_topics': missing,
            'hold_sec': hold_sec,
            'elapsed_sec': timeout_sec,
            'required_topics': required_topics,
            'latest': latest,
            'sample_n': len(samples),
        }


def write_result(path: Path | None, result: dict) -> None:
    text = json.dumps(result, indent=2) + '\n'
    if path:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)
    print(f'CLEAR_NAV_RESULT: {json.dumps(result)}', flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description='取消 Nav2 任务并确认 cmd_vel 静默')
    parser.add_argument('--cancel-timeout', type=float, default=8.0)
    parser.add_argument('--inactive-timeout', type=float, default=10.0)
    parser.add_argument('--quiet-hold-sec', type=float, default=0.5)
    parser.add_argument('--quiet-timeout', type=float, default=5.0)
    parser.add_argument('--quiet-linear', type=float, default=0.02)
    parser.add_argument('--quiet-angular', type=float, default=0.05)
    parser.add_argument(
        '--require-cmd-topics',
        default='/cmd_vel,/cmd_vel_gazebo',
        help='Comma list that must actually receive messages during quiet window',
    )
    parser.add_argument('--skip-cancel', action='store_true',
                        help='仅检查/清零速度（无 Nav2 时）')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    required = [t.strip() for t in args.require_cmd_topics.split(',') if t.strip()]

    rclpy.init()
    node = ClearNavControl(args.quiet_linear, args.quiet_angular)
    # Allow discovery / first status messages.
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    cancel_reports = []
    inactive = {'ok': True, 'skipped': True}
    if not args.skip_cancel:
        for action in ('navigate_to_pose', 'navigate_through_poses'):
            cancel_reports.append(node.cancel_all(action, args.cancel_timeout))
        inactive = node.wait_inactive(args.inactive_timeout)
    quiet = node.quiet_cmd_vel(args.quiet_hold_sec, args.quiet_timeout, required)

    result = {
        'clear_ok': False,
        'cancel': cancel_reports,
        'inactive': inactive,
        'cmd_vel_quiet': quiet,
        'active_goals_at_end': node.active_goals(),
        'status_seen': dict(node.status_seen),
        'cmd_topic_counts': node.cmd_counts,
    }

    if args.skip_cancel:
        result['cancel_confirmed'] = None
        result['proceed_ok'] = bool(quiet.get('ok'))
        result['clear_ok'] = bool(quiet.get('ok'))
        if not quiet.get('ok'):
            result['reason'] = quiet.get('reason')
    else:
        cancel_calls_ok = all(r.get('ok') for r in cancel_reports)
        inactive_ok = bool(inactive.get('ok'))
        quiet_ok = bool(quiet.get('ok'))
        status_seen = all(node.status_seen.values())
        mode = inactive.get('mode')
        # Strict evidence: only when status was observed empty.
        result['cancel_confirmed'] = bool(
            cancel_calls_ok and inactive_ok and status_seen
            and mode == 'status_observed_idle'
        )
        # Soft proceed: idle mute servers + cancel OK + quiet cmd (for reset/dwell).
        result['proceed_ok'] = bool(cancel_calls_ok and inactive_ok and quiet_ok)
        result['clear_ok'] = bool(result['proceed_ok'])
        result['idle_status_unobserved'] = bool(mode == 'idle_status_unobserved')
        if not cancel_calls_ok:
            result['reason'] = 'cancel_call_failed'
        elif not inactive_ok:
            result['reason'] = inactive.get('reason') or 'inactive_check_failed'
        elif not quiet_ok:
            result['reason'] = quiet.get('reason') or 'cmd_vel_not_quiet'
        elif result['idle_status_unobserved']:
            result['reason'] = 'proceed_with_unobserved_status'

    write_result(args.output, result)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if result['clear_ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
