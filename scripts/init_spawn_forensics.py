#!/usr/bin/env python3
"""Record spawn / GT / cmd_vel / odom / initialpose / AMCL evidence.

Formal baseline rule: publish the *configured* initial pose, never reseed from GT.
GT is a measurement ruler only. Position mismatch → INIT_FAIL evidence, not auto-fix.

This script does not claim residual cmd_vel is the root cause; it records the chain
so a reviewer can decide.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
    qos_profile_system_default,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from coords import load_map_origin


AMCL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

ROOT = Path(__file__).resolve().parents[1]


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def pose_dict_from_odom(msg: Odometry) -> dict:
    p = msg.pose.pose
    return {
        'x': round(float(p.position.x), 4),
        'y': round(float(p.position.y), 4),
        'yaw': round(yaw_from_quaternion(p.orientation), 4),
        'v': round(float(msg.twist.twist.linear.x), 4),
        'w': round(float(msg.twist.twist.angular.z), 4),
        'stamp_sec': round(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, 3),
    }


def twist_dict(msg: Twist | None) -> dict | None:
    if msg is None:
        return None
    return {
        'linear_x': round(float(msg.linear.x), 4),
        'angular_z': round(float(msg.angular.z), 4),
    }


class ForensicsProbe(Node):
    def __init__(self) -> None:
        super().__init__(
            'init_spawn_forensics',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.gt = None
        self.odom = None
        self.amcl = None
        self.cmd = {
            '/cmd_vel': None,
            '/cmd_vel_gazebo': None,
            '/cmd_vel_nav': None,
        }
        self.counts = {
            'ground_truth': 0,
            'odom': 0,
            'amcl_pose': 0,
            '/cmd_vel': 0,
            '/cmd_vel_gazebo': 0,
            '/cmd_vel_nav': 0,
        }
        self.gt_trace: list[dict] = []
        self.odom_trace: list[dict] = []
        self.cmd_trace: list[dict] = []
        self.amcl_trace: list[dict] = []
        self.events: list[dict] = []
        self.t0 = time.monotonic()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.origin = (0.0, 0.0, 0.0)

        self.create_subscription(Odometry, '/ground_truth', self._on_gt, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, AMCL_QOS)
        for topic in self.cmd:
            self.create_subscription(
                Twist, topic, lambda msg, key=topic: self._on_cmd(key, msg),
                qos_profile_system_default if topic == '/cmd_vel' else qos_profile_sensor_data,
            )

    def set_origin(self, origin: tuple[float, float, float]) -> None:
        self.origin = origin

    def _rel(self) -> float:
        return round(time.monotonic() - self.t0, 3)

    def mark(self, name: str, **extra) -> None:
        event = {'t_wall': self._rel(), 'event': name}
        event.update(extra)
        self.events.append(event)
        print(f'FORENSICS_EVENT: {json.dumps(event)}', flush=True)

    def _on_gt(self, msg: Odometry) -> None:
        self.gt = msg
        self.counts['ground_truth'] += 1
        if len(self.gt_trace) < 2000:
            row = pose_dict_from_odom(msg)
            row['t_wall'] = self._rel()
            self.gt_trace.append(row)

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg
        self.counts['odom'] += 1
        if len(self.odom_trace) < 2000:
            row = pose_dict_from_odom(msg)
            row['t_wall'] = self._rel()
            self.odom_trace.append(row)

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl = msg
        self.counts['amcl_pose'] += 1
        if len(self.amcl_trace) < 500:
            p = msg.pose.pose
            self.amcl_trace.append({
                't_wall': self._rel(),
                'x': round(float(p.position.x), 4),
                'y': round(float(p.position.y), 4),
                'yaw': round(yaw_from_quaternion(p.orientation), 4),
                'cov': [round(float(msg.pose.covariance[i]), 4) for i in (0, 7, 35)],
                'stamp_sec': round(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, 3),
            })

    def _on_cmd(self, key: str, msg: Twist) -> None:
        self.cmd[key] = msg
        self.counts[key] += 1
        if len(self.cmd_trace) < 3000:
            self.cmd_trace.append({
                't_wall': self._rel(),
                'topic': key,
                **(twist_dict(msg) or {}),
            })

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def latest_snapshot(self) -> dict:
        tf_info = {'ok': False, 'reason': 'lookup_failed'}
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(), timeout=Duration(seconds=0.05),
            )
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            now = self.get_clock().now().nanoseconds * 1e-9
            tf_info = {
                'ok': True,
                'stamp_sec': round(stamp, 3),
                'clock_sec': round(now, 3),
                # Keep raw delta for review. AMCL may stamp into the future via
                # transform_tolerance; do not auto-fail here.
                'stamp_minus_clock_sec': round(stamp - now, 3) if now > 0 else None,
                'translation': {
                    'x': round(float(tf.transform.translation.x), 4),
                    'y': round(float(tf.transform.translation.y), 4),
                },
            }
        except TransformException as exc:
            tf_info = {'ok': False, 'reason': str(exc)}

        gt = pose_dict_from_odom(self.gt) if self.gt is not None else None
        odom = pose_dict_from_odom(self.odom) if self.odom is not None else None
        amcl = None
        amcl_vs_gt = None
        if self.amcl is not None:
            p = self.amcl.pose.pose
            amcl = {
                'x': round(float(p.position.x), 4),
                'y': round(float(p.position.y), 4),
                'yaw': round(yaw_from_quaternion(p.orientation), 4),
                'cov': [round(float(self.amcl.pose.covariance[i]), 4) for i in (0, 7, 35)],
            }
            if gt is not None:
                ox, oy, _ = self.origin
                gt_map_x = gt['x'] - ox
                gt_map_y = gt['y'] - oy
                amcl_vs_gt = round(
                    math.hypot(amcl['x'] - gt_map_x, amcl['y'] - gt_map_y), 4,
                )
        return {
            'ground_truth': gt,
            'odom': odom,
            'amcl': amcl,
            'amcl_error_m': amcl_vs_gt,
            'cmd_vel': {k: twist_dict(v) for k, v in self.cmd.items()},
            'tf_map_base_footprint': tf_info,
            'topic_counts': dict(self.counts),
        }


def run_cmd(cmd: list[str], timeout: float) -> dict:
    try:
        completed = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout, check=False,
        )
        return {
            'cmd': cmd,
            'returncode': completed.returncode,
            'stdout_tail': completed.stdout[-2000:],
            'stderr_tail': completed.stderr[-2000:],
        }
    except subprocess.TimeoutExpired as exc:
        return {
            'cmd': cmd,
            'returncode': None,
            'reason': 'timeout',
            'stdout_tail': (exc.stdout or '')[-2000:] if isinstance(exc.stdout, str) else '',
            'stderr_tail': (exc.stderr or '')[-2000:] if isinstance(exc.stderr, str) else '',
        }


def summarize(spawn_xy: tuple[float, float], spawn_gate_m: float, stopped_v: float,
              stopped_w: float, probe: ForensicsProbe, post_spawn_start_idx: int) -> dict:
    gt_after = probe.gt_trace[post_spawn_start_idx:]
    if not gt_after:
        return {
            'spawn_pose_ok': False,
            'robot_stopped': False,
            'reason': 'no_gt_after_spawn',
            'gt_first_after_spawn': None,
            'gt_last': None,
            'gt_delta_from_spawn_m': None,
            'gt_travel_after_spawn_m': None,
        }

    first = gt_after[0]
    last = gt_after[-1]
    peak = max(gt_after, key=lambda row: math.hypot(row['x'] - spawn_xy[0], row['y'] - spawn_xy[1]))
    travel = 0.0
    for a, b in zip(gt_after, gt_after[1:]):
        travel += math.hypot(b['x'] - a['x'], b['y'] - a['y'])
    delta = math.hypot(last['x'] - spawn_xy[0], last['y'] - spawn_xy[1])
    peak_delta = math.hypot(peak['x'] - spawn_xy[0], peak['y'] - spawn_xy[1])
    stopped = abs(last['v']) < stopped_v and abs(last['w']) < stopped_w
    # Formal gate: near configured birth pose AND stopped. No GT reseeding.
    spawn_pose_ok = delta <= spawn_gate_m and stopped
    return {
        'spawn_pose_ok': spawn_pose_ok,
        'robot_stopped': stopped,
        'spawn_gate_m': spawn_gate_m,
        'gt_first_after_spawn': first,
        'gt_last': last,
        'gt_peak_offset': peak,
        'gt_delta_from_spawn_m': round(delta, 4),
        'gt_peak_delta_from_spawn_m': round(peak_delta, 4),
        'gt_travel_after_spawn_m': round(travel, 4),
        'reason': None if spawn_pose_ok else (
            'not_near_spawn' if delta > spawn_gate_m else 'not_stopped'
        ),
    }


def write_markdown(path: Path, result: dict) -> None:
    summary = result['summary']
    clear = result.get('clear', {})
    lines = [
        '# 初始化偏移取证',
        '',
        f"- 时间: {result.get('stamp')}",
        f"- HEAD: `{result.get('git_head', 'unknown')}`",
        f"- dirty: `{result.get('git_dirty', 'unknown')}`",
        f"- 规定出生位姿: `({result['spawn']['x']}, {result['spawn']['y']}, yaw={result['spawn']['yaw']})`",
        f"- initialpose: **规定位姿（非 GT 重设）**",
        '',
        '## 结论（证据，非定因）',
        '',
        f"- clear_ok: `{clear.get('clear_ok')}` / cancel_confirmed: `{clear.get('cancel_confirmed')}`",
        f"- spawn_pose_ok: `{summary.get('spawn_pose_ok')}`",
        f"- GT 最终相对出生点: `{summary.get('gt_delta_from_spawn_m')} m`",
        f"- GT 峰值相对出生点: `{summary.get('gt_peak_delta_from_spawn_m')} m`",
        f"- 出生后 GT 路径积分: `{summary.get('gt_travel_after_spawn_m')} m`",
        f"- 静止: `{summary.get('robot_stopped')}`",
        f"- 正式流程判定: `{'INIT_POSE_READY' if summary.get('spawn_pose_ok') else 'INIT_FAIL_SPAWN_OFFSET'}`",
        '',
        '## 说明',
        '',
        '- “残留控制自溜”仍是待证假设；请结合 `cmd_trace` 与 clear 结果判断。',
        '- 本记录不会用 GT 改写 initialpose。',
        '- 静止时 AMCL 无新消息可能正常（update_min_d/a），勿单独当作失效。',
        '',
        '## 文件',
        '',
        '- `result.json`：完整结构化结果',
        '- `clear.json`：清控证据（若有）',
        '',
    ]
    path.write_text('\n'.join(lines) + '\n')


def main() -> int:
    parser = argparse.ArgumentParser(description='出生偏移取证（不 GT 重设种子）')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--map-yaml', type=Path, default=ROOT / 'ros2_ws/src/navigation_config/maps/test_room.yaml')
    parser.add_argument('--spawn-x', type=float, default=0.9)
    parser.add_argument('--spawn-y', type=float, default=3.0)
    parser.add_argument('--spawn-z', type=float, default=0.05)
    parser.add_argument('--spawn-yaw', type=float, default=0.0)
    parser.add_argument('--spawn-gate-m', type=float, default=0.10,
                        help='正式协议：静止后相对规定出生点的允许偏差')
    parser.add_argument('--post-spawn-sec', type=float, default=8.0)
    parser.add_argument('--post-initialpose-sec', type=float, default=5.0)
    parser.add_argument('--skip-clear', action='store_true')
    parser.add_argument('--skip-initialpose', action='store_true',
                        help='只取证出生漂移，不发 initialpose')
    args = parser.parse_args()

    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime('%Y-%m-%dT%H:%M:%S%z')

    git_head = subprocess.run(
        ['git', '-C', str(ROOT), 'rev-parse', '--short', 'HEAD'],
        capture_output=True, text=True, check=False,
    ).stdout.strip() or 'unknown'
    dirty = subprocess.run(
        ['git', '-C', str(ROOT), 'status', '--porcelain'],
        capture_output=True, text=True, check=False,
    ).stdout.strip()
    git_dirty = bool(dirty)

    clear_result = None
    if not args.skip_clear:
        clear_path = out / 'clear.json'
        clear_run = run_cmd(
            [sys.executable, str(ROOT / 'scripts/clear_nav_control.py'),
             '--output', str(clear_path)],
            timeout=40.0,
        )
        if clear_path.is_file():
            clear_result = json.loads(clear_path.read_text())
        else:
            clear_result = {'clear_ok': False, 'reason': 'clear_script_no_output', 'run': clear_run}
        if not clear_result.get('clear_ok'):
            result = {
                'stamp': stamp,
                'git_head': git_head,
                'git_dirty': git_dirty,
                'spawn': {
                    'x': args.spawn_x, 'y': args.spawn_y,
                    'z': args.spawn_z, 'yaw': args.spawn_yaw,
                },
                'clear': clear_result,
                'summary': {
                    'spawn_pose_ok': False,
                    'reason': 'clear_failed_before_spawn',
                },
                'protocol': {
                    'initialpose_source': 'configured_spawn_pose',
                    'gt_reseed': False,
                },
            }
            (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
            write_markdown(out / 'results.md', result)
            print(f'FORENSICS_RESULT: {json.dumps(result["summary"])}', flush=True)
            return 2

    origin = load_map_origin(args.map_yaml)
    rclpy.init()
    probe = ForensicsProbe()
    probe.set_origin(origin)
    probe.mark('probe_started')

    # Wait briefly for clock / topics before touching Gazebo entities.
    probe.spin_for(1.0)

    delete = run_cmd(
        ['ros2', 'service', 'call', '/delete_entity', 'gazebo_msgs/srv/DeleteEntity',
         "{name: 'robot_v0'}"],
        timeout=25.0,
    )
    probe.mark('delete_entity', returncode=delete.get('returncode'))
    probe.spin_for(2.0)

    spawn_cmd = [
        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
        '-entity', 'robot_v0',
        '-topic', 'robot_description',
        '-x', str(args.spawn_x),
        '-y', str(args.spawn_y),
        '-z', str(args.spawn_z),
        '-Y', str(args.spawn_yaw),
    ]
    probe.mark('spawn_requested', spawn={
        'x': args.spawn_x, 'y': args.spawn_y, 'z': args.spawn_z, 'yaw': args.spawn_yaw,
    })
    gt_idx_before_spawn = len(probe.gt_trace)
    spawn = run_cmd(spawn_cmd, timeout=50.0)
    probe.mark('spawn_finished', returncode=spawn.get('returncode'))
    if spawn.get('returncode') not in (0,):
        result = {
            'stamp': stamp,
            'git_head': git_head,
            'git_dirty': git_dirty,
            'spawn': {
                'x': args.spawn_x, 'y': args.spawn_y,
                'z': args.spawn_z, 'yaw': args.spawn_yaw,
            },
            'clear': clear_result,
            'delete': delete,
            'spawn_run': spawn,
            'summary': {'spawn_pose_ok': False, 'reason': 'spawn_failed'},
            'protocol': {
                'initialpose_source': 'configured_spawn_pose',
                'gt_reseed': False,
            },
        }
        (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        write_markdown(out / 'results.md', result)
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 3

    # Continuously record GT / odom / cmd_vel after birth.
    post_spawn_start_idx = len(probe.gt_trace)
    probe.mark('post_spawn_record_begin', gt_count=probe.counts['ground_truth'])
    probe.spin_for(args.post_spawn_sec)
    probe.mark('post_spawn_record_end', gt_count=probe.counts['ground_truth'])
    summary = summarize(
        (args.spawn_x, args.spawn_y), args.spawn_gate_m, 0.02, 0.05,
        probe, post_spawn_start_idx,
    )
    snapshot_before_init = probe.latest_snapshot()
    probe.mark('snapshot_before_initialpose', **{
        k: snapshot_before_init.get(k) for k in ('amcl_error_m', 'tf_map_base_footprint')
    })

    initialpose_run = None
    snapshot_after_init = None
    if not args.skip_initialpose:
        # Formal protocol: configured birth pose only. Never pass measured GT.
        init_cmd = [
            sys.executable, str(ROOT / 'scripts/publish_amcl_initial_pose.py'),
            '--world-x', str(args.spawn_x),
            '--world-y', str(args.spawn_y),
            '--yaw', str(args.spawn_yaw),
            '--map-yaml', str(args.map_yaml),
        ]
        probe.mark('initialpose_requested_configured_spawn')
        initialpose_run = run_cmd(init_cmd, timeout=35.0)
        probe.mark('initialpose_finished', returncode=initialpose_run.get('returncode'))
        probe.spin_for(args.post_initialpose_sec)
        snapshot_after_init = probe.latest_snapshot()
        probe.mark('snapshot_after_initialpose', amcl_error_m=snapshot_after_init.get('amcl_error_m'))

    result = {
        'stamp': stamp,
        'git_head': git_head,
        'git_dirty': git_dirty,
        'spawn': {
            'x': args.spawn_x, 'y': args.spawn_y,
            'z': args.spawn_z, 'yaw': args.spawn_yaw,
        },
        'map_origin': list(origin),
        'protocol': {
            'initialpose_source': 'configured_spawn_pose',
            'gt_reseed': False,
            'spawn_gate_m': args.spawn_gate_m,
            'note': 'GT is measurement only; mismatch => INIT_FAIL evidence',
        },
        'clear': clear_result,
        'delete': {'returncode': delete.get('returncode')},
        'spawn_run': {'returncode': spawn.get('returncode')},
        'initialpose_run': (
            {'returncode': initialpose_run.get('returncode')} if initialpose_run else None
        ),
        'summary': summary,
        'snapshot_before_initialpose': snapshot_before_init,
        'snapshot_after_initialpose': snapshot_after_init,
        'events': probe.events,
        'topic_counts': probe.counts,
        'gt_trace': probe.gt_trace,
        'odom_trace': probe.odom_trace,
        'cmd_trace': probe.cmd_trace,
        'amcl_trace': probe.amcl_trace,
        'gt_idx_before_spawn': gt_idx_before_spawn,
        'gt_idx_post_spawn_start': post_spawn_start_idx,
    }
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    write_markdown(out / 'results.md', result)
    print(f'FORENSICS_RESULT: {json.dumps(summary)}', flush=True)

    probe.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    # Exit codes for automation:
    # 0 = clear ok and spawn pose within formal gate
    # 1 = clear ok but spawn offset / not stopped (INIT_FAIL evidence)
    # 2/3 = infra
    if not summary.get('spawn_pose_ok'):
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
