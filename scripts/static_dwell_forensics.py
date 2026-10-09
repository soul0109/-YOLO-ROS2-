#!/usr/bin/env python3
"""Zero-command static dwell forensics (simulation contact creep diagnosis).

Protocol:
  1) clear_nav_control (cancel confirmed + real cmd receipts)
  2) delete + spawn at configured pose/yaw (no GT reseeding)
  3) hold zero /cmd_vel for N seconds
  4) record GT, odom, cmd layers, wheel joint velocity, optional Gazebo entity state

Does not modify Nav2 params or reseed AMCL from GT.
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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import JointState

ROOT = Path(__file__).resolve().parents[1]


def yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def run_cmd(cmd: list[str], timeout: float) -> dict:
    try:
        completed = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout, check=False,
        )
        return {
            'cmd': cmd,
            'returncode': completed.returncode,
            'stdout_tail': (completed.stdout or '')[-2000:],
            'stderr_tail': (completed.stderr or '')[-2000:],
        }
    except subprocess.TimeoutExpired as exc:
        return {
            'cmd': cmd,
            'returncode': None,
            'reason': 'timeout',
            'stdout_tail': (exc.stdout or '')[-2000:] if isinstance(exc.stdout, str) else '',
            'stderr_tail': (exc.stderr or '')[-2000:] if isinstance(exc.stderr, str) else '',
        }


class DwellProbe(Node):
    def __init__(self) -> None:
        super().__init__(
            'static_dwell_forensics',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )
        self.gt = None
        self.odom = None
        self.joints = None
        self.cmd = {
            '/cmd_vel': None,
            '/cmd_vel_gazebo': None,
            '/cmd_vel_nav': None,
        }
        self.counts = {
            'ground_truth': 0,
            'odom': 0,
            'joint_states': 0,
            '/cmd_vel': 0,
            '/cmd_vel_gazebo': 0,
            '/cmd_vel_nav': 0,
        }
        # Full-run counters (never capped). Trace buffers may truncate.
        self.cmd_nonzero = {name: 0 for name in self.cmd}
        self.cmd_abs_max = {
            name: {'linear_x': 0.0, 'angular_z': 0.0} for name in self.cmd
        }
        self.gt_trace: list[dict] = []
        self.odom_trace: list[dict] = []
        self.cmd_trace: list[dict] = []
        self.joint_trace: list[dict] = []
        self.entity_trace: list[dict] = []
        self.events: list[dict] = []
        self.t0 = time.monotonic()
        self.zero_pub = self.create_publisher(Twist, '/cmd_vel', qos_profile_system_default)
        self._cmd_eps = 1e-4
        self._cmd_trace_cap = 30000
        self._joint_trace_cap = 20000

        self.create_subscription(Odometry, '/ground_truth', self._on_gt, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self._on_joint, 10)
        for topic in self.cmd:
            self.create_subscription(
                Twist, topic, lambda msg, key=topic: self._on_cmd(key, msg),
                qos_profile_system_default if topic == '/cmd_vel' else qos_profile_sensor_data,
            )

    def _rel(self) -> float:
        return round(time.monotonic() - self.t0, 3)

    def mark(self, name: str, **extra) -> None:
        event = {'t_wall': self._rel(), 'event': name}
        event.update(extra)
        self.events.append(event)
        print(f'DWELL_EVENT: {json.dumps(event)}', flush=True)

    def _pose_row(self, msg: Odometry) -> dict:
        p = msg.pose.pose
        return {
            't_wall': self._rel(),
            'x': round(float(p.position.x), 6),
            'y': round(float(p.position.y), 6),
            'yaw': round(yaw_from_quaternion(p.orientation), 6),
            'v': round(float(msg.twist.twist.linear.x), 6),
            'w': round(float(msg.twist.twist.angular.z), 6),
            'stamp_sec': round(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, 3),
        }

    def _on_gt(self, msg: Odometry) -> None:
        self.gt = msg
        self.counts['ground_truth'] += 1
        if len(self.gt_trace) < 20000:
            self.gt_trace.append(self._pose_row(msg))

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg
        self.counts['odom'] += 1
        if len(self.odom_trace) < 20000:
            self.odom_trace.append(self._pose_row(msg))

    def _on_cmd(self, key: str, msg: Twist) -> None:
        self.cmd[key] = msg
        self.counts[key] += 1
        lin = float(msg.linear.x)
        ang = float(msg.angular.z)
        self.cmd_abs_max[key]['linear_x'] = max(self.cmd_abs_max[key]['linear_x'], abs(lin))
        self.cmd_abs_max[key]['angular_z'] = max(self.cmd_abs_max[key]['angular_z'], abs(ang))
        if abs(lin) > self._cmd_eps or abs(ang) > self._cmd_eps:
            self.cmd_nonzero[key] += 1
        if len(self.cmd_trace) < self._cmd_trace_cap:
            self.cmd_trace.append({
                't_wall': self._rel(),
                'topic': key,
                'linear_x': round(lin, 6),
                'angular_z': round(ang, 6),
            })

    def _on_joint(self, msg: JointState) -> None:
        self.joints = msg
        self.counts['joint_states'] += 1
        if len(self.joint_trace) >= self._joint_trace_cap:
            return
        names = list(msg.name)
        velocities = list(msg.velocity) if msg.velocity else []
        positions = list(msg.position) if msg.position else []
        row = {
            't_wall': self._rel(),
            'stamp_sec': round(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, 3),
        }
        for joint in ('left_wheel_joint', 'right_wheel_joint'):
            if joint in names:
                idx = names.index(joint)
                row[f'{joint}_pos'] = (
                    round(float(positions[idx]), 6) if idx < len(positions) else None
                )
                row[f'{joint}_vel'] = (
                    round(float(velocities[idx]), 6) if idx < len(velocities) else None
                )
        self.joint_trace.append(row)

    def spin_for(self, seconds: float, publish_zero: bool = False) -> None:
        deadline = time.monotonic() + seconds
        zero = Twist()
        while time.monotonic() < deadline:
            if publish_zero:
                self.zero_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.05)

    def sample_entity_state(self, entity: str = 'robot_v0') -> dict | None:
        # Best-effort Gazebo model pose via CLI; absence is recorded, not fatal.
        try:
            completed = subprocess.run(
                [
                    'ros2', 'service', 'call', '/get_entity_state',
                    'gazebo_msgs/srv/GetEntityState',
                    f"{{name: '{entity}', reference_frame: 'world'}}",
                ],
                capture_output=True, text=True, timeout=8.0, check=False,
            )
        except Exception as exc:  # noqa: BLE001 — diagnostics only
            return {'ok': False, 'reason': str(exc)}
        text = (completed.stdout or '') + '\n' + (completed.stderr or '')
        if completed.returncode != 0:
            return {'ok': False, 'reason': 'service_failed', 'tail': text[-800:]}
        # Parse loosely: look for position x/y and orientation z/w if present.
        return {
            'ok': True,
            't_wall': self._rel(),
            'raw_tail': text[-1200:],
        }


def fit_velocity(trace: list[dict], skip_sec: float = 2.0) -> dict:
    if len(trace) < 2:
        return {'ok': False, 'reason': 'too_few_samples'}
    t0 = trace[0]['stamp_sec'] if trace[0].get('stamp_sec') is not None else trace[0]['t_wall']
    use_stamp = trace[0].get('stamp_sec') is not None
    late = []
    for row in trace:
        t = row['stamp_sec'] if use_stamp else row['t_wall']
        if t - t0 >= skip_sec:
            late.append(row)
    if len(late) < 2:
        return {'ok': False, 'reason': 'too_few_after_skip'}
    ts = [(r['stamp_sec'] if use_stamp else r['t_wall']) - (
        late[0]['stamp_sec'] if use_stamp else late[0]['t_wall']
    ) for r in late]
    xs = [r['x'] for r in late]
    ys = [r['y'] for r in late]
    n = len(ts)
    mx = sum(ts) / n
    def slope(vals):
        my = sum(vals) / n
        num = sum((t - mx) * (v - my) for t, v in zip(ts, vals))
        den = sum((t - mx) ** 2 for t in ts) or 1e-12
        return num / den
    vx = slope(xs)
    vy = slope(ys)
    first, last = late[0], late[-1]
    dt = (last['stamp_sec'] if use_stamp else last['t_wall']) - (
        first['stamp_sec'] if use_stamp else first['t_wall']
    )
    return {
        'ok': True,
        'skip_sec': skip_sec,
        'sample_n': n,
        'dt_sec': round(dt, 3),
        'dx_m': round(last['x'] - first['x'], 6),
        'dy_m': round(last['y'] - first['y'], 6),
        'vx_m_s': round(vx, 6),
        'vy_m_s': round(vy, 6),
        'speed_m_s': round(math.hypot(vx, vy), 6),
        'vx_mm_s': round(vx * 1000.0, 3),
        'vy_mm_s': round(vy * 1000.0, 3),
        'first': first,
        'last': last,
    }


def body_frame_drift(world_vx: float, world_vy: float, yaw: float) -> dict:
    # Rotate world velocity into body frame at mean yaw.
    c, s = math.cos(yaw), math.sin(yaw)
    body_x = c * world_vx + s * world_vy
    body_y = -s * world_vx + c * world_vy
    return {
        'yaw_ref': round(yaw, 6),
        'body_vx_m_s': round(body_x, 6),
        'body_vy_m_s': round(body_y, 6),
        'body_vx_mm_s': round(body_x * 1000.0, 3),
        'body_vy_mm_s': round(body_y * 1000.0, 3),
    }


def cmd_full_stats(probe: DwellProbe) -> dict:
    """Use full-run counters; trace buffer may be truncated."""
    nonzero_total = sum(probe.cmd_nonzero.values())
    received = {t: probe.counts[t] for t in probe.cmd}
    return {
        'eps': probe._cmd_eps,
        'received_by_topic': received,
        'nonzero_by_topic': dict(probe.cmd_nonzero),
        'nonzero_n': nonzero_total,
        'abs_max_by_topic': probe.cmd_abs_max,
        'trace_saved_n': len(probe.cmd_trace),
        'trace_capped': any(received[t] > probe._cmd_trace_cap for t in probe.cmd),
        'coverage_note': (
            'nonzero_n counts every callback; cmd_trace may be capped'
        ),
    }


def joint_motion_stats(
    joint_trace: list[dict],
    stamp_lo: float | None = None,
    stamp_hi: float | None = None,
    wheel_radius: float = 0.05,
) -> dict:
    """Stats over an optional sim-stamp window (same robot / same dwell)."""
    rows = joint_trace
    if stamp_lo is not None and stamp_hi is not None:
        rows = [
            r for r in joint_trace
            if r.get('stamp_sec') is not None and stamp_lo <= r['stamp_sec'] <= stamp_hi
        ]
    if len(rows) < 2:
        return {
            'ok': False,
            'reason': 'too_few',
            'window': {'stamp_lo': stamp_lo, 'stamp_hi': stamp_hi},
            'sample_n': len(rows),
        }
    left = [r.get('left_wheel_joint_vel') for r in rows if r.get('left_wheel_joint_vel') is not None]
    right = [r.get('right_wheel_joint_vel') for r in rows if r.get('right_wheel_joint_vel') is not None]
    left_pos = [r.get('left_wheel_joint_pos') for r in rows if r.get('left_wheel_joint_pos') is not None]
    right_pos = [r.get('right_wheel_joint_pos') for r in rows if r.get('right_wheel_joint_pos') is not None]
    result = {
        'ok': True,
        'sample_n': len(rows),
        'window': {'stamp_lo': stamp_lo, 'stamp_hi': stamp_hi},
        'wheel_radius_m': wheel_radius,
    }
    if left:
        result['left_vel_max_abs'] = round(max(abs(v) for v in left), 6)
        result['left_vel_mean_abs'] = round(sum(abs(v) for v in left) / len(left), 6)
    if right:
        result['right_vel_max_abs'] = round(max(abs(v) for v in right), 6)
        result['right_vel_mean_abs'] = round(sum(abs(v) for v in right) / len(right), 6)
    if len(left_pos) >= 2:
        d = left_pos[-1] - left_pos[0]
        result['left_pos_delta'] = round(d, 6)
        result['left_roll_arc_m'] = round(d * wheel_radius, 6)
    if len(right_pos) >= 2:
        d = right_pos[-1] - right_pos[0]
        result['right_pos_delta'] = round(d, 6)
        result['right_roll_arc_m'] = round(d * wheel_radius, 6)
    return result


def judge_cause(fit: dict, body: dict, cmd_stats: dict, joints: dict, spawn_yaw: float) -> dict:
    """Heuristic cause label for reviewer; not a final physics root cause."""
    notes = []
    if cmd_stats.get('nonzero_n', 0) > 0:
        notes.append('nonzero_cmd_present')
    speed = fit.get('speed_m_s') or 0.0
    if speed < 5e-5:
        label = 'no_significant_pose_creep'
    elif abs(body.get('body_vx_m_s') or 0.0) > 2.0 * abs(body.get('body_vy_m_s') or 0.0):
        label = 'creep_aligned_with_body_x'
        notes.append('supports_heading_following_contact_creep_hypothesis')
    elif abs(fit.get('vx_m_s') or 0.0) > 2.0 * abs(fit.get('vy_m_s') or 0.0) and abs(spawn_yaw) < 0.2:
        label = 'creep_aligned_with_world_x_at_yaw0'
        notes.append('ambiguous_until_second_yaw_compared')
    else:
        label = 'creep_direction_mixed_or_world_fixed'
        notes.append('compare_with_orthogonal_yaw_run')

    if cmd_stats.get('nonzero_n', 0) == 0 and speed >= 5e-5:
        notes.append('saved_and_full_counters_show_zero_cmd')
    elif cmd_stats.get('nonzero_n', 0) == 0:
        notes.append('full_cmd_counters_zero')
    wheel_spin = max(
        abs(joints.get('left_pos_delta') or 0.0),
        abs(joints.get('right_pos_delta') or 0.0),
    )
    if wheel_spin > 0.05:
        notes.append('wheel_angle_changed_in_fit_window')
        notes.append('rolling_compatible_not_proof_of_solver_microtorque')
    elif joints.get('ok'):
        notes.append('wheel_joint_rotation_small_in_fit_window')
    return {
        'label': label,
        'notes': notes,
        'extrapolated_cm_per_min': round(speed * 60.0 * 100.0, 3),
    }


def write_markdown(path: Path, result: dict) -> None:
    fit = result['gt_fit']
    body = result.get('body_frame') or {}
    cause = result.get('cause_judgment') or {}
    lines = [
        '# 零指令静置漂移诊断',
        '',
        f"- 时间: {result.get('stamp')}",
        f"- HEAD: `{result.get('git_head')}` dirty=`{result.get('git_dirty')}`",
        f"- 规定出生: `({result['spawn']['x']}, {result['spawn']['y']}, yaw={result['spawn']['yaw']})`",
        f"- 请求静置（仿真时）: `{result['dwell_sec']} s`",
        f"- 实际计时: `{json.dumps(result.get('timing'), ensure_ascii=False)}`",
        f"- clear_ok / cancel_confirmed / idle_unobserved: "
        f"`{result.get('clear', {}).get('clear_ok')}` / "
        f"`{result.get('clear', {}).get('cancel_confirmed')}` / "
        f"`{result.get('clear', {}).get('idle_status_unobserved')}`",
        '',
        '## GT 漂移（跳过前 2 s 仿真时）',
        '',
        f"- vx: `{fit.get('vx_mm_s')} mm/s`, vy: `{fit.get('vy_mm_s')} mm/s`, speed: `{fit.get('speed_m_s')} m/s`",
        f"- body_vx: `{body.get('body_vx_mm_s')} mm/s`, body_vy: `{body.get('body_vy_mm_s')} mm/s`",
        f"- 外推: `{cause.get('extrapolated_cm_per_min')} cm/min`（推算，非正式验收）",
        '',
        '## 原因判断（启发式，待双朝向对照）',
        '',
        f"- label: `{cause.get('label')}`",
        f"- notes: `{', '.join(cause.get('notes') or [])}`",
        '',
        '## 指令 / 关节',
        '',
        f"- cmd nonzero（全程回调计数）: `{result.get('cmd_stats', {}).get('nonzero_n')}`",
        f"- cmd received: `{json.dumps(result.get('cmd_stats', {}).get('received_by_topic'), ensure_ascii=False)}`",
        f"- cmd trace saved/capped: "
        f"`{result.get('cmd_stats', {}).get('trace_saved_n')}` / "
        f"`{result.get('cmd_stats', {}).get('trace_capped')}`",
        f"- coverage: `{result.get('cmd_stats', {}).get('coverage_note')}`",
        f"- joints（GT 拟合窗）: `{json.dumps(result.get('joint_stats'), ensure_ascii=False)}`",
        f"- entity samples: `{len(result.get('entity_trace') or [])}` "
        f"（默认关闭 GetEntityState，避免阻塞）",
        '',
        '## 文件',
        '',
        '- `result.json` 完整轨迹',
        '- `clear.json` 清控证据',
        '',
    ]
    path.write_text('\n'.join(lines) + '\n')


def main() -> int:
    parser = argparse.ArgumentParser(description='零指令静置漂移取证')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--dwell-sec', type=float, default=90.0)
    parser.add_argument('--spawn-x', type=float, default=0.9)
    parser.add_argument('--spawn-y', type=float, default=3.0)
    parser.add_argument('--spawn-z', type=float, default=0.05)
    parser.add_argument('--spawn-yaw', type=float, default=0.0)
    parser.add_argument('--entity-sample-period', type=float, default=0.0,
                        help='Gazebo GetEntityState period; default 0=disabled (blocks ROS spin)')
    parser.add_argument('--skip-clear', action='store_true')
    parser.add_argument('--skip-respawn', action='store_true',
                        help='Do not delete/spawn; dwell in place (debug only)')
    args = parser.parse_args()

    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime('%Y-%m-%dT%H:%M:%S%z')
    git_head = subprocess.run(
        ['git', '-C', str(ROOT), 'rev-parse', '--short', 'HEAD'],
        capture_output=True, text=True, check=False,
    ).stdout.strip() or 'unknown'
    git_dirty = bool(subprocess.run(
        ['git', '-C', str(ROOT), 'status', '--porcelain'],
        capture_output=True, text=True, check=False,
    ).stdout.strip())

    clear_result = None
    if not args.skip_clear:
        clear_path = out / 'clear.json'
        clear_run = run_cmd(
            [sys.executable, str(ROOT / 'scripts/clear_nav_control.py'),
             '--output', str(clear_path)],
            timeout=45.0,
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
                'dwell_sec': args.dwell_sec,
                'clear': clear_result,
                'gt_fit': {'ok': False, 'reason': 'clear_failed'},
                'cause_judgment': {'label': 'aborted_clear_failed', 'notes': []},
            }
            (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
            write_markdown(out / 'results.md', result)
            print(f'DWELL_RESULT: {json.dumps(result["cause_judgment"])}', flush=True)
            return 2

    rclpy.init()
    probe = DwellProbe()
    probe.mark('probe_started', spawn_yaw=args.spawn_yaw)
    probe.spin_for(1.0)

    delete = spawn = None
    if not args.skip_respawn:
        delete = run_cmd(
            ['ros2', 'service', 'call', '/delete_entity', 'gazebo_msgs/srv/DeleteEntity',
             "{name: 'robot_v0'}"],
            timeout=25.0,
        )
        probe.mark('delete_entity', returncode=delete.get('returncode'))
        probe.spin_for(2.0)
        spawn = run_cmd(
            [
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'robot_v0',
                '-topic', 'robot_description',
                '-x', str(args.spawn_x),
                '-y', str(args.spawn_y),
                '-z', str(args.spawn_z),
                '-Y', str(args.spawn_yaw),
            ],
            timeout=50.0,
        )
        probe.mark('spawn_finished', returncode=spawn.get('returncode'), yaw=args.spawn_yaw)
        if spawn.get('returncode') not in (0,):
            result = {
                'stamp': stamp,
                'git_head': git_head,
                'git_dirty': git_dirty,
                'spawn': {
                    'x': args.spawn_x, 'y': args.spawn_y,
                    'z': args.spawn_z, 'yaw': args.spawn_yaw,
                },
                'dwell_sec': args.dwell_sec,
                'clear': clear_result,
                'spawn_run': spawn,
                'gt_fit': {'ok': False, 'reason': 'spawn_failed'},
                'cause_judgment': {'label': 'aborted_spawn_failed', 'notes': []},
            }
            (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
            write_markdown(out / 'results.md', result)
            probe.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            return 3
        # Brief settle before starting the formal dwell window.
        probe.spin_for(2.0, publish_zero=True)

    # Wait until sim clock is alive (use_sim_time).
    clock_deadline = time.monotonic() + 30.0
    while time.monotonic() < clock_deadline and probe.get_clock().now().nanoseconds == 0:
        rclpy.spin_once(probe, timeout_sec=0.05)
    if probe.get_clock().now().nanoseconds == 0:
        result = {
            'stamp': stamp,
            'git_head': git_head,
            'git_dirty': git_dirty,
            'spawn': {
                'x': args.spawn_x, 'y': args.spawn_y,
                'z': args.spawn_z, 'yaw': args.spawn_yaw,
            },
            'dwell_sec': args.dwell_sec,
            'clear': clear_result,
            'gt_fit': {'ok': False, 'reason': 'sim_clock_zero'},
            'cause_judgment': {'label': 'aborted_sim_clock_zero', 'notes': []},
        }
        (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        write_markdown(out / 'results.md', result)
        probe.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 4

    dwell_start_idx = len(probe.gt_trace)
    sim_start = probe.get_clock().now()
    wall_start = time.monotonic()
    probe.mark(
        'dwell_begin',
        gt_count=probe.counts['ground_truth'],
        sim_sec=round(sim_start.nanoseconds * 1e-9, 3),
        note='duration counted in sim time (/clock), not wall clock',
    )
    next_entity = time.monotonic()
    next_progress = 0.0
    zero = Twist()
    # Wall safety: if Gazebo freezes, don't hang forever (10x realtime budget).
    wall_budget = max(args.dwell_sec * 10.0, args.dwell_sec + 120.0)
    while True:
        probe.zero_pub.publish(zero)
        rclpy.spin_once(probe, timeout_sec=0.05)
        sim_now = probe.get_clock().now()
        sim_elapsed = (sim_now - sim_start).nanoseconds * 1e-9
        wall_elapsed = time.monotonic() - wall_start
        if sim_elapsed >= args.dwell_sec:
            break
        if wall_elapsed >= wall_budget:
            probe.mark(
                'dwell_wall_budget_exceeded',
                sim_elapsed=round(sim_elapsed, 3),
                wall_elapsed=round(wall_elapsed, 3),
            )
            break
        if sim_elapsed >= next_progress:
            print(
                f'DWELL_PROGRESS: sim={sim_elapsed:.1f}/{args.dwell_sec:.1f}s '
                f'wall={wall_elapsed:.1f}s gt={probe.counts["ground_truth"]}',
                flush=True,
            )
            next_progress += 10.0
        if args.entity_sample_period > 0 and time.monotonic() >= next_entity:
            sample = probe.sample_entity_state()
            if sample is not None:
                probe.entity_trace.append(sample)
            next_entity = time.monotonic() + args.entity_sample_period
    sim_end = probe.get_clock().now()
    wall_end = time.monotonic()
    timing = {
        'requested_sim_sec': args.dwell_sec,
        'elapsed_sim_sec': round((sim_end - sim_start).nanoseconds * 1e-9, 3),
        'elapsed_wall_sec': round(wall_end - wall_start, 3),
        'realtime_factor': None,
    }
    if timing['elapsed_wall_sec'] > 1e-3:
        timing['realtime_factor'] = round(
            timing['elapsed_sim_sec'] / timing['elapsed_wall_sec'], 3,
        )
    probe.mark(
        'dwell_end',
        gt_count=probe.counts['ground_truth'],
        **timing,
    )

    gt_dwell = probe.gt_trace[dwell_start_idx:]
    odom_dwell = [row for row in probe.odom_trace if row['t_wall'] >= (
        gt_dwell[0]['t_wall'] if gt_dwell else 0
    )]
    fit = fit_velocity(gt_dwell, skip_sec=2.0)
    mean_yaw = args.spawn_yaw
    stamp_lo = stamp_hi = None
    if fit.get('ok') and fit.get('first') and fit.get('last'):
        mean_yaw = 0.5 * (fit['first']['yaw'] + fit['last']['yaw'])
        stamp_lo = fit['first'].get('stamp_sec')
        stamp_hi = fit['last'].get('stamp_sec')
    body = body_frame_drift(fit.get('vx_m_s') or 0.0, fit.get('vy_m_s') or 0.0, mean_yaw) if fit.get('ok') else None
    cmd_stats = cmd_full_stats(probe)
    # Joint deltas only over the same post-skip GT fit window (same spawned robot).
    joint_stats = joint_motion_stats(
        probe.joint_trace, stamp_lo=stamp_lo, stamp_hi=stamp_hi,
    )
    cause = judge_cause(fit if fit.get('ok') else {'speed_m_s': 0.0}, body or {}, cmd_stats, joint_stats, args.spawn_yaw)

    result = {
        'stamp': stamp,
        'git_head': git_head,
        'git_dirty': git_dirty,
        'spawn': {
            'x': args.spawn_x, 'y': args.spawn_y,
            'z': args.spawn_z, 'yaw': args.spawn_yaw,
        },
        'dwell_sec': args.dwell_sec,
        'timing': timing,
        'protocol': {
            'zero_command': True,
            'gt_reseed': False,
            'skip_first_sec_for_fit': 2.0,
            'duration_base': 'sim_time',
        },
        'clear': clear_result,
        'delete': {'returncode': None if delete is None else delete.get('returncode')},
        'spawn_run': {'returncode': None if spawn is None else spawn.get('returncode')},
        'topic_counts': probe.counts,
        'gt_fit': fit,
        'odom_fit': fit_velocity(odom_dwell, skip_sec=2.0),
        'body_frame': body,
        'cmd_stats': cmd_stats,
        'joint_stats': joint_stats,
        'cause_judgment': cause,
        'events': probe.events,
        'gt_trace': gt_dwell,
        'odom_trace': odom_dwell,
        'cmd_trace': probe.cmd_trace,
        'joint_trace': probe.joint_trace,
        'entity_trace': probe.entity_trace,
    }
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    write_markdown(out / 'results.md', result)
    print(f'DWELL_RESULT: {json.dumps({"fit": fit, "body": body, "cause": cause})}', flush=True)

    probe.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if fit.get('ok') else 1


if __name__ == '__main__':
    sys.exit(main())
