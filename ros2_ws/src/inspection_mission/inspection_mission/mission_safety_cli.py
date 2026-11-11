"""Operator recovery CLI for robot safety barriers (V2-02 / §19).

Online (preferred): forward to live Mission Runtime via recover_req/resp files.
Offline: only after acquiring the same robot flock; never inject expected tokens.

Copy-paste examples (use temp dirs in tests; production uses default runtime)::

  python3 -m inspection_mission.mission_safety_cli status \\
    --runtime-dir /tmp/robot_rt --robot-id inspection_sim_01

  python3 -m inspection_mission.mission_safety_cli arm \\
    --runtime-dir /tmp/robot_rt --robot-id inspection_sim_01

  python3 -m inspection_mission.mission_safety_cli confirm \\
    --runtime-dir /tmp/robot_rt --robot-id inspection_sim_01 \\
    --instance-id <id> --token <token> \\
    --archive-db /tmp/robot_rt/archive.sqlite3
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from inspection_mission.mission_archive import MissionArchive
from inspection_mission.mission_recovery_ipc import (
    live_runtime_present,
    submit_online_request,
)
from inspection_mission.robot_safety import (
    DEFAULT_ROBOT_ID,
    RobotInstanceGuard,
    default_runtime_dir,
)


def _print(obj: dict) -> None:
    print(json.dumps(obj, indent=2, ensure_ascii=False))


def cmd_status(args: argparse.Namespace) -> int:
    runtime_dir = Path(args.runtime_dir)
    robot_id = args.robot_id
    probe = RobotInstanceGuard(robot_id=robot_id, runtime_dir=runtime_dir)
    if live_runtime_present(probe):
        out = submit_online_request(
            runtime_dir=runtime_dir, robot_id=robot_id, cmd='status',
            timeout_sec=args.timeout,
        )
        _print(out)
        return 0 if out.get('ok', True) else 1
    # Offline status: report disk + meta; never invent a "current" live instance.
    meta = probe.read_instance_meta()
    snap = probe.read_safety()
    _print({
        'ok': True,
        'mode': 'offline',
        'robot_id': robot_id,
        'instance_id': meta.get('instance_id') or snap.payload.get('instance_id') or '',
        'live_runtime': False,
        'runtime_dir': str(runtime_dir),
        'safety_path': str(probe.safety_path),
        'safety_ok': snap.ok,
        'safety_refuse': snap.refuse,
        'safety_reason': snap.reason,
        'payload': snap.payload,
        'instance_meta': meta,
    })
    return 0


def cmd_arm(args: argparse.Namespace) -> int:
    runtime_dir = Path(args.runtime_dir)
    robot_id = args.robot_id
    probe = RobotInstanceGuard(robot_id=robot_id, runtime_dir=runtime_dir)
    if live_runtime_present(probe):
        out = submit_online_request(
            runtime_dir=runtime_dir, robot_id=robot_id, cmd='arm',
            timeout_sec=args.timeout,
        )
        _print(out)
        return 0 if out.get('ok') else 1
    # Offline exclusive: must hold flock.
    g = RobotInstanceGuard(robot_id=robot_id, runtime_dir=runtime_dir)
    ok, msg = g.acquire_flock()
    if not ok:
        _print({'ok': False, 'message': f'offline arm refused: {msg}'})
        return 1
    try:
        # Use this offline session's own instance_id. Do NOT adopt a stale mark
        # owner (that would impersonate). Confirm path does explicit takeover.
        token = g.arm_operator_confirm(persist=True)
        # Do NOT write live instance_meta here — that is Runtime-only.
        _print({
            'ok': True,
            'mode': 'offline',
            'instance_id': g.instance_id,
            'token': token,
            'message': (
                'armed under exclusive flock; confirm with same instance_id/token '
                '(stale foreign marks are taken over explicitly on confirm)'
            ),
        })
        return 0
    finally:
        g.release_flock()


def cmd_confirm(args: argparse.Namespace) -> int:
    if not args.instance_id or not args.token:
        print('confirm requires --instance-id and --token', file=sys.stderr)
        return 2
    runtime_dir = Path(args.runtime_dir)
    robot_id = args.robot_id
    probe = RobotInstanceGuard(robot_id=robot_id, runtime_dir=runtime_dir)
    if live_runtime_present(probe):
        out = submit_online_request(
            runtime_dir=runtime_dir,
            robot_id=robot_id,
            cmd='confirm',
            instance_id=args.instance_id,
            token=args.token,
            timeout_sec=args.timeout,
        )
        _print(out)
        return 0 if out.get('ok') else 1

    # Offline exclusive recovery writer.
    g = RobotInstanceGuard(
        robot_id=robot_id,
        runtime_dir=runtime_dir,
        instance_id=args.instance_id,
    )
    ok, msg = g.acquire_flock()
    if not ok:
        _print({'ok': False, 'message': f'offline confirm refused: {msg}'})
        return 1
    interrupted: list[str] = []
    try:
        tok_ok, tok_msg = g.validate_and_consume_token(
            instance_id=args.instance_id,
            token=args.token,
            allow_persisted=True,
        )
        if not tok_ok:
            _print({'ok': False, 'message': tok_msg, 'interrupted': []})
            return 1
        take_ok, take_msg, take_audit = g.takeover_stale_mark_for_recovery()
        if not take_ok:
            _print({
                'ok': False,
                'message': f'takeover failed: {take_msg}',
                'interrupted': [],
                'takeover': take_audit,
            })
            return 1
        if args.archive_db:
            arch = MissionArchive(Path(args.archive_db))
            try:
                interrupted, err = arch.interrupt_running_missions(
                    reason=(
                        f'offline CLI confirm instance_id={args.instance_id} '
                        '(exclusive flock held)'
                    ),
                )
                if err:
                    _print({'ok': False, 'message': err, 'interrupted': interrupted})
                    return 1
            finally:
                arch.close()
        clear_ok, clear_msg = g.clear_mark_if_owner(instance_id=args.instance_id)
        if not clear_ok:
            _print({
                'ok': False,
                'message': f'token ok but clear failed: {clear_msg}',
                'interrupted': interrupted,
                'takeover': take_audit,
            })
            return 1
        _print({
            'ok': True,
            'mode': 'offline',
            'message': 'confirmed under exclusive flock',
            'interrupted': interrupted,
            'takeover': take_audit,
        })
        return 0
    finally:
        g.release_flock()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description='Robot safety operator recovery CLI (online→Runtime, offline→flock)',
    )
    parser.add_argument(
        '--runtime-dir',
        default=str(default_runtime_dir()),
        help='Robot-level runtime dir (independent of mission DB)',
    )
    parser.add_argument('--robot-id', default=DEFAULT_ROBOT_ID)
    parser.add_argument(
        '--timeout', type=float, default=5.0,
        help='Seconds to wait for online Runtime response',
    )
    sub = parser.add_subparsers(dest='cmd', required=True)

    p_status = sub.add_parser('status', help='Show live Runtime or offline safety status')
    p_status.set_defaults(func=cmd_status)

    p_arm = sub.add_parser('arm', help='Arm one-shot confirm token on current instance')
    p_arm.set_defaults(func=cmd_arm)

    p_confirm = sub.add_parser('confirm', help='Confirm with instance-bound token')
    p_confirm.add_argument('--instance-id', required=True)
    p_confirm.add_argument('--token', required=True)
    p_confirm.add_argument(
        '--archive-db',
        default='',
        help='Offline only: SQLite path for INTERRUPT RUNNING (requires flock)',
    )
    p_confirm.set_defaults(func=cmd_confirm)

    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == '__main__':
    raise SystemExit(main())
