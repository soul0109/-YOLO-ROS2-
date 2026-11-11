"""File-based recovery IPC between CLI and live Mission Runtime (§19 R1).

Online path: CLI writes recover_req.json; Runtime timer processes with its
in-memory armed token / archive writer; writes recover_resp.json.

Offline path: CLI must acquire the same robot flock; only then may it arm/
confirm/interrupt as the exclusive writer. Holding flock failure → refuse.
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Any, Callable, Optional

from inspection_mission.robot_safety import DEFAULT_ROBOT_ID, RobotInstanceGuard


def req_path(runtime_dir: Path, robot_id: str) -> Path:
    return Path(runtime_dir) / f'robot_{robot_id}_recover_req.json'


def resp_path(runtime_dir: Path, robot_id: str) -> Path:
    return Path(runtime_dir) / f'robot_{robot_id}_recover_resp.json'


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    os.replace(tmp, path)


def read_json(path: Path) -> Optional[dict[str, Any]]:
    if not path.is_file():
        return None
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
        return data if isinstance(data, dict) else None
    except Exception:
        return None


def live_runtime_present(guard: RobotInstanceGuard) -> bool:
    """True only when another holder currently owns the robot flock.

    Stale instance_meta alone is not enough (offline arm must not look "live").
    """
    meta = guard.read_instance_meta()
    if not meta.get('instance_id'):
        return False
    probe = RobotInstanceGuard(
        robot_id=guard.robot_id,
        runtime_dir=guard.runtime_dir,
    )
    ok, _ = probe.acquire_flock()
    if ok:
        # Nobody holds the lock → no live Runtime writer.
        probe.release_flock()
        return False
    return True


def wait_response(
    runtime_dir: Path,
    robot_id: str,
    *,
    request_id: str,
    timeout_sec: float = 5.0,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_sec
    path = resp_path(runtime_dir, robot_id)
    while time.monotonic() < deadline:
        data = read_json(path)
        if data and data.get('request_id') == request_id:
            return data
        time.sleep(0.05)
    return {
        'ok': False,
        'request_id': request_id,
        'message': 'timeout waiting for Runtime recover response',
    }


class RecoveryRequestProcessor:
    """Runtime-side processor: bind to live node methods."""

    def __init__(
        self,
        *,
        guard: RobotInstanceGuard,
        handle_status: Callable[[], dict[str, Any]],
        handle_arm: Callable[[], dict[str, Any]],
        handle_confirm: Callable[[str, str], dict[str, Any]],
    ) -> None:
        self.guard = guard
        self.handle_status = handle_status
        self.handle_arm = handle_arm
        self.handle_confirm = handle_confirm

    def poll_once(self) -> bool:
        path = req_path(self.guard.runtime_dir, self.guard.robot_id)
        data = read_json(path)
        if not data:
            return False
        request_id = str(data.get('request_id') or '')
        cmd = str(data.get('cmd') or '')
        try:
            path.unlink()
        except OSError:
            pass
        if cmd == 'status':
            out = self.handle_status()
        elif cmd == 'arm':
            out = self.handle_arm()
        elif cmd == 'confirm':
            out = self.handle_confirm(
                str(data.get('instance_id') or ''),
                str(data.get('token') or ''),
            )
        else:
            out = {'ok': False, 'message': f'unknown cmd={cmd}'}
        out = dict(out)
        out['request_id'] = request_id
        write_json(resp_path(self.guard.runtime_dir, self.guard.robot_id), out)
        return True


def submit_online_request(
    *,
    runtime_dir: Path,
    robot_id: str,
    cmd: str,
    instance_id: str = '',
    token: str = '',
    timeout_sec: float = 5.0,
) -> dict[str, Any]:
    import uuid
    request_id = uuid.uuid4().hex
    # Drop stale response
    rp = resp_path(runtime_dir, robot_id)
    if rp.is_file():
        try:
            rp.unlink()
        except OSError:
            pass
    write_json(req_path(runtime_dir, robot_id), {
        'request_id': request_id,
        'cmd': cmd,
        'instance_id': instance_id,
        'token': token,
        'pid': os.getpid(),
    })
    return wait_response(
        runtime_dir, robot_id, request_id=request_id, timeout_sec=timeout_sec,
    )
