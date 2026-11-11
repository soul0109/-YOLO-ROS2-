"""Robot-level single-instance flock + motion safety mark (V2-02 / §19).

Contract:
- Fixed per-robot runtime directory; NOT tied to mission SQLite path.
- Runtime holds an exclusive flock FD until process exit.
- Independent safety JSON, atomically replaced (tmp + os.replace).
- Before motion: mark active. Missing/corrupt/last-active → refuse new goals.
- Token arm/consume is separate from clear_mark (R3).
- Clear only after control confirmed + archive commit (+ recovery interrupt).
- Same-user same-host only; not a cross-host safety claim.
"""

from __future__ import annotations

import json
import os
import tempfile
import threading
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

try:
    import fcntl
except ImportError:  # pragma: no cover
    fcntl = None  # type: ignore


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


DEFAULT_ROBOT_ID = 'inspection_sim_01'


def default_runtime_dir() -> Path:
    raw = os.environ.get('INSPECTION_ROBOT_RUNTIME_DIR', '').strip()
    if raw:
        return Path(raw).expanduser()
    return Path.home() / 'inspection-robot' / 'runtime'


@dataclass
class SafetySnapshot:
    ok: bool
    refuse: bool
    reason: str
    payload: dict[str, Any]
    path: Path


class RobotInstanceGuard:
    """Flock + atomic safety JSON for one robot_id on this VM/user."""

    def __init__(
        self,
        *,
        robot_id: str = DEFAULT_ROBOT_ID,
        runtime_dir: Optional[Path] = None,
        instance_id: Optional[str] = None,
    ) -> None:
        self.robot_id = str(robot_id or DEFAULT_ROBOT_ID).strip() or DEFAULT_ROBOT_ID
        self.runtime_dir = Path(runtime_dir) if runtime_dir else default_runtime_dir()
        self.instance_id = instance_id or uuid.uuid4().hex
        self.lock_path = self.runtime_dir / f'robot_{self.robot_id}.lock'
        self.safety_path = self.runtime_dir / f'robot_{self.robot_id}_safety.json'
        self.instance_meta_path = self.runtime_dir / f'robot_{self.robot_id}_instance.json'
        self.armed_token_path = self.runtime_dir / f'robot_{self.robot_id}_armed_token.json'
        self._lock_fd: Optional[int] = None
        self._thread_lock = threading.RLock()
        self._operator_confirm_token: Optional[str] = None
        self.flock_failed = False
        self.flock_fail_reason = ''

    @property
    def held(self) -> bool:
        return self._lock_fd is not None

    def acquire_flock(self) -> tuple[bool, str]:
        """Exclusive non-blocking flock; hold FD until release/exit."""
        with self._thread_lock:
            if self._lock_fd is not None:
                return True, 'already held'
            if fcntl is None:
                self.flock_failed = True
                self.flock_fail_reason = 'fcntl unavailable on this platform'
                return False, self.flock_fail_reason
            fd = None
            try:
                self.runtime_dir.mkdir(parents=True, exist_ok=True)
                fd = os.open(str(self.lock_path), os.O_RDWR | os.O_CREAT, 0o644)
                try:
                    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    os.close(fd)
                    self.flock_failed = True
                    self.flock_fail_reason = f'robot flock busy: {self.lock_path}'
                    return False, self.flock_fail_reason
                os.ftruncate(fd, 0)
                os.write(
                    fd,
                    (
                        f'pid={os.getpid()} instance_id={self.instance_id} '
                        f'robot_id={self.robot_id}\n'
                    ).encode('utf-8'),
                )
                os.fsync(fd)
                self._lock_fd = fd
                self.flock_failed = False
                self.flock_fail_reason = ''
                return True, 'acquired'
            except OSError as exc:
                if fd is not None:
                    try:
                        os.close(fd)
                    except OSError:
                        pass
                self.flock_failed = True
                self.flock_fail_reason = f'flock open failed: {exc}'
                return False, self.flock_fail_reason

    def release_flock(self) -> None:
        with self._thread_lock:
            fd = self._lock_fd
            self._lock_fd = None
            if fd is None:
                return
            try:
                if fcntl is not None:
                    fcntl.flock(fd, fcntl.LOCK_UN)
            except OSError:
                pass
            try:
                os.close(fd)
            except OSError:
                pass

    def write_instance_meta(self) -> None:
        """Publish current live instance identity (holder only)."""
        if not self.held:
            return
        self._write_json_atomic(self.instance_meta_path, {
            'robot_id': self.robot_id,
            'instance_id': self.instance_id,
            'pid': os.getpid(),
            'updated_at': utc_now_iso(),
        })

    def read_instance_meta(self) -> dict[str, Any]:
        if not self.instance_meta_path.is_file():
            return {}
        try:
            data = json.loads(self.instance_meta_path.read_text(encoding='utf-8'))
            return data if isinstance(data, dict) else {}
        except Exception:
            return {}

    def read_safety(self) -> SafetySnapshot:
        path = self.safety_path
        if not path.is_file():
            return SafetySnapshot(
                ok=False, refuse=True, reason='safety mark missing',
                payload={}, path=path,
            )
        try:
            data = json.loads(path.read_text(encoding='utf-8'))
            if not isinstance(data, dict):
                raise ValueError('safety root must be object')
        except Exception as exc:  # noqa: BLE001
            return SafetySnapshot(
                ok=False, refuse=True, reason=f'safety mark corrupt: {exc}',
                payload={}, path=path,
            )
        state = str(data.get('state') or '').strip().lower()
        if state == 'clear':
            return SafetySnapshot(
                ok=True, refuse=False, reason='clear', payload=data, path=path,
            )
        if state in ('', 'active', 'refuse'):
            reason = {
                'active': 'last run left active mark',
                'refuse': 'refuse mark set',
                '': 'empty state',
            }.get(state, f'state={state}')
            return SafetySnapshot(
                ok=True, refuse=True, reason=reason, payload=data, path=path,
            )
        return SafetySnapshot(
            ok=False, refuse=True,
            reason=f'unsupported safety state={state}',
            payload=data, path=path,
        )

    def _write_json_atomic(self, path: Path, payload: dict[str, Any]) -> tuple[bool, str]:
        with self._thread_lock:
            try:
                self.runtime_dir.mkdir(parents=True, exist_ok=True)
                text = json.dumps(payload, indent=2, ensure_ascii=False) + '\n'
                fd, tmp_name = tempfile.mkstemp(
                    prefix=f'.{path.name}.', dir=str(self.runtime_dir),
                )
                try:
                    with os.fdopen(fd, 'w', encoding='utf-8') as fh:
                        fh.write(text)
                        fh.flush()
                        os.fsync(fh.fileno())
                    os.replace(tmp_name, path)
                    dir_fd = os.open(str(self.runtime_dir), os.O_RDONLY)
                    try:
                        os.fsync(dir_fd)
                    finally:
                        os.close(dir_fd)
                except Exception:
                    try:
                        os.unlink(tmp_name)
                    except OSError:
                        pass
                    raise
                return True, 'written'
            except Exception as exc:  # noqa: BLE001
                return False, f'write failed: {exc}'

    def write_safety_atomic(self, payload: dict[str, Any]) -> tuple[bool, str]:
        return self._write_json_atomic(self.safety_path, payload)

    def mark_active(self, *, mission_id: str) -> tuple[bool, str]:
        return self.write_safety_atomic({
            'schema': 'v2_02_robot_safety_v1',
            'robot_id': self.robot_id,
            'instance_id': self.instance_id,
            'state': 'active',
            'mission_id': mission_id,
            'updated_at': utc_now_iso(),
        })

    def mark_refuse(self, *, reason: str, mission_id: str = '') -> tuple[bool, str]:
        return self.write_safety_atomic({
            'schema': 'v2_02_robot_safety_v1',
            'robot_id': self.robot_id,
            'instance_id': self.instance_id,
            'state': 'refuse',
            'reason': reason,
            'mission_id': mission_id,
            'updated_at': utc_now_iso(),
        })

    def clear_mark(self) -> tuple[bool, str]:
        return self.write_safety_atomic({
            'schema': 'v2_02_robot_safety_v1',
            'robot_id': self.robot_id,
            'instance_id': self.instance_id,
            'state': 'clear',
            'updated_at': utc_now_iso(),
        })

    def clear_mark_if_owner(
        self,
        *,
        instance_id: str,
        mission_id: str = '',
    ) -> tuple[bool, str]:
        """Clear only when caller instance matches mark owner (or mark empty).

        Normal seal / post-takeover clear path. Does NOT adopt foreign owners.
        """
        if instance_id != self.instance_id:
            return False, (
                f'clear refused: caller instance={instance_id} '
                f'guard={self.instance_id}'
            )
        snap = self.read_safety()
        marked_instance = str((snap.payload or {}).get('instance_id') or '')
        marked_mission = str((snap.payload or {}).get('mission_id') or '')
        if marked_instance and marked_instance != instance_id:
            return False, (
                f'clear refused: safety owned by instance={marked_instance}'
            )
        if mission_id and marked_mission and marked_mission != mission_id:
            return False, (
                f'clear refused: safety mission={marked_mission} '
                f'!= {mission_id}'
            )
        return self.clear_mark()

    def takeover_stale_mark_for_recovery(self) -> tuple[bool, str, dict[str, Any]]:
        """Explicit operator recovery takeover of a foreign/stale safety mark.

        Preconditions (enforced here):
        - this process holds the robot flock (old Runtime is gone / offline exclusive)
        Caller must already have: recovery claim + consumed current-instance token.

        Behavior:
        - Missing mark / already owned by self → success, no rewrite needed
        - Foreign owner active/refuse → rewrite as refuse owned by *this* instance,
          preserving previous owner/mission/reason/updated_at under ``takeover_of``
        - Never clears here; caller still interrupt-archive → clear_mark_if_owner
        - Does not relax ``clear_mark_if_owner`` for normal seals
        """
        if not self.held:
            return False, 'takeover refused: flock not held', {}
        snap = self.read_safety()
        payload = dict(snap.payload or {})
        marked_instance = str(payload.get('instance_id') or '')
        if not marked_instance or marked_instance == self.instance_id:
            return True, 'takeover not required', {
                'needed': False,
                'instance_id': self.instance_id,
                'marked_instance': marked_instance,
            }
        audit = {
            'previous_instance_id': marked_instance,
            'previous_mission_id': str(payload.get('mission_id') or ''),
            'previous_state': str(payload.get('state') or ''),
            'previous_reason': str(payload.get('reason') or ''),
            'previous_updated_at': str(payload.get('updated_at') or ''),
            'takeover_instance_id': self.instance_id,
            'takeover_at': utc_now_iso(),
        }
        ok, msg = self.write_safety_atomic({
            'schema': 'v2_02_robot_safety_v1',
            'robot_id': self.robot_id,
            'instance_id': self.instance_id,
            'state': 'refuse',
            'reason': (
                f'operator recovery takeover from instance={marked_instance}'
            ),
            'mission_id': str(payload.get('mission_id') or ''),
            'takeover_of': audit,
            'updated_at': utc_now_iso(),
        })
        if not ok:
            return False, f'takeover write failed: {msg}', audit
        return True, 'took over stale mark as refuse', {
            'needed': True,
            **audit,
        }

    def bootstrap_clear_if_missing(self) -> tuple[bool, str]:
        """First-use helper: write clear when no safety file exists.

        Only for missing file. Active/refuse/corrupt marks still require
        operator confirm (+ takeover when foreign). Requires flock held.
        """
        if not self.held:
            return False, 'bootstrap refused: flock not held'
        if self.safety_path.is_file():
            return False, 'bootstrap skipped: safety mark already present'
        return self.clear_mark()

    def startup_gate(self) -> tuple[bool, str, SafetySnapshot]:
        snap = self.read_safety()
        if not snap.ok or snap.refuse:
            return False, snap.reason, snap
        return True, 'clear', snap

    def arm_operator_confirm(self, *, persist: bool = False) -> str:
        """Create one-shot token bound to this instance_id. Never accepts user token."""
        token = uuid.uuid4().hex
        self._operator_confirm_token = token
        if persist:
            self._write_json_atomic(self.armed_token_path, {
                'instance_id': self.instance_id,
                'token': token,
                'updated_at': utc_now_iso(),
            })
        return token

    def validate_and_consume_token(
        self,
        *,
        instance_id: str,
        token: str,
        allow_persisted: bool = False,
    ) -> tuple[bool, str]:
        """Validate + one-shot consume token WITHOUT clearing safety mark.

        Never treats caller-supplied token as the expected value.
        """
        if instance_id != self.instance_id:
            return False, (
                f'instance_id mismatch: got={instance_id} '
                f'current={self.instance_id}'
            )
        expected = self._operator_confirm_token
        if expected is None and allow_persisted and self.armed_token_path.is_file():
            try:
                data = json.loads(self.armed_token_path.read_text(encoding='utf-8'))
                if (
                    isinstance(data, dict)
                    and data.get('instance_id') == instance_id
                    and data.get('token')
                ):
                    expected = str(data['token'])
                    self._operator_confirm_token = expected
            except Exception:
                expected = None
        if not expected or token != expected:
            return False, 'confirm token invalid or already consumed'
        # Consume in memory and delete persisted arm file.
        self._operator_confirm_token = None
        if self.armed_token_path.is_file():
            try:
                self.armed_token_path.unlink()
            except OSError:
                pass
        return True, 'token consumed'

    def close(self) -> None:
        try:
            if self.instance_meta_path.is_file() and self.held:
                self.instance_meta_path.unlink()
        except OSError:
            pass
        self.release_flock()
