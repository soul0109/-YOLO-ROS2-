"""MVP-R4/R5: Runtime-owned SQLite archive for missions + station results.

Writer: MissionArchive (Runtime sole writer; creates schema).
Reader: MissionArchiveReader (CLI / FastAPI; mode=ro, no migrate, no write).
Does not recompute vision — stores paths and objects_json from existing artifacts.
"""

from __future__ import annotations

import json
import sqlite3
import threading
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional


SCHEMA_VERSION = 1

_SCHEMA_SQL = """
CREATE TABLE IF NOT EXISTS schema_meta (
  key TEXT PRIMARY KEY,
  value TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS missions (
  mission_id TEXT PRIMARY KEY,
  route_id TEXT NOT NULL,
  execution_status TEXT NOT NULL,
  error_code TEXT NOT NULL,
  message TEXT NOT NULL DEFAULT '',
  started_at TEXT NOT NULL,
  ended_at TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS stations (
  id INTEGER PRIMARY KEY AUTOINCREMENT,
  mission_id TEXT NOT NULL REFERENCES missions(mission_id),
  station_id TEXT NOT NULL,
  inspection_status TEXT NOT NULL DEFAULT '',
  image_stamp TEXT NOT NULL DEFAULT '',
  screenshot_path TEXT NOT NULL DEFAULT '',
  annotated_path TEXT NOT NULL DEFAULT '',
  station_result_path TEXT NOT NULL DEFAULT '',
  model_version TEXT NOT NULL DEFAULT '',
  roi_rules_version TEXT NOT NULL DEFAULT '',
  objects_json TEXT NOT NULL DEFAULT '',
  UNIQUE(mission_id, station_id)
);
"""


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def _row_to_mission(row: sqlite3.Row) -> dict[str, Any]:
    return dict(row)


def _row_to_station(row: sqlite3.Row) -> dict[str, Any]:
    d = dict(row)
    raw = d.get('objects_json') or ''
    if raw:
        try:
            d['objects'] = json.loads(raw)
        except json.JSONDecodeError:
            d['objects'] = raw
    else:
        d['objects'] = None
    return d


@dataclass
class ArchiveWriteResult:
    ok: bool
    error_code: str = 'NONE'
    message: str = ''


class MissionArchiveReader:
    """Read-only archive access. Never creates DB/files, never migrates, never writes."""

    def __init__(self, db_path: Path) -> None:
        self.db_path = Path(db_path).expanduser()
        if not self.db_path.is_file():
            raise FileNotFoundError(f'archive db not found: {self.db_path}')
        uri = f'file:{self.db_path.resolve().as_posix()}?mode=ro'
        self._lock = threading.RLock()
        self._conn = sqlite3.connect(
            uri,
            uri=True,
            check_same_thread=False,
        )
        self._conn.row_factory = sqlite3.Row
        # Defensive: refuse writes even if a caller issues INSERT by mistake.
        self._conn.execute('PRAGMA query_only = ON')

    def close(self) -> None:
        with self._lock:
            self._conn.close()

    def list_mission_ids(self) -> set[str]:
        with self._lock:
            rows = self._conn.execute('SELECT mission_id FROM missions').fetchall()
            return {str(r['mission_id']) for r in rows}

    def has_mission(self, mission_id: str) -> bool:
        with self._lock:
            row = self._conn.execute(
                'SELECT 1 FROM missions WHERE mission_id=?',
                (mission_id,),
            ).fetchone()
            return row is not None

    def get_mission(self, mission_id: str) -> Optional[dict[str, Any]]:
        with self._lock:
            row = self._conn.execute(
                'SELECT * FROM missions WHERE mission_id=?',
                (mission_id,),
            ).fetchone()
            if row is None:
                return None
            return _row_to_mission(row)

    def get_stations(self, mission_id: str) -> list[dict[str, Any]]:
        with self._lock:
            rows = self._conn.execute(
                'SELECT * FROM stations WHERE mission_id=? ORDER BY id',
                (mission_id,),
            ).fetchall()
            return [_row_to_station(r) for r in rows]

    def query_mission_bundle(self, mission_id: str) -> Optional[dict[str, Any]]:
        mission = self.get_mission(mission_id)
        if mission is None:
            return None
        return {
            'mission': mission,
            'stations': self.get_stations(mission_id),
            'db_path': str(self.db_path),
        }

    def list_recent_missions(self, limit: int = 20) -> list[dict[str, Any]]:
        limit = max(1, min(int(limit), 200))
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT * FROM missions
                ORDER BY ended_at DESC, mission_id DESC
                LIMIT ?
                """,
                (limit,),
            ).fetchall()
            return [_row_to_mission(r) for r in rows]

    def get_station(
        self, mission_id: str, station_id: str,
    ) -> Optional[dict[str, Any]]:
        with self._lock:
            row = self._conn.execute(
                """
                SELECT * FROM stations
                WHERE mission_id=? AND station_id=?
                """,
                (mission_id, station_id),
            ).fetchone()
            if row is None:
                return None
            return _row_to_station(row)

    def resolve_station_image_path(
        self,
        mission_id: str,
        station_id: str,
        kind: str,
    ) -> Optional[Path]:
        """Map mission/station + kind → archived file path from DB (no arbitrary paths)."""
        station = self.get_station(mission_id, station_id)
        if station is None:
            return None
        if kind == 'screenshot':
            raw = station.get('screenshot_path') or ''
        elif kind == 'annotated':
            raw = station.get('annotated_path') or ''
        else:
            raise ValueError(f'unsupported image kind: {kind}')
        if not raw:
            return None
        path = Path(raw)
        if not path.is_file():
            return None
        return path


class MissionArchive:
    """Process-local SQLite store (Runtime writer). Thread-safe via lock."""

    def __init__(self, db_path: Path) -> None:
        self.db_path = Path(db_path).expanduser()
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._conn = sqlite3.connect(
            str(self.db_path),
            check_same_thread=False,
            isolation_level=None,
        )
        self._conn.row_factory = sqlite3.Row
        self._conn.execute('PRAGMA foreign_keys = ON')
        self._init_schema()

    def close(self) -> None:
        with self._lock:
            self._conn.close()

    def _init_schema(self) -> None:
        with self._lock:
            self._conn.executescript(_SCHEMA_SQL)
            cur = self._conn.execute(
                "SELECT value FROM schema_meta WHERE key='schema_version'",
            )
            row = cur.fetchone()
            if row is None:
                self._conn.execute(
                    "INSERT INTO schema_meta(key, value) VALUES('schema_version', ?)",
                    (str(SCHEMA_VERSION),),
                )
            self._conn.commit()

    def list_mission_ids(self) -> set[str]:
        with self._lock:
            rows = self._conn.execute('SELECT mission_id FROM missions').fetchall()
            return {str(r['mission_id']) for r in rows}

    def has_mission(self, mission_id: str) -> bool:
        with self._lock:
            row = self._conn.execute(
                'SELECT 1 FROM missions WHERE mission_id=?',
                (mission_id,),
            ).fetchone()
            return row is not None

    def begin_mission(
        self,
        *,
        mission_id: str,
        route_id: str,
        started_at: str,
    ) -> ArchiveWriteResult:
        """Insert mission shell (RUNNING) so stations can be appended safely."""
        mission_id = str(mission_id).strip()
        if not mission_id:
            return ArchiveWriteResult(False, 'archive_invalid', 'empty mission_id')
        with self._lock:
            try:
                self._conn.execute('BEGIN IMMEDIATE')
                if self._conn.execute(
                    'SELECT 1 FROM missions WHERE mission_id=?', (mission_id,),
                ).fetchone():
                    self._conn.execute('ROLLBACK')
                    return ArchiveWriteResult(
                        False,
                        'archive_duplicate_mission',
                        f"mission_id '{mission_id}' already archived",
                    )
                self._conn.execute(
                    """
                    INSERT INTO missions(
                      mission_id, route_id, execution_status, error_code, message,
                      started_at, ended_at
                    ) VALUES (?,?,?,?,?,?,?)
                    """,
                    (
                        mission_id,
                        str(route_id),
                        'RUNNING',
                        'NONE',
                        '',
                        str(started_at),
                        '',
                    ),
                )
                self._conn.execute('COMMIT')
            except Exception as exc:
                try:
                    self._conn.execute('ROLLBACK')
                except Exception:
                    pass
                return ArchiveWriteResult(
                    False, 'archive_write_failed', f'SQLite write failed: {exc}',
                )
        return ArchiveWriteResult(True, 'NONE', 'mission begun')

    def _station_insp_ok(self, station: dict[str, Any]) -> tuple[bool, str, str]:
        insp = str(station.get('inspection_status') or '')
        if insp not in ('', 'PASS', 'FAIL', 'UNKNOWN'):
            return False, 'archive_invalid', f'illegal inspection_status={insp!r}'
        return True, insp, ''

    def _objects_json(self, station: dict[str, Any]) -> str:
        objects = station.get('objects')
        if objects is None or objects == '':
            return ''
        if isinstance(objects, str):
            return objects
        return json.dumps(objects, ensure_ascii=False)

    def persist_station(
        self,
        *,
        mission_id: str,
        station: dict[str, Any],
    ) -> ArchiveWriteResult:
        """Append one station row. Requires mission row; duplicate (mission,station) → error."""
        mission_id = str(mission_id).strip()
        if not mission_id:
            return ArchiveWriteResult(False, 'archive_invalid', 'empty mission_id')
        sid = str(station.get('station_id') or '').strip()
        if not sid:
            return ArchiveWriteResult(False, 'archive_invalid', 'empty station_id')
        ok, insp, err = self._station_insp_ok(station)
        if not ok:
            return ArchiveWriteResult(False, 'archive_invalid', err)

        with self._lock:
            try:
                self._conn.execute('BEGIN IMMEDIATE')
                if not self._conn.execute(
                    'SELECT 1 FROM missions WHERE mission_id=?', (mission_id,),
                ).fetchone():
                    self._conn.execute('ROLLBACK')
                    return ArchiveWriteResult(
                        False, 'archive_missing_mission',
                        f"mission_id '{mission_id}' not begun",
                    )
                if self._conn.execute(
                    'SELECT 1 FROM stations WHERE mission_id=? AND station_id=?',
                    (mission_id, sid),
                ).fetchone():
                    self._conn.execute('ROLLBACK')
                    return ArchiveWriteResult(
                        False, 'archive_duplicate_station',
                        f"station '{sid}' already archived for '{mission_id}'",
                    )
                self._conn.execute(
                    """
                    INSERT INTO stations(
                      mission_id, station_id, inspection_status, image_stamp,
                      screenshot_path, annotated_path, station_result_path,
                      model_version, roi_rules_version, objects_json
                    ) VALUES (?,?,?,?,?,?,?,?,?,?)
                    """,
                    (
                        mission_id,
                        sid,
                        insp,
                        str(station.get('image_stamp') or ''),
                        str(station.get('screenshot_path') or ''),
                        str(station.get('annotated_path') or ''),
                        str(station.get('station_result_path') or ''),
                        str(station.get('model_version') or ''),
                        str(station.get('roi_rules_version') or ''),
                        self._objects_json(station),
                    ),
                )
                self._conn.execute('COMMIT')
            except Exception as exc:
                try:
                    self._conn.execute('ROLLBACK')
                except Exception:
                    pass
                return ArchiveWriteResult(
                    False, 'archive_write_failed', f'SQLite write failed: {exc}',
                )
        return ArchiveWriteResult(True, 'NONE', 'station archived')

    def finalize_mission(
        self,
        *,
        mission_id: str,
        execution_status: str,
        error_code: str,
        message: str,
        ended_at: str,
    ) -> ArchiveWriteResult:
        """Update mission terminal fields (after begin_mission)."""
        mission_id = str(mission_id).strip()
        with self._lock:
            try:
                self._conn.execute('BEGIN IMMEDIATE')
                cur = self._conn.execute(
                    """
                    UPDATE missions
                    SET execution_status=?, error_code=?, message=?, ended_at=?
                    WHERE mission_id=?
                    """,
                    (
                        str(execution_status),
                        str(error_code),
                        str(message or ''),
                        str(ended_at),
                        mission_id,
                    ),
                )
                if cur.rowcount != 1:
                    self._conn.execute('ROLLBACK')
                    return ArchiveWriteResult(
                        False, 'archive_missing_mission',
                        f"mission_id '{mission_id}' not found for finalize",
                    )
                self._conn.execute('COMMIT')
            except Exception as exc:
                try:
                    self._conn.execute('ROLLBACK')
                except Exception:
                    pass
                return ArchiveWriteResult(
                    False, 'archive_write_failed', f'SQLite write failed: {exc}',
                )
        return ArchiveWriteResult(True, 'NONE', 'mission finalized')

    def list_running_mission_ids(self) -> list[str]:
        with self._lock:
            rows = self._conn.execute(
                "SELECT mission_id FROM missions WHERE execution_status='RUNNING' "
                'ORDER BY started_at ASC',
            ).fetchall()
            return [str(r['mission_id']) for r in rows]

    def interrupt_running_missions(
        self,
        *,
        reason: str,
        ended_at: Optional[str] = None,
    ) -> tuple[list[str], Optional[str]]:
        """Mark RUNNING shells INTERRUPTED. Keeps stations/timestamps; never deletes.

        Returns (interrupted_ids, error_message_or_none).
        """
        ended = ended_at or utc_now_iso()
        msg = str(reason or 'operator confirmed interrupt after unsafe/refuse restart')
        with self._lock:
            try:
                self._conn.execute('BEGIN IMMEDIATE')
                rows = self._conn.execute(
                    "SELECT mission_id, message FROM missions "
                    "WHERE execution_status='RUNNING'",
                ).fetchall()
                ids: list[str] = []
                for row in rows:
                    mid = str(row['mission_id'])
                    prior = str(row['message'] or '')
                    combined = msg if not prior else f'{prior}; {msg}'
                    self._conn.execute(
                        """
                        UPDATE missions
                        SET execution_status=?, error_code=?, message=?, ended_at=?
                        WHERE mission_id=? AND execution_status='RUNNING'
                        """,
                        ('INTERRUPTED', 'operator_interrupt', combined, ended, mid),
                    )
                    ids.append(mid)
                self._conn.execute('COMMIT')
                return ids, None
            except Exception as exc:
                try:
                    self._conn.execute('ROLLBACK')
                except Exception:
                    pass
                return [], f'interrupt_running failed: {exc}'

    def persist_mission(
        self,
        *,
        mission_id: str,
        route_id: str,
        execution_status: str,
        error_code: str,
        message: str,
        started_at: str,
        ended_at: str,
        station: Optional[dict[str, Any]] = None,
    ) -> ArchiveWriteResult:
        """Insert one mission (+ optional one station). Duplicate mission_id → error.

        Kept for R4/R5 single-shot callers/tests. Multi-station R7 uses
        begin_mission + persist_station* + finalize_mission.
        """
        mission_id = str(mission_id).strip()
        if not mission_id:
            return ArchiveWriteResult(False, 'archive_invalid', 'empty mission_id')

        insp = ''
        if station is not None:
            ok, insp, err = self._station_insp_ok(station)
            if not ok:
                return ArchiveWriteResult(False, 'archive_invalid', err)

        with self._lock:
            try:
                self._conn.execute('BEGIN IMMEDIATE')
                if self._conn.execute(
                    'SELECT 1 FROM missions WHERE mission_id=?', (mission_id,),
                ).fetchone():
                    self._conn.execute('ROLLBACK')
                    return ArchiveWriteResult(
                        False,
                        'archive_duplicate_mission',
                        f"mission_id '{mission_id}' already archived",
                    )
                self._conn.execute(
                    """
                    INSERT INTO missions(
                      mission_id, route_id, execution_status, error_code, message,
                      started_at, ended_at
                    ) VALUES (?,?,?,?,?,?,?)
                    """,
                    (
                        mission_id,
                        str(route_id),
                        str(execution_status),
                        str(error_code),
                        str(message or ''),
                        str(started_at),
                        str(ended_at),
                    ),
                )
                if station is not None and station.get('station_id'):
                    self._conn.execute(
                        """
                        INSERT INTO stations(
                          mission_id, station_id, inspection_status, image_stamp,
                          screenshot_path, annotated_path, station_result_path,
                          model_version, roi_rules_version, objects_json
                        ) VALUES (?,?,?,?,?,?,?,?,?,?)
                        """,
                        (
                            mission_id,
                            str(station['station_id']),
                            insp,
                            str(station.get('image_stamp') or ''),
                            str(station.get('screenshot_path') or ''),
                            str(station.get('annotated_path') or ''),
                            str(station.get('station_result_path') or ''),
                            str(station.get('model_version') or ''),
                            str(station.get('roi_rules_version') or ''),
                            self._objects_json(station),
                        ),
                    )
                self._conn.execute('COMMIT')
            except Exception as exc:
                try:
                    self._conn.execute('ROLLBACK')
                except Exception:
                    pass
                return ArchiveWriteResult(
                    False, 'archive_write_failed', f'SQLite write failed: {exc}',
                )
        return ArchiveWriteResult(True, 'NONE', 'archived')

    def get_mission(self, mission_id: str) -> Optional[dict[str, Any]]:
        with self._lock:
            row = self._conn.execute(
                'SELECT * FROM missions WHERE mission_id=?',
                (mission_id,),
            ).fetchone()
            if row is None:
                return None
            return _row_to_mission(row)

    def get_stations(self, mission_id: str) -> list[dict[str, Any]]:
        with self._lock:
            rows = self._conn.execute(
                'SELECT * FROM stations WHERE mission_id=? ORDER BY id',
                (mission_id,),
            ).fetchall()
            return [_row_to_station(r) for r in rows]

    def query_mission_bundle(self, mission_id: str) -> Optional[dict[str, Any]]:
        mission = self.get_mission(mission_id)
        if mission is None:
            return None
        return {
            'mission': mission,
            'stations': self.get_stations(mission_id),
            'db_path': str(self.db_path),
        }


def load_station_payload_from_result_json(path: Path) -> dict[str, Any]:
    """Read existing R3 station_result JSON without recomputing vision."""
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    return {
        'station_id': str(data.get('station_id') or ''),
        'inspection_status': str(data.get('inspection_status') or ''),
        'image_stamp': str(data.get('image_stamp') or ''),
        'screenshot_path': str(data.get('screenshot_path') or ''),
        'annotated_path': str(data.get('annotated_path') or ''),
        'station_result_path': str(path),
        'model_version': str(data.get('model_version') or ''),
        'roi_rules_version': str(data.get('roi_rules_version') or ''),
        'objects': data.get('objects') or {},
    }
