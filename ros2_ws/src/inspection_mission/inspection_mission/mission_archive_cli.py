#!/usr/bin/env python3
"""Read-only CLI: query archived mission + station artifacts by mission_id.

Uses MissionArchiveReader (mode=ro). Does not create/migrate/write the DB.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from inspection_mission.mission_archive import MissionArchiveReader

DEFAULT_DB = Path.home() / 'inspection-robot' / 'bags' / 'inspection_archive.sqlite3'


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--db', type=Path, default=DEFAULT_DB, help='SQLite path')
    ap.add_argument('--mission-id', required=True)
    ap.add_argument('--json', action='store_true', help='print raw JSON bundle')
    args = ap.parse_args(argv)

    if not args.db.is_file():
        print(f'ERROR: db not found: {args.db}', file=sys.stderr)
        return 2

    try:
        arch = MissionArchiveReader(args.db)
    except FileNotFoundError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 2

    try:
        bundle = arch.query_mission_bundle(args.mission_id)
    finally:
        arch.close()

    if bundle is None:
        print(f'NOT_FOUND mission_id={args.mission_id} db={args.db}')
        return 1

    if args.json:
        print(json.dumps(bundle, indent=2, ensure_ascii=False))
        return 0

    m = bundle['mission']
    print(f"db: {bundle['db_path']}")
    print('--- mission ---')
    for k in (
        'mission_id', 'route_id', 'execution_status', 'error_code',
        'message', 'started_at', 'ended_at',
    ):
        print(f'  {k}: {m.get(k)}')
    print(f"--- stations ({len(bundle['stations'])}) ---")
    for s in bundle['stations']:
        print(f"  station_id: {s.get('station_id')}")
        print(f"    inspection_status: {s.get('inspection_status')}")
        print(f"    image_stamp: {s.get('image_stamp')}")
        print(f"    model_version: {s.get('model_version')}")
        print(f"    roi_rules_version: {s.get('roi_rules_version')}")
        print(f"    screenshot_path: {s.get('screenshot_path')}")
        print(f"    annotated_path: {s.get('annotated_path')}")
        print(f"    station_result_path: {s.get('station_result_path')}")
        objs = s.get('objects')
        if isinstance(objs, dict):
            for name, payload in objs.items():
                if isinstance(payload, dict):
                    print(
                        f"    object[{name}]: verdict={payload.get('verdict')} "
                        f"state={payload.get('state') or payload.get('model_states')}",
                    )
                else:
                    print(f'    object[{name}]: {payload}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
