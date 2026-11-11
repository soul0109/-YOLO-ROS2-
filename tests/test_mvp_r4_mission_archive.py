#!/usr/bin/env python3
"""MVP-R4 offline tests for SQLite mission archive (no Gazebo)."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'ros2_ws' / 'src' / 'inspection_mission'))

from inspection_mission.mission_archive import (  # noqa: E402
    MissionArchive,
    MissionArchiveReader,
    load_station_payload_from_result_json,
)
from inspection_mission.mission_state import (  # noqa: E402
    ErrorCode,
    MissionRuntimeState,
)


class TestMissionArchive(unittest.TestCase):
    def test_persist_and_query_completed_fail(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            db = Path(td) / 'a.sqlite3'
            arch = MissionArchive(db)
            wr = arch.persist_mission(
                mission_id='m1',
                route_id='test_room_A',
                execution_status='COMPLETED',
                error_code='NONE',
                message='ok',
                started_at='2026-11-09T08:00:00+00:00',
                ended_at='2026-11-09T08:01:00+00:00',
                station={
                    'station_id': 'A',
                    'inspection_status': 'FAIL',
                    'image_stamp': '1.0',
                    'screenshot_path': '/tmp/a.png',
                    'annotated_path': '/tmp/a_ann.png',
                    'station_result_path': '/tmp/a.json',
                    'model_version': 'train1_dev:best.pt:deadbeef',
                    'roi_rules_version': 'roi_rules_v1',
                    'objects': {
                        'leds': {'verdict': 'FAIL', 'model_states': ['green', 'green', 'off']},
                        'panel': {'verdict': 'FAIL', 'state': 'dark'},
                        'leak': {'verdict': 'PASS', 'state': 'none'},
                    },
                },
            )
            self.assertTrue(wr.ok, wr.message)
            bundle = arch.query_mission_bundle('m1')
            self.assertIsNotNone(bundle)
            self.assertEqual(bundle['mission']['execution_status'], 'COMPLETED')
            self.assertEqual(bundle['stations'][0]['inspection_status'], 'FAIL')
            self.assertEqual(
                bundle['stations'][0]['objects']['leds']['model_states'][2], 'off',
            )
            arch.close()

    def test_duplicate_mission_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            db = Path(td) / 'b.sqlite3'
            arch = MissionArchive(db)
            args = dict(
                mission_id='dup',
                route_id='test_room_A',
                execution_status='FAILED',
                error_code='nav_failed',
                message='x',
                started_at='t0',
                ended_at='t1',
            )
            self.assertTrue(arch.persist_mission(**args).ok)
            wr2 = arch.persist_mission(**args)
            self.assertFalse(wr2.ok)
            self.assertEqual(wr2.error_code, 'archive_duplicate_mission')
            arch.close()

    def test_unknown_not_coerced_and_seed_used_ids(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            db = Path(td) / 'c.sqlite3'
            arch = MissionArchive(db)
            wr = arch.persist_mission(
                mission_id='unk1',
                route_id='test_room_A',
                execution_status='COMPLETED',
                error_code='NONE',
                message='ok',
                started_at='t0',
                ended_at='t1',
                station={
                    'station_id': 'A',
                    'inspection_status': 'UNKNOWN',
                    'objects': {'panel': {'verdict': 'UNKNOWN'}},
                },
            )
            self.assertTrue(wr.ok)
            self.assertEqual(
                arch.get_stations('unk1')[0]['inspection_status'], 'UNKNOWN',
            )
            state = MissionRuntimeState()
            state.seed_used_mission_ids(arch.list_mission_ids())
            ok, code, _ = state.validate_goal('unk1', 'test_room_A')
            self.assertFalse(ok)
            self.assertEqual(code, ErrorCode.DUPLICATE_MISSION_ID)
            arch.close()

    def test_load_station_payload_from_json(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / 's.json'
            p.write_text(
                json.dumps({
                    'station_id': 'A',
                    'inspection_status': 'FAIL',
                    'image_stamp': '2.5',
                    'screenshot_path': '/x.png',
                    'annotated_path': '/x_a.png',
                    'model_version': 'm',
                    'roi_rules_version': 'roi_rules_v1',
                    'objects': {'leak': {'verdict': 'PASS', 'state': 'none'}},
                }),
                encoding='utf-8',
            )
            payload = load_station_payload_from_result_json(p)
            self.assertEqual(payload['inspection_status'], 'FAIL')
            self.assertEqual(payload['objects']['leak']['state'], 'none')
            self.assertEqual(payload['station_result_path'], str(p))

    def test_reader_mode_ro_no_create(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            missing = Path(td) / 'missing.sqlite3'
            with self.assertRaises(FileNotFoundError):
                MissionArchiveReader(missing)
            self.assertFalse(missing.exists())
            db = Path(td) / 'ok.sqlite3'
            w = MissionArchive(db)
            w.persist_mission(
                mission_id='r',
                route_id='test_room_A',
                execution_status='FAILED',
                error_code='nav_failed',
                message='x',
                started_at='t0',
                ended_at='t1',
            )
            w.close()
            r = MissionArchiveReader(db)
            self.assertTrue(r.has_mission('r'))
            r.close()


if __name__ == '__main__':
    unittest.main()
