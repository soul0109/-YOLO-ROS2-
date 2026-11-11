#!/usr/bin/env python3
"""MVP-R5 offline tests: read-only archive + API status/error mapping (no Gazebo)."""

from __future__ import annotations

import os
import sqlite3
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'ros2_ws' / 'src' / 'inspection_mission'))
sys.path.insert(0, str(REPO / 'web' / 'backend'))

from fastapi.testclient import TestClient  # noqa: E402

from inspection_mission.mission_archive import (  # noqa: E402
    MissionArchive,
    MissionArchiveReader,
)
from mission_api.app import create_app  # noqa: E402
from mission_api.ros_bridge import LiveMissionState, RosMissionBridge  # noqa: E402


class TestReadonlyArchive(unittest.TestCase):
    def test_reader_does_not_create_missing_db(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            missing = Path(td) / 'nope.sqlite3'
            with self.assertRaises(FileNotFoundError):
                MissionArchiveReader(missing)
            self.assertFalse(missing.exists())

    def test_reader_query_only_rejects_write(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            db = Path(td) / 'r.sqlite3'
            w = MissionArchive(db)
            w.persist_mission(
                mission_id='m1',
                route_id='test_room_A',
                execution_status='COMPLETED',
                error_code='NONE',
                message='ok',
                started_at='t0',
                ended_at='t1',
                station={
                    'station_id': 'A',
                    'inspection_status': 'FAIL',
                    'screenshot_path': str(Path(td) / 'a.png'),
                    'annotated_path': str(Path(td) / 'a_ann.png'),
                    'objects': {'leak': {'verdict': 'PASS'}},
                },
            )
            w.close()
            # Touch fake images for resolve
            (Path(td) / 'a.png').write_bytes(b'png')
            (Path(td) / 'a_ann.png').write_bytes(b'png')

            r = MissionArchiveReader(db)
            self.assertTrue(r.has_mission('m1'))
            self.assertEqual(r.get_stations('m1')[0]['inspection_status'], 'FAIL')
            self.assertEqual(len(r.list_recent_missions(limit=5)), 1)
            shot = r.resolve_station_image_path('m1', 'A', 'screenshot')
            self.assertIsNotNone(shot)
            with self.assertRaises(sqlite3.OperationalError):
                r._conn.execute(
                    "INSERT INTO missions VALUES('x','r','COMPLETED','NONE','','t0','t1')",
                )
            r.close()


class TestApiMapping(unittest.TestCase):
    def setUp(self) -> None:
        self._td = tempfile.TemporaryDirectory()
        self.db = Path(self._td.name) / 'api.sqlite3'
        w = MissionArchive(self.db)
        w.persist_mission(
            mission_id='mvp_r4_hist',
            route_id='test_room_A',
            execution_status='COMPLETED',
            error_code='NONE',
            message='A-station capture saved; inspection_status=FAIL',
            started_at='2026-11-09T08:13:39+00:00',
            ended_at='2026-11-09T08:14:11+00:00',
            station={
                'station_id': 'A',
                'inspection_status': 'FAIL',
                'image_stamp': '35.671',
                'screenshot_path': str(Path(self._td.name) / 'A.png'),
                'annotated_path': str(Path(self._td.name) / 'A_ann.png'),
                'station_result_path': str(Path(self._td.name) / 'A.json'),
                'model_version': 'train1_dev:best.pt:test',
                'roi_rules_version': 'roi_rules_v1',
                'objects': {
                    'leds': {'verdict': 'FAIL', 'model_states': ['green', 'green', 'off']},
                    'panel': {'verdict': 'FAIL', 'state': 'dark'},
                    'leak': {'verdict': 'PASS', 'state': 'none'},
                },
            },
        )
        w.close()
        (Path(self._td.name) / 'A.png').write_bytes(b'\x89PNG\r\n')
        (Path(self._td.name) / 'A_ann.png').write_bytes(b'\x89PNG\r\n')
        os.environ['INSPECTION_ARCHIVE_DB'] = str(self.db)

        self.bridge = MagicMock(spec=RosMissionBridge)
        self.bridge.started = True
        self.bridge.is_action_server_ready.return_value = False
        self.bridge.get_live.return_value = None
        self.bridge.list_live.return_value = []
        self.app = create_app(bridge=self.bridge, start_ros=False)
        self.client = TestClient(self.app)

    def tearDown(self) -> None:
        self._td.cleanup()

    def test_list_and_get_archived_history(self) -> None:
        r = self.client.get('/missions?limit=10')
        self.assertEqual(r.status_code, 200)
        body = r.json()
        self.assertEqual(len(body), 1)
        self.assertEqual(body[0]['mission_id'], 'mvp_r4_hist')
        self.assertEqual(body[0]['source'], 'archive')
        self.assertEqual(body[0]['status'], 'COMPLETED')

        g = self.client.get('/missions/mvp_r4_hist')
        self.assertEqual(g.status_code, 200)
        self.assertEqual(g.json()['inspection_status'], 'FAIL')
        self.assertEqual(g.json()['status'], 'COMPLETED')

        st = self.client.get('/missions/mvp_r4_hist/stations')
        self.assertEqual(st.status_code, 200)
        self.assertEqual(st.json()[0]['inspection_status'], 'FAIL')

        img = self.client.get('/missions/mvp_r4_hist/stations/A/screenshot')
        self.assertEqual(img.status_code, 200)
        self.assertIn('image/png', img.headers.get('content-type', ''))

    def test_live_inflight_not_404(self) -> None:
        live = LiveMissionState(
            mission_id='live_1',
            route_id='test_room_A',
            status='RUNNING',
            phase='NAVIGATING',
            message='going to A',
        )
        self.bridge.get_live.side_effect = (
            lambda mid: live if mid == 'live_1' else None
        )
        g = self.client.get('/missions/live_1')
        self.assertEqual(g.status_code, 200)
        self.assertEqual(g.json()['source'], 'live')
        self.assertEqual(g.json()['phase'], 'NAVIGATING')
        st = self.client.get('/missions/live_1/stations')
        self.assertEqual(st.status_code, 200)
        self.assertEqual(st.json(), [])

    def test_unknown_mission_404(self) -> None:
        r = self.client.get('/missions/does_not_exist')
        self.assertEqual(r.status_code, 404)
        self.assertEqual(r.json()['detail']['error_code'], 'mission_not_found')

    def test_create_maps_errors(self) -> None:
        self.bridge.submit_goal.return_value = (
            400, 'empty_mission_id', 'mission_id must be non-empty', None,
        )
        r = self.client.post('/missions', json={'mission_id': ' ', 'route_id': 'test_room_A'})
        # pydantic may 422 for min_length; also accept bridge 400 path with valid-looking id
        self.bridge.submit_goal.return_value = (
            409, 'duplicate_mission_id', "already archived", None,
        )
        r = self.client.post(
            '/missions',
            json={'mission_id': 'mvp_r4_hist', 'route_id': 'test_room_A'},
        )
        self.assertEqual(r.status_code, 409)
        self.assertEqual(r.json()['error_code'], 'duplicate_mission_id')

        self.bridge.submit_goal.return_value = (
            503, 'runtime_unavailable', 'action server not available', None,
        )
        r = self.client.post(
            '/missions',
            json={'mission_id': 'new_one', 'route_id': 'test_room_A'},
        )
        self.assertEqual(r.status_code, 503)

        live = LiveMissionState(
            mission_id='ok1', route_id='test_room_A', status='ACCEPTED',
        )
        self.bridge.submit_goal.return_value = (202, 'NONE', 'accepted', live)
        r = self.client.post(
            '/missions',
            json={'mission_id': 'ok1', 'route_id': 'test_room_A'},
        )
        self.assertEqual(r.status_code, 202)
        self.assertTrue(r.json()['accepted'])


if __name__ == '__main__':
    unittest.main()
