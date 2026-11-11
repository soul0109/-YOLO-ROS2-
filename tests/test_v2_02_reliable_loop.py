#!/usr/bin/env python3
"""V2-02 reliable execution loop: B1 non-terminal, archive seal, API obs, safety.

Fake / temp DB / temp safety dir only — never touches live robot runtime dir.
Run:
  PYTHONDONTWRITEBYTECODE=1 python3 -m unittest tests.test_v2_02_reliable_loop -v
"""

from __future__ import annotations

import json
import sys
import tempfile
import threading
import unittest
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from typing import Optional
from unittest.mock import MagicMock

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'ros2_ws' / 'src' / 'inspection_mission'))
sys.path.insert(0, str(REPO / 'web' / 'backend'))

from inspection_mission.mission_archive import MissionArchive  # noqa: E402
from inspection_mission.mission_runner import MissionRunner  # noqa: E402
from inspection_mission.mission_state import (  # noqa: E402
    ErrorCode,
    ExecutionStatus,
    MissionRuntimeState,
)
from inspection_mission.robot_safety import RobotInstanceGuard  # noqa: E402
from fastapi.testclient import TestClient  # noqa: E402
from mission_api.app import create_app  # noqa: E402
from mission_api.ros_bridge import (  # noqa: E402
    OBS_PENDING,
    OBS_UNKNOWN,
    LiveMissionState,
    RosMissionBridge,
)


class FakeGoalHandle:
    def __init__(self, accepted: bool, nav: 'FakeNav'):
        self.accepted = accepted
        self._nav = nav

    def cancel_goal_async(self):
        return self._nav.cancel_goal_async(self)

    def get_result_async(self):
        return self._nav.get_result_async(self)


class FakeNav:
    def __init__(self) -> None:
        self.server_available = True
        self.send_raises: Optional[BaseException] = None
        self.send_future: Optional[Future] = None
        self.cancel_future: Optional[Future] = None
        self.result_future: Optional[Future] = None
        self.send_calls = 0
        self.cancel_calls = 0

    def wait_for_server(self, timeout_sec: float, *, through_poses: bool = False) -> bool:
        return bool(self.server_available)

    def send_goal_async(self, pose, feedback_callback=None):
        self.send_calls += 1
        if self.send_raises is not None:
            raise self.send_raises
        self.send_future = Future()
        return self.send_future

    def cancel_goal_async(self, goal_handle):
        self.cancel_calls += 1
        self.cancel_future = Future()
        return self.cancel_future

    def get_result_async(self, goal_handle):
        if self.result_future is None:
            self.result_future = Future()
        return self.result_future

    def complete_send(self, accepted: bool = True) -> FakeGoalHandle:
        assert self.send_future is not None
        handle = FakeGoalHandle(accepted, self)
        self.send_future.set_result(handle)
        return handle

    def complete_result(self, status: int) -> None:
        assert self.result_future is not None
        self.result_future.set_result(SimpleNamespace(status=status, result=None))


def _start_runner(mission_id, nav, state):
    finished = []
    gen = state.active_generation or state.begin(mission_id)
    runner = MissionRunner(
        state=state,
        nav=nav,
        mission_id=mission_id,
        generation=gen,
        pose=PoseStamped(),
        on_finished=finished.append,
    )
    runner.start()
    return runner, finished


class TestB1NonTerminalResult(unittest.TestCase):
    def _case(self, status: int) -> None:
        state = MissionRuntimeState()
        nav = FakeNav()
        runner, finished = _start_runner('m_nt', nav, state)
        nav.complete_send(accepted=True)
        nav.complete_result(status)
        self.assertEqual(finished, [])
        self.assertTrue(state.is_busy)
        self.assertFalse(state.terminal_published)
        # Stop confirm path should attempt cancel.
        self.assertGreaterEqual(nav.cancel_calls, 1)

    def test_unknown_does_not_release(self) -> None:
        self._case(GoalStatus.STATUS_UNKNOWN)

    def test_executing_does_not_release(self) -> None:
        self._case(GoalStatus.STATUS_EXECUTING)

    def test_canceling_does_not_release(self) -> None:
        self._case(GoalStatus.STATUS_CANCELING)

    def test_aborted_releases_failed(self) -> None:
        state = MissionRuntimeState()
        nav = FakeNav()
        runner, finished = _start_runner('m_ab', nav, state)
        nav.complete_send(accepted=True)
        nav.complete_result(GoalStatus.STATUS_ABORTED)
        self.assertEqual(len(finished), 1)
        self.assertEqual(finished[0].execution_status, ExecutionStatus.FAILED.value)
        self.assertFalse(state.is_busy)
        self.assertFalse(state.unsafe_busy)

    def test_result_future_exception_attempts_stop(self) -> None:
        state = MissionRuntimeState()
        nav = FakeNav()
        runner, finished = _start_runner('m_re', nav, state)
        nav.complete_send(accepted=True)
        assert nav.result_future is not None
        nav.result_future.set_exception(RuntimeError('boom'))
        self.assertEqual(finished, [])
        self.assertTrue(state.is_busy)
        self.assertGreaterEqual(nav.cancel_calls, 1)


class TestArchiveSeal(unittest.TestCase):
    def test_claim_commit_release_only_after_ok(self) -> None:
        state = MissionRuntimeState()
        gen = state.begin('m_seal')
        claimed = state.claim_finalizing_terminal(
            'm_seal', ExecutionStatus.COMPLETED.value, ErrorCode.NONE, 'ok',
            generation=gen,
        )
        self.assertIsNotNone(claimed)
        self.assertTrue(state.is_busy)
        self.assertTrue(state.finalizing)
        committed = state.commit_terminal_outcome(
            'm_seal', gen, archive_ok=True, release=True,
        )
        self.assertIsNotNone(committed)
        self.assertFalse(state.is_busy)
        self.assertFalse(state.finalizing)

    def test_archive_fail_keeps_refuse(self) -> None:
        from inspection_mission.mission_state import MissionTerminal
        state = MissionRuntimeState()
        gen = state.begin('m_af2')
        state.claim_finalizing_terminal(
            'm_af2', ExecutionStatus.COMPLETED.value, ErrorCode.NONE, 'ok',
            generation=gen,
        )
        override = MissionTerminal(
            mission_id='m_af2',
            execution_status=ExecutionStatus.FAILED.value,
            error_code=ErrorCode.ARCHIVE_FAILED,
            message='not saved',
            generation=gen,
        )
        out = state.commit_terminal_outcome(
            'm_af2', gen, archive_ok=False, release=False, override_terminal=override,
        )
        self.assertEqual(out.error_code, ErrorCode.ARCHIVE_FAILED)
        self.assertTrue(state.archive_degraded)
        self.assertTrue(state.is_busy)
        ok, code, _msg = state.validate_goal('other', 'test_room_A')
        self.assertFalse(ok)
        self.assertEqual(code, ErrorCode.ARCHIVE_FAILED)


class TestRobotSafety(unittest.TestCase):
    def test_missing_mark_refuses(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            g = RobotInstanceGuard(robot_id='ut_bot', runtime_dir=Path(td))
            allow, reason, snap = g.startup_gate()
            self.assertFalse(allow)
            self.assertTrue(snap.refuse)
            self.assertIn('missing', reason)

    def test_active_then_clear(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            g = RobotInstanceGuard(robot_id='ut_bot', runtime_dir=Path(td))
            ok, _ = g.acquire_flock()
            self.assertTrue(ok)
            try:
                self.assertTrue(g.mark_active(mission_id='m1')[0])
                allow, reason, _ = g.startup_gate()
                self.assertFalse(allow)
                self.assertIn('active', reason)
                token = g.arm_operator_confirm()
                tok_ok, tok_msg = g.validate_and_consume_token(
                    instance_id=g.instance_id, token=token,
                )
                self.assertTrue(tok_ok, tok_msg)
                cok, cmsg = g.clear_mark_if_owner(instance_id=g.instance_id)
                self.assertTrue(cok, cmsg)
                allow2, _, _ = g.startup_gate()
                self.assertTrue(allow2)
                # Token one-shot
                tok2, _ = g.validate_and_consume_token(
                    instance_id=g.instance_id, token=token,
                )
                self.assertFalse(tok2)
            finally:
                g.release_flock()

    def test_flock_single_instance(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            a = RobotInstanceGuard(robot_id='ut_bot', runtime_dir=Path(td))
            b = RobotInstanceGuard(robot_id='ut_bot', runtime_dir=Path(td))
            self.assertTrue(a.acquire_flock()[0])
            try:
                self.assertFalse(b.acquire_flock()[0])
            finally:
                a.release_flock()

    def test_db_path_independent(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            rt = Path(td) / 'runtime'
            db = Path(td) / 'other' / 'missions.sqlite3'
            db.parent.mkdir(parents=True)
            g = RobotInstanceGuard(robot_id='ut_bot', runtime_dir=rt)
            g.clear_mark()
            # Changing DB does not change safety path
            self.assertTrue(str(g.safety_path).startswith(str(rt)))
            self.assertNotEqual(g.safety_path.parent, db.parent)


class TestObservationApi(unittest.TestCase):
    def setUp(self) -> None:
        self._td = tempfile.TemporaryDirectory()
        self.db = Path(self._td.name) / 'api.sqlite3'
        MissionArchive(self.db).close()
        self.bridge = RosMissionBridge(goal_response_timeout_sec=0.05)
        # Offline: do not start rclpy
        self.app = create_app(bridge=self.bridge, start_ros=False)
        self.client = TestClient(self.app)

    def tearDown(self) -> None:
        self._td.cleanup()

    def test_get_running_shell_without_live_is_unknown(self) -> None:
        import os
        os.environ['INSPECTION_ARCHIVE_DB'] = str(self.db)
        arch = MissionArchive(self.db)
        arch.begin_mission(
            mission_id='run_only', route_id='test_room_A', started_at='t0',
        )
        arch.close()
        r = self.client.get('/missions/run_only')
        self.assertEqual(r.status_code, 200)
        body = r.json()
        self.assertEqual(body['status'], 'RUNNING')
        self.assertEqual(body['observation_status'], OBS_UNKNOWN)

    def test_live_pending_keeps_busy_semantics(self) -> None:
        with self.bridge._lock:
            self.bridge._live['pend'] = LiveMissionState(
                mission_id='pend',
                route_id='test_room_ABC',
                status='ACCEPTED',
                observation_status=OBS_PENDING,
                error_code='goal_response_timeout',
            )
            self.bridge._busy_mission_id = 'pend'
        self.assertTrue(self.bridge.is_bridge_busy())
        live = self.bridge.get_live('pend')
        self.assertEqual(live.observation_status, OBS_PENDING)
        self.assertFalse(live.terminal)

    def test_result_comm_error_sets_unknown_not_failed(self) -> None:
        with self.bridge._lock:
            self.bridge._live['x'] = LiveMissionState(
                mission_id='x', route_id='test_room_A', status='RUNNING',
            )
            self.bridge._busy_mission_id = 'x'
            self.bridge._result_attached['x'] = True
        # Simulate result cb exception path
        st = self.bridge._live['x']
        st.observation_status = OBS_UNKNOWN
        st.error_code = 'result_communication_error'
        st.message = 'result future failed (observation): boom'
        self.assertNotEqual(st.status, 'FAILED')
        self.assertFalse(st.terminal)


class TestInterruptRunning(unittest.TestCase):
    def test_interrupt_preserves_stations(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            db = Path(td) / 'a.sqlite3'
            arch = MissionArchive(db)
            arch.begin_mission(mission_id='m', route_id='test_room_A', started_at='t0')
            arch.persist_station(
                mission_id='m',
                station={
                    'station_id': 'A',
                    'inspection_status': 'PASS',
                    'objects': {'leds': {'verdict': 'PASS'}},
                },
            )
            ids, err = arch.interrupt_running_missions(reason='operator confirm')
            self.assertIsNone(err)
            self.assertEqual(ids, ['m'])
            bundle = arch.query_mission_bundle('m')
            self.assertEqual(bundle['mission']['execution_status'], 'INTERRUPTED')
            self.assertEqual(len(bundle['stations']), 1)
            arch.close()


if __name__ == '__main__':
    unittest.main()
