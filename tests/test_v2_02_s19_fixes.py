#!/usr/bin/env python3
"""V2-02 §19 batch fixes: recovery/token/lock, seal order, HTTP converge.

Uses temp DB/runtime dirs and fake orchestration — never default robot runtime.
Run:
  PYTHONDONTWRITEBYTECODE=1 python3 -m unittest tests.test_v2_02_s19_fixes -v
"""

from __future__ import annotations

import json
import os
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
from inspection_mission.mission_recovery_ipc import (  # noqa: E402
    RecoveryRequestProcessor,
    submit_online_request,
)
from inspection_mission.mission_runner import MissionRunner  # noqa: E402
from inspection_mission.mission_safety_cli import main as cli_main  # noqa: E402
from inspection_mission.mission_state import (  # noqa: E402
    ErrorCode,
    ExecutionStatus,
    MissionContext,
    MissionRuntimeState,
    MissionTerminal,
)
from inspection_mission.robot_safety import RobotInstanceGuard  # noqa: E402
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


class TestR1TokenAndLock(unittest.TestCase):
    def test_cli_forged_token_fails_offline(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            rt = Path(td)
            holder = RobotInstanceGuard(robot_id='bot', runtime_dir=rt)
            self.assertTrue(holder.acquire_flock()[0])
            try:
                holder.mark_active(mission_id='m1')
                # Another process cannot arm/confirm without flock.
                rc = cli_main([
                    '--runtime-dir', str(rt), '--robot-id', 'bot',
                    'confirm', '--instance-id', 'x', '--token', 'never_armed',
                ])
                self.assertNotEqual(rc, 0)
                snap = holder.read_safety()
                self.assertTrue(snap.refuse)
                self.assertEqual(snap.payload.get('state'), 'active')
            finally:
                holder.release_flock()

    def test_cli_cannot_clear_while_other_holds_flock(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            rt = Path(td)
            holder = RobotInstanceGuard(robot_id='bot', runtime_dir=rt)
            self.assertTrue(holder.acquire_flock()[0])
            holder.mark_active(mission_id='m1')
            try:
                rc = cli_main([
                    '--runtime-dir', str(rt), '--robot-id', 'bot',
                    'arm',
                ])
                self.assertNotEqual(rc, 0)
                snap = holder.read_safety()
                self.assertEqual(snap.payload.get('state'), 'active')
            finally:
                holder.release_flock()

    def test_offline_arm_confirm_under_flock(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            rt = Path(td)
            db = Path(td) / 'a.sqlite3'
            arch = MissionArchive(db)
            arch.begin_mission(mission_id='run1', route_id='test_room_A', started_at='t0')
            arch.close()
            rc = cli_main(['--runtime-dir', str(rt), '--robot-id', 'bot', 'arm'])
            self.assertEqual(rc, 0)
            armed = json.loads((rt / 'robot_bot_armed_token.json').read_text())
            rc = cli_main([
                '--runtime-dir', str(rt), '--robot-id', 'bot',
                'confirm',
                '--instance-id', armed['instance_id'],
                '--token', armed['token'],
                '--archive-db', str(db),
            ])
            self.assertEqual(rc, 0)
            g = RobotInstanceGuard(robot_id='bot', runtime_dir=rt)
            allow, _, _ = g.startup_gate()
            self.assertTrue(allow)
            arch = MissionArchive(db)
            self.assertEqual(
                arch.get_mission('run1')['execution_status'], 'INTERRUPTED',
            )
            arch.close()

    def test_online_ipc_uses_runtime_token(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            rt = Path(td)
            guard = RobotInstanceGuard(robot_id='bot', runtime_dir=rt)
            self.assertTrue(guard.acquire_flock()[0])
            guard.write_instance_meta()
            state = {'token': '', 'cleared': False}

            def handle_arm():
                state['token'] = guard.arm_operator_confirm()
                return {
                    'ok': True,
                    'instance_id': guard.instance_id,
                    'token': state['token'],
                }

            def handle_confirm(iid, tok):
                ok, msg = guard.validate_and_consume_token(
                    instance_id=iid, token=tok,
                )
                if not ok:
                    return {'ok': False, 'message': msg}
                cok, cmsg = guard.clear_mark_if_owner(instance_id=iid)
                state['cleared'] = cok
                return {'ok': cok, 'message': cmsg or 'ok'}

            proc = RecoveryRequestProcessor(
                guard=guard,
                handle_status=lambda: {'ok': True, 'instance_id': guard.instance_id},
                handle_arm=handle_arm,
                handle_confirm=handle_confirm,
            )
            # Background poller
            stop = threading.Event()

            def loop():
                while not stop.wait(0.02):
                    proc.poll_once()

            t = threading.Thread(target=loop, daemon=True)
            t.start()
            try:
                guard.mark_active(mission_id='m')
                arm = submit_online_request(
                    runtime_dir=rt, robot_id='bot', cmd='arm', timeout_sec=2.0,
                )
                self.assertTrue(arm.get('ok'), arm)
                # Forged token fails
                bad = submit_online_request(
                    runtime_dir=rt, robot_id='bot', cmd='confirm',
                    instance_id=guard.instance_id, token='forged', timeout_sec=2.0,
                )
                self.assertFalse(bad.get('ok'))
                # Real token works once
                good = submit_online_request(
                    runtime_dir=rt, robot_id='bot', cmd='confirm',
                    instance_id=arm['instance_id'], token=arm['token'],
                    timeout_sec=2.0,
                )
                self.assertTrue(good.get('ok'), good)
                self.assertTrue(state['cleared'])
                # Replay fails
                again = submit_online_request(
                    runtime_dir=rt, robot_id='bot', cmd='confirm',
                    instance_id=arm['instance_id'], token=arm['token'],
                    timeout_sec=2.0,
                )
                self.assertFalse(again.get('ok'))
            finally:
                stop.set()
                guard.release_flock()


class TestR2SealOrder(unittest.TestCase):
    def test_managed_runner_aborted_keeps_busy_until_seal(self) -> None:
        state = MissionRuntimeState()
        gen = state.begin('m_ab')
        nav = FakeNav()
        finished = []
        runner = MissionRunner(
            state=state, nav=nav, mission_id='m_ab', generation=gen,
            pose=PoseStamped(), on_finished=finished.append,
            managed_occupancy=True,
        )
        runner.start()
        nav.complete_send(True)
        nav.complete_result(GoalStatus.STATUS_ABORTED)
        self.assertEqual(len(finished), 1)
        self.assertTrue(state.is_busy)
        ok, code, _ = state.validate_goal('next', 'test_room_A')
        self.assertFalse(ok)
        self.assertEqual(code, ErrorCode.BUSY)

    def test_seal_clear_while_busy_then_release(self) -> None:
        """Call real MissionRuntimeNode._seal via sourced interfaces if available."""
        try:
            from inspection_mission.mission_runtime_node import MissionRuntimeNode
        except ModuleNotFoundError:
            self.skipTest('inspection_interfaces not sourced')
            return
        with tempfile.TemporaryDirectory() as td:
            rt = Path(td)
            db = Path(td) / 'a.sqlite3'
            state = MissionRuntimeState()
            guard = RobotInstanceGuard(robot_id='bot', runtime_dir=rt)
            self.assertTrue(guard.acquire_flock()[0])
            arch = MissionArchive(db)
            try:
                gen = state.begin('m1')
                arch.begin_mission(
                    mission_id='m1', route_id='test_room_A', started_at='t0',
                )
                guard.mark_active(mission_id='m1')
                node = object.__new__(MissionRuntimeNode)
                node.state = state
                node._guard = guard
                node._archive = arch
                node.get_logger = lambda: SimpleNamespace(
                    error=lambda *a, **k: None,
                    info=lambda *a, **k: None,
                    warn=lambda *a, **k: None,
                )
                ctx = MissionContext(mission_id='m1', generation=gen)
                terminal = MissionTerminal(
                    mission_id='m1',
                    execution_status=ExecutionStatus.COMPLETED.value,
                    error_code=ErrorCode.NONE,
                    message='ok',
                    generation=gen,
                )
                seen_busy = []
                real_clear = guard.clear_mark_if_owner

                def wrapped_clear(**kwargs):
                    seen_busy.append(state.is_busy)
                    return real_clear(**kwargs)

                guard.clear_mark_if_owner = wrapped_clear  # type: ignore
                out = MissionRuntimeNode._seal_mission_terminal(
                    node, ctx=ctx, terminal=terminal, release=True, mark_unsafe=False,
                )
                self.assertEqual(out.execution_status, ExecutionStatus.COMPLETED.value)
                self.assertEqual(seen_busy, [True])
                self.assertFalse(state.is_busy)
                allow, _, _ = guard.startup_gate()
                self.assertTrue(allow)
            finally:
                arch.close()
                guard.release_flock()

    def test_old_owner_cannot_pollute_new_terminal(self) -> None:
        state = MissionRuntimeState()
        g1 = state.begin('old')
        state.claim_finalizing_terminal(
            'old', ExecutionStatus.COMPLETED.value, ErrorCode.NONE, 'old',
            generation=g1,
        )
        # Force release then new owner
        state.release_occupancy_if_owner('old', g1)
        g2 = state.begin('new')
        polluted = state.commit_archive_result(
            'old', g1, archive_ok=True,
            override_terminal=MissionTerminal(
                mission_id='old', execution_status='FAILED',
                error_code='x', message='pollute', generation=g1,
            ),
        )
        self.assertIsNone(polluted)
        self.assertEqual(state.active_mission_id, 'new')
        self.assertEqual(state.active_generation, g2)


class TestR3RecoveryOrder(unittest.TestCase):
    def test_confirm_refuses_finalizing(self) -> None:
        state = MissionRuntimeState()
        gen = state.begin('m')
        state.claim_finalizing_terminal(
            'm', ExecutionStatus.COMPLETED.value, ErrorCode.NONE, 'x',
            generation=gen,
        )
        ok, msg = state.can_accept_operator_confirm()
        self.assertFalse(ok)
        self.assertIn('finalizing', msg)

    def test_token_consume_separate_from_clear(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            g = RobotInstanceGuard(robot_id='bot', runtime_dir=Path(td))
            token = g.arm_operator_confirm()
            g.mark_active(mission_id='m')
            ok, _ = g.validate_and_consume_token(
                instance_id=g.instance_id, token=token,
            )
            self.assertTrue(ok)
            # Still active until explicit clear
            snap = g.read_safety()
            self.assertEqual(snap.payload.get('state'), 'active')
            self.assertTrue(g.clear_mark_if_owner(instance_id=g.instance_id)[0])


class TestR4HttpConverge(unittest.TestCase):
    def test_late_reject_clears_busy_via_callback(self) -> None:
        bridge = RosMissionBridge(goal_response_timeout_sec=0.05)
        bridge._started = True
        bridge._client = MagicMock()
        bridge._node = MagicMock()
        send_future = Future()

        def send_goal_async(goal, feedback_callback=None):
            return send_future

        bridge._client.send_goal_async = send_goal_async
        bridge._client.wait_for_server = lambda timeout_sec=None: True
        bridge._client.server_is_ready = lambda: True

        # Submit in thread so we can complete after 504
        box: dict = {}

        def worker():
            box['ret'] = bridge.submit_goal('late1', 'test_room_A')

        t = threading.Thread(target=worker)
        t.start()
        t.join(timeout=10.0)
        self.assertFalse(
            t.is_alive(),
            'submit_goal worker still running after join; box keys=%s' % (list(box),),
        )
        self.assertIn('ret', box)
        code, err, _msg, live = box['ret']
        self.assertEqual(code, 504)
        self.assertEqual(live.observation_status, OBS_PENDING)
        self.assertTrue(bridge.is_bridge_busy())
        # Late reject via real callback
        handle = SimpleNamespace(accepted=False)
        send_future.set_result(handle)
        # Allow callback
        import time
        time.sleep(0.05)
        self.assertFalse(bridge.is_bridge_busy())
        self.assertIsNone(bridge.get_live('late1'))

    def test_get_result_sync_exception_unknown(self) -> None:
        bridge = RosMissionBridge()
        with bridge._lock:
            bridge._live['m'] = LiveMissionState(
                mission_id='m', route_id='test_room_A', status='ACCEPTED',
            )
            bridge._busy_mission_id = 'm'

        class BoomHandle:
            accepted = True

            def get_result_async(self):
                raise RuntimeError('sync boom')

        bridge._attach_result_once('m', 'test_room_A', BoomHandle())
        st = bridge.get_live('m')
        self.assertEqual(st.observation_status, OBS_UNKNOWN)
        self.assertFalse(bridge.is_bridge_busy())

    def test_archive_reconcile_clears_busy(self) -> None:
        bridge = RosMissionBridge()
        with bridge._lock:
            bridge._live['done1'] = LiveMissionState(
                mission_id='done1', route_id='test_room_A', status='RUNNING',
            )
            bridge._busy_mission_id = 'done1'
        bridge.reconcile_from_archive_terminal(
            'done1', execution_status='COMPLETED', error_code='NONE',
        )
        self.assertFalse(bridge.is_bridge_busy())
        st = bridge.get_live('done1')
        self.assertTrue(st.terminal)
        self.assertEqual(st.status, 'COMPLETED')


if __name__ == '__main__':
    unittest.main()
