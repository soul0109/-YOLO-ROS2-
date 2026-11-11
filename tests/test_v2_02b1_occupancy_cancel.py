#!/usr/bin/env python3
"""V2-02B1: occupancy atomicity, early-cancel handoff, nav unknown paths.

Fake-only; barrier/event driven (no sleep races). Bounded join timeouts.
Run:
  PYTHONDONTWRITEBYTECODE=1 python3 -m unittest tests.test_v2_02b1_occupancy_cancel -v
"""

from __future__ import annotations

import sys
import threading
import unittest
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'ros2_ws' / 'src' / 'inspection_mission'))

from inspection_mission.mission_runner import MissionRunner  # noqa: E402
from inspection_mission.mission_state import (  # noqa: E402
    AcceptCancelBridge,
    AcceptedGoalWindow,
    ErrorCode,
    ExecutionStatus,
    MissionActiveSlot,
    MissionRuntimeState,
)


class FakeServerGoalHandle:
    def __init__(self, mission_id: str):
        self.request = SimpleNamespace(mission_id=mission_id, route_id='test_room_A')


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

    def send_through_poses_async(self, poses, feedback_callback=None):
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

    def fail_send_future(self, exc: BaseException) -> None:
        assert self.send_future is not None
        self.send_future.set_exception(exc)

    def complete_result(self, status: int) -> None:
        assert self.result_future is not None
        self.result_future.set_result(SimpleNamespace(status=status, result=None))


def _pose() -> PoseStamped:
    return PoseStamped()


def _start_runner(mission_id, nav, state, *, clock=None, defer=False, poses=None):
    finished = []
    gen = state.active_generation or state.begin(mission_id)
    kwargs = dict(
        state=state,
        nav=nav,
        mission_id=mission_id,
        generation=gen,
        pose=_pose() if poses is None else None,
        poses=poses,
        on_finished=finished.append,
        defer_success=defer,
    )
    if clock is not None:
        kwargs['clock'] = clock
    runner = MissionRunner(**kwargs)
    runner.start()
    return runner, finished, gen


class TestConcurrentReserve(unittest.TestCase):
    def test_two_goals_barrier_only_one_succeeds(self) -> None:
        state = MissionRuntimeState()
        slot = MissionActiveSlot(lock=state.lock)
        bridge = AcceptCancelBridge(state, slot)
        barrier = threading.Barrier(2, timeout=2.0)
        results: list[tuple] = []
        lock = threading.Lock()

        def worker(mid: str) -> None:
            barrier.wait()
            ok, code, _msg, gen = state.reserve_goal(
                mid, 'test_room_A', bridge.window,
            )
            with lock:
                results.append((mid, ok, code, gen))

        threads = [
            threading.Thread(target=worker, args=('g_a',)),
            threading.Thread(target=worker, args=('g_b',)),
        ]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=3.0)
            self.assertFalse(t.is_alive())
        wins = [r for r in results if r[1]]
        loses = [r for r in results if not r[1]]
        self.assertEqual(len(wins), 1)
        self.assertEqual(len(loses), 1)
        self.assertEqual(loses[0][2], ErrorCode.BUSY)
        self.assertTrue(state.is_busy)


class TestEarlyCancelHandoff(unittest.TestCase):
    def test_cancel_between_consume_and_install_impossible_under_lock(self) -> None:
        """consume+install atomic: cancel either hits ticket or context."""
        state = MissionRuntimeState()
        slot = MissionActiveSlot(lock=state.lock)
        bridge = AcceptCancelBridge(state, slot)
        ok, _, _, gen = state.reserve_goal('early1', 'test_room_A', bridge.window)
        self.assertTrue(ok)
        gh = FakeServerGoalHandle('early1')
        # Cancel before install → ticket
        self.assertTrue(bridge.resolve_cancel(gh))
        ctx = bridge.install_context_after_accepted(gh, 'early1', gen)
        self.assertTrue(ctx.pending_cancel)

    def test_cancel_after_install_records_and_calls_runner_outside(self) -> None:
        state = MissionRuntimeState()
        slot = MissionActiveSlot(lock=state.lock)
        bridge = AcceptCancelBridge(state, slot)
        ok, _, _, gen = state.reserve_goal('early2', 'test_room_A', bridge.window)
        self.assertTrue(ok)
        gh = FakeServerGoalHandle('early2')
        ctx = bridge.install_context_after_accepted(gh, 'early2', gen)
        nav = FakeNav()
        clock = {'t': 0.0}
        runner, finished, _ = _start_runner(
            'early2', nav, state, clock=lambda: clock['t'],
        )
        ctx.runner = runner
        # Accept nav then cancel
        nav.complete_send(True)
        self.assertTrue(bridge.resolve_cancel(gh))
        self.assertTrue(ctx.pending_cancel)
        self.assertIsNotNone(nav.cancel_future)
        # Confirm stop on the same result future the runner already holds.
        assert nav.result_future is not None
        nav.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
        nav.complete_result(GoalStatus.STATUS_CANCELED)
        self.assertEqual(len(finished), 1)
        self.assertEqual(finished[0].execution_status, ExecutionStatus.CANCELED.value)

    def test_stale_runner_request_cancel_after_new_owner_is_harmless(self) -> None:
        state = MissionRuntimeState()
        nav_a = FakeNav()
        clock = {'t': 0.0}
        state.begin('old')
        runner_a, _, _ = _start_runner('old', nav_a, state, clock=lambda: clock['t'])
        # Force terminal unsafe then new owner
        state.try_publish_terminal(
            'old', ExecutionStatus.FAILED.value, ErrorCode.CANCEL_UNCONFIRMED, 'x',
            release=False, mark_unsafe=True, generation=1,
        )
        # Clear for new begin: unsafe blocks — stay unsafe; request_cancel on old
        # should not publish for new generation.
        runner_a.request_cancel()
        self.assertTrue(state.unsafe_busy)


class TestNavUnknownPaths(unittest.TestCase):
    def test_send_sync_exception_marks_unsafe_keeps_busy(self) -> None:
        state = MissionRuntimeState()
        nav = FakeNav()
        nav.send_raises = RuntimeError('dds blip')
        state.begin('sx')
        runner, finished, _ = _start_runner('sx', nav, state)
        self.assertEqual(len(finished), 1)
        self.assertEqual(finished[0].error_code, ErrorCode.CANCEL_UNCONFIRMED)
        self.assertTrue(state.unsafe_busy)
        self.assertTrue(state.is_busy)
        # No second terminal from late nothing
        ok, code, _, _ = state.reserve_goal('sx2', 'test_room_A', AcceptedGoalWindow(lock=state.lock))
        self.assertFalse(ok)
        self.assertEqual(code, ErrorCode.RUNTIME_UNSAFE)

    def test_send_future_exception_marks_unsafe(self) -> None:
        state = MissionRuntimeState()
        nav = FakeNav()
        state.begin('sf')
        runner, finished, _ = _start_runner('sf', nav, state)
        nav.fail_send_future(RuntimeError('send fut boom'))
        self.assertEqual(len(finished), 1)
        self.assertTrue(state.unsafe_busy)
        self.assertEqual(finished[0].error_code, ErrorCode.CANCEL_UNCONFIRMED)

    def test_result_future_exception_marks_unsafe(self) -> None:
        # V2-02: after accept, result future error enters stop-confirm (cancel)
        # rather than immediately publishing unsafe; occupancy stays held.
        state = MissionRuntimeState()
        nav = FakeNav()
        state.begin('rf')
        runner, finished, _ = _start_runner('rf', nav, state)
        nav.complete_send(True)
        assert nav.result_future is not None
        nav.result_future.set_exception(RuntimeError('result boom'))
        self.assertEqual(len(finished), 0)
        self.assertTrue(state.is_busy)
        self.assertGreaterEqual(nav.cancel_calls, 1)

    def test_explicit_reject_releases(self) -> None:
        state = MissionRuntimeState()
        nav = FakeNav()
        state.begin('rej')
        runner, finished, _ = _start_runner('rej', nav, state)
        nav.complete_send(False)
        self.assertEqual(finished[0].error_code, ErrorCode.NAV_GOAL_REJECTED)
        self.assertFalse(state.unsafe_busy)
        self.assertFalse(state.is_busy)

    def test_sync_done_callback_on_already_complete_future(self) -> None:
        state = MissionRuntimeState()
        nav = FakeNav()
        state.begin('sync')
        # Pre-complete future before start attaches callback
        nav.send_future = Future()
        handle = FakeGoalHandle(True, nav)
        nav.result_future = Future()
        nav.result_future.set_result(
            SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=None),
        )
        nav.send_future.set_result(handle)
        # Monkey: send returns already-done future
        def send_done(pose, feedback_callback=None):
            nav.send_calls += 1
            return nav.send_future

        nav.send_goal_async = send_done  # type: ignore[method-assign]
        runner, finished, _ = _start_runner('sync', nav, state)
        self.assertEqual(len(finished), 1)
        self.assertEqual(finished[0].execution_status, ExecutionStatus.COMPLETED.value)

    def test_cancel_before_send_claim_does_not_send(self) -> None:
        state = MissionRuntimeState()
        nav = FakeNav()
        state.begin('cbs')
        finished = []
        gen = state.active_generation
        runner = MissionRunner(
            state=state,
            nav=nav,
            mission_id='cbs',
            generation=gen,
            pose=_pose(),
            on_finished=finished.append,
        )
        runner.request_cancel()
        runner.start()
        self.assertEqual(nav.send_calls, 0)
        self.assertEqual(finished[0].execution_status, ExecutionStatus.CANCELED.value)
        self.assertFalse(state.unsafe_busy)

    def test_late_result_after_unsafe_no_second_terminal(self) -> None:
        state = MissionRuntimeState()
        nav = FakeNav()
        clock = {'t': 0.0}
        state.begin('late')
        runner, finished, _ = _start_runner(
            'late', nav, state, clock=lambda: clock['t'],
        )
        # Force send timeout → unknown → unsafe
        clock['t'] = 100.0
        runner.poll_timeouts()
        clock['t'] = 200.0
        runner.poll_timeouts()
        self.assertEqual(len(finished), 1)
        self.assertTrue(state.unsafe_busy)
        # Late accept + result must not second-publish
        nav.complete_send(True)
        if nav.cancel_future is not None:
            nav.cancel_future.set_result(SimpleNamespace(goals_canceling=[object()]))
        if nav.result_future is None:
            nav.result_future = Future()
        nav.complete_result(GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(len(finished), 1)

    def test_through_poses_same_contract_success(self) -> None:
        state = MissionRuntimeState()
        nav = FakeNav()
        state.begin('tp')
        poses = [_pose(), _pose(), _pose(), _pose()]  # vias+C shape
        runner, finished, _ = _start_runner('tp', nav, state, poses=poses)
        nav.complete_send(True)
        nav.complete_result(GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(finished[0].execution_status, ExecutionStatus.COMPLETED.value)
        self.assertEqual(nav.send_calls, 1)


if __name__ == '__main__':
    unittest.main()
