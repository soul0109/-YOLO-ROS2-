#!/usr/bin/env python3
"""MVP-R1 offline tests for Mission Runtime / ownership / cancel semantics.

No Gazebo. Requires ROS Humble Python libs (geometry_msgs, action_msgs).
Run after: source /opt/ros/humble/setup.bash
  PYTHONPATH=ros2_ws/src/inspection_mission:$PYTHONPATH \\
    python3 -m unittest tests.test_mvp_r1_mission_runtime -v
"""

from __future__ import annotations

import sys
import threading
import unittest
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'ros2_ws' / 'src' / 'inspection_mission'))

from inspection_mission.map_pose import pose_from_route_values  # noqa: E402
from inspection_mission.mission_runner import MissionRunner  # noqa: E402
from inspection_mission.mission_state import (  # noqa: E402
    ALLOWED_ROUTE_ID,
    AcceptCancelBridge,
    ErrorCode,
    ExecutionStatus,
    MissionActiveSlot,
    MissionContext,
    MissionRuntimeState,
    MissionTerminal,
    Phase,
)
from inspection_mission.nav_adapter import NavAdapter  # noqa: E402


class FakeServerGoalHandle:
    """Minimal Action ServerGoalHandle stand-in for Runtime callback tests."""

    def __init__(self, mission_id: str, route_id: str = ALLOWED_ROUTE_ID) -> None:
        self.request = SimpleNamespace(mission_id=mission_id, route_id=route_id)


class FakeGoalHandle:
    def __init__(self, accepted: bool, nav: 'FakeNavAdapter') -> None:
        self.accepted = accepted
        self._nav = nav

    def cancel_goal_async(self):
        return self._nav.cancel_goal_async(self)

    def get_result_async(self):
        return self._nav.get_result_async(self)


class FakeNavAdapter:
    """Controllable NavAdapterProtocol stand-in for offline tests."""

    def __init__(self) -> None:
        self.server_available = True
        self.wait_calls = 0
        self.send_calls = 0
        self.cancel_calls = 0
        self.get_result_calls = 0
        self.last_pose: PoseStamped | None = None
        self.last_poses: list[PoseStamped] | None = None
        self.last_through_poses = False
        self.send_future: Future | None = None
        self.cancel_future: Future | None = None
        self.result_future: Future | None = None
        self.goal_handle: FakeGoalHandle | None = None
        self.cancel_handles: list = []

    def wait_for_server(self, timeout_sec: float, *, through_poses: bool = False) -> bool:
        self.wait_calls += 1
        self.last_through_poses = bool(through_poses)
        return bool(self.server_available)

    def send_goal_async(self, pose: PoseStamped, feedback_callback=None):
        self.send_calls += 1
        self.last_pose = pose
        self.last_poses = None
        self.send_future = Future()
        return self.send_future

    def send_through_poses_async(self, poses, feedback_callback=None):
        self.send_calls += 1
        self.last_poses = list(poses)
        self.last_pose = poses[-1] if poses else None
        self.send_future = Future()
        return self.send_future

    def cancel_goal_async(self, goal_handle):
        self.cancel_calls += 1
        self.cancel_handles.append(goal_handle)
        self.cancel_future = Future()
        return self.cancel_future

    def get_result_async(self, goal_handle):
        self.get_result_calls += 1
        if self.result_future is None:
            self.result_future = Future()
        return self.result_future

    def complete_send(self, accepted: bool = True) -> FakeGoalHandle:
        assert self.send_future is not None
        handle = FakeGoalHandle(accepted, self)
        self.goal_handle = handle
        self.send_future.set_result(handle)
        return handle

    def complete_cancel_response(self, response=None) -> None:
        assert self.cancel_future is not None
        # Default: non-empty goals_canceling (cancel accepted for at least one goal).
        # Empty list means cancel was not accepted — still not proof of stop.
        if response is None:
            response = SimpleNamespace(goals_canceling=[SimpleNamespace(goal_id='g')])
        self.cancel_future.set_result(response)

    def complete_result(self, status: int) -> None:
        if self.result_future is None:
            self.result_future = Future()
        if not self.result_future.done():
            self.result_future.set_result(
                SimpleNamespace(status=status, result=None),
            )


def _pose() -> PoseStamped:
    return pose_from_route_values([3.15, 1.75, -1.57079632679], [-0.0134, 0.0519, 0.0])


def _start_runner(
    mission_id: str,
    nav: FakeNavAdapter,
    state: MissionRuntimeState,
    *,
    clock=None,
    send_timeout_sec: float = 2.0,
    result_timeout_sec: float = 2.0,
    cancel_timeout_sec: float = 2.0,
):
    generation = state.begin(mission_id)
    feedback = []
    finished = []
    done = threading.Event()
    ctx = MissionContext(mission_id=mission_id, generation=generation)

    def on_finished(terminal):
        finished.append(terminal)
        ctx.publish_finished(terminal)
        done.set()

    kwargs = dict(
        state=state,
        nav=nav,
        pose=_pose(),
        mission_id=mission_id,
        generation=generation,
        server_timeout_sec=1.0,
        send_timeout_sec=send_timeout_sec,
        result_timeout_sec=result_timeout_sec,
        cancel_timeout_sec=cancel_timeout_sec,
        on_feedback=lambda *a: feedback.append(a),
        on_finished=on_finished,
    )
    if clock is not None:
        kwargs['clock'] = clock
    runner = MissionRunner(**kwargs)
    ctx.runner = runner
    return runner, state, feedback, finished, done, ctx, generation


class TestMapPose(unittest.TestCase):
    def test_matches_patrol_origin_rule(self):
        origin = [-0.0134, 0.0519, 0.0]
        values = [3.15, 1.75, -1.57079632679]
        pose = pose_from_route_values(values, origin)
        self.assertEqual(pose.header.frame_id, 'map')
        self.assertAlmostEqual(pose.pose.position.x, 3.15 - (-0.0134), places=6)
        self.assertAlmostEqual(pose.pose.position.y, 1.75 - 0.0519, places=6)


class TestGoalValidation(unittest.TestCase):
    def test_accept_test_room_a(self):
        state = MissionRuntimeState()
        ok, code, _ = state.validate_goal('mvp_r1_a_001', ALLOWED_ROUTE_ID)
        self.assertTrue(ok)
        self.assertEqual(code, ErrorCode.NONE)

    def test_reject_empty_mission_id(self):
        state = MissionRuntimeState()
        ok, code, msg = state.validate_goal('', ALLOWED_ROUTE_ID)
        self.assertFalse(ok)
        self.assertEqual(code, ErrorCode.EMPTY_MISSION_ID)
        self.assertIn('mission_id', msg)

    def test_reject_other_route(self):
        state = MissionRuntimeState()
        ok, code, msg = state.validate_goal('m1', 'test_room_XYZ')
        self.assertFalse(ok)
        self.assertEqual(code, ErrorCode.UNSUPPORTED_ROUTE)
        self.assertIn('test_room_A', msg)
        self.assertIn('test_room_ABC', msg)

    def test_accept_abc_route(self):
        state = MissionRuntimeState()
        ok, code, _ = state.validate_goal('m_abc', 'test_room_ABC')
        self.assertTrue(ok)
        self.assertEqual(code, ErrorCode.NONE)

    def test_reject_busy_second_goal(self):
        state = MissionRuntimeState()
        state.begin('m1')
        ok, code, _ = state.validate_goal('m2', ALLOWED_ROUTE_ID)
        self.assertFalse(ok)
        self.assertEqual(code, ErrorCode.BUSY)

    def test_reject_duplicate_mission_id(self):
        state = MissionRuntimeState()
        gen = state.begin('m1')
        state.try_publish_terminal(
            'm1', ExecutionStatus.COMPLETED.value, ErrorCode.NONE, 'ok',
            release=True, generation=gen,
        )
        ok, code, _ = state.validate_goal('m1', ALLOWED_ROUTE_ID)
        self.assertFalse(ok)
        self.assertEqual(code, ErrorCode.DUPLICATE_MISSION_ID)


class TestMissionRunnerNavPaths(unittest.TestCase):
    def _run(self, nav: FakeNavAdapter, state: MissionRuntimeState | None = None):
        state = state or MissionRuntimeState()
        return _start_runner('m1', nav, state)

    def test_nav_server_unavailable(self):
        nav = FakeNavAdapter()
        nav.server_available = False
        runner, state, _, finished, done, _, _ = self._run(nav)
        runner.start()
        self.assertTrue(done.wait(1.0))
        self.assertEqual(finished[0].execution_status, ExecutionStatus.FAILED.value)
        self.assertEqual(finished[0].error_code, ErrorCode.NAV_SERVER_UNAVAILABLE)
        self.assertEqual(nav.send_calls, 0)
        self.assertFalse(state.is_busy)

    def test_nav_goal_rejected(self):
        nav = FakeNavAdapter()
        runner, state, feedback, finished, done, _, _ = self._run(nav)
        runner.start()
        self.assertTrue(any(f[2] == Phase.NAVIGATING.value for f in feedback))
        nav.complete_send(accepted=False)
        self.assertTrue(done.wait(1.0))
        self.assertEqual(finished[0].error_code, ErrorCode.NAV_GOAL_REJECTED)
        self.assertFalse(state.is_busy)

    def test_nav_success(self):
        nav = FakeNavAdapter()
        runner, state, _, finished, done, _, _ = self._run(nav)
        runner.start()
        nav.complete_send(accepted=True)
        nav.complete_result(GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(done.wait(1.0))
        self.assertEqual(finished[0].execution_status, ExecutionStatus.COMPLETED.value)
        self.assertEqual(finished[0].error_code, ErrorCode.NONE)
        self.assertIn('A-station navigation', finished[0].message)
        self.assertFalse(state.is_busy)

    def test_nav_failed(self):
        nav = FakeNavAdapter()
        runner, _, _, finished, done, _, _ = self._run(nav)
        runner.start()
        nav.complete_send(accepted=True)
        nav.complete_result(GoalStatus.STATUS_ABORTED)
        self.assertTrue(done.wait(1.0))
        self.assertEqual(finished[0].execution_status, ExecutionStatus.FAILED.value)
        self.assertEqual(finished[0].error_code, ErrorCode.NAV_FAILED)

    def test_cancel_while_navigating_confirmed(self):
        nav = FakeNavAdapter()
        runner, state, feedback, finished, done, _, _ = self._run(nav)
        runner.start()
        nav.complete_send(accepted=True)
        runner.request_cancel()
        self.assertTrue(any(f[2] == Phase.CANCELING.value for f in feedback))
        self.assertEqual(nav.cancel_calls, 1)
        nav.complete_cancel_response()
        nav.complete_result(GoalStatus.STATUS_CANCELED)
        self.assertTrue(done.wait(1.0))
        self.assertEqual(finished[0].execution_status, ExecutionStatus.CANCELED.value)
        self.assertFalse(state.unsafe_busy)
        self.assertFalse(state.is_busy)

    def test_cancel_interleaved_with_goal_acceptance(self):
        nav = FakeNavAdapter()
        runner, state, _, finished, done, _, _ = self._run(nav)
        runner.start()
        runner.request_cancel()
        self.assertEqual(nav.cancel_calls, 0)
        nav.complete_send(accepted=True)
        self.assertEqual(nav.cancel_calls, 1)
        nav.complete_cancel_response()
        nav.complete_result(GoalStatus.STATUS_CANCELED)
        self.assertTrue(done.wait(1.0))
        self.assertEqual(finished[0].execution_status, ExecutionStatus.CANCELED.value)
        self.assertFalse(state.is_busy)

    def test_cancel_before_accept_no_false_success_when_rejected(self):
        nav = FakeNavAdapter()
        runner, _, _, finished, done, _, _ = self._run(nav)
        runner.start()
        runner.request_cancel()
        nav.complete_send(accepted=False)
        self.assertTrue(done.wait(1.0))
        self.assertEqual(finished[0].execution_status, ExecutionStatus.CANCELED.value)
        self.assertEqual(nav.cancel_calls, 0)

    def test_cancel_unconfirmed_keeps_busy(self):
        nav = FakeNavAdapter()
        runner, state, _, finished, done, _, _ = self._run(nav)
        runner.start()
        nav.complete_send(accepted=True)
        runner.request_cancel()
        nav.complete_cancel_response()
        nav.complete_result(GoalStatus.STATUS_UNKNOWN)
        self.assertTrue(done.wait(1.0))
        self.assertEqual(finished[0].execution_status, ExecutionStatus.FAILED.value)
        self.assertEqual(finished[0].error_code, ErrorCode.CANCEL_UNCONFIRMED)
        self.assertTrue(state.unsafe_busy)
        self.assertTrue(state.is_busy)
        ok, code, _ = state.validate_goal('m2', ALLOWED_ROUTE_ID)
        self.assertFalse(ok)
        self.assertEqual(code, ErrorCode.RUNTIME_UNSAFE)

    def test_cancel_timeout_unconfirmed(self):
        clock = {'t': 0.0}

        def now():
            return clock['t']

        nav = FakeNavAdapter()
        state = MissionRuntimeState()
        gen = state.begin('m1')
        finished = []
        done = threading.Event()
        runner = MissionRunner(
            state=state,
            nav=nav,
            pose=_pose(),
            mission_id='m1',
            generation=gen,
            cancel_timeout_sec=1.0,
            on_finished=lambda t: (finished.append(t), done.set()),
            clock=now,
        )
        runner.start()
        nav.complete_send(accepted=True)
        runner.request_cancel()
        self.assertEqual(nav.cancel_calls, 1)
        clock['t'] = 5.0
        runner.poll_timeouts()
        self.assertTrue(done.wait(1.0))
        self.assertEqual(finished[0].error_code, ErrorCode.CANCEL_UNCONFIRMED)
        self.assertTrue(state.unsafe_busy)

    def test_late_result_does_not_republish_terminal(self):
        nav = FakeNavAdapter()
        runner, state, _, finished, done, _, _ = self._run(nav)
        runner.start()
        nav.complete_send(accepted=True)
        nav.complete_result(GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(done.wait(1.0))
        self.assertEqual(len(finished), 1)
        late = Future()
        late.set_result(SimpleNamespace(status=GoalStatus.STATUS_ABORTED, result=None))
        runner._on_result_done(late)
        self.assertEqual(len(finished), 1)
        self.assertEqual(state.last_terminal.execution_status, ExecutionStatus.COMPLETED.value)

    def test_single_terminal_per_mission(self):
        state = MissionRuntimeState()
        gen = state.begin('m1')
        first = state.try_publish_terminal(
            'm1', ExecutionStatus.COMPLETED.value, ErrorCode.NONE, 'one',
            release=True, generation=gen,
        )
        second = state.try_publish_terminal(
            'm1', ExecutionStatus.FAILED.value, ErrorCode.NAV_FAILED, 'two',
            release=True, generation=gen,
        )
        self.assertIsNotNone(first)
        self.assertIsNone(second)
        self.assertEqual(first.mission_id, 'm1')


class TestOwnershipIsolation(unittest.TestCase):
    """R1 fix A: owner/generation isolation across A→B handoff."""

    def test_a_terminal_does_not_wake_b_finish_event(self):
        """A on_finished must not set B's context finish_event/terminal."""
        state = MissionRuntimeState()
        nav_a = FakeNavAdapter()
        runner_a, state, _, finished_a, done_a, ctx_a, gen_a = _start_runner(
            'mission_a', nav_a, state,
        )
        runner_a.start()
        nav_a.complete_send(accepted=True)
        nav_a.complete_result(GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(done_a.wait(1.0))
        self.assertEqual(finished_a[0].mission_id, 'mission_a')
        self.assertTrue(ctx_a.finish_event.is_set())

        nav_b = FakeNavAdapter()
        runner_b, state, _, finished_b, done_b, ctx_b, gen_b = _start_runner(
            'mission_b', nav_b, state,
        )
        self.assertNotEqual(gen_a, gen_b)
        self.assertFalse(ctx_b.finish_event.is_set())
        self.assertIsNone(ctx_b.finish_terminal)

        # Simulate late A finish callback writing again via production path.
        late = MissionTerminal(
            mission_id='mission_a',
            execution_status=ExecutionStatus.COMPLETED.value,
            error_code=ErrorCode.NONE,
            message='late-a',
            generation=gen_a,
        )
        ctx_a.publish_finished(late)

        self.assertFalse(ctx_b.finish_event.is_set())
        self.assertIsNone(ctx_b.finish_terminal)
        self.assertFalse(done_b.is_set())
        self.assertEqual(len(finished_b), 0)

        # B still owns state and can complete independently.
        runner_b.start()
        nav_b.complete_send(accepted=True)
        nav_b.complete_result(GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(done_b.wait(1.0))
        self.assertEqual(finished_b[0].mission_id, 'mission_b')
        self.assertEqual(ctx_b.finish_terminal.mission_id, 'mission_b')

    def test_late_a_send_callback_does_not_pollute_b_phase(self):
        state = MissionRuntimeState()
        nav_a = FakeNavAdapter()
        runner_a, state, feedback_a, finished_a, done_a, _, gen_a = _start_runner(
            'mission_a', nav_a, state,
        )
        runner_a.start()
        # Hold A's send future open; force-complete A via ownership release path.
        # Publish A terminal + release without resolving send (simulates timeout-ish).
        terminal_a = state.try_publish_terminal(
            'mission_a', ExecutionStatus.FAILED.value, ErrorCode.NAV_SEND_TIMEOUT, 'a-done',
            release=True, generation=gen_a,
        )
        self.assertIsNotNone(terminal_a)
        runner_a._local_terminal_done = True
        finished_a.append(terminal_a)
        done_a.set()

        nav_b = FakeNavAdapter()
        runner_b, state, feedback_b, _, _, _, gen_b = _start_runner(
            'mission_b', nav_b, state,
        )
        runner_b.start()
        self.assertEqual(state.phase, Phase.NAVIGATING.value)
        self.assertTrue(state.is_owner('mission_b', gen_b))

        # Late A send resolves after B is navigating; must not flip B to CANCELING.
        runner_a._cancel_requested = True
        nav_a.complete_send(accepted=True)
        self.assertEqual(state.phase, Phase.NAVIGATING.value)
        self.assertTrue(state.is_owner('mission_b', gen_b))
        self.assertFalse(any(f[2] == Phase.CANCELING.value for f in feedback_b))
        self.assertEqual(nav_a.cancel_calls, 0)

    def test_late_a_result_callback_does_not_pollute_b(self):
        state = MissionRuntimeState()
        nav_a = FakeNavAdapter()
        runner_a, state, _, finished_a, done_a, _, gen_a = _start_runner(
            'mission_a', nav_a, state,
        )
        runner_a.start()
        nav_a.complete_send(accepted=True)
        # Capture A's result future, then finish A without using it.
        result_future_a = nav_a.result_future
        self.assertIsNotNone(result_future_a)
        terminal_a = state.try_publish_terminal(
            'mission_a', ExecutionStatus.COMPLETED.value, ErrorCode.NONE, 'a-ok',
            release=True, generation=gen_a,
        )
        self.assertIsNotNone(terminal_a)
        runner_a._local_terminal_done = True
        finished_a.append(terminal_a)
        done_a.set()

        nav_b = FakeNavAdapter()
        runner_b, state, _, finished_b, done_b, _, gen_b = _start_runner(
            'mission_b', nav_b, state,
        )
        runner_b.start()
        nav_b.complete_send(accepted=True)
        self.assertEqual(state.phase, Phase.NAVIGATING.value)

        # Late A result arrives — must not publish a second terminal or change B.
        result_future_a.set_result(
            SimpleNamespace(status=GoalStatus.STATUS_ABORTED, result=None),
        )
        # Explicitly invoke production callback (also fired by Future if same thread).
        runner_a._on_result_done(result_future_a)

        self.assertTrue(state.is_owner('mission_b', gen_b))
        self.assertEqual(state.phase, Phase.NAVIGATING.value)
        self.assertEqual(len(finished_b), 0)
        self.assertEqual(finished_a[0].mission_id, 'mission_a')
        self.assertIn('mission_a', state.used_mission_ids)
        # begin(B) clears last_terminal; A must not have rewritten it via late callback.
        self.assertIsNone(state.last_terminal)

        nav_b.complete_result(GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(done_b.wait(1.0))
        self.assertEqual(finished_b[0].mission_id, 'mission_b')
        self.assertEqual(finished_b[0].execution_status, ExecutionStatus.COMPLETED.value)
        self.assertEqual(state.last_terminal.mission_id, 'mission_b')

    def test_ab_finally_interleaved_slot_clear(self):
        """Old finally must not clear newer active context/runner."""
        slot = MissionActiveSlot()
        gh_a = SimpleNamespace(name='goal_a')
        gh_b = SimpleNamespace(name='goal_b')
        ctx_a = MissionContext(mission_id='mission_a', generation=1, goal_handle=gh_a)
        ctx_b = MissionContext(mission_id='mission_b', generation=2, goal_handle=gh_b)
        ctx_a.runner = SimpleNamespace(name='runner_a')
        ctx_b.runner = SimpleNamespace(name='runner_b')

        slot.install(ctx_a)
        self.assertIs(slot.active, ctx_a)

        # B accepted while A still tearing down.
        slot.install(ctx_b)
        self.assertIs(slot.active, ctx_b)
        self.assertIs(slot.active.runner, ctx_b.runner)

        # A's finally uses production clear_if_owner semantics.
        cleared = slot.clear_if_owner(ctx_a)
        self.assertFalse(cleared)
        self.assertIs(slot.active, ctx_b)
        self.assertIs(slot.active.runner, ctx_b.runner)

        cleared_b = slot.clear_if_owner(ctx_b)
        self.assertTrue(cleared_b)
        self.assertIsNone(slot.active)

    def test_b_cancel_still_sends_nav_cancel_after_canceling_phase(self):
        """Second cancel while CANCELING must still cancel B's Nav2 goal if pending."""
        state = MissionRuntimeState()
        nav = FakeNavAdapter()
        runner, state, _, finished, done, _, _ = _start_runner('mission_b', nav, state)
        runner.start()
        # First cancel before handle exists → phase CANCELING, no cancel_goal yet.
        runner.request_cancel()
        self.assertEqual(state.phase, Phase.CANCELING.value)
        self.assertEqual(nav.cancel_calls, 0)

        nav.complete_send(accepted=True)
        # _on_send_done should have issued cancel; if not, second request_cancel must.
        if nav.cancel_calls == 0:
            runner.request_cancel()
        self.assertGreaterEqual(nav.cancel_calls, 1)
        self.assertIs(nav.cancel_handles[-1], nav.goal_handle)

        nav.complete_cancel_response()
        nav.complete_result(GoalStatus.STATUS_CANCELED)
        self.assertTrue(done.wait(1.0))
        self.assertEqual(finished[0].mission_id, 'mission_b')
        self.assertEqual(finished[0].execution_status, ExecutionStatus.CANCELED.value)

    def test_b_second_cancel_sends_when_handle_ready_but_cancel_not_sent(self):
        """If phase already CANCELING and handle appears without auto-cancel, re-cancel works."""
        state = MissionRuntimeState()
        nav = FakeNavAdapter()
        runner, state, _, _, _, _, _ = _start_runner('mission_b', nav, state)
        runner.start()
        nav.complete_send(accepted=True)
        # Manually mark CANCELING without going through cancel_goal (pollution-style).
        state.set_phase(
            Phase.CANCELING.value,
            mission_id='mission_b',
            generation=runner.generation,
        )
        self.assertEqual(nav.cancel_calls, 0)
        runner.request_cancel()
        self.assertEqual(nav.cancel_calls, 1)
        self.assertIs(nav.cancel_handles[0], nav.goal_handle)

    def test_each_mission_one_terminal_with_correct_id(self):
        state = MissionRuntimeState()
        terminals = []

        nav_a = FakeNavAdapter()
        runner_a, state, _, finished_a, done_a, ctx_a, _ = _start_runner(
            'mission_a', nav_a, state,
        )
        runner_a.start()
        nav_a.complete_send(True)
        nav_a.complete_result(GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(done_a.wait(1.0))
        terminals.append(ctx_a.finish_terminal)

        nav_b = FakeNavAdapter()
        runner_b, state, _, finished_b, done_b, ctx_b, _ = _start_runner(
            'mission_b', nav_b, state,
        )
        runner_b.start()
        nav_b.complete_send(True)
        nav_b.complete_result(GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(done_b.wait(1.0))
        terminals.append(ctx_b.finish_terminal)

        self.assertEqual([t.mission_id for t in terminals], ['mission_a', 'mission_b'])
        self.assertEqual(len(finished_a), 1)
        self.assertEqual(len(finished_b), 1)
        # Wrong-owner publish rejected.
        self.assertIsNone(
            state.try_publish_terminal(
                'mission_a', ExecutionStatus.FAILED.value, ErrorCode.NAV_FAILED, 'nope',
                release=True, generation=1,
            ),
        )

    def test_state_rejects_wrong_owner_phase_and_terminal(self):
        state = MissionRuntimeState()
        gen_a = state.begin('mission_a')
        state.try_publish_terminal(
            'mission_a', ExecutionStatus.COMPLETED.value, ErrorCode.NONE, 'ok',
            release=True, generation=gen_a,
        )
        gen_b = state.begin('mission_b')
        self.assertFalse(
            state.set_phase(
                Phase.CANCELING.value, mission_id='mission_a', generation=gen_a,
            ),
        )
        self.assertEqual(state.phase, Phase.VALIDATING.value)
        self.assertIsNone(
            state.try_publish_terminal(
                'mission_a', ExecutionStatus.FAILED.value, ErrorCode.NAV_FAILED, 'x',
                release=True, generation=gen_a,
            ),
        )
        self.assertTrue(state.is_owner('mission_b', gen_b))


class TestActionServerAcceptCancelRace(unittest.TestCase):
    """Production ActionServer callback interleaving (not Runner send-future race).

    Sequence under test:
      _goal_callback ACCEPT → _cancel_callback → _handle_accepted_callback
    Uses production AcceptCancelBridge / MissionRunner / MissionRuntimeState.
    """

    def _bridge(self):
        state = MissionRuntimeState()
        slot = MissionActiveSlot()
        bridge = AcceptCancelBridge(state, slot)
        return state, slot, bridge

    def test_cancel_before_context_install_is_accepted_and_consumed(self):
        state, slot, bridge = self._bridge()
        mission_id = 'mvp_r1_early_cancel'
        # --- _goal_callback ACCEPT ---
        generation = state.begin(mission_id)
        bridge.on_goal_accepted(mission_id, generation)
        self.assertIsNone(slot.active)

        gh = FakeServerGoalHandle(mission_id)
        # --- _cancel_callback before Context ---
        self.assertTrue(bridge.resolve_cancel(gh))
        self.assertTrue(bridge.window.ticket.pending_cancel)
        self.assertIs(bridge.window.ticket.goal_handle, gh)

        # --- _handle_accepted_callback installs Context ---
        ctx = bridge.install_context_after_accepted(gh, mission_id, generation)
        self.assertIs(slot.active, ctx)
        self.assertTrue(ctx.pending_cancel)
        self.assertIsNone(bridge.window.ticket)

        # Execute path (same order as MissionRuntimeNode._execute_callback):
        # consume pending_cancel → request_cancel → start.
        nav = FakeNavAdapter()
        finished = []
        done = threading.Event()
        runner = MissionRunner(
            state=state,
            nav=nav,
            pose=_pose(),
            mission_id=mission_id,
            generation=generation,
            on_finished=lambda t: (finished.append(t), done.set(), ctx.publish_finished(t)),
        )
        ctx.runner = runner
        pending = ctx.pending_cancel
        ctx.pending_cancel = False
        self.assertTrue(pending)
        runner.request_cancel()
        runner.start()
        self.assertTrue(done.wait(1.0))
        # Cancel before Nav2 send ⇒ confirmed CANCELED (never started).
        self.assertEqual(finished[0].execution_status, ExecutionStatus.CANCELED.value)
        self.assertEqual(finished[0].mission_id, mission_id)
        self.assertEqual(finished[0].error_code, ErrorCode.NONE)
        self.assertEqual(nav.send_calls, 0)
        self.assertFalse(state.is_busy)

    def test_foreign_cancel_before_context_still_rejected(self):
        state, slot, bridge = self._bridge()
        mission_id = 'mvp_r1_owner'
        generation = state.begin(mission_id)
        bridge.on_goal_accepted(mission_id, generation)

        foreign = FakeServerGoalHandle('someone_else')
        self.assertFalse(bridge.resolve_cancel(foreign))
        self.assertFalse(bridge.window.ticket.pending_cancel)

        # Owner cancel still works afterward.
        owner = FakeServerGoalHandle(mission_id)
        self.assertTrue(bridge.resolve_cancel(owner))
        self.assertTrue(bridge.window.ticket.pending_cancel)

        # Installing with owner handle consumes; foreign handle must not steal.
        ctx = bridge.install_context_after_accepted(owner, mission_id, generation)
        self.assertTrue(ctx.pending_cancel)
        self.assertIs(ctx.goal_handle, owner)
        self.assertIsNone(slot.get_by_goal_handle(foreign))

    def test_cancel_with_no_accepted_goal_rejected(self):
        _, _, bridge = self._bridge()
        gh = FakeServerGoalHandle('orphan')
        self.assertFalse(bridge.resolve_cancel(gh))

    def test_cancel_after_context_install_unconfirmed_keeps_busy(self):
        """After Context exists and Nav2 accepted, unconfirmed cancel stays busy."""
        state, slot, bridge = self._bridge()
        mission_id = 'mvp_r1_cancel_unconfirmed'
        generation = state.begin(mission_id)
        bridge.on_goal_accepted(mission_id, generation)
        gh = FakeServerGoalHandle(mission_id)
        # No cancel in ACCEPT→install window.
        ctx = bridge.install_context_after_accepted(gh, mission_id, generation)
        self.assertFalse(ctx.pending_cancel)

        nav = FakeNavAdapter()
        finished = []
        done = threading.Event()
        runner = MissionRunner(
            state=state,
            nav=nav,
            pose=_pose(),
            mission_id=mission_id,
            generation=generation,
            on_finished=lambda t: (finished.append(t), done.set(), ctx.publish_finished(t)),
        )
        ctx.runner = runner
        runner.start()
        nav.complete_send(accepted=True)
        # Production cancel_callback after Context install.
        self.assertTrue(bridge.resolve_cancel(gh))
        self.assertGreaterEqual(nav.cancel_calls, 1)
        nav.complete_cancel_response()
        nav.complete_result(GoalStatus.STATUS_UNKNOWN)
        self.assertTrue(done.wait(1.0))
        self.assertEqual(finished[0].execution_status, ExecutionStatus.FAILED.value)
        self.assertEqual(finished[0].error_code, ErrorCode.CANCEL_UNCONFIRMED)
        self.assertEqual(finished[0].mission_id, mission_id)
        self.assertTrue(state.unsafe_busy)
        self.assertTrue(state.is_busy)
        ok, code, _ = state.validate_goal('next', ALLOWED_ROUTE_ID)
        self.assertFalse(ok)
        self.assertEqual(code, ErrorCode.RUNTIME_UNSAFE)

    def test_cancel_after_context_install_confirmed_returns_canceled(self):
        state, slot, bridge = self._bridge()
        mission_id = 'mvp_r1_cancel_confirmed'
        generation = state.begin(mission_id)
        bridge.on_goal_accepted(mission_id, generation)
        gh = FakeServerGoalHandle(mission_id)
        ctx = bridge.install_context_after_accepted(gh, mission_id, generation)

        nav = FakeNavAdapter()
        finished = []
        done = threading.Event()
        runner = MissionRunner(
            state=state,
            nav=nav,
            pose=_pose(),
            mission_id=mission_id,
            generation=generation,
            on_finished=lambda t: (finished.append(t), done.set(), ctx.publish_finished(t)),
        )
        ctx.runner = runner
        runner.start()
        nav.complete_send(accepted=True)
        self.assertTrue(bridge.resolve_cancel(gh))
        nav.complete_cancel_response()
        nav.complete_result(GoalStatus.STATUS_CANCELED)
        self.assertTrue(done.wait(1.0))
        self.assertEqual(finished[0].execution_status, ExecutionStatus.CANCELED.value)
        self.assertEqual(finished[0].error_code, ErrorCode.NONE)
        self.assertEqual(finished[0].mission_id, mission_id)
        self.assertFalse(state.unsafe_busy)
        self.assertFalse(state.is_busy)

    def test_node_callbacks_delegate_to_accept_cancel_bridge(self):
        """MissionRuntimeNode wires production AcceptCancelBridge (import surface)."""
        try:
            from inspection_interfaces.action import ExecuteInspection  # noqa: F401
        except ImportError:
            self.skipTest('inspection_interfaces not built/sourced yet')
        import inspection_mission.mission_runtime_node as runtime_mod

        # Mirror the exact callback order using the same bridge type the node holds.
        state = MissionRuntimeState()
        slot = MissionActiveSlot()
        bridge = AcceptCancelBridge(state, slot)
        self.assertEqual(type(bridge), runtime_mod.AcceptCancelBridge)

        mission_id = 'mvp_r1_node_path'
        generation = state.begin(mission_id)
        bridge.on_goal_accepted(mission_id, generation)
        gh = FakeServerGoalHandle(mission_id)
        # cancel before handle_accepted — must ACCEPT (True), not REJECT
        self.assertTrue(bridge.resolve_cancel(gh))
        ctx = bridge.install_context_after_accepted(gh, mission_id, generation)
        self.assertTrue(ctx.pending_cancel)
        # Second cancel after install still ACCEPT and marks pending again
        ctx.pending_cancel = False
        self.assertTrue(bridge.resolve_cancel(gh))
        self.assertTrue(ctx.pending_cancel)


class TestTimeoutStopConfirmation(unittest.TestCase):
    """Fix B: send/result timeout must confirm Nav2 stop before releasing mutex."""

    def test_send_timeout_then_late_accept_keeps_busy_until_stop_confirmed(self):
        clock = {'t': 0.0}
        state = MissionRuntimeState()
        nav = FakeNavAdapter()
        runner, state, _, finished, done, _, _ = _start_runner(
            'm_send_to', nav, state, clock=lambda: clock['t'],
            send_timeout_sec=1.0, cancel_timeout_sec=5.0,
        )
        runner.start()
        self.assertEqual(nav.send_calls, 1)
        clock['t'] = 2.0
        runner.poll_timeouts()
        # Must not release / finish yet — acceptance unknown.
        self.assertFalse(done.is_set())
        self.assertTrue(state.is_busy)
        ok, code, _ = state.validate_goal('other', ALLOWED_ROUTE_ID)
        self.assertFalse(ok)
        self.assertEqual(code, ErrorCode.BUSY)

        nav.complete_send(accepted=True)
        self.assertGreaterEqual(nav.cancel_calls, 1)
        self.assertTrue(state.is_busy)
        nav.complete_cancel_response()
        nav.complete_result(GoalStatus.STATUS_CANCELED)
        self.assertTrue(done.wait(1.0))
        self.assertEqual(finished[0].execution_status, ExecutionStatus.FAILED.value)
        self.assertEqual(finished[0].error_code, ErrorCode.NAV_SEND_TIMEOUT)
        self.assertFalse(state.is_busy)

    def test_send_timeout_unknown_accept_marks_unsafe_busy(self):
        clock = {'t': 0.0}
        state = MissionRuntimeState()
        nav = FakeNavAdapter()
        runner, state, _, finished, done, _, _ = _start_runner(
            'm_send_unknown', nav, state, clock=lambda: clock['t'],
            send_timeout_sec=1.0, cancel_timeout_sec=1.0,
        )
        runner.start()
        clock['t'] = 2.0
        runner.poll_timeouts()
        self.assertTrue(state.is_busy)
        self.assertFalse(done.is_set())
        clock['t'] = 4.0
        runner.poll_timeouts()
        self.assertTrue(done.wait(1.0))
        self.assertEqual(finished[0].error_code, ErrorCode.NAV_SEND_TIMEOUT)
        self.assertTrue(state.unsafe_busy)
        self.assertTrue(state.is_busy)
        ok, code, _ = state.validate_goal('next', ALLOWED_ROUTE_ID)
        self.assertFalse(ok)
        self.assertEqual(code, ErrorCode.RUNTIME_UNSAFE)

    def test_user_cancel_then_send_timeout_then_late_accept(self):
        clock = {'t': 0.0}
        state = MissionRuntimeState()
        nav = FakeNavAdapter()
        runner, state, _, finished, done, _, _ = _start_runner(
            'm_cancel_send_to', nav, state, clock=lambda: clock['t'],
            send_timeout_sec=1.0, cancel_timeout_sec=5.0,
        )
        runner.start()
        runner.request_cancel()
        clock['t'] = 2.0
        runner.poll_timeouts()  # send timeout while user-cancel already set
        self.assertFalse(done.is_set())
        self.assertTrue(state.is_busy)
        nav.complete_send(accepted=True)
        self.assertGreaterEqual(nav.cancel_calls, 1)
        nav.complete_cancel_response()
        nav.complete_result(GoalStatus.STATUS_CANCELED)
        self.assertTrue(done.wait(1.0))
        # User cancel wins confirmed semantics.
        self.assertEqual(finished[0].execution_status, ExecutionStatus.CANCELED.value)
        self.assertEqual(finished[0].error_code, ErrorCode.NONE)
        self.assertFalse(state.is_busy)

    def test_result_timeout_waits_cancel_response_keeps_mutex(self):
        clock = {'t': 0.0}
        state = MissionRuntimeState()
        nav = FakeNavAdapter()
        runner, state, _, finished, done, _, _ = _start_runner(
            'm_result_to', nav, state, clock=lambda: clock['t'],
            result_timeout_sec=1.0, cancel_timeout_sec=5.0,
        )
        runner.start()
        nav.complete_send(accepted=True)
        clock['t'] = 2.0
        runner.poll_timeouts()
        self.assertFalse(done.is_set())
        self.assertTrue(state.is_busy)
        self.assertGreaterEqual(nav.cancel_calls, 1)
        # Cancel response pending — still busy.
        ok, code, _ = state.validate_goal('other', ALLOWED_ROUTE_ID)
        self.assertFalse(ok)
        self.assertIn(code, (ErrorCode.BUSY, ErrorCode.RUNTIME_UNSAFE))
        self.assertEqual(len(finished), 0)

    def test_cancel_response_without_result_keeps_mutex(self):
        clock = {'t': 0.0}
        state = MissionRuntimeState()
        nav = FakeNavAdapter()
        runner, state, _, finished, done, _, _ = _start_runner(
            'm_cancel_resp_only', nav, state, clock=lambda: clock['t'],
            result_timeout_sec=1.0, cancel_timeout_sec=5.0,
        )
        runner.start()
        nav.complete_send(accepted=True)
        clock['t'] = 2.0
        runner.poll_timeouts()
        nav.complete_cancel_response()
        # Result not terminal yet — must not finish/release.
        self.assertFalse(done.is_set())
        self.assertTrue(state.is_busy)
        self.assertEqual(len(finished), 0)

    def test_result_timeout_confirmed_stop_returns_nav_result_timeout(self):
        clock = {'t': 0.0}
        state = MissionRuntimeState()
        nav = FakeNavAdapter()
        runner, state, _, finished, done, _, _ = _start_runner(
            'm_result_confirmed', nav, state, clock=lambda: clock['t'],
            result_timeout_sec=1.0, cancel_timeout_sec=5.0,
        )
        runner.start()
        nav.complete_send(accepted=True)
        clock['t'] = 2.0
        runner.poll_timeouts()
        nav.complete_cancel_response()
        nav.complete_result(GoalStatus.STATUS_CANCELED)
        self.assertTrue(done.wait(1.0))
        self.assertEqual(finished[0].execution_status, ExecutionStatus.FAILED.value)
        self.assertEqual(finished[0].error_code, ErrorCode.NAV_RESULT_TIMEOUT)
        self.assertFalse(state.unsafe_busy)
        self.assertFalse(state.is_busy)

    def test_result_timeout_unconfirmed_keeps_unsafe_busy(self):
        clock = {'t': 0.0}
        state = MissionRuntimeState()
        nav = FakeNavAdapter()
        runner, state, _, finished, done, _, _ = _start_runner(
            'm_result_unconf', nav, state, clock=lambda: clock['t'],
            result_timeout_sec=1.0, cancel_timeout_sec=1.0,
        )
        runner.start()
        nav.complete_send(accepted=True)
        clock['t'] = 2.0
        runner.poll_timeouts()
        nav.complete_cancel_response()
        # Never deliver Nav2 result; cancel confirm times out.
        clock['t'] = 4.0
        runner.poll_timeouts()
        self.assertTrue(done.wait(1.0))
        self.assertEqual(finished[0].execution_status, ExecutionStatus.FAILED.value)
        self.assertEqual(finished[0].error_code, ErrorCode.CANCEL_UNCONFIRMED)
        self.assertTrue(state.unsafe_busy)
        ok, code, _ = state.validate_goal('next', ALLOWED_ROUTE_ID)
        self.assertFalse(ok)
        self.assertEqual(code, ErrorCode.RUNTIME_UNSAFE)

    def test_user_cancel_and_result_timeout_no_early_release(self):
        clock = {'t': 0.0}
        state = MissionRuntimeState()
        nav = FakeNavAdapter()
        runner, state, _, finished, done, _, _ = _start_runner(
            'm_cancel_vs_result_to', nav, state, clock=lambda: clock['t'],
            result_timeout_sec=1.0, cancel_timeout_sec=5.0,
        )
        runner.start()
        nav.complete_send(accepted=True)
        runner.request_cancel()
        clock['t'] = 2.0
        # Result deadline fires while user-cancel already in progress — must not release.
        runner.poll_timeouts()
        self.assertFalse(done.is_set())
        self.assertTrue(state.is_busy)
        nav.complete_cancel_response()
        nav.complete_result(GoalStatus.STATUS_CANCELED)
        self.assertTrue(done.wait(1.0))
        self.assertEqual(finished[0].execution_status, ExecutionStatus.CANCELED.value)
        self.assertEqual(len(finished), 1)

    def test_empty_goals_canceling_not_treated_as_stopped(self):
        clock = {'t': 0.0}
        state = MissionRuntimeState()
        nav = FakeNavAdapter()
        runner, state, _, finished, done, _, _ = _start_runner(
            'm_empty_cancel', nav, state, clock=lambda: clock['t'],
            result_timeout_sec=1.0, cancel_timeout_sec=5.0,
        )
        runner.start()
        nav.complete_send(accepted=True)
        clock['t'] = 2.0
        runner.poll_timeouts()
        nav.complete_cancel_response(SimpleNamespace(goals_canceling=[]))
        self.assertFalse(done.is_set())
        self.assertTrue(state.is_busy)
        nav.complete_result(GoalStatus.STATUS_ABORTED)
        self.assertTrue(done.wait(1.0))
        self.assertEqual(finished[0].error_code, ErrorCode.NAV_RESULT_TIMEOUT)

    def test_late_callbacks_after_timeout_terminal_no_second_terminal(self):
        clock = {'t': 0.0}
        state = MissionRuntimeState()
        nav = FakeNavAdapter()
        runner, state, _, finished, done, _, gen = _start_runner(
            'm_late_after_unsafe', nav, state, clock=lambda: clock['t'],
            send_timeout_sec=1.0, cancel_timeout_sec=1.0,
        )
        runner.start()
        clock['t'] = 2.0
        runner.poll_timeouts()
        clock['t'] = 4.0
        runner.poll_timeouts()
        self.assertTrue(done.wait(1.0))
        self.assertEqual(len(finished), 1)
        self.assertTrue(state.unsafe_busy)
        # Late accept must be canceled as orphan cleanup, not publish again.
        nav.complete_send(accepted=True)
        self.assertGreaterEqual(nav.cancel_calls, 1)
        nav.complete_cancel_response()
        nav.complete_result(GoalStatus.STATUS_CANCELED)
        self.assertEqual(len(finished), 1)
        self.assertEqual(finished[0].mission_id, 'm_late_after_unsafe')
        # Still cannot accept new mission.
        ok, code, _ = state.validate_goal('newbie', ALLOWED_ROUTE_ID)
        self.assertFalse(ok)
        self.assertEqual(code, ErrorCode.RUNTIME_UNSAFE)

    def test_goal_callback_rejects_second_mission_while_stop_unconfirmed(self):
        """Runtime-layer: validate_goal (used by _goal_callback) rejects during unsafe."""
        clock = {'t': 0.0}
        state = MissionRuntimeState()
        nav = FakeNavAdapter()
        runner, state, _, finished, done, _, _ = _start_runner(
            'm_runtime_reject', nav, state, clock=lambda: clock['t'],
            result_timeout_sec=1.0, cancel_timeout_sec=1.0,
        )
        runner.start()
        nav.complete_send(accepted=True)
        clock['t'] = 2.0
        runner.poll_timeouts()
        nav.complete_cancel_response()
        clock['t'] = 4.0
        runner.poll_timeouts()
        self.assertTrue(done.wait(1.0))
        self.assertTrue(state.unsafe_busy)

        # Same check MissionRuntimeNode._goal_callback uses before ACCEPT.
        ok, code, msg = state.validate_goal('second_mission', ALLOWED_ROUTE_ID)
        self.assertFalse(ok)
        self.assertEqual(code, ErrorCode.RUNTIME_UNSAFE)
        self.assertIn('unsafe', msg)
        with self.assertRaises(RuntimeError):
            state.begin('second_mission')


class TestNavAdapterSurface(unittest.TestCase):
    def test_nav_adapter_class_exposes_required_methods(self):
        required = (
            'wait_for_server',
            'send_goal_async',
            'send_through_poses_async',
            'cancel_goal_async',
            'get_result_async',
        )
        for name in required:
            self.assertTrue(callable(getattr(NavAdapter, name)))


class TestRuntimeModuleImport(unittest.TestCase):
    def test_runtime_module_loads_policy_constants(self):
        try:
            from inspection_interfaces.action import ExecuteInspection  # noqa: F401
        except ImportError:
            self.skipTest('inspection_interfaces not built/sourced yet')
        import inspection_mission.mission_runtime_node as runtime_mod

        self.assertEqual(runtime_mod.ALLOWED_ROUTE_ID, 'test_room_A')
        self.assertTrue(callable(runtime_mod.MissionRuntimeNode))
        self.assertTrue(hasattr(runtime_mod.MissionRuntimeNode, 'clear_active_context_if_owner'))
        self.assertTrue(hasattr(runtime_mod.MissionRuntimeNode, 'install_active_context'))
        self.assertTrue(hasattr(ExecuteInspection, 'Goal'))
        self.assertTrue(hasattr(runtime_mod, 'AcceptCancelBridge'))


if __name__ == '__main__':
    unittest.main()
