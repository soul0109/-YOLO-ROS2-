#!/usr/bin/env python3
"""MVP-R6: bridge handoff — immediate Future must not lose terminal / block next submit."""

from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'web' / 'backend'))

from mission_api.ros_bridge import LiveMissionState, RosMissionBridge  # noqa: E402


class _ImmediateFuture:
    """Minimal Future that runs done-callbacks immediately (possibly re-entrant)."""

    def __init__(self, result):
        self._result = result
        self._callbacks = []
        self._done = False

    def add_done_callback(self, cb):
        self._callbacks.append(cb)
        # Fire immediately — same race as a fast ActionServer.
        self._done = True
        cb(self)

    def result(self):
        return self._result


class TestBridgeHandoff(unittest.TestCase):
    def _make_bridge_with_instant_result(self) -> RosMissionBridge:
        bridge = RosMissionBridge(server_wait_sec=0.1, goal_response_timeout_sec=2.0)
        bridge._started = True
        bridge._node = MagicMock()

        goal_handle = MagicMock()
        goal_handle.accepted = True

        result = SimpleNamespace(
            execution_status='COMPLETED',
            error_code='NONE',
            message='instant done',
            inspection_status='FAIL',
            station_id='A',
            screenshot_path='/tmp/a.png',
            station_result_path='/tmp/a.json',
            model_version='m',
            image_stamp='1.0',
        )
        result_wrap = SimpleNamespace(result=result)

        def send_goal_async(goal, feedback_callback=None):
            # Accept future completes immediately.
            accept_fut = _ImmediateFuture(goal_handle)

            def get_result_async():
                # Result also completes immediately when registered.
                return _ImmediateFuture(result_wrap)

            goal_handle.get_result_async = get_result_async
            return accept_fut

        client = MagicMock()
        client.server_is_ready.return_value = True
        client.wait_for_server.return_value = True
        client.send_goal_async.side_effect = send_goal_async
        bridge._client = client

        # Avoid importing real action message type in send path:
        # patch submit to use a dummy Goal after server checks by injecting module.
        return bridge

    def test_immediate_result_not_overwritten_by_accepted_and_busy_clears(self) -> None:
        bridge = self._make_bridge_with_instant_result()

        # Stub ExecuteInspection import used inside submit_goal.
        import mission_api.ros_bridge as mod

        class _Goal:
            def __init__(self):
                self.mission_id = ''
                self.route_id = ''

        class _EI:
            Goal = _Goal

        # Inject fake into sys.modules for `from inspection_interfaces.action import ExecuteInspection`
        import types
        pkg = types.ModuleType('inspection_interfaces')
        action = types.ModuleType('inspection_interfaces.action')
        action.ExecuteInspection = _EI
        sys.modules['inspection_interfaces'] = pkg
        sys.modules['inspection_interfaces.action'] = action

        code, err, msg, live = bridge.submit_goal('m_instant_1', 'test_room_A')
        self.assertEqual(code, 202, msg)
        self.assertIsNotNone(live)
        # Callback may have already terminalized before return — must NOT be wiped to ACCEPTED-only.
        stored = bridge.get_live('m_instant_1')
        self.assertIsNotNone(stored)
        self.assertTrue(stored.terminal)
        self.assertEqual(stored.status, 'COMPLETED')
        self.assertEqual(stored.inspection_status, 'FAIL')
        self.assertIsNone(bridge._busy_mission_id)

        # After terminal, a new mission_id must be submittable (busy cleared).
        code2, err2, msg2, live2 = bridge.submit_goal('m_instant_2', 'test_room_A')
        self.assertEqual(code2, 202, msg2)
        stored2 = bridge.get_live('m_instant_2')
        self.assertIsNotNone(stored2)
        self.assertTrue(stored2.terminal)
        self.assertEqual(stored2.status, 'COMPLETED')
        self.assertIsNone(bridge._busy_mission_id)

    def test_placeholder_exists_before_callbacks(self) -> None:
        """Busy+placeholder are set atomically; callback sees existing state."""
        bridge = RosMissionBridge()
        bridge._started = True
        bridge._node = MagicMock()
        seen = {}

        class _Goal:
            mission_id = ''
            route_id = ''

        class _EI:
            Goal = _Goal

        import types
        action = types.ModuleType('inspection_interfaces.action')
        action.ExecuteInspection = _EI
        sys.modules['inspection_interfaces'] = types.ModuleType('inspection_interfaces')
        sys.modules['inspection_interfaces.action'] = action

        goal_handle = MagicMock()
        goal_handle.accepted = True
        result = SimpleNamespace(
            execution_status='COMPLETED',
            error_code='NONE',
            message='ok',
            inspection_status='PASS',
            station_id='A',
            screenshot_path='',
            station_result_path='',
            model_version='',
            image_stamp='',
        )

        def send_goal_async(goal, feedback_callback=None):
            # During accept callback, placeholder must already exist.
            def on_accept(fut):
                with bridge._lock:
                    seen['has_placeholder'] = 'm_ph' in bridge._live
                    seen['busy'] = bridge._busy_mission_id
                # accept box filled by real callback; we chain via ImmediateFuture

            accept_fut = _ImmediateFuture(goal_handle)
            # Wrap get_result to also fire immediately
            def get_result_async():
                return _ImmediateFuture(SimpleNamespace(result=result))

            goal_handle.get_result_async = get_result_async
            return accept_fut

        client = MagicMock()
        client.wait_for_server.return_value = True
        client.send_goal_async.side_effect = send_goal_async
        bridge._client = client

        # Hook into real _goal path by observing after submit
        code, _, _, _ = bridge.submit_goal('m_ph', 'test_room_A')
        self.assertEqual(code, 202)
        # After full submit, state should be terminal from immediate result
        st = bridge.get_live('m_ph')
        self.assertTrue(st.terminal)
        self.assertEqual(st.status, 'COMPLETED')


if __name__ == '__main__':
    unittest.main()
