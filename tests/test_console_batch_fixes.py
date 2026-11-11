#!/usr/bin/env python3
"""Console-batch offline tests: takeover, recovery admit gate, map coords.

Uses temp dirs only — never default robot runtime or evidence bags.
Run:
  PYTHONDONTWRITEBYTECODE=1 python3 -m unittest tests.test_console_batch_fixes -v
"""

from __future__ import annotations

import json
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'ros2_ws' / 'src' / 'inspection_mission'))
sys.path.insert(0, str(REPO / 'web' / 'backend'))

from inspection_mission.mission_archive import MissionArchive  # noqa: E402
from inspection_mission.mission_safety_cli import main as cli_main  # noqa: E402
from inspection_mission.mission_state import (  # noqa: E402
    ErrorCode,
    MissionRuntimeState,
)
from inspection_mission.robot_safety import RobotInstanceGuard  # noqa: E402
from mission_api.map_coords import (  # noqa: E402
    map_xy_to_canvas,
    map_xy_to_grid_cell,
    world_xy_yaw_to_map,
    yaw_to_canvas_angle,
)


class TestRecoveryAdmitGate(unittest.TestCase):
    def test_recovery_claimed_blocks_new_goal(self) -> None:
        state = MissionRuntimeState()
        state.set_safety_refuse(True, 'stale mark')
        # Simulate operator confirm claiming recovery while refuse still set:
        # clear refuse first like after partial path? Directly claim recovery.
        state.set_safety_refuse(False)
        self.assertTrue(state.claim_recovery())
        ok, code, msg = state.validate_goal('next', 'test_room_A')
        self.assertFalse(ok)
        self.assertEqual(code, ErrorCode.BUSY)
        self.assertIn('recovery', msg)
        self.assertTrue(state.is_busy)
        # Concurrent admit attempt under claim
        ok2, code2, _, gen = state.reserve_goal(
            'next2', 'test_room_A',
            window=SimpleNamespace(open_unlocked=lambda *a, **k: None),
        )
        self.assertFalse(ok2)
        self.assertEqual(code2, ErrorCode.BUSY)
        self.assertIsNone(gen)
        state.clear_recovery_claim()
        ok3, code3, _ = state.validate_goal('next3', 'test_room_A')
        self.assertTrue(ok3)
        self.assertEqual(code3, ErrorCode.NONE)


class TestTakeoverStaleMark(unittest.TestCase):
    def test_clear_if_owner_still_refuses_foreign(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            rt = Path(td)
            old = RobotInstanceGuard(robot_id='bot', runtime_dir=rt, instance_id='old')
            self.assertTrue(old.acquire_flock()[0])
            old.mark_active(mission_id='m_old')
            old.release_flock()

            new = RobotInstanceGuard(robot_id='bot', runtime_dir=rt, instance_id='new')
            self.assertTrue(new.acquire_flock()[0])
            try:
                cok, cmsg = new.clear_mark_if_owner(instance_id='new')
                self.assertFalse(cok)
                self.assertIn('owned by instance=old', cmsg)
            finally:
                new.release_flock()

    def test_takeover_then_clear_with_new_instance(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            rt = Path(td)
            old = RobotInstanceGuard(robot_id='bot', runtime_dir=rt, instance_id='old')
            self.assertTrue(old.acquire_flock()[0])
            old.mark_active(mission_id='m_old')
            old.release_flock()

            new = RobotInstanceGuard(robot_id='bot', runtime_dir=rt, instance_id='new')
            self.assertTrue(new.acquire_flock()[0])
            try:
                tok = new.arm_operator_confirm()
                ok, _ = new.validate_and_consume_token(
                    instance_id='new', token=tok,
                )
                self.assertTrue(ok)
                take_ok, take_msg, audit = new.takeover_stale_mark_for_recovery()
                self.assertTrue(take_ok, take_msg)
                self.assertTrue(audit.get('needed'))
                self.assertEqual(audit.get('previous_instance_id'), 'old')
                snap = new.read_safety()
                self.assertEqual(snap.payload.get('instance_id'), 'new')
                self.assertEqual(snap.payload.get('state'), 'refuse')
                self.assertIn('takeover_of', snap.payload)
                cok, cmsg = new.clear_mark_if_owner(instance_id='new')
                self.assertTrue(cok, cmsg)
                allow, _, _ = new.startup_gate()
                self.assertTrue(allow)
            finally:
                new.release_flock()

    def test_online_style_operator_confirm_takeover(self) -> None:
        """Mirror Runtime.operator_confirm order without spinning ROS."""
        try:
            from inspection_mission.mission_runtime_node import MissionRuntimeNode
        except ModuleNotFoundError:
            self.skipTest('inspection_interfaces not sourced')
            return
        with tempfile.TemporaryDirectory() as td:
            rt = Path(td)
            db = Path(td) / 'a.sqlite3'
            old = RobotInstanceGuard(robot_id='bot', runtime_dir=rt, instance_id='old')
            self.assertTrue(old.acquire_flock()[0])
            old.mark_active(mission_id='left')
            old.release_flock()

            arch = MissionArchive(db)
            arch.begin_mission(
                mission_id='left', route_id='test_room_A', started_at='t0',
            )
            node = object.__new__(MissionRuntimeNode)
            node.state = MissionRuntimeState()
            node.state.set_safety_refuse(True, 'last run left active mark')
            node._guard = RobotInstanceGuard(
                robot_id='bot', runtime_dir=rt, instance_id='new',
            )
            self.assertTrue(node._guard.acquire_flock()[0])
            node._archive = arch
            node._writer_enabled = True
            try:
                token = node.arm_operator_confirm()
                ok, msg, interrupted = node.operator_confirm(
                    instance_id='new', token=token, interrupt_running=True,
                )
                self.assertTrue(ok, msg)
                self.assertEqual(interrupted, ['left'])
                self.assertFalse(node.state.safety_refuse)
                allow, _, _ = node._guard.startup_gate()
                self.assertTrue(allow)
                self.assertEqual(
                    arch.get_mission('left')['execution_status'], 'INTERRUPTED',
                )
            finally:
                arch.close()
                node._guard.release_flock()

    def test_offline_cli_takeover_foreign_mark(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            rt = Path(td)
            old = RobotInstanceGuard(robot_id='bot', runtime_dir=rt, instance_id='old')
            self.assertTrue(old.acquire_flock()[0])
            old.mark_active(mission_id='m1')
            old.release_flock()

            rc = cli_main(['--runtime-dir', str(rt), '--robot-id', 'bot', 'arm'])
            self.assertEqual(rc, 0)
            armed = json.loads((rt / 'robot_bot_armed_token.json').read_text())
            self.assertNotEqual(armed['instance_id'], 'old')
            rc = cli_main([
                '--runtime-dir', str(rt), '--robot-id', 'bot',
                'confirm',
                '--instance-id', armed['instance_id'],
                '--token', armed['token'],
            ])
            self.assertEqual(rc, 0)
            g = RobotInstanceGuard(robot_id='bot', runtime_dir=rt)
            allow, _, _ = g.startup_gate()
            self.assertTrue(allow)


class TestBootstrapMissingMark(unittest.TestCase):
    def test_bootstrap_only_when_missing(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            rt = Path(td)
            g = RobotInstanceGuard(robot_id='bot', runtime_dir=rt)
            self.assertTrue(g.acquire_flock()[0])
            try:
                allow, reason, _ = g.startup_gate()
                self.assertFalse(allow)
                self.assertEqual(reason, 'safety mark missing')
                ok, msg = g.bootstrap_clear_if_missing()
                self.assertTrue(ok, msg)
                allow2, _, _ = g.startup_gate()
                self.assertTrue(allow2)
                # Active mark must NOT be bootstrapped away
                g.mark_active(mission_id='m')
                ok2, msg2 = g.bootstrap_clear_if_missing()
                self.assertFalse(ok2)
                self.assertIn('already present', msg2)
                allow3, _, snap = g.startup_gate()
                self.assertFalse(allow3)
                self.assertEqual(snap.payload.get('state'), 'active')
            finally:
                g.release_flock()


class TestMapCoords(unittest.TestCase):
    def test_world_to_map_and_canvas_flip(self) -> None:
        origin = [-0.0134, 0.0519, 0.0]
        mx, my, myaw = world_xy_yaw_to_map(3.15, 1.75, -1.57, origin)
        self.assertAlmostEqual(mx, 3.15 - origin[0], places=5)
        self.assertAlmostEqual(my, 1.75 - origin[1], places=5)
        cell = map_xy_to_grid_cell(
            mx, my,
            origin_x=origin[0], origin_y=origin[1],
            resolution=0.05, width=200, height=100,
        )
        self.assertIsNotNone(cell)
        canvas = map_xy_to_canvas(
            mx, my,
            origin_x=origin[0], origin_y=origin[1],
            resolution=0.05, width=200, height=100, flip_y=True,
        )
        self.assertIsNotNone(canvas)
        # Y flip: canvas y should be near height - grid_y
        self.assertGreater(canvas[1], 0)
        self.assertAlmostEqual(yaw_to_canvas_angle(1.0), -1.0)

    def test_amcl_map_pose_must_not_subtract_origin_again(self) -> None:
        # Documented contract: amcl already map-frame; helper is only for route YAML.
        origin = [-0.0134, 0.0519, 0.0]
        amcl_x, amcl_y = 1.0, 2.0
        # Wrong double-apply would shift by origin again — detect the difference.
        wrong_x = amcl_x - origin[0]
        self.assertNotAlmostEqual(wrong_x, amcl_x)
        cell = map_xy_to_grid_cell(
            amcl_x, amcl_y,
            origin_x=origin[0], origin_y=origin[1],
            resolution=0.05, width=400, height=200,
        )
        self.assertEqual(cell, (20, 38))  # floor((1-(-0.0134))/0.05)=20


class TestRecoveryRaceStatic(unittest.TestCase):
    def test_admit_blocked_during_claim_window(self) -> None:
        state = MissionRuntimeState()
        barrier = threading.Event()
        results = []

        def claim_and_hold():
            self.assertTrue(state.claim_recovery())
            barrier.set()
            time.sleep(0.05)
            state.clear_recovery_claim()

        def try_admit():
            barrier.wait(timeout=1.0)
            ok, code, _ = state.validate_goal('race', 'test_room_A')
            results.append((ok, code))

        t1 = threading.Thread(target=claim_and_hold)
        t2 = threading.Thread(target=try_admit)
        t1.start()
        t2.start()
        t1.join(timeout=2.0)
        t2.join(timeout=2.0)
        self.assertEqual(results, [(False, ErrorCode.BUSY)])


if __name__ == '__main__':
    unittest.main()
