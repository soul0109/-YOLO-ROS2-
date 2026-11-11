#!/usr/bin/env python3
"""Offline mock tests for t02_ros_adapter.py — no ROS daemon / Gazebo."""

from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))

import t02_collect_raw as core  # noqa: E402
import t02_ros_adapter as ad  # noqa: E402


def _bgr(color=(10, 20, 30)):
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:] = color
    return img


class MemWorld:
    """In-memory gazebo stand-in for offline adapter tests."""

    def __init__(self) -> None:
        self.entities: Dict[str, Dict[str, Any]] = {}
        self.query_mode = "ok"  # ok | fail
        self.fail_ops: Dict[str, str] = {}
        self.calls: List[tuple] = []

    def query(self, name: str) -> ad.Existence:
        if self.query_mode == "fail":
            return ad.Existence.QUERY_FAILED
        return ad.Existence.PRESENT if name in self.entities else ad.Existence.ABSENT

    def get_pose(self, name: str) -> List[float]:
        if name not in self.entities:
            raise KeyError(name)
        return list(self.entities[name]["pose"])

    def set_pose(self, name: str, pose: Sequence[float], timeout_s: float = 5.0) -> None:
        self.calls.append(("set_pose", name, list(pose), timeout_s))
        if self.fail_ops.get("set_pose") == name:
            raise RuntimeError("set_pose_boom")
        if name not in self.entities:
            raise KeyError(name)
        self.entities[name]["pose"] = list(pose)

    def spawn(self, name: str, xml: str, pose: Sequence[float], timeout_s: float = 5.0) -> None:
        self.calls.append(("spawn", name, list(pose), timeout_s))
        if self.fail_ops.get("spawn") == name:
            raise RuntimeError("spawn_boom")
        if name in self.entities:
            raise RuntimeError("exists")
        self.entities[name] = {"xml": xml, "pose": list(pose)}

    def delete(self, name: str, timeout_s: float = 5.0) -> None:
        self.calls.append(("delete", name, timeout_s))
        if self.fail_ops.get("delete") == name:
            raise RuntimeError("delete_boom")
        self.entities.pop(name, None)


def _adapter(world: MemWorld, **kwargs) -> ad.GazeboSceneAdapter:
    return ad.GazeboSceneAdapter(
        query_fn=world.query,
        get_pose_fn=world.get_pose,
        set_pose_fn=world.set_pose,
        spawn_fn=world.spawn,
        delete_fn=world.delete,
        **kwargs,
    )


class TestImportNoRos(unittest.TestCase):
    def test_import_and_dry_run_zero_ros(self):
        self.assertNotIn("rclpy", sys.modules)
        plan = ad.build_pilot_c2a_plan()
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "dry"
            # ensure adapter import did not pull rclpy
            self.assertNotIn("rclpy", sys.modules)
            report = core.dry_run(plan, out, source_registry=None)
            self.assertTrue(report["ok"])
            self.assertEqual(report["execution_readiness"]["status"], "not_execution_ready")
        self.assertNotIn("rclpy", sys.modules)


class TestServiceOutcomes(unittest.TestCase):
    def test_success_false_exception_timeout(self):
        ok = ad.interpret_future(done=True, result=type("R", (), {"success": True, "status_message": ""})(), exc=None, timed_out=False)
        self.assertTrue(ok.success)
        bad = ad.interpret_future(done=True, result=type("R", (), {"success": False, "status_message": "nope"})(), exc=None, timed_out=False)
        self.assertFalse(bad.success)
        with self.assertRaises(RuntimeError):
            ad.raise_from_outcome("spawn", "x", bad)
        to = ad.interpret_future(done=False, result=None, exc=None, timed_out=True)
        with self.assertRaises(ad.UncertainOpError):
            ad.raise_from_outcome("spawn", "x", to)
        ex = ad.interpret_future(done=True, result=None, exc=RuntimeError("boom"), timed_out=False)
        with self.assertRaises(ad.UncertainOpError):
            ad.raise_from_outcome("delete", "x", ex)


class TestEntityQueryTriState(unittest.TestCase):
    def test_query_failed_not_absent_blocks_spawn(self):
        world = MemWorld()
        world.query_mode = "fail"
        adapter = _adapter(world)
        adapter.set_allowlist([ad.ROBOT_ENTITY, "t02c2a_tmp_occ_A_1"])
        with self.assertRaises(ad.EntityQueryError):
            adapter.entity_exists("t02c2a_tmp_occ_A_1")
        with self.assertRaises(ad.EntityQueryError):
            adapter.spawn("t02c2a_tmp_occ_A_1", ad.visible_box_sdf("t02c2a_tmp_occ_A_1"), [0, 0, 0.4, 0, 0, 0])
        self.assertNotIn("t02c2a_tmp_occ_A_1", world.entities)


class TestClockAndFrames(unittest.TestCase):
    def test_clock_missing_backwards_stale(self):
        ck = ad.ClockState()
        with self.assertRaises(ad.ClockInvalidError):
            ck.validate_fresh(now_mono=10.0)
        ck.update(100.0, 1.0)
        self.assertEqual(ck.validate_fresh(now_mono=1.5), 100.0)
        ck.update(90.0, 2.0)
        with self.assertRaises(ad.ClockInvalidError) as cm:
            ck.validate_fresh(now_mono=2.1)
        self.assertIn("backwards", str(cm.exception))
        ck2 = ad.ClockState()
        ck2.update(50.0, 1.0)
        with self.assertRaises(ad.ClockInvalidError):
            ck2.validate_fresh(max_age_mono=0.2, now_mono=2.0)

    def test_clock_100_90_91_stays_invalid_until_epoch_reset(self):
        ck = ad.ClockState()
        ck.update(100.0, 1.0)
        ck.update(90.0, 2.0)
        ck.update(91.0, 3.0)
        with self.assertRaises(ad.ClockInvalidError):
            ck.validate_fresh(now_mono=3.1)
        ep = ck.reset_epoch()
        self.assertEqual(ep, 1)
        ck.update(92.0, 4.0)
        self.assertEqual(ck.validate_fresh(now_mono=4.1), 92.0)

    def test_frame_validate_and_no_forged_token(self):
        with self.assertRaises(ValueError):
            ad.validate_bgr_frame(_bgr(), "wrong_link")
        with self.assertRaises(ValueError):
            ad.validate_bgr_frame(np.zeros((100, 100, 3), np.uint8), "camera_link")
        ad.validate_bgr_frame(_bgr(), "camera_link")
        ring = ad.FrameRing()
        ring.push(ad.ImageFrame(_bgr(), 1.0, "camera_link", 1, 10.0))
        fr = ring.newest()
        self.assertIsNotNone(fr)
        # association fields must remain unset at capture
        cand = core.FrameCandidate(fr.recv_mono, fr.stamp_sec, fr.frame_id, None, None, fr.bgr, fr.seq)
        self.assertIsNone(cand.case_id)
        self.assertIsNone(cand.scene_op_done_token)

    def test_real_grab_deadline_and_dup_seq(self):
        """Call actual LiveRosBundle.grab via object.__new__ + mocks — no ROS init."""
        import time as _t

        b = object.__new__(ad.LiveRosBundle)
        b.clock_state = ad.ClockState()
        b.frames = ad.FrameRing()
        b.spins = 0

        def spin(timeout_sec):
            b.spins += 1

        b._spin_once = spin  # type: ignore
        b.clock_state.update(1000.0, _t.monotonic())
        with self.assertRaises(TimeoutError):
            ad.LiveRosBundle.grab(b, _t.monotonic() + 0.05)
        self.assertGreater(b.spins, 0)

        # duplicate seq not returned twice
        b.clock_state = ad.ClockState()
        b.clock_state.update(2000.0, _t.monotonic())
        b.frames = ad.FrameRing()
        fr = ad.ImageFrame(_bgr(), 2000.0, "camera_link", 7, _t.monotonic())
        b.frames.push(fr)
        b.frames.seq = 7
        c1 = ad.LiveRosBundle.grab(b, _t.monotonic() + 1.0)
        self.assertEqual(c1.seq, 7)
        # same seq still newest → must wait/timeout rather than re-return
        with self.assertRaises(TimeoutError):
            ad.LiveRosBundle.grab(b, _t.monotonic() + 0.03)


class TestPerCaseClock(unittest.TestCase):
    def test_ros_stamp_revalidated_each_case(self):
        calls = {"n": 0}

        def ros_stamp():
            # call1: batch-level stamp contract probe; call2: case c1; call3+: fail
            calls["n"] += 1
            if calls["n"] >= 3:
                raise ad.ClockInvalidError("ros_clock_missing")
            return 100.0 + float(calls["n"])

        world = MemWorld()
        world.entities[core.ROBOT_ENTITY] = {"xml": "<r/>", "pose": [0.9, 3.0, 0.05, 0, 0, 0]}
        adapter = _adapter(world)
        plan = {
            "session_id": "s",
            "batch_id": "b",
            "usage": core.USAGE_PILOT,
            "execution_ready": True,
            "cases": [
                {
                    "case_id": "c1",
                    "station_id": "A",
                    "distance_m": 1.2,
                    "robot_pose_world": dict(core.STATION_POSE_WORLD["A"]),
                    "pose_frame_id": "world",
                    "usage": core.USAGE_PILOT,
                    "target_states_intent": {"leds": ["green", "green", "green"], "panel": "lit", "leak": "none"},
                    "scene_recipe": {"kind": "normal", "seed": 701},
                    "entity_allowlist": [core.ROBOT_ENTITY],
                    "scene_ops": [],
                },
                {
                    "case_id": "c2",
                    "station_id": "B",
                    "distance_m": 1.2,
                    "robot_pose_world": dict(core.STATION_POSE_WORLD["B"]),
                    "pose_frame_id": "world",
                    "usage": core.USAGE_PILOT,
                    "target_states_intent": {"leds": ["green", "green", "green"], "panel": "lit", "leak": "none"},
                    "scene_recipe": {"kind": "normal", "seed": 702},
                    "entity_allowlist": [core.ROBOT_ENTITY],
                    "scene_ops": [],
                },
            ],
        }
        clock = {"t": 0.0}

        def mono():
            return clock["t"]

        def sleep_fn(s):
            clock["t"] += s

        cam_stamp = {"v": 200.0}

        def grab(deadline):
            img = _bgr()
            return core.FrameCandidate(mono(), cam_stamp["v"], "camera_link", None, None, img, 1)

        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "batch"
            summary = core.run_batch(
                plan,
                out,
                adapter=adapter,
                buffer=core.BoundedFrameBuffer(),
                grab=grab,
                mono=mono,
                ros_stamp_now=ros_stamp,
                sleep_fn=sleep_fn,
                settle_s=0.0,
                wait_timeout_s=0.5,
                capture_source="live_ros",
                source_registry={
                    "known_seeds": [],
                    "known_content_hashes": [],
                    "known_provenance_hashes": [],
                    "completeness": {"declared_complete": True, "statement": "unit"},
                },
            )
            self.assertGreaterEqual(calls["n"], 3)
            self.assertFalse(summary["ok"])
            self.assertEqual(summary["cases"][0]["status"], "captured")
            self.assertEqual(summary["cases"][1]["status"], "clock_or_stamp_error")


class TestActualSnapshotRestore(unittest.TestCase):
    def test_inject_uses_actual_pose_not_home(self):
        world = MemWorld()
        actual = [1.5, 2.5, 0.05, 0, 0, 0.3]
        home = [0.9, 3.0, 0.05, 0, 0, 0]
        world.entities[core.ROBOT_ENTITY] = {"xml": "<r/>", "pose": list(actual)}
        adapter = _adapter(world)
        adapter.set_allowlist([core.ROBOT_ENTITY])
        ops = [
            {
                "op": "set_pose",
                "name": core.ROBOT_ENTITY,
                "pose": ad.station_target_pose("A"),
                "previous_pose": list(home),
            }
        ]
        out = adapter.inject_actual_snapshots(ops)
        self.assertEqual(out[0]["previous_pose"], actual)
        self.assertEqual(out[0]["previous_pose_source"], "actual_world_snapshot")
        self.assertNotEqual(out[0]["previous_pose"], home)

    def test_unverifiable_xml_refuses_delete(self):
        world = MemWorld()
        world.entities["cabinet_a_panel"] = {"xml": "<mystery/>", "pose": [1, 2, 3, 0, 0, 0]}
        adapter = _adapter(world)
        adapter.set_allowlist(["cabinet_a_panel"])
        with self.assertRaises(RuntimeError) as cm:
            adapter.resolve_restore_source_for_delete(
                "cabinet_a_panel",
                {"xml": "<mystery/>", "pose": [1, 2, 3, 0, 0, 0]},
            )
        self.assertIn("unverifiable", str(cm.exception))

    def test_plan_approved_generator_and_fake_hash_pose_not_trusted(self):
        world = MemWorld()
        world.entities["cabinet_a_panel"] = {"xml": "<x/>", "pose": [1.0, 2.0, 0.3, 0, 0, 0]}
        adapter = _adapter(world)
        adapter.set_allowlist(["cabinet_a_panel"])
        xml = ad.approved_visual_xml("cabinet_a_panel", "lit")
        prev = {
            "source_kind": "approved_generator",
            "state": "lit",
            "content_hash": ad.sha256_text(xml),
            "pose": [9.0, 9.0, 9.0, 0, 0, 0],
            "xml": xml,
            "verified": True,
        }
        with self.assertRaises(RuntimeError) as cm:
            adapter.resolve_restore_source_for_delete("cabinet_a_panel", prev)
        self.assertIn("unverifiable", str(cm.exception))


class TestAllowlist(unittest.TestCase):
    def test_whitelist_and_unknown_name(self):
        world = MemWorld()
        adapter = _adapter(world)
        adapter.set_allowlist([core.ROBOT_ENTITY])
        with self.assertRaises(PermissionError):
            adapter.spawn("evil_box", "<sdf/>", [0, 0, 0, 0, 0, 0])
        adapter.set_allowlist([core.ROBOT_ENTITY, "random_name"])
        with self.assertRaises(PermissionError):
            adapter._check_allow("random_name")  # structural policy


class TestPostOpAndRestoreVerify(unittest.TestCase):
    def test_set_pose_readback_fail(self):
        world = MemWorld()
        world.entities[core.ROBOT_ENTITY] = {"xml": "<r/>", "pose": [0, 0, 0, 0, 0, 0]}

        def set_pose_lie(name, pose):
            world.calls.append(("set_pose", name, list(pose)))
            # pretend success but leave old pose
            return

        adapter = ad.GazeboSceneAdapter(
            query_fn=world.query,
            get_pose_fn=world.get_pose,
            set_pose_fn=set_pose_lie,
            spawn_fn=world.spawn,
            delete_fn=world.delete,
            verify_after=True,
        )
        adapter.set_allowlist([core.ROBOT_ENTITY])
        with self.assertRaises(RuntimeError) as cm:
            adapter.set_pose(core.ROBOT_ENTITY, [1, 1, 0.05, 0, 0, 0])
        self.assertIn("readback_mismatch", str(cm.exception))

    def test_restore_readback_fail_aborts_batch(self):
        world = MemWorld()
        world.entities[core.ROBOT_ENTITY] = {"xml": "<r/>", "pose": [0.9, 3.0, 0.05, 0, 0, 0]}
        ent = "t02c2a_tmp_occ_A_1"
        adapter = _adapter(world)
        plan = {
            "session_id": "s",
            "batch_id": "b",
            "usage": core.USAGE_PILOT,
            "execution_ready": True,
            "cases": [
                {
                    "case_id": "c1",
                    "station_id": "A",
                    "distance_m": 1.2,
                    "robot_pose_world": dict(core.STATION_POSE_WORLD["A"]),
                    "pose_frame_id": "world",
                    "usage": core.USAGE_PILOT,
                    "target_states_intent": {"leds": ["green", "green", "green"], "panel": "lit", "leak": "none"},
                    "scene_recipe": {"kind": "partial_occlusion", "seed": 710},
                    "entity_allowlist": [core.ROBOT_ENTITY, ent],
                    "scene_ops": [
                        {
                            "op": "spawn",
                            "name": ent,
                            "xml": ad.visible_box_sdf(ent),
                            "pose": [3.15, 1.4, 0.45, 0, 0, -math.pi / 2],
                        }
                    ],
                },
                {
                    "case_id": "c2",
                    "station_id": "B",
                    "distance_m": 1.2,
                    "robot_pose_world": dict(core.STATION_POSE_WORLD["B"]),
                    "pose_frame_id": "world",
                    "usage": core.USAGE_PILOT,
                    "target_states_intent": {"leds": ["green", "green", "green"], "panel": "lit", "leak": "none"},
                    "scene_recipe": {"kind": "normal", "seed": 711},
                    "entity_allowlist": [core.ROBOT_ENTITY],
                    "scene_ops": [],
                },
            ],
        }
        # After spawn success, arm delete fail on restore
        orig_spawn = world.spawn

        def spawn_arm(name, xml, pose):
            orig_spawn(name, xml, pose)
            world.fail_ops["delete"] = ent

        world.spawn = spawn_arm  # type: ignore
        adapter.spawn_fn = world.spawn

        clock = {"t": 0.0}

        def mono():
            return clock["t"]

        def sleep_fn(s):
            clock["t"] += s

        def grab(deadline):
            return core.FrameCandidate(mono(), 1000.0, "camera_link", None, None, _bgr(), 1)

        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "rb"
            summary = core.run_batch(
                plan,
                out,
                adapter=adapter,
                buffer=core.BoundedFrameBuffer(),
                grab=grab,
                mono=mono,
                ros_stamp_now=lambda: 1000.0,
                sleep_fn=sleep_fn,
                settle_s=0.0,
                wait_timeout_s=0.5,
                capture_source="fake",
                source_registry={
                    "known_seeds": [],
                    "known_content_hashes": [],
                    "known_provenance_hashes": [],
                    "completeness": {"declared_complete": True, "statement": "u"},
                },
            )
            self.assertFalse(summary["ok"])
            self.assertTrue(summary["cases"][0]["restore_errors"])
            self.assertEqual(summary["cases"][1]["status"], "skipped_after_abort")


class TestFakeLiveSeparation(unittest.TestCase):
    def test_sources_not_confused(self):
        self.assertIn("fake", core.CAPTURE_SOURCES)
        self.assertIn("live_ros", core.CAPTURE_SOURCES)
        plan = ad.build_pilot_c2a_plan()
        self.assertEqual(plan["usage"], core.USAGE_PILOT)
        self.assertFalse(plan.get("formal_test_eligible", True) is True and plan["usage"] == core.USAGE_FORMAL)

    def test_formal_refused(self):
        plan = {"usage": core.USAGE_FORMAL, "cases": []}
        with self.assertRaises(PermissionError):
            ad.refuse_if_formal(plan)


class TestPilotPlanGeometry(unittest.TestCase):
    def test_twelve_station_facing_occlusion(self):
        plan = ad.build_pilot_c2a_plan()
        self.assertEqual(len(plan["cases"]), 12)
        errs = ad.static_check_pilot_plan(plan)
        self.assertEqual(errs, [], errs)
        for c in plan["cases"]:
            if not c["case_id"].startswith("pilot_hard_"):
                continue
            sid = c["station_id"]
            spawn = [
                o
                for o in c["scene_ops"]
                if o["op"] == "spawn" and ad.is_tmp_occlusion_name(o["name"])
            ][0]
            rep = ad.occlusion_fov_overlap_report(sid)
            self.assertTrue(rep["ok"], rep)
            self.assertGreaterEqual(rep["overlap_frac"], 0.05)
            self.assertLessEqual(rep["overlap_frac"], 0.85)
            # occluder between camera and cabinet along station facing
            cam = ad.compute_occlusion_geometry(sid)["camera_xyz"]
            panel = ad.compute_occlusion_geometry(sid)["target_panel_xyz"]
            occ = spawn["pose"][:3]
            # param t along cam→panel
            vx, vy = panel[0] - cam[0], panel[1] - cam[1]
            t = ((occ[0] - cam[0]) * vx + (occ[1] - cam[1]) * vy) / (vx * vx + vy * vy)
            self.assertGreater(t, 0.1)
            self.assertLess(t, 0.95)


class TestCmdRunnerContract(unittest.TestCase):
    def test_cmd_timeout_marked(self):
        def runner(argv, timeout_s):
            return ad.CmdResult(124, "", "timeout", timed_out=True)

        world = MemWorld()
        adapter = _adapter(world, cmd_runner=runner)
        self.assertTrue(adapter.cmd_runner(["gz", "model", "-h"], 1.0).timed_out)


class TestCapsSummary(unittest.TestCase):
    def test_capability_preflight_shape(self):
        caps = ad.GazeboCapabilities(
            spawn_entity=True,
            delete_entity=True,
            gz_model_pose_candidate=True,
            gz_model_pose_verified=False,
            get_entity_state=False,
            set_entity_state=False,
        )
        s = caps.summary()
        self.assertTrue(s["can_spawn_delete"])
        self.assertFalse(s["can_set_pose"])  # candidate ≠ verified
        self.assertFalse(s["can_read_pose"])
        caps.gz_model_pose_verified = True
        self.assertTrue(caps.can_read_pose())
        self.assertTrue(caps.can_set_pose())


class TestCliEmitNoRos(unittest.TestCase):
    def test_emit_c2a_plan_cli(self):
        self.assertNotIn("rclpy", sys.modules)
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "pilot.json"
            rc = core.main(["--emit-pilot-c2a-plan", str(p)])
            self.assertEqual(rc, 0)
            plan = json.loads(p.read_text())
            self.assertEqual(len(plan["cases"]), 12)
            self.assertTrue(any(o["op"] == "delete" for o in plan["cases"][0]["scene_ops"]))
        self.assertNotIn("rclpy", sys.modules)


class TestR1ControlledSourceLifecycle(unittest.TestCase):
    def test_register_replace_restore_source_and_pose(self):
        world = MemWorld()
        name = "cabinet_a_panel"
        old_xml = ad.approved_visual_xml(name, "dark")
        new_xml = ad.approved_visual_xml(name, "lit")
        old_pose = [3.15, 0.555, 0.32, 0, 0, 0]
        new_pose = [3.15, 0.555, 0.32, 0, 0, 0]
        world.entities[name] = {"xml": old_xml, "pose": list(old_pose)}
        adapter = _adapter(world)
        adapter.set_allowlist([name])
        with self.assertRaises(PermissionError):
            adapter.register_world_sdf_source(
                name=name,
                xml=old_xml,
                world_path="/tmp/test.world",
                world_hash="abc",
                pose=old_pose,
                clean_world_attested=False,
            )
        adapter.register_world_sdf_source(
            name=name,
            xml=old_xml,
            world_path="/tmp/test.world",
            world_hash="abc123",
            pose=old_pose,
            clean_world_attested=True,  # offline attestation fixture — not field claim
        )
        ops = [
            {
                "op": "delete",
                "name": name,
                "previous": {"xml": "<plan/>", "pose": [9, 9, 9, 0, 0, 0]},
            },
            {"op": "spawn", "name": name, "xml": new_xml, "pose": new_pose},
        ]
        inj = adapter.inject_actual_snapshots(ops)
        self.assertEqual(inj[0]["previous"]["pose"], old_pose)
        self.assertNotEqual(inj[0]["previous"]["pose"], [9, 9, 9, 0, 0, 0])
        restore: list = []
        core.execute_ops_accounting(adapter, inj, restore)
        self.assertEqual(adapter.restore_sources[name].source_kind, "session_spawn")
        self.assertEqual(adapter.restore_sources[name].content_hash, ad.sha256_text(new_xml))
        errs = core.restore_scene(adapter, restore)
        self.assertEqual(errs, [])
        self.assertEqual(adapter.restore_sources[name].source_kind, "world_sdf")
        self.assertEqual(adapter.restore_sources[name].content_hash, ad.sha256_text(old_xml))
        self.assertEqual(world.entities[name]["pose"], old_pose)


class TestR1ServiceAndPose(unittest.TestCase):
    def test_call_service_outcomes_with_deadline(self):
        b = object.__new__(ad.LiveRosBundle)
        b._rclpy = type("R", (), {})()
        spins = {"n": 0}

        def spin_until(node, fut, timeout_sec=1.0):
            spins["n"] += 1
            # leave fut not done to simulate timeout within deadline
            return None

        b._rclpy.spin_until_future_complete = spin_until
        b.node = object()

        class Cli:
            def wait_for_service(self, timeout_sec=1.0):
                return True

            def call_async(self, req):
                return type("F", (), {"done": lambda self: False, "result": lambda self: None})()

        out = ad.LiveRosBundle._call_service(b, Cli(), object(), 0.01)
        self.assertTrue(out.timed_out)

        class CliFail:
            def wait_for_service(self, timeout_sec=1.0):
                return False

            def call_async(self, req):
                raise AssertionError("should not call")

        out2 = ad.LiveRosBundle._call_service(b, CliFail(), object(), 1.0)
        self.assertTrue(out2.timed_out)
        self.assertEqual(out2.message, "service_unavailable")

        class Fut:
            def done(self):
                return True

            def result(self):
                raise RuntimeError("boom")

        class CliExc:
            def wait_for_service(self, timeout_sec=1.0):
                return True

            def call_async(self, req):
                return Fut()

        def spin_ok(node, fut, timeout_sec=1.0):
            return None

        b._rclpy.spin_until_future_complete = spin_ok
        out3 = ad.LiveRosBundle._call_service(b, CliExc(), object(), 1.0)
        self.assertIsNotNone(out3.exception)

        class FutFalse:
            def done(self):
                return True

            def result(self):
                return type("R", (), {"success": False, "status_message": "nope", "model_names": []})()

        class CliFalse:
            def wait_for_service(self, timeout_sec=1.0):
                return True

            def call_async(self, req):
                return FutFalse()

        out4 = ad.LiveRosBundle._call_service(b, CliFalse(), object(), 1.0)
        self.assertIs(out4.success, False)

    def test_get_model_list_success_false_is_query_failed(self):
        def query_fail_success(_name):
            # emulate live query branch
            out = ad.interpret_future(
                done=True,
                result=type("R", (), {"success": False, "status_message": "x", "model_names": ["a"]})(),
                exc=None,
                timed_out=False,
            )
            if out.success is False:
                return ad.Existence.QUERY_FAILED
            return ad.Existence.ABSENT

        self.assertIs(query_fail_success("a"), ad.Existence.QUERY_FAILED)

    def test_pose6_and_strict_cli(self):
        self.assertFalse(ad.pose6_close([0, 0, 0, 0, 0, 0], [float("nan"), 0, 0, 0, 0, 0]))
        self.assertFalse(ad.pose6_close([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, float("inf")]))
        # roll mismatch
        self.assertFalse(ad.pose6_close([0, 0, 0, 0, 0, 0], [0, 0, 0, 0.5, 0, 0]))
        self.assertEqual(ad.parse_gz_pose_stdout("1 2 3 0.1 0.2 0.3"), [1, 2, 3, 0.1, 0.2, 0.3])
        with self.assertRaises(RuntimeError):
            ad.parse_gz_pose_stdout("name: robot 1 2 3 0 0 0")

    def test_quat_full_rpy(self):
        q = ad.rpy_to_quat(0.2, -0.1, 1.0)
        r, p, y = ad.quat_to_rpy(*q)
        self.assertAlmostEqual(r, 0.2, places=5)
        self.assertAlmostEqual(p, -0.1, places=5)
        self.assertAlmostEqual(y, 1.0, places=5)

    def test_grab_fails_on_clock_rollback_during_wait(self):
        import time as _t

        b = object.__new__(ad.LiveRosBundle)
        b.clock_state = ad.ClockState()
        b.frames = ad.FrameRing()
        b.clock_state.update(100.0, _t.monotonic())

        def spin(_tmo):
            # inject rollback while waiting
            b.clock_state.update(50.0, _t.monotonic())

        b._spin_once = spin  # type: ignore
        with self.assertRaises(ad.ClockInvalidError):
            ad.LiveRosBundle.grab(b, _t.monotonic() + 1.0)
        self.assertEqual(len(b.frames), 0)

    def test_timeout_propagates_to_backend(self):
        world = MemWorld()
        world.entities["t02c2a_tmp_occ_A_1"] = {"xml": "<a/>", "pose": [0, 0, 0, 0, 0, 0]}
        # entity absent for spawn test
        world.entities.clear()
        adapter = _adapter(world)
        adapter.set_allowlist(["t02c2a_tmp_occ_A_1"])
        adapter.spawn("t02c2a_tmp_occ_A_1", ad.visible_box_sdf("t02c2a_tmp_occ_A_1"), [0, 0, 0.4, 0, 0, 0], timeout_s=3.5)
        self.assertEqual(world.calls[-1][-1], 3.5)


if __name__ == "__main__":
    unittest.main()
