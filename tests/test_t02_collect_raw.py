#!/usr/bin/env python3
"""Noros tests for t02_collect_raw.py (C1-R1/R2 contracts). No ROS/Gazebo."""

from __future__ import annotations

import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
import t02_collect_raw as t02  # noqa: E402


def _pose(station="A"):
    return dict(t02.STATION_POSE_WORLD[station])


def _registry(**extra):
    reg = {
        "known_seeds": [],
        "known_content_hashes": [],
        "known_provenance_hashes": [],
        "completeness": {
            "declared_complete": True,
            "statement": "unit-test fixture: intentionally empty known sets; not independence certification",
        },
    }
    reg.update(extra)
    return reg


def _base_case(cid="c1", station="A", usage=t02.USAGE_PILOT, **extra):
    c = {
        "case_id": cid,
        "station_id": station,
        "distance_m": 1.2,
        "robot_pose_world": _pose(station),
        "pose_frame_id": "world",
        "usage": usage,
        "execution_ready": True,
        "target_states_intent": {"leds": ["green", "green", "green"], "panel": "lit", "leak": "none"},
        "scene_recipe": {"kind": "normal", "seed": 301},
        "scene_ops": [],
        "entity_allowlist": [],
    }
    c.update(extra)
    return c


def _plan(cases, usage=t02.USAGE_PILOT, batch="b_test", session="s_test", execution_ready=True):
    return {
        "session_id": session,
        "batch_id": batch,
        "usage": usage,
        "execution_ready": execution_ready,
        "cases": cases,
    }


class Clock:
    def __init__(self, t0=0.0):
        self.t = t0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


class StampClock:
    """Separate ROS/sim stamp channel — never subtract from mono."""

    def __init__(self, t0=5000.0):
        self.t = t0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def _run_ok_batch(plan, out: Path, **kwargs):
    clock = kwargs.pop("clock", Clock())
    stamp = kwargs.pop("stamp", StampClock())
    cam = kwargs.pop("cam", t02.FakeCamera(stamp=stamp()))
    adapter = kwargs.pop("adapter", t02.FakeSceneAdapter())
    if t02.ROBOT_ENTITY not in adapter.entities:
        adapter.entities[t02.ROBOT_ENTITY] = {"xml": "<r/>", "pose": [0.9, 3.0, 0.05, 0, 0, 0]}
    buf = t02.BoundedFrameBuffer()
    settle = kwargs.pop("settle_s", 0.02)
    wait = kwargs.pop("wait_timeout_s", 1.0)
    registry = kwargs.pop("source_registry", _registry())

    def grab(deadline):
        return cam.grab(deadline, clock)

    summary = t02.run_batch(
        plan,
        out,
        adapter=adapter,
        buffer=buf,
        grab=grab,
        mono=clock,
        ros_stamp_now=stamp,
        sleep_fn=lambda s: clock.advance(s),
        settle_s=settle,
        wait_timeout_s=wait,
        capture_source="fake",
        source_registry=registry,
    )
    return summary, adapter, cam, clock, stamp

# ----- retained / updated original contracts -----


class TestDryRunNoRos(unittest.TestCase):
    def test_dry_run_does_not_import_rclpy(self):
        sys.modules.pop("rclpy", None)
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "dry"
            report = t02.dry_run(_plan([_base_case()]), out, source_registry=None)
            self.assertTrue(report["ok"])
            self.assertNotIn("rclpy", sys.modules)
            self.assertEqual(report["source_conflict_review"]["isolation_status"], "not_checked")


class TestOutdirRefuse(unittest.TestCase):
    def test_refuse_existing_outdir(self):
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "exists"
            out.mkdir()
            with self.assertRaises(FileExistsError):
                t02.dry_run(_plan([_base_case()]), out)


class TestUsageAndIds(unittest.TestCase):
    def test_usage_mix_rejected(self):
        cases = [
            _base_case("a", usage=t02.USAGE_PILOT),
            _base_case("b", usage=t02.USAGE_FORMAL, scene_recipe={"kind": "x", "seed": 302}),
        ]
        errs = t02.validate_plan(_plan(cases, usage=t02.USAGE_PILOT))
        self.assertTrue(any("usage_mix" in e for e in errs))

    def test_duplicate_case_id_rejected(self):
        errs = t02.validate_plan(_plan([_base_case("x"), _base_case("x", scene_recipe={"seed": 303})]))
        self.assertTrue(any("duplicate" in e for e in errs))

    def test_seed42_forbidden(self):
        errs = t02.validate_plan(_plan([_base_case(scene_recipe={"kind": "bad", "seed": 42})]))
        self.assertTrue(any("seed42" in e for e in errs))

    def test_unsafe_id_rejected(self):
        errs = t02.validate_plan(_plan([_base_case("a/../b")]))
        self.assertTrue(any("unsafe_id" in e or "bad_id" in e for e in errs))

    def test_nan_pose_rejected(self):
        c = _base_case()
        c["robot_pose_world"]["x"] = float("nan")
        errs = t02.validate_plan(_plan([c]))
        self.assertTrue(any("non_finite" in e for e in errs))

    def test_empty_allowlist_with_ops_rejected(self):
        c = _base_case(
            scene_ops=[{"op": "spawn", "name": "tmp1", "xml": "<a/>", "pose": [0, 0, 0, 0, 0, 0]}],
            entity_allowlist=[],
        )
        errs = t02.validate_plan(_plan([c]))
        self.assertTrue(any("not_in_explicit_allowlist" in e for e in errs))

    def test_delete_requires_previous(self):
        c = _base_case(
            scene_ops=[{"op": "delete", "name": "e1"}],
            entity_allowlist=["e1"],
        )
        errs = t02.validate_plan(_plan([c]))
        self.assertTrue(any("delete_requires_previous" in e for e in errs))


class TestSourceRegistry(unittest.TestCase):
    def test_no_registry_not_checked(self):
        rep = t02.check_source_conflicts(_plan([_base_case()]), None)
        self.assertEqual(rep["isolation_status"], "not_checked")

    def test_seed_note_change_same_content_hash(self):
        c1 = _base_case(scene_recipe={"kind": "n", "seed": 350, "note": "a"})
        c2 = _base_case(scene_recipe={"kind": "n", "seed": 999, "note": "b"})
        self.assertEqual(t02.content_hash(c1), t02.content_hash(c2))
        self.assertNotEqual(
            t02.provenance_hash(c1, _plan([c1])),
            t02.provenance_hash(c2, _plan([c2])),
        )

    def test_content_conflict_via_registry(self):
        c = _base_case(scene_recipe={"kind": "n", "seed": 350})
        ch = t02.content_hash(c)
        reg = _registry(known_content_hashes=[ch])
        rep = t02.check_source_conflicts(_plan([c]), reg)
        self.assertEqual(rep["n_conflicts"], 1)
        # changing only seed/note must still collide on content
        c2 = _base_case(scene_recipe={"kind": "n", "seed": 351, "note": "other"})
        rep2 = t02.check_source_conflicts(_plan([c2]), reg)
        self.assertEqual(rep2["n_conflicts"], 1)


class TestBuffer(unittest.TestCase):
    def test_buffer_bounded(self):
        buf = t02.BoundedFrameBuffer(maxlen=4)
        for i in range(20):
            fr = t02.FrameCandidate(float(i), 10.0 + i, "camera_link", None, None, np.zeros((4, 4, 3), np.uint8), i)
            self.assertIsNone(buf.push(fr))
        self.assertEqual(len(buf), 4)
        self.assertEqual(buf.dropped, 16)

    def test_stamp_backwards(self):
        buf = t02.BoundedFrameBuffer()
        self.assertIsNone(buf.push(t02.FrameCandidate(1, 5, "c", None, None, np.zeros((2, 2, 3), np.uint8), 1)))
        self.assertEqual(
            buf.push(t02.FrameCandidate(1.1, 4, "c", None, None, np.zeros((2, 2, 3), np.uint8), 2)),
            "image_stamp_went_backwards",
        )

    def test_late_old_stamp_rejected(self):
        buf = t02.BoundedFrameBuffer()
        # new recv_mono but stamp older than ros lower bound
        fr = t02.FrameCandidate(100.0, 1.0, "camera_link", None, None, np.zeros((8, 8, 3), np.uint8), 1)
        buf.push(fr)
        chosen, reason = buf.select_fresh(
            case_id="c", scene_op_done_token="t", after_recv_mono=50.0, min_image_stamp_sec=10.0
        )
        self.assertIsNone(chosen)
        self.assertEqual(reason, "stamp_before_ros_lower_bound")

    def test_wrong_case_token_not_mutated(self):
        buf = t02.BoundedFrameBuffer()
        fr = t02.FrameCandidate(100.0, 20.0, "camera_link", "other", "tokX", np.zeros((8, 8, 3), np.uint8), 1)
        orig_case, orig_tok = fr.case_id, fr.scene_op_done_token
        buf.push(fr)
        chosen, reason = buf.select_fresh(
            case_id="mine", scene_op_done_token="tokY", after_recv_mono=0.0, min_image_stamp_sec=0.0
        )
        self.assertIsNone(chosen)
        self.assertEqual(fr.case_id, orig_case)
        self.assertEqual(fr.scene_op_done_token, orig_tok)


class TestIntentReview(unittest.TestCase):
    def test_unknown_forbidden_in_intent(self):
        c = _base_case()
        c["target_states_intent"]["leds"] = ["UNKNOWN", "green", "green"]
        errs = t02.validate_plan(_plan([c]))
        self.assertTrue(any("UNKNOWN_not_allowed" in e for e in errs))

    def test_sidecar_fake_marks(self):
        plan = _plan([_base_case()])
        side = t02.build_sidecar(
            case=plan["cases"][0],
            plan=plan,
            image_rel="images/x.png",
            image_stamp_sec=1.0,
            image_frame_id="camera_link",
            recv_mono=1.0,
            scene_op_done_token="t",
            timing={"scene_op_done_mono": 1.0, "settle_done_mono": 1.1, "ros_stamp_lower_bound": 9.0},
            capture_source="fake",
        )
        self.assertEqual(side["capture_source"], "fake")
        self.assertTrue(side["synthetic"])
        self.assertFalse(side["formal_test_eligible"])
        self.assertEqual(side["review"]["status"], "unreviewed")
        self.assertFalse(side["boxes_review_passed"])


class TestWorldFields(unittest.TestCase):
    def test_map_without_calib(self):
        c = _base_case()
        c["robot_pose_map"] = {"x": 1, "y": 2, "z": 0, "yaw": 0}
        self.assertTrue(any("map_pose_without_calib" in e for e in t02.validate_plan(_plan([c]))))

    def test_pose_frame_world(self):
        c = _base_case()
        c["pose_frame_id"] = "map"
        self.assertTrue(any("pose_frame_must_be_world" in e for e in t02.validate_plan(_plan([c]))))


# ----- R1 restore / interrupt / grab paths -----


class TestRestoreContracts(unittest.TestCase):
    def test_spawn_then_grab_raises_restores(self):
        ent = "tmp_occ1"
        xml = t02._visible_box_sdf(ent)
        case = _base_case(
            "gfail",
            scene_ops=[{"op": "spawn", "name": ent, "xml": xml, "pose": [1, 2, 0.4, 0, 0, 0]}],
            entity_allowlist=[ent],
            scene_recipe={"kind": "partial_occlusion", "seed": 360},
        )
        plan = _plan([case, _base_case("after", scene_recipe={"kind": "n", "seed": 361})])
        clock = Clock()
        stamp = StampClock()
        cam = t02.FakeCamera(stamp=stamp())
        cam.raise_exc = RuntimeError("grab_boom")
        adapter = t02.FakeSceneAdapter()
        buf = t02.BoundedFrameBuffer()

        def grab(deadline):
            return cam.grab(deadline, clock)

        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "b"
            summary = t02.run_batch(
                plan,
                out,
                adapter=adapter,
                buffer=buf,
                grab=grab,
                mono=clock,
                ros_stamp_now=stamp,
                sleep_fn=lambda s: clock.advance(s),
                capture_source="fake",
                source_registry=_registry(),
            )
            self.assertFalse(summary["ok"])
            self.assertEqual(summary["status"], "batch_aborted")
            self.assertNotIn(ent, adapter.entities)  # restored (deleted)
            self.assertEqual(summary["cases"][1]["status"], "skipped_after_abort")
            self.assertNotEqual(summary["status"], "batch_complete")
            self.assertFalse(summary["formal_test_eligible"])

    def test_settle_keyboard_interrupt(self):
        plan = _plan([_base_case("intr")])
        clock = Clock()
        stamp = StampClock()
        cam = t02.FakeCamera(stamp=stamp())
        adapter = t02.FakeSceneAdapter()
        buf = t02.BoundedFrameBuffer()
        calls = {"n": 0}

        def sleep_fn(s):
            calls["n"] += 1
            if calls["n"] == 1:
                raise KeyboardInterrupt()
            clock.advance(s)

        def grab(deadline):
            return cam.grab(deadline, clock)

        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "bi"
            summary = t02.run_batch(
                plan,
                out,
                adapter=adapter,
                buffer=buf,
                grab=grab,
                mono=clock,
                ros_stamp_now=stamp,
                sleep_fn=sleep_fn,
                capture_source="fake",
                source_registry=_registry(),
            )
            self.assertFalse(summary["ok"])
            self.assertIn(summary["status"], {"batch_interrupted", "batch_aborted"})
            st = json.loads((out / "batch_status.json").read_text())["status"]
            self.assertNotEqual(st, "running")
            self.assertNotEqual(st, "batch_complete")

    def test_write_fail_and_restore_fail_skips_rest(self):
        ent = "tmp_w"
        xml = t02._visible_box_sdf(ent)
        case = _base_case(
            "wfail",
            scene_ops=[{"op": "spawn", "name": ent, "xml": xml, "pose": [0, 0, 0.4, 0, 0, 0]}],
            entity_allowlist=[ent],
            scene_recipe={"kind": "partial_occlusion", "seed": 370},
        )
        plan = _plan([case, _base_case("later", scene_recipe={"kind": "n", "seed": 371})])
        clock = Clock()
        stamp = StampClock()
        cam = t02.FakeCamera(stamp=stamp())
        adapter = t02.FakeSceneAdapter()
        buf = t02.BoundedFrameBuffer()
        real_imwrite = cv2.imwrite
        cv2.imwrite = lambda *a, **k: False  # type: ignore
        # after spawn, arm delete fail for restore
        orig_spawn = adapter.spawn

        def spawn_arm(name, xml, pose, timeout_s=5.0):
            orig_spawn(name, xml, pose, timeout_s=timeout_s)
            adapter.fail_on["delete"] = ent

        adapter.spawn = spawn_arm  # type: ignore

        def grab(deadline):
            return cam.grab(deadline, clock)

        try:
            with tempfile.TemporaryDirectory() as td:
                out = Path(td) / "bw"
                summary = t02.run_batch(
                    plan,
                    out,
                    adapter=adapter,
                    buffer=buf,
                    grab=grab,
                    mono=clock,
                    ros_stamp_now=stamp,
                    sleep_fn=lambda s: clock.advance(s),
                    capture_source="fake",
                    source_registry=_registry(),
                )
                self.assertFalse(summary["ok"])
                self.assertEqual(summary["cases"][0]["status"], "write_failed")
                self.assertTrue(summary["cases"][0]["restore_errors"])
                self.assertEqual(summary["cases"][1]["status"], "skipped_after_abort")
        finally:
            cv2.imwrite = real_imwrite  # type: ignore

    def test_grab_deadline_timeout(self):
        plan = _plan([_base_case("dl")])
        clock = Clock(0.0)
        stamp = StampClock()
        cam = t02.FakeCamera(stamp=stamp())
        cam.block_until_mono = 100.0  # beyond short deadline
        adapter = t02.FakeSceneAdapter()
        buf = t02.BoundedFrameBuffer()

        def grab(deadline):
            return cam.grab(deadline, clock)

        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "dl"
            summary = t02.run_batch(
                plan,
                out,
                adapter=adapter,
                buffer=buf,
                grab=grab,
                mono=clock,
                ros_stamp_now=stamp,
                sleep_fn=lambda s: clock.advance(s),
                settle_s=0.0,
                wait_timeout_s=0.05,
                capture_source="fake",
                source_registry=_registry(),
            )
            self.assertFalse(summary["ok"])
            self.assertEqual(summary["cases"][0]["status"], "grab_deadline")


class TestFakeFormal(unittest.TestCase):
    def test_formal_fake_refused(self):
        plan = _plan(
            [_base_case(usage=t02.USAGE_FORMAL, scene_recipe={"kind": "n", "seed": 380})],
            usage=t02.USAGE_FORMAL,
        )
        adapter = t02.FakeSceneAdapter()
        buf = t02.BoundedFrameBuffer()
        clock = Clock()
        cam = t02.FakeCamera()
        grabs = {"n": 0}

        def grab(deadline):
            grabs["n"] += 1
            return cam.grab(deadline, clock)

        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "ff"
            summary = t02.run_batch(
                plan,
                out,
                adapter=adapter,
                buffer=buf,
                grab=grab,
                mono=clock,
                capture_source="fake",
                source_registry=_registry(),
            )
            self.assertFalse(summary["ok"])
            self.assertEqual(summary["status"], "fake_refuses_formal_test_candidate")
            self.assertEqual(adapter.ops, [])
            self.assertEqual(grabs["n"], 0)


class TestExamplePilot(unittest.TestCase):
    def test_twelve_pilot_honest(self):
        plan = t02.example_pilot_plan()
        self.assertEqual(len(plan["cases"]), 12)
        self.assertFalse(plan["execution_ready"])
        errs = t02.validate_plan(plan)
        self.assertEqual(errs, [], errs)
        readiness = t02.plan_execution_readiness(plan)
        self.assertEqual(readiness["status"], "not_execution_ready")
        normals = [c for c in plan["cases"] if c["case_id"].startswith("pilot_normal_")]
        hards = [c for c in plan["cases"] if c["case_id"].startswith("pilot_hard_")]
        self.assertEqual(len(normals), 6)
        self.assertEqual(len(hards), 6)
        for c in normals:
            ops = c["scene_ops"]
            self.assertTrue(any(o["op"] == "set_pose" and o["name"] == "robot_v0" for o in ops))
            self.assertIn("previous_pose", ops[0])
            self.assertEqual(c["entity_allowlist"], ["robot_v0"])
        for c in hards:
            self.assertNotIn("UNKNOWN", json.dumps(c["target_states_intent"]))
            spawn = [o for o in c["scene_ops"] if o["op"] == "spawn"][0]
            xml = spawn["xml"]
            self.assertIn("link", xml)
            self.assertIn("visual", xml)
            self.assertIn("geometry", xml)
            self.assertIn("robot_v0", c["entity_allowlist"])
            self.assertTrue(any(x.startswith("t02c1_tmp_") for x in c["entity_allowlist"]))

    def test_dry_run_marks_not_execution_ready(self):
        plan = t02.example_pilot_plan()
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "dryp"
            report = t02.dry_run(plan, out, source_registry=None)
            self.assertTrue(report["ok"])
            self.assertEqual(report["execution_readiness"]["status"], "not_execution_ready")
            st = json.loads((out / "batch_status.json").read_text())["status"]
            self.assertEqual(st, "dry_run_ok_not_execution_ready")


class TestHappyFakePath(unittest.TestCase):
    def test_fake_success_not_formal_raw(self):
        plan = _plan([_base_case("ok1")])
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "batch"
            summary, *_ = _run_ok_batch(plan, out)
            self.assertTrue(summary["ok"])
            self.assertEqual(summary["status"], "batch_fake_complete")
            self.assertEqual(summary["capture_source"], "fake")
            self.assertFalse(summary["formal_test_eligible"])
            side = json.loads(next((out / "meta").glob("*.json")).read_text())
            self.assertTrue(side["synthetic"])
            self.assertEqual(side["capture_source"], "fake")
            self.assertFalse(side["formal_test_eligible"])


class TestCli(unittest.TestCase):
    def test_cli_dry_run(self):
        plan = _plan([_base_case("cli")])
        with tempfile.TemporaryDirectory() as td:
            p = Path(td)
            plan_path = p / "plan.json"
            out = p / "out"
            plan_path.write_text(json.dumps(plan), encoding="utf-8")
            rc = t02.main(["--plan", str(plan_path), "--out", str(out)])
            self.assertEqual(rc, 0)


# ----- R2 gates + per-op restore accounting -----


def _formal_live_ros_batch(plan, out, *, adapter, grab, mono, stamp, registry):
    """Isolated fake deps with capture_source=live_ros — gate tests only, not real capture."""
    if t02.ROBOT_ENTITY not in adapter.entities:
        adapter.entities[t02.ROBOT_ENTITY] = {"xml": "<r/>", "pose": [0.9, 3.0, 0.05, 0, 0, 0]}
    return t02.run_batch(
        plan,
        out,
        adapter=adapter,
        buffer=t02.BoundedFrameBuffer(),
        grab=grab,
        mono=mono,
        ros_stamp_now=stamp,
        sleep_fn=lambda s: mono.advance(s) if hasattr(mono, "advance") else None,
        settle_s=0.0,
        wait_timeout_s=0.5,
        capture_source="live_ros",
        source_registry=registry,
    )


class TestR2ExecutionGates(unittest.TestCase):
    def test_not_execution_ready_zero_scene_and_grab(self):
        plan = t02.example_pilot_plan()
        self.assertFalse(plan["execution_ready"])
        adapter = t02.FakeSceneAdapter()
        clock = Clock()
        cam = t02.FakeCamera()
        grabs = {"n": 0}

        def grab(deadline):
            grabs["n"] += 1
            return cam.grab(deadline, clock)

        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "nr"
            summary = t02.run_batch(
                plan,
                out,
                adapter=adapter,
                buffer=t02.BoundedFrameBuffer(),
                grab=grab,
                mono=clock,
                ros_stamp_now=StampClock(),
                sleep_fn=lambda s: clock.advance(s),
                capture_source="fake",
                source_registry=_registry(),
            )
            self.assertFalse(summary["ok"])
            self.assertEqual(summary["status"], "not_execution_ready")
            self.assertEqual(summary["n_scene_ops_attempted"], 0)
            self.assertEqual(summary["n_grabs"], 0)
            self.assertEqual(adapter.ops, [])
            self.assertEqual(grabs["n"], 0)

    def test_formal_registry_missing_pre_scene(self):
        plan = _plan(
            [_base_case("f1", usage=t02.USAGE_FORMAL, scene_recipe={"kind": "n", "seed": 401})],
            usage=t02.USAGE_FORMAL,
        )
        adapter = t02.FakeSceneAdapter()
        clock = Clock()
        stamp = StampClock()
        cam = t02.FakeCamera(stamp=stamp())
        grabs = {"n": 0}

        def grab(deadline):
            grabs["n"] += 1
            return cam.grab(deadline, clock)

        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "frm"
            summary = _formal_live_ros_batch(
                plan, out, adapter=adapter, grab=grab, mono=clock, stamp=stamp, registry=None
            )
            self.assertEqual(summary["status"], "formal_registry_missing_or_not_checked")
            self.assertEqual(adapter.ops, [])
            self.assertEqual(grabs["n"], 0)

    def test_formal_registry_incomplete_pre_scene(self):
        plan = _plan(
            [_base_case("f2", usage=t02.USAGE_FORMAL, scene_recipe={"kind": "n", "seed": 402})],
            usage=t02.USAGE_FORMAL,
        )
        incomplete = {
            "known_seeds": [],
            "known_content_hashes": [],
            "known_provenance_hashes": [],
            "completeness": {"declared_complete": False, "statement": ""},
        }
        adapter = t02.FakeSceneAdapter()
        clock = Clock()
        stamp = StampClock()
        grabs = {"n": 0}
        cam = t02.FakeCamera(stamp=stamp())

        def grab(deadline):
            grabs["n"] += 1
            return cam.grab(deadline, clock)

        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "fri"
            summary = _formal_live_ros_batch(
                plan, out, adapter=adapter, grab=grab, mono=clock, stamp=stamp, registry=incomplete
            )
            self.assertEqual(summary["status"], "formal_registry_incomplete_or_invalid")
            self.assertEqual(adapter.ops, [])
            self.assertEqual(grabs["n"], 0)

    def test_formal_unreviewed_conflict_pre_scene(self):
        c = _base_case("f3", usage=t02.USAGE_FORMAL, scene_recipe={"kind": "n", "seed": 403})
        plan = _plan([c], usage=t02.USAGE_FORMAL)
        reg = _registry(known_seeds=[403])
        adapter = t02.FakeSceneAdapter()
        clock = Clock()
        stamp = StampClock()
        grabs = {"n": 0}
        cam = t02.FakeCamera(stamp=stamp())

        def grab(deadline):
            grabs["n"] += 1
            return cam.grab(deadline, clock)

        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "frc"
            summary = _formal_live_ros_batch(
                plan, out, adapter=adapter, grab=grab, mono=clock, stamp=stamp, registry=reg
            )
            self.assertEqual(summary["status"], "formal_blocked_unreviewed_conflict")
            self.assertEqual(adapter.ops, [])
            self.assertEqual(grabs["n"], 0)

    def test_illegal_capture_source(self):
        plan = _plan([_base_case("ics")])
        adapter = t02.FakeSceneAdapter()
        clock = Clock()
        grabs = {"n": 0}

        def grab(deadline):
            grabs["n"] += 1
            raise AssertionError("should not grab")

        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "ics"
            summary = t02.run_batch(
                plan,
                out,
                adapter=adapter,
                buffer=t02.BoundedFrameBuffer(),
                grab=grab,
                mono=clock,
                capture_source="gazebo_cheat",
                source_registry=_registry(),
            )
            self.assertEqual(summary["status"], "illegal_capture_source")
            self.assertEqual(adapter.ops, [])
            self.assertEqual(grabs["n"], 0)

    def test_non_fake_requires_ros_stamp(self):
        plan = _plan([_base_case("nst")])
        adapter = t02.FakeSceneAdapter()
        clock = Clock()
        grabs = {"n": 0}

        def grab(deadline):
            grabs["n"] += 1
            raise AssertionError("no")

        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "nst"
            summary = t02.run_batch(
                plan,
                out,
                adapter=adapter,
                buffer=t02.BoundedFrameBuffer(),
                grab=grab,
                mono=clock,
                ros_stamp_now=None,
                capture_source="live_ros",
                source_registry=_registry(),
            )
            self.assertEqual(summary["status"], "ros_stamp_contract_failed")
            self.assertIn("non_fake_requires_explicit_ros_stamp_now", summary["error"])
            self.assertEqual(adapter.ops, [])
            self.assertEqual(grabs["n"], 0)

    def test_ros_stamp_zero_nan_inf_rejected(self):
        plan = _plan([_base_case("badstamp")])
        for bad in (0.0, float("nan"), float("inf")):
            adapter = t02.FakeSceneAdapter()
            clock = Clock()
            grabs = {"n": 0}

            def grab(deadline):
                grabs["n"] += 1
                raise AssertionError("no")

            with tempfile.TemporaryDirectory() as td:
                out = Path(td) / f"bs_{bad}"
                summary = t02.run_batch(
                    plan,
                    out,
                    adapter=adapter,
                    buffer=t02.BoundedFrameBuffer(),
                    grab=grab,
                    mono=clock,
                    ros_stamp_now=lambda bad=bad: bad,
                    capture_source="live_ros",
                    source_registry=_registry(),
                )
                self.assertEqual(summary["status"], "ros_stamp_contract_failed", msg=str(bad))
                self.assertEqual(adapter.ops, [])
                self.assertEqual(grabs["n"], 0)


class TestR2PerOpRestore(unittest.TestCase):
    def test_delete_spawn_same_name_restores_original(self):
        name = "panel_swap"
        old_xml, new_xml = "<old/>", "<new/>"
        old_pose = [1.0, 2.0, 0.1, 0, 0, 0]
        new_pose = [3.0, 4.0, 0.2, 0, 0, 0.5]
        case = _base_case(
            "swap",
            scene_ops=[
                {"op": "delete", "name": name, "previous": {"xml": old_xml, "pose": old_pose}},
                {"op": "spawn", "name": name, "xml": new_xml, "pose": new_pose},
            ],
            entity_allowlist=[name],
            scene_recipe={"kind": "n", "seed": 410},
        )
        plan = _plan([case])
        adapter = t02.FakeSceneAdapter()
        adapter.entities[name] = {"xml": old_xml, "pose": list(old_pose)}
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "swap"
            summary, *_ = _run_ok_batch(plan, out, adapter=adapter)
            self.assertTrue(summary["ok"])
            self.assertIn(name, adapter.entities)
            self.assertEqual(adapter.entities[name]["xml"], old_xml)
            self.assertEqual(adapter.entities[name]["pose"], old_pose)
            kinds = [o["op"] for o in adapter.ops]
            # scene delete+spawn, then restore delete+spawn
            self.assertEqual(kinds[:2], ["delete", "spawn"])
            self.assertEqual(kinds[2:4], ["delete", "spawn"])

    def test_first_op_fail_no_later_attempt_or_restore(self):
        a, b = "ent_a", "ent_b"
        case = _base_case(
            "firstfail",
            scene_ops=[
                {"op": "spawn", "name": a, "xml": t02._visible_box_sdf(a), "pose": [0, 0, 0.4, 0, 0, 0]},
                {"op": "spawn", "name": b, "xml": t02._visible_box_sdf(b), "pose": [1, 0, 0.4, 0, 0, 0]},
            ],
            entity_allowlist=[a, b],
            scene_recipe={"kind": "partial_occlusion", "seed": 411},
        )
        plan = _plan([case])
        adapter = t02.FakeSceneAdapter(fail_on={"spawn": a})
        clock = Clock()
        stamp = StampClock()
        cam = t02.FakeCamera(stamp=stamp())
        grabs = {"n": 0}

        def grab(deadline):
            grabs["n"] += 1
            return cam.grab(deadline, clock)

        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "ff1"
            summary = t02.run_batch(
                plan,
                out,
                adapter=adapter,
                buffer=t02.BoundedFrameBuffer(),
                grab=grab,
                mono=clock,
                ros_stamp_now=stamp,
                sleep_fn=lambda s: clock.advance(s),
                capture_source="fake",
                source_registry=_registry(),
            )
            self.assertFalse(summary["ok"])
            self.assertEqual(summary["cases"][0]["status"], "scene_op_failed")
            self.assertNotIn(a, adapter.entities)
            self.assertNotIn(b, adapter.entities)
            # fail_before: no mutate logged; restore delete is idempotent no-op (no extra op)
            self.assertEqual(adapter.ops, [])
            self.assertEqual(grabs["n"], 0)
            self.assertFalse(summary["cases"][0]["restore_errors"])

    def test_mutate_then_raise_restores_original(self):
        name = "tmp_after"
        xml = t02._visible_box_sdf(name)
        case = _base_case(
            "aftermut",
            scene_ops=[{"op": "spawn", "name": name, "xml": xml, "pose": [0, 1, 0.4, 0, 0, 0]}],
            entity_allowlist=[name],
            scene_recipe={"kind": "partial_occlusion", "seed": 412},
        )
        plan = _plan([case])
        adapter = t02.FakeSceneAdapter(fail_after_mutate={"spawn": name})
        clock = Clock()
        stamp = StampClock()
        cam = t02.FakeCamera(stamp=stamp())

        def grab(deadline):
            return cam.grab(deadline, clock)

        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "am"
            summary = t02.run_batch(
                plan,
                out,
                adapter=adapter,
                buffer=t02.BoundedFrameBuffer(),
                grab=grab,
                mono=clock,
                ros_stamp_now=stamp,
                sleep_fn=lambda s: clock.advance(s),
                capture_source="fake",
                source_registry=_registry(),
            )
            self.assertFalse(summary["ok"])
            self.assertEqual(summary["cases"][0]["status"], "scene_op_failed")
            self.assertNotIn(name, adapter.entities)  # restored via delete
            self.assertEqual([o["op"] for o in adapter.ops], ["spawn", "delete"])
            self.assertFalse(summary["cases"][0]["restore_errors"])

    def test_raise_before_mutate_restore_no_extra_change(self):
        name = "tmp_before"
        before = dict(t02.FakeSceneAdapter().entities)  # empty baseline copy
        xml = t02._visible_box_sdf(name)
        case = _base_case(
            "beforemut",
            scene_ops=[{"op": "spawn", "name": name, "xml": xml, "pose": [0, 1, 0.4, 0, 0, 0]}],
            entity_allowlist=[name],
            scene_recipe={"kind": "partial_occlusion", "seed": 413},
        )
        plan = _plan([case])
        adapter = t02.FakeSceneAdapter(fail_on={"spawn": name})
        snap = dict(adapter.entities)
        clock = Clock()
        stamp = StampClock()

        def grab(deadline):
            raise AssertionError("no grab")

        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "bm"
            summary = t02.run_batch(
                plan,
                out,
                adapter=adapter,
                buffer=t02.BoundedFrameBuffer(),
                grab=grab,
                mono=clock,
                ros_stamp_now=stamp,
                sleep_fn=lambda s: clock.advance(s),
                capture_source="fake",
                source_registry=_registry(),
            )
            self.assertEqual(summary["cases"][0]["status"], "scene_op_failed")
            self.assertEqual(adapter.entities, snap)
            self.assertEqual(adapter.ops, [])  # restore idempotent missing — no extra mutate
            self.assertFalse(summary["cases"][0]["restore_errors"])
            self.assertEqual(before, {})  # sanity

    def test_mid_replace_fail_and_restore_fail_skips_rest(self):
        name = "panel_mid"
        old_xml, new_xml = "<old_mid/>", "<new_mid/>"
        old_pose = [1.0, 0.0, 0.1, 0, 0, 0]
        new_pose = [2.0, 0.0, 0.1, 0, 0, 0]
        case = _base_case(
            "midfail",
            scene_ops=[
                {"op": "delete", "name": name, "previous": {"xml": old_xml, "pose": old_pose}},
                {"op": "spawn", "name": name, "xml": new_xml, "pose": new_pose},
            ],
            entity_allowlist=[name],
            scene_recipe={"kind": "n", "seed": 414},
        )
        plan = _plan([case, _base_case("later2", scene_recipe={"kind": "n", "seed": 415})])
        adapter = t02.FakeSceneAdapter()
        adapter.entities[name] = {"xml": old_xml, "pose": list(old_pose)}
        state = {"phase": "scene"}
        real_spawn = adapter.spawn

        def spawn_w(name_, xml, pose, timeout_s=5.0):
            if state["phase"] == "scene":
                adapter._check_allow(name_)
                adapter.ops.append({"op": "spawn", "name": name_, "pose": list(pose), "timeout_s": timeout_s})
                adapter.entities[name_] = {"xml": xml, "pose": list(pose)}
                state["phase"] = "restore"
                raise RuntimeError("spawn_after_mutate_boom")
            raise RuntimeError("restore_spawn_boom")

        adapter.spawn = spawn_w  # type: ignore
        clock = Clock()
        stamp = StampClock()
        cam = t02.FakeCamera(stamp=stamp())

        def grab(deadline):
            return cam.grab(deadline, clock)

        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "mr"
            summary = t02.run_batch(
                plan,
                out,
                adapter=adapter,
                buffer=t02.BoundedFrameBuffer(),
                grab=grab,
                mono=clock,
                ros_stamp_now=stamp,
                sleep_fn=lambda s: clock.advance(s),
                capture_source="fake",
                source_registry=_registry(),
            )
            self.assertFalse(summary["ok"])
            self.assertEqual(summary["status"], "batch_aborted")
            c0 = summary["cases"][0]
            self.assertEqual(c0["status"], "scene_op_failed")
            self.assertTrue(c0["restore_errors"])
            self.assertTrue(c0["restore_uncertain"])
            self.assertEqual(summary["cases"][1]["status"], "skipped_after_abort")
            # after restore delete of new entity + failed spawn-previous: entity absent, uncertain
            self.assertNotIn(name, adapter.entities)
            # scene: delete + spawn(mutate); restore: delete; restore spawn failed (no op log if raise before)
            self.assertEqual([o["op"] for o in adapter.ops if o["op"] == "delete"], ["delete", "delete"])
            _ = real_spawn  # keep reference for readability


if __name__ == "__main__":
    unittest.main()
