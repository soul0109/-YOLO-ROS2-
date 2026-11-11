#!/usr/bin/env python3
"""M1-T02-C1-R2: isolated collection tool — gates + per-op restore accounting.

Default: dry-run only (no ROS import). Fake execute is synthetic/dev-only.
Real ROS adapter is NOT implemented — cannot real-capture in this task.

R2 contracts (on top of R1):
- execution gate: readiness=false → not_execution_ready, zero scene/grab
- formal: registry missing/incomplete/unreviewed conflicts refuse pre-scene
- capture_source enum; non-fake requires explicit finite ros_stamp_now > 0
- per-op restore registration before each attempt; unattempted ops not restored
- idempotent restore contracts; previous XML/pose are declarations not snapshots
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
import time
from collections import deque
from copy import deepcopy
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Deque, Dict, List, Optional, Protocol, Sequence, Tuple

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[1]
SCRIPTS = Path(__file__).resolve().parent
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

STATION_POSE_WORLD: Dict[str, Dict[str, float]] = {
    "A": {"x": 3.15, "y": 1.75, "z": 0.05, "yaw": -math.pi / 2},
    "B": {"x": 8.45, "y": 1.75, "z": 0.05, "yaw": -math.pi / 2},
    "C": {"x": 5.25, "y": 4.25, "z": 0.05, "yaw": math.pi / 2},
}

USAGE_PILOT = "pilot_dev"
USAGE_FORMAL = "formal_test_candidate"
ALLOWED_USAGE = frozenset({USAGE_PILOT, USAGE_FORMAL})
CAPTURE_SOURCES = frozenset({"fake", "live_ros"})
SCHEMA = "vision_5b_frame_t02_v1"
LABEL_VERSION = "t02-c1-r2-v0.1"
SAFE_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
ALLOWED_LED = frozenset({"green", "red", "yellow", "off"})
ALLOWED_PANEL = frozenset({"lit", "dark"})
ALLOWED_LEAK = frozenset({"none", "fixed_fail"})
ALLOWED_OPS = frozenset({"delete", "spawn", "set_pose"})
ROBOT_ENTITY = "robot_v0"
REGISTRY_KEYS = ("known_seeds", "known_content_hashes", "known_provenance_hashes", "completeness")

GrabFn = Callable[[float], "FrameCandidate"]  # deadline_mono -> frame


# --- hashes ------------------------------------------------------------------


def stable_json_hash(obj: Any) -> str:
    blob = json.dumps(obj, sort_keys=True, ensure_ascii=False, separators=(",", ":"))
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()


def imaging_content_view(case: Dict[str, Any]) -> Dict[str, Any]:
    """Fields that affect imaging — excludes case_id/seed/notes/checklist chatter."""
    recipe = case.get("scene_recipe") or {}
    content_recipe = {
        k: v
        for k, v in recipe.items()
        if k
        not in {
            "seed",
            "note",
            "notes",
            "case_id",
            "annotator_comment",
        }
    }
    # Resolved physical state (seed-resolved intent must be explicit in plan)
    return {
        "station_id": case.get("station_id"),
        "distance_m": case.get("distance_m"),
        "robot_pose_world": case.get("robot_pose_world"),
        "pose_frame_id": case.get("pose_frame_id", "world"),
        "target_states_intent": case.get("target_states_intent"),
        "scene_ops": case.get("scene_ops") or [],
        "scene_recipe_imaging": content_recipe,
        "hard_case": case.get("hard_case") or [],
    }


def provenance_view(case: Dict[str, Any], plan: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "session_id": plan.get("session_id"),
        "batch_id": plan.get("batch_id"),
        "case_id": case.get("case_id"),
        "usage": case.get("usage"),
        "scene_recipe": case.get("scene_recipe") or {},
        "checklist_ids": case.get("checklist_ids") or [],
        "notes": case.get("notes") or "",
        "imaging_content": imaging_content_view(case),
    }


def content_hash(case: Dict[str, Any]) -> str:
    return stable_json_hash(imaging_content_view(case))


def provenance_hash(case: Dict[str, Any], plan: Dict[str, Any]) -> str:
    return stable_json_hash(provenance_view(case, plan))


# --- camera buffer -----------------------------------------------------------


@dataclass
class FrameCandidate:
    recv_mono: float
    image_stamp_sec: float
    image_frame_id: str
    case_id: Optional[str]
    scene_op_done_token: Optional[str]
    bgr: np.ndarray
    seq: int


class BoundedFrameBuffer:
    def __init__(self, maxlen: int = 8) -> None:
        if maxlen < 1:
            raise ValueError("maxlen must be >= 1")
        self.maxlen = maxlen
        self._buf: Deque[FrameCandidate] = deque(maxlen=maxlen)
        self.received = 0
        self.dropped = 0
        self.last_stamp: Optional[float] = None

    def __len__(self) -> int:
        return len(self._buf)

    def clear(self) -> None:
        self._buf.clear()
        # stamp monotonicity is per-case after clear; reset tracker
        self.last_stamp = None

    def push(self, fr: FrameCandidate) -> Optional[str]:
        self.received += 1
        if self.last_stamp is not None and fr.image_stamp_sec < self.last_stamp:
            return "image_stamp_went_backwards"
        if len(self._buf) == self.maxlen:
            self.dropped += 1
        self._buf.append(fr)
        self.last_stamp = fr.image_stamp_sec
        return None

    def select_fresh(
        self,
        *,
        case_id: str,
        scene_op_done_token: str,
        after_recv_mono: float,
        min_image_stamp_sec: float,
    ) -> Tuple[Optional[FrameCandidate], Optional[str]]:
        """Newest usable frame. Never mutates association fields.

        after_recv_mono: settle-complete monotonic lower bound (recv time).
        min_image_stamp_sec: ROS/sim stamp lower bound for this case (not mono).
        """
        reject_reason: Optional[str] = None
        chosen: Optional[FrameCandidate] = None
        for fr in self._buf:
            if fr.case_id is not None and fr.case_id != case_id:
                reject_reason = "case_id_mismatch"
                continue
            if fr.scene_op_done_token is not None and fr.scene_op_done_token != scene_op_done_token:
                reject_reason = "token_mismatch"
                continue
            if fr.recv_mono < after_recv_mono:
                continue
            if fr.image_stamp_sec < min_image_stamp_sec:
                # late delivery of old stamp — reject for selection
                reject_reason = "stamp_before_ros_lower_bound"
                continue
            if not math.isfinite(fr.image_stamp_sec) or fr.image_stamp_sec <= 0:
                reject_reason = "invalid_stamp"
                continue
            chosen = fr
        return chosen, (None if chosen is not None else reject_reason)


# --- scene adapter -----------------------------------------------------------


class SceneAdapter(Protocol):
    def entity_exists(self, name: str) -> bool: ...

    def delete(self, name: str, timeout_s: float = 5.0) -> None: ...

    def spawn(self, name: str, xml: str, pose: Sequence[float], timeout_s: float = 5.0) -> None: ...

    def set_pose(self, name: str, pose: Sequence[float], timeout_s: float = 5.0) -> None: ...


@dataclass
class FakeSceneAdapter:
    entities: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    fail_on: Dict[str, str] = field(default_factory=dict)  # fail before mutate
    fail_after_mutate: Dict[str, str] = field(default_factory=dict)  # mutate then raise
    ops: List[Dict[str, Any]] = field(default_factory=list)
    allowlist: Optional[frozenset] = None

    def _check_allow(self, name: str) -> None:
        if self.allowlist is None:
            raise PermissionError("allowlist not set")
        if name not in self.allowlist:
            raise PermissionError(f"entity not in plan allowlist: {name}")

    def _fail_before(self, op: str, name: str) -> None:
        target = self.fail_on.get(op)
        if target is not None and target in ("*", name):
            raise RuntimeError(f"fake {op} failed before mutate for {name}")

    def _fail_after(self, op: str, name: str) -> None:
        target = self.fail_after_mutate.get(op)
        if target is not None and target in ("*", name):
            raise RuntimeError(f"fake {op} failed after mutate for {name}")

    def entity_exists(self, name: str) -> bool:
        return name in self.entities

    def delete(self, name: str, timeout_s: float = 5.0) -> None:
        self._check_allow(name)
        self._fail_before("delete", name)
        self.ops.append({"op": "delete", "name": name, "timeout_s": timeout_s})
        self.entities.pop(name, None)
        self._fail_after("delete", name)

    def spawn(self, name: str, xml: str, pose: Sequence[float], timeout_s: float = 5.0) -> None:
        self._check_allow(name)
        self._fail_before("spawn", name)
        if name in self.entities:
            raise RuntimeError(f"spawn refuses overwrite existing entity: {name}")
        self.ops.append({"op": "spawn", "name": name, "pose": list(pose), "timeout_s": timeout_s})
        self.entities[name] = {"xml": xml, "pose": list(pose)}
        self._fail_after("spawn", name)

    def set_pose(self, name: str, pose: Sequence[float], timeout_s: float = 5.0) -> None:
        self._check_allow(name)
        self._fail_before("set_pose", name)
        if name not in self.entities:
            raise KeyError(f"set_pose: unknown entity {name}")
        self.ops.append({"op": "set_pose", "name": name, "pose": list(pose), "timeout_s": timeout_s})
        self.entities[name]["pose"] = list(pose)
        self._fail_after("set_pose", name)


@dataclass
class FakeCamera:
    stamp: float = 1000.0
    frame_id: str = "camera_link"
    seq: int = 0
    case_id: Optional[str] = None
    scene_op_done_token: Optional[str] = None
    color: tuple = (40, 80, 120)
    block_until_mono: Optional[float] = None  # simulate blocking grab
    raise_exc: Optional[BaseException] = None

    def grab(self, deadline_mono: float, mono: Callable[[], float]) -> FrameCandidate:
        if self.raise_exc is not None:
            raise self.raise_exc
        # Honor deadline: if configured to block past deadline, timeout
        if self.block_until_mono is not None and self.block_until_mono > deadline_mono:
            raise TimeoutError("grab_deadline_exceeded")
        if mono() >= deadline_mono:
            raise TimeoutError("grab_deadline_exceeded")
        self.seq += 1
        self.stamp += 0.1
        bgr = np.zeros((480, 640, 3), dtype=np.uint8)
        bgr[:] = self.color
        cv2.putText(bgr, f"seq={self.seq}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        return FrameCandidate(
            recv_mono=mono(),
            image_stamp_sec=self.stamp,
            image_frame_id=self.frame_id,
            case_id=self.case_id,
            scene_op_done_token=self.scene_op_done_token,
            bgr=bgr,
            seq=self.seq,
        )


# --- validation --------------------------------------------------------------


def _finite_num(v: Any, name: str, errs: List[str], prefix: str) -> None:
    try:
        x = float(v)
    except (TypeError, ValueError):
        errs.append(f"{prefix}:not_numeric:{name}")
        return
    if not math.isfinite(x):
        errs.append(f"{prefix}:non_finite:{name}")


def _safe_id(value: Any, field: str, errs: List[str], prefix: str) -> None:
    if not isinstance(value, str) or not value:
        errs.append(f"{prefix}:empty_id:{field}")
        return
    if "/" in value or "\\" in value or ".." in value:
        errs.append(f"{prefix}:unsafe_id:{field}")
        return
    if not SAFE_ID_RE.match(value):
        errs.append(f"{prefix}:bad_id_charset:{field}")


def validate_plan(plan: Dict[str, Any]) -> List[str]:
    errs: List[str] = []
    for key in ("session_id", "batch_id", "usage", "cases"):
        if key not in plan:
            errs.append(f"missing_plan_field:{key}")
    for fid in ("session_id", "batch_id"):
        if fid in plan:
            _safe_id(plan.get(fid), fid, errs, "plan")
    usage = plan.get("usage")
    if usage is not None and usage not in ALLOWED_USAGE:
        errs.append(f"bad_usage:{usage}")
    cases = plan.get("cases") or []
    if not isinstance(cases, list) or not cases:
        errs.append("cases_empty")
        return errs
    seen: set[str] = set()
    for i, c in enumerate(cases):
        prefix = f"case[{i}]"
        for k in (
            "case_id",
            "station_id",
            "distance_m",
            "robot_pose_world",
            "target_states_intent",
            "scene_recipe",
            "usage",
            "entity_allowlist",
            "scene_ops",
        ):
            if k not in c:
                errs.append(f"{prefix}:missing:{k}")
        cid = c.get("case_id")
        _safe_id(cid, "case_id", errs, prefix)
        if isinstance(cid, str):
            if cid in seen:
                errs.append(f"duplicate_case_id:{cid}")
            seen.add(cid)
        if c.get("usage") not in ALLOWED_USAGE:
            errs.append(f"{prefix}:bad_usage:{c.get('usage')}")
        if usage and c.get("usage") and c.get("usage") != usage:
            errs.append(f"{prefix}:usage_mix_with_plan")
        if c.get("station_id") not in STATION_POSE_WORLD:
            errs.append(f"{prefix}:bad_station:{c.get('station_id')}")
        _finite_num(c.get("distance_m"), "distance_m", errs, prefix)
        try:
            d = float(c.get("distance_m"))
            if math.isfinite(d) and d not in (1.0, 1.2, 1.5):
                errs.append(f"{prefix}:distance_not_in_allowed_set")
        except (TypeError, ValueError):
            pass
        pose = c.get("robot_pose_world") or {}
        for axis in ("x", "y", "z", "yaw"):
            if axis not in pose:
                errs.append(f"{prefix}:pose_missing:{axis}")
            else:
                _finite_num(pose.get(axis), axis, errs, prefix)
        if c.get("pose_frame_id", "world") != "world":
            errs.append(f"{prefix}:pose_frame_must_be_world")
        if "robot_pose_map" in c and not c.get("map_calib"):
            errs.append(f"{prefix}:map_pose_without_calib")
        intent = c.get("target_states_intent") or {}
        leds = intent.get("leds")
        if not isinstance(leds, list) or len(leds) != 3:
            errs.append(f"{prefix}:leds_must_be_3")
        else:
            for li, led in enumerate(leds):
                if led not in ALLOWED_LED:
                    errs.append(f"{prefix}:bad_led[{li}]:{led}")
        if intent.get("panel") not in ALLOWED_PANEL:
            errs.append(f"{prefix}:bad_panel:{intent.get('panel')}")
        if intent.get("leak") not in ALLOWED_LEAK:
            errs.append(f"{prefix}:bad_leak:{intent.get('leak')}")
        # UNKNOWN is review-only — not allowed in physical intent
        blob = json.dumps(intent)
        if "UNKNOWN" in blob:
            errs.append(f"{prefix}:UNKNOWN_not_allowed_in_scene_state_intent")
        seed = (c.get("scene_recipe") or {}).get("seed")
        if seed == 42:
            errs.append(f"{prefix}:forbidden_hardcoded_seed42")
        allow = c.get("entity_allowlist")
        ops = c.get("scene_ops")
        if not isinstance(allow, list):
            errs.append(f"{prefix}:entity_allowlist_must_be_list")
            allow = []
        if not isinstance(ops, list):
            errs.append(f"{prefix}:scene_ops_must_be_list")
            ops = []
        for name in allow:
            _safe_id(name, "allowlist_entity", errs, prefix)
        allow_set = set(allow)
        for j, op in enumerate(ops):
            op_p = f"{prefix}.op[{j}]"
            kind = op.get("op")
            if kind not in ALLOWED_OPS:
                errs.append(f"{op_p}:bad_op:{kind}")
            name = op.get("name")
            _safe_id(name, "op_name", errs, op_p)
            if name not in allow_set:
                errs.append(f"{op_p}:name_not_in_explicit_allowlist:{name}")
            if kind == "delete":
                prev = op.get("previous")
                if not isinstance(prev, dict) or "xml" not in prev or "pose" not in prev:
                    errs.append(f"{op_p}:delete_requires_previous_xml_pose")
                else:
                    if not isinstance(prev.get("pose"), (list, tuple)) or len(prev["pose"]) < 6:
                        errs.append(f"{op_p}:previous_pose_bad")
                    else:
                        for k, pv in enumerate(prev["pose"][:6]):
                            _finite_num(pv, f"previous_pose[{k}]", errs, op_p)
            if kind == "set_pose":
                if "previous_pose" not in op:
                    errs.append(f"{op_p}:set_pose_requires_previous_pose")
                else:
                    pp = op.get("previous_pose")
                    if not isinstance(pp, (list, tuple)) or len(pp) < 6:
                        errs.append(f"{op_p}:previous_pose_bad")
                    else:
                        for k, pv in enumerate(pp[:6]):
                            _finite_num(pv, f"previous_pose[{k}]", errs, op_p)
                pose = op.get("pose")
                if not isinstance(pose, (list, tuple)) or len(pose) < 6:
                    errs.append(f"{op_p}:pose_bad")
                else:
                    for k, pv in enumerate(pose[:6]):
                        _finite_num(pv, f"pose[{k}]", errs, op_p)
            if kind == "spawn":
                if "xml" not in op or "pose" not in op:
                    errs.append(f"{op_p}:spawn_requires_xml_pose")
                else:
                    pose = op.get("pose")
                    if not isinstance(pose, (list, tuple)) or len(pose) < 6:
                        errs.append(f"{op_p}:pose_bad")
                    else:
                        for k, pv in enumerate(pose[:6]):
                            _finite_num(pv, f"pose[{k}]", errs, op_p)
                    xml = str(op.get("xml") or "")
                    if "link" not in xml or "visual" not in xml or "geometry" not in xml:
                        # only enforce for occlusion-like temp entities
                        if "occ" in str(name) or "occlusion" in str((c.get("scene_recipe") or {}).get("kind")):
                            errs.append(f"{op_p}:occlusion_sdf_needs_link_visual_geometry")
    return errs


def plan_execution_readiness(plan: Dict[str, Any]) -> Dict[str, Any]:
    """Honest readiness — placeholders / pending poses => not_execution_ready."""
    blockers: List[str] = []
    if plan.get("execution_ready") is False:
        blockers.append("plan.execution_ready=false")
    for c in plan.get("cases") or []:
        notes = str(c.get("notes") or "")
        recipe = c.get("scene_recipe") or {}
        if "pending_pilot_confirm" in notes or recipe.get("occlusion_pose_status") == "pending_pilot_confirm":
            blockers.append(f"{c.get('case_id')}:occlusion_pose_pending_pilot_confirm")
        if recipe.get("state_apply_status") == "design_placeholder":
            blockers.append(f"{c.get('case_id')}:state_apply_design_placeholder")
        if c.get("execution_ready") is False:
            blockers.append(f"{c.get('case_id')}:execution_ready=false")
    ready = not blockers and not validate_plan(plan)
    return {
        "execution_ready": ready,
        "status": "execution_ready" if ready else "not_execution_ready",
        "blockers": blockers,
    }


# --- source registry ---------------------------------------------------------


def load_source_registry(path: Optional[Path]) -> Optional[Dict[str, Any]]:
    if path is None:
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def validate_registry_schema(registry: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Minimal schema + completeness declaration. Completeness ≠ independence certification."""
    if registry is None:
        return {
            "ok": False,
            "status": "registry_missing",
            "note": "no registry — isolation not_checked; empty/missing is not a full source audit",
        }
    missing = [k for k in REGISTRY_KEYS if k not in registry]
    if missing:
        return {"ok": False, "status": "registry_schema_invalid", "missing_keys": missing}
    for k in ("known_seeds", "known_content_hashes", "known_provenance_hashes"):
        if not isinstance(registry.get(k), list):
            return {"ok": False, "status": "registry_schema_invalid", "bad_key": k}
    comp = registry.get("completeness")
    if not isinstance(comp, dict) or "declared_complete" not in comp:
        return {
            "ok": False,
            "status": "registry_schema_invalid",
            "note": "completeness.declared_complete required",
        }
    if comp.get("declared_complete") is not True:
        return {
            "ok": False,
            "status": "registry_incomplete",
            "note": "completeness not declared — empty/incomplete registry is not a full source audit",
        }
    if not str(comp.get("statement") or "").strip():
        return {
            "ok": False,
            "status": "registry_incomplete",
            "note": "completeness.statement required when declared_complete=true (traceability premise only)",
        }
    return {
        "ok": True,
        "status": "registry_schema_ok",
        "note": "declared_complete is a traceability premise, NOT independence certification",
    }


def check_source_conflicts(
    plan: Dict[str, Any],
    registry: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    """Conflict review against explicit registry. Never claims isolation pass without registry."""
    schema = validate_registry_schema(registry)
    if registry is None or not schema["ok"]:
        return {
            "isolation_status": "not_checked" if registry is None else schema["status"],
            "registry_schema": schema,
            "n_conflicts": None,
            "cases": [],
            "note": schema.get("note")
            or "no source registry provided — must not claim isolation passed",
        }
    known_seeds = set(registry.get("known_seeds") or [])
    known_content = set(registry.get("known_content_hashes") or [])
    known_prov = set(registry.get("known_provenance_hashes") or [])
    hits: List[Dict[str, Any]] = []
    for c in plan.get("cases") or []:
        recipe = c.get("scene_recipe") or {}
        seed = recipe.get("seed")
        ch = content_hash(c)
        ph = provenance_hash(c, plan)
        reasons: List[str] = []
        if seed in known_seeds:
            reasons.append(f"seed_in_registry:{seed}")
        if ch in known_content:
            reasons.append(f"content_hash_collision:{ch[:12]}")
        if ph in known_prov:
            reasons.append(f"provenance_hash_collision:{ph[:12]}")
        hits.append(
            {
                "case_id": c.get("case_id"),
                "seed": seed,
                "content_hash": ch,
                "provenance_hash": ph,
                "conflict": bool(reasons),
                "reasons": reasons,
                "note": "same physical state alone is not leakage; full recipe/image/session linkage matters",
            }
        )
    n_conflicts = sum(1 for h in hits if h["conflict"])
    usage = plan.get("usage")
    if usage == USAGE_FORMAL and n_conflicts:
        isolation_status = "formal_blocked_unreviewed_conflict"
    elif usage == USAGE_PILOT and n_conflicts:
        isolation_status = "pilot_conflicts_recorded_not_independent"
    else:
        isolation_status = "no_registry_conflicts_not_independence_proof"
    return {
        "isolation_status": isolation_status,
        "registry_schema": schema,
        "n_conflicts": n_conflicts,
        "cases": hits,
        "note": "seed/content hashes are provenance aids — not independence certification",
    }


def validate_capture_source(capture_source: str) -> Optional[str]:
    if capture_source not in CAPTURE_SOURCES:
        return f"illegal_capture_source:{capture_source}"
    return None


def validate_ros_stamp_fn(ros_stamp_now: Optional[Callable[[], float]], capture_source: str) -> Optional[str]:
    if capture_source == "fake":
        return None
    if ros_stamp_now is None:
        return "non_fake_requires_explicit_ros_stamp_now"
    try:
        v = float(ros_stamp_now())
    except Exception as exc:  # noqa: BLE001
        return f"ros_stamp_now_raise:{exc}"
    if not math.isfinite(v) or v <= 0:
        return f"ros_stamp_invalid:{v}"
    return None


# --- sidecar -----------------------------------------------------------------


def build_sidecar(
    *,
    case: Dict[str, Any],
    plan: Dict[str, Any],
    image_rel: str,
    image_stamp_sec: float,
    image_frame_id: str,
    recv_mono: float,
    scene_op_done_token: str,
    timing: Dict[str, float],
    capture_source: str,
) -> Dict[str, Any]:
    recipe = dict(case["scene_recipe"])
    is_fake = capture_source == "fake"
    return {
        "schema": SCHEMA,
        "image_id": f"{plan['batch_id']}_{case['case_id']}",
        "file_name": image_rel,
        "session_id": plan["session_id"],
        "batch_id": plan["batch_id"],
        "case_id": case["case_id"],
        "usage": case["usage"],
        "station_id": case["station_id"],
        "distance_m": float(case["distance_m"]),
        "robot_pose_world": dict(case["robot_pose_world"]),
        "pose_frame_id": "world",
        "image_frame_id": image_frame_id,
        "image_stamp_sec": image_stamp_sec,
        "recv_monotonic": recv_mono,
        "timing": timing,
        "scene_op_done_token": scene_op_done_token,
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "scene_recipe": recipe,
        "content_hash": content_hash(case),
        "provenance_hash": provenance_hash(case, plan),
        "seed": recipe.get("seed"),
        "scene_state_intent": dict(case["target_states_intent"]),
        "review": {
            "status": "unreviewed",
            "states": None,
            "detections": None,
            "note": "physical intent ≠ review GT; occlusion does not auto-write UNKNOWN",
        },
        "boxes_candidate": True,
        "boxes_review_passed": False,
        "boxes_untrusted": False,
        "capture_source": capture_source,
        "synthetic": True if is_fake else False,
        "formal_test_eligible": False,
        "label_version": LABEL_VERSION,
        "split": None,
        "checklist_ids": list(case.get("checklist_ids") or []),
        "hard_case": list(case.get("hard_case") or []),
        "notes": case.get("notes") or "",
    }


# --- restore / ops -----------------------------------------------------------


@dataclass
class CaseResult:
    case_id: str
    ok: bool
    status: str
    detail: str = ""
    restore_errors: List[Dict[str, Any]] = field(default_factory=list)
    restore_uncertain: bool = False
    image_path: Optional[str] = None
    sidecar_path: Optional[str] = None
    interrupted: bool = False


def ensure_fresh_outdir(out_dir: Path) -> None:
    if out_dir.exists():
        raise FileExistsError(f"REFUSE: out dir exists (no --force): {out_dir}")


def write_batch_status(out_dir: Path, status: str, payload: Dict[str, Any]) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    data = {"status": status, "updated_at": datetime.now(timezone.utc).isoformat(), **payload}
    (out_dir / "batch_status.json").write_text(
        json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )


def validate_ops_preflight(adapter: SceneAdapter, ops: List[Dict[str, Any]]) -> None:
    """Read-only precondition check. Must NOT mutate the scene.

    Maintains expected presence so delete→spawn same name is allowed in plan order.
    previous XML/pose are restore *declarations* — not trusted snapshots without verify.
    """
    expected_present: Dict[str, bool] = {}

    def present(name: str) -> bool:
        if name in expected_present:
            return expected_present[name]
        return adapter.entity_exists(name)

    for op in ops:
        kind = op["op"]
        name = op["name"]
        if kind == "delete":
            prev = op.get("previous")
            if not isinstance(prev, dict) or "xml" not in prev or "pose" not in prev:
                raise ValueError(f"delete {name}: previous xml/pose declaration required")
            if not present(name):
                raise ValueError(f"delete {name}: entity not present (cannot verify restore declaration)")
            expected_present[name] = False
        elif kind == "spawn":
            if present(name):
                raise RuntimeError(f"spawn {name}: entity already exists — refuse overwrite")
            expected_present[name] = True
        elif kind == "set_pose":
            if "previous_pose" not in op:
                raise ValueError(f"set_pose {name}: previous_pose declaration required")
            if not present(name):
                raise ValueError(f"set_pose {name}: entity not present")
        else:
            raise ValueError(f"unknown op {kind}")


def execute_ops_accounting(
    adapter: SceneAdapter,
    ops: List[Dict[str, Any]],
    restore_stack: Optional[List[Dict[str, Any]]] = None,
) -> List[Dict[str, Any]]:
    """Execute ops in order. Register restore responsibility *before each attempt*.

    Unattempted later ops are never added to the restore stack.
    Exception after/before mutate still leaves registered responsibility for attempted ops
    (caller must pass a mutable restore_stack so finally can restore after raise).
    """
    if restore_stack is None:
        restore_stack = []
    for op in ops:
        kind = op["op"]
        name = op["name"]
        timeout = float(op.get("timeout_s", 5.0))
        if kind == "delete":
            prev = op["previous"]
            # Register restore BEFORE attempt — timeout/raise may still have mutated.
            # Controlled source metadata (if present) is reinstated after restore spawn.
            restore_entry: Dict[str, Any] = {
                "op": "spawn",
                "name": name,
                "xml": prev["xml"],
                "pose": list(prev["pose"]),
                "idempotent_missing_ok": False,
                "refuse_if_exists": True,
                "note": "previous is declared restore target; verify before blind overwrite",
            }
            if prev.get("source_kind"):
                restore_entry["controlled_restore_source"] = {
                    "source_kind": prev.get("source_kind"),
                    "source_path": prev.get("source_path"),
                    "content_hash": prev.get("content_hash"),
                    "world_hash": prev.get("world_hash"),
                    "verified": bool(prev.get("verified")),
                    "xml": prev["xml"],
                }
            restore_stack.append(restore_entry)
            adapter.delete(name, timeout_s=timeout)
        elif kind == "spawn":
            restore_stack.append(
                {
                    "op": "delete",
                    "name": name,
                    "idempotent_missing_ok": True,
                    "note": "delete temp entity; missing is ok (idempotent)",
                }
            )
            adapter.spawn(name, op["xml"], op["pose"], timeout_s=timeout)
        elif kind == "set_pose":
            restore_stack.append(
                {
                    "op": "set_pose",
                    "name": name,
                    "pose": list(op["previous_pose"]),
                    "idempotent_missing_ok": False,
                    "note": "previous_pose is declaration",
                }
            )
            adapter.set_pose(name, op["pose"], timeout_s=timeout)
        else:
            raise ValueError(f"unknown op {kind}")
    return restore_stack


def restore_scene(adapter: SceneAdapter, restore_recipe: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Reverse restore with idempotent contracts. Never blind-spawn over existing entities."""
    errors: List[Dict[str, Any]] = []
    for op in reversed(restore_recipe):
        try:
            kind = op["op"]
            name = op["name"]
            if kind == "delete":
                if adapter.entity_exists(name):
                    adapter.delete(name)
                elif not op.get("idempotent_missing_ok", False):
                    errors.append(
                        {
                            "op": kind,
                            "name": name,
                            "error": "delete_target_missing",
                            "uncertain": True,
                        }
                    )
            elif kind == "spawn":
                if adapter.entity_exists(name):
                    if op.get("refuse_if_exists", True):
                        errors.append(
                            {
                                "op": kind,
                                "name": name,
                                "error": "restore_spawn_refused_entity_exists",
                                "uncertain": True,
                                "note": "will not blind-overwrite; previous declaration unverified as live snapshot",
                            }
                        )
                        continue
                if hasattr(adapter, "spawn_restoring"):
                    adapter.spawn_restoring(  # type: ignore[attr-defined]
                        name, op["xml"], op["pose"], controlled=op.get("controlled_restore_source")
                    )
                else:
                    adapter.spawn(name, op["xml"], op["pose"])
            elif kind == "set_pose":
                if not adapter.entity_exists(name):
                    errors.append(
                        {
                            "op": kind,
                            "name": name,
                            "error": "set_pose_target_missing",
                            "uncertain": True,
                        }
                    )
                    continue
                adapter.set_pose(name, op["pose"])
            else:
                errors.append({"op": kind, "name": name, "error": "unknown_restore_op", "uncertain": True})
        except Exception as exc:  # noqa: BLE001
            errors.append(
                {
                    "op": op.get("op"),
                    "name": op.get("name"),
                    "error": str(exc),
                    "uncertain": True,
                    "note": "exception ≠ proof scene unchanged; needs status check",
                }
            )
    return errors

def run_one_case(
    *,
    case: Dict[str, Any],
    plan: Dict[str, Any],
    out_dir: Path,
    adapter: SceneAdapter,
    buffer: BoundedFrameBuffer,
    grab: GrabFn,
    mono: Callable[[], float],
    ros_stamp_now: Callable[[], float],
    settle_s: float = 0.05,
    wait_timeout_s: float = 2.0,
    sleep_fn: Callable[[float], None] = time.sleep,
    capture_source: str = "fake",
) -> CaseResult:
    cid = str(case["case_id"])
    token = f"op_done_{cid}"
    ops = list(case.get("scene_ops") or [])
    allow = case.get("entity_allowlist")
    if not isinstance(allow, list):
        return CaseResult(cid, False, "plan_invalid", "entity_allowlist missing")

    # Scene switch: clear candidate cache (and live camera ring / clock epoch if provided)
    buffer.clear()
    if hasattr(adapter, "begin_capture_cycle"):
        try:
            adapter.begin_capture_cycle()  # type: ignore[attr-defined]
        except Exception:  # noqa: BLE001
            pass
    elif hasattr(adapter, "clear_frame_candidates"):
        try:
            adapter.clear_frame_candidates()  # type: ignore[attr-defined]
        except Exception:  # noqa: BLE001
            pass

    restore: List[Dict[str, Any]] = []
    restore_errors: List[Dict[str, Any]] = []
    status = "running"
    detail = ""
    ok = False
    interrupted = False
    image_path: Optional[str] = None
    sidecar_path: Optional[str] = None
    timing: Dict[str, float] = {}

    try:
        try:
            # Allowlist before any query/inject.
            if hasattr(adapter, "set_allowlist"):
                adapter.set_allowlist(allow)  # type: ignore[attr-defined]
            elif hasattr(adapter, "allowlist"):
                adapter.allowlist = frozenset(allow)  # type: ignore[attr-defined]
            # Optional live hook: replace design previous_* with actual snapshots / verified sources.
            # Must not mutate scene (queries only). Fake adapters typically omit this method.
            if hasattr(adapter, "inject_actual_snapshots"):
                ops = list(adapter.inject_actual_snapshots(ops))  # type: ignore[attr-defined]
            # Read-only preflight (no mutation), then per-op register-then-attempt.
            # restore list is mutated in-place so raise mid-ops still restores attempted work.
            validate_ops_preflight(adapter, ops)
            execute_ops_accounting(adapter, ops, restore)
        except Exception as exc:  # noqa: BLE001
            status = "scene_op_failed"
            detail = str(exc)
            raise

        timing["scene_op_done_mono"] = mono()
        # ROS/sim stamp lower bound taken at scene-op completion (NOT mono arithmetic).
        # Live path must re-validate /clock here per case — not only at batch start.
        try:
            timing["ros_stamp_lower_bound"] = float(ros_stamp_now())
        except Exception as exc:  # noqa: BLE001
            status = "clock_or_stamp_error"
            detail = str(exc)
            raise

        try:
            sleep_fn(settle_s)
        except BaseException as exc:
            status = "settle_failed"
            detail = str(exc)
            if isinstance(exc, KeyboardInterrupt):
                interrupted = True
                status = "interrupted"
            raise

        timing["settle_done_mono"] = mono()
        deadline = timing["settle_done_mono"] + wait_timeout_s
        chosen: Optional[FrameCandidate] = None
        last_reject: Optional[str] = None

        while mono() < deadline:
            try:
                fr = grab(deadline)
            except TimeoutError as exc:
                status = "grab_deadline"
                detail = str(exc)
                raise
            except KeyboardInterrupt:
                interrupted = True
                status = "interrupted"
                detail = "KeyboardInterrupt_during_grab"
                raise
            except Exception as exc:  # noqa: BLE001
                status = "grab_failed"
                detail = str(exc)
                raise

            # Never mutate association fields on the frame object
            err = buffer.push(fr)
            if err:
                status = "clock_or_stamp_error"
                detail = err
                raise RuntimeError(err)

            chosen, last_reject = buffer.select_fresh(
                case_id=cid,
                scene_op_done_token=token,
                after_recv_mono=timing["settle_done_mono"],
                min_image_stamp_sec=timing["ros_stamp_lower_bound"],
            )
            if chosen is not None:
                break
            sleep_fn(0.01)

        if chosen is None:
            status = "wait_timeout"
            detail = last_reject or "wait_timeout_no_fresh_frame"
            raise RuntimeError(detail)

        img_name = f"{plan['batch_id']}_{cid}.png"
        img_rel = f"images/{img_name}"
        img_path_p = out_dir / "images" / img_name
        side_path_p = out_dir / "meta" / f"{plan['batch_id']}_{cid}.json"
        img_path_p.parent.mkdir(parents=True, exist_ok=True)
        side_path_p.parent.mkdir(parents=True, exist_ok=True)

        try:
            if not cv2.imwrite(str(img_path_p), chosen.bgr):
                raise OSError("cv2.imwrite returned False")
            if cv2.imread(str(img_path_p), cv2.IMREAD_COLOR) is None:
                raise OSError("PNG reopen failed")
            sidecar = build_sidecar(
                case=case,
                plan=plan,
                image_rel=img_rel,
                image_stamp_sec=chosen.image_stamp_sec,
                image_frame_id=chosen.image_frame_id,
                recv_mono=chosen.recv_mono,
                scene_op_done_token=token,
                timing=timing,
                capture_source=capture_source,
            )
            side_path_p.write_text(json.dumps(sidecar, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
            image_path = str(img_path_p)
            sidecar_path = str(side_path_p)
        except Exception as exc:  # noqa: BLE001
            status = "write_failed"
            detail = str(exc)
            raise

        status = "captured"
        detail = "ok"
        ok = True

    except KeyboardInterrupt:
        interrupted = True
        if status == "running":
            status = "interrupted"
            detail = "KeyboardInterrupt"
    except Exception:
        if status == "running":
            status = "case_failed"
        ok = False
    finally:
        restore_errors = restore_scene(adapter, restore)
        if restore_errors:
            ok = False
            if status in {"captured", "running"}:
                status = "restore_failed"
            # keep original failure status but mark restore failure structurally

    return CaseResult(
        case_id=cid,
        ok=ok and not restore_errors and not interrupted,
        status=("interrupted" if interrupted and status == "captured" else status),
        detail=detail,
        restore_errors=restore_errors,
        restore_uncertain=bool(restore_errors),
        image_path=image_path,
        sidecar_path=sidecar_path,
        interrupted=interrupted,
    )


def run_batch(
    plan: Dict[str, Any],
    out_dir: Path,
    *,
    adapter: SceneAdapter,
    buffer: BoundedFrameBuffer,
    grab: GrabFn,
    mono: Callable[[], float] = time.monotonic,
    ros_stamp_now: Optional[Callable[[], float]] = None,
    sleep_fn: Callable[[float], None] = time.sleep,
    settle_s: float = 0.05,
    wait_timeout_s: float = 2.0,
    capture_source: str = "fake",
    source_registry: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Execute batch. All gates run before any scene op / grab."""

    def _gate_return(status: str, **extra: Any) -> Dict[str, Any]:
        payload = {
            "ok": False,
            "status": status,
            "capture_source": capture_source,
            "formal_test_eligible": False,
            "n_scene_ops_attempted": 0,
            "n_grabs": 0,
            **extra,
        }
        write_batch_status(out_dir, status, payload)
        (out_dir / "batch_summary.json").write_text(
            json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
        return payload

    ensure_fresh_outdir(out_dir)
    out_dir.mkdir(parents=True, exist_ok=False)
    (out_dir / "images").mkdir()
    (out_dir / "meta").mkdir()

    cs_err = validate_capture_source(capture_source)
    if cs_err:
        return _gate_return("illegal_capture_source", error=cs_err)

    if capture_source == "fake" and plan.get("usage") == USAGE_FORMAL:
        return _gate_return("fake_refuses_formal_test_candidate")

    verrs = validate_plan(plan)
    if verrs:
        return _gate_return("plan_invalid", errors=verrs)

    readiness = plan_execution_readiness(plan)
    (out_dir / "execution_readiness.json").write_text(
        json.dumps(readiness, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    # Gate: readiness=false → zero scene ops, zero grabs (fake and non-fake)
    if not readiness.get("execution_ready"):
        return _gate_return(
            "not_execution_ready",
            execution_readiness=readiness,
            note="structure may be valid; execution refused before any scene/grab",
        )

    reg_schema = validate_registry_schema(source_registry)
    conflicts = check_source_conflicts(plan, source_registry)
    (out_dir / "source_conflict_review.json").write_text(
        json.dumps(conflicts, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    if plan.get("usage") == USAGE_FORMAL:
        # formal: missing / incomplete / not_checked / unreviewed conflicts — all refuse pre-scene
        if source_registry is None or conflicts.get("isolation_status") == "not_checked":
            return _gate_return(
                "formal_registry_missing_or_not_checked",
                source_conflicts=conflicts,
                registry_schema=reg_schema,
            )
        if not reg_schema.get("ok"):
            return _gate_return(
                "formal_registry_incomplete_or_invalid",
                source_conflicts=conflicts,
                registry_schema=reg_schema,
            )
        if conflicts.get("isolation_status") == "formal_blocked_unreviewed_conflict":
            return _gate_return(
                "formal_blocked_unreviewed_conflict",
                source_conflicts=conflicts,
                registry_schema=reg_schema,
            )

    stamp_err = validate_ros_stamp_fn(ros_stamp_now, capture_source)
    if stamp_err:
        return _gate_return("ros_stamp_contract_failed", error=stamp_err)

    if capture_source == "fake" and ros_stamp_now is None:
        _stamp = {"v": 1000.0}

        def ros_stamp_now() -> float:
            return _stamp["v"]

    assert ros_stamp_now is not None

    write_batch_status(out_dir, "running", {"batch_id": plan["batch_id"], "capture_source": capture_source})
    results: List[CaseResult] = []
    abort = False
    abort_reason = ""
    interrupted = False

    try:
        for case in plan["cases"]:
            if abort:
                results.append(
                    CaseResult(str(case["case_id"]), False, "skipped_after_abort", abort_reason)
                )
                continue
            cr = run_one_case(
                case=case,
                plan=plan,
                out_dir=out_dir,
                adapter=adapter,
                buffer=buffer,
                grab=grab,
                mono=mono,
                ros_stamp_now=ros_stamp_now,
                settle_s=settle_s,
                wait_timeout_s=wait_timeout_s,
                sleep_fn=sleep_fn,
                capture_source=capture_source,
            )
            results.append(cr)
            if cr.interrupted:
                interrupted = True
                abort = True
                abort_reason = f"{cr.case_id}:interrupted"
            elif cr.restore_errors or cr.status == "restore_failed":
                abort = True
                abort_reason = f"{cr.case_id}:restore_failed"
            elif not cr.ok and cr.status in {
                "scene_op_failed",
                "grab_failed",
                "settle_failed",
                "grab_deadline",
                "write_failed",
                "clock_or_stamp_error",
                "wait_timeout",
            }:
                abort = True
                abort_reason = f"{cr.case_id}:{cr.status}"
    except KeyboardInterrupt:
        interrupted = True
        abort = True
        abort_reason = "batch_KeyboardInterrupt"

    all_ok = bool(results) and all(r.ok for r in results) and not abort and not interrupted
    if interrupted:
        status = "batch_interrupted"
    elif abort:
        status = "batch_aborted"
    elif all_ok:
        status = "batch_fake_complete" if capture_source == "fake" else "batch_complete"
    else:
        status = "batch_failed"

    summary = {
        "ok": all_ok,
        "status": status,
        "batch_id": plan["batch_id"],
        "session_id": plan["session_id"],
        "usage": plan["usage"],
        "capture_source": capture_source,
        "synthetic_batch": capture_source == "fake",
        "formal_test_eligible": False,
        "n_ok": sum(1 for r in results if r.ok),
        "n_fail": sum(1 for r in results if not r.ok),
        "cases": [asdict(r) for r in results],
        "buffer_stats": {
            "maxlen": buffer.maxlen,
            "received": buffer.received,
            "dropped": buffer.dropped,
            "buffered": len(buffer),
        },
        "source_conflicts": conflicts,
        "execution_readiness": readiness,
        "registry_schema": reg_schema,
    }
    write_batch_status(out_dir, status, summary)
    (out_dir / "batch_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return summary

def dry_run(
    plan: Dict[str, Any],
    out_dir: Path,
    *,
    source_registry: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    ensure_fresh_outdir(out_dir)
    out_dir.mkdir(parents=True, exist_ok=False)
    verrs = validate_plan(plan)
    readiness = plan_execution_readiness(plan)
    conflicts = check_source_conflicts(plan, source_registry)
    report = {
        "mode": "dry_run",
        "ok": not verrs,
        "plan_errors": verrs,
        "execution_readiness": readiness,
        "source_conflict_review": conflicts,
        "session_id": plan.get("session_id"),
        "batch_id": plan.get("batch_id"),
        "usage": plan.get("usage"),
        "n_cases": len(plan.get("cases") or []),
        "formal_test_eligible": False,
        "capture_source": "none_dry_run",
        "cases_preview": [
            {
                "case_id": c.get("case_id"),
                "station_id": c.get("station_id"),
                "distance_m": c.get("distance_m"),
                "usage": c.get("usage"),
                "content_hash": content_hash(c),
                "provenance_hash": provenance_hash(c, plan),
                "entity_allowlist": c.get("entity_allowlist"),
                "n_scene_ops": len(c.get("scene_ops") or []),
            }
            for c in (plan.get("cases") or [])
        ],
        "notes": [
            "dry-run does not import/init ROS",
            "not a formal capture completion",
            "real ROS adapter not implemented",
        ],
    }
    # Success of validation ≠ execution ready
    if report["ok"] and readiness["status"] == "not_execution_ready":
        batch_status = "dry_run_ok_not_execution_ready"
    elif report["ok"]:
        batch_status = "dry_run_ok"
    else:
        batch_status = "dry_run_invalid"
    (out_dir / "dry_run_report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    write_batch_status(out_dir, batch_status, report)
    return report


def _visible_box_sdf(name: str, size=(0.15, 0.4, 0.3), color=(0.8, 0.1, 0.1, 1.0)) -> str:
    sx, sy, sz = size
    r, g, b, a = color
    return f"""<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <visual name='visual'>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse></material>
      </visual>
      <collision name='collision'>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
"""


def example_pilot_plan() -> Dict[str, Any]:
    """12-frame pilot_dev design plan — honestly not_execution_ready until pilot confirms poses."""
    cases: List[Dict[str, Any]] = []
    # Pretend session start pose for robot (required previous for set_pose)
    home_pose = [0.9, 3.0, 0.05, 0.0, 0.0, 0.0]

    for sid in ("A", "B", "C"):
        for n in (1, 2):
            pose = dict(STATION_POSE_WORLD[sid])
            target_pose = [pose["x"], pose["y"], pose["z"], 0.0, 0.0, pose["yaw"]]
            intent = {
                "leds": ["green", "green", "red"] if sid != "C" else ["green", "off", "green"],
                "panel": "lit",
                "leak": "fixed_fail" if sid == "B" else "none",
            }
            cases.append(
                {
                    "case_id": f"pilot_normal_{sid}_{n}",
                    "station_id": sid,
                    "distance_m": 1.2,
                    "robot_pose_world": pose,
                    "pose_frame_id": "world",
                    "usage": USAGE_PILOT,
                    "execution_ready": False,
                    "target_states_intent": intent,
                    "scene_recipe": {
                        "kind": "normal",
                        "seed": 300 + ord(sid) + n,
                        "state_apply_status": "design_placeholder",
                        "state_apply_method": "future_authorized_material_swap_or_gt_apply",
                        "note": "seed identifies recipe family only",
                    },
                    "entity_allowlist": [ROBOT_ENTITY],
                    "scene_ops": [
                        {
                            "op": "set_pose",
                            "name": ROBOT_ENTITY,
                            "pose": target_pose,
                            "previous_pose": list(home_pose),
                            "timeout_s": 5.0,
                        }
                    ],
                    "checklist_ids": [f"PILOT-N-{sid}"],
                    "hard_case": [],
                    "notes": "robot_v0 teleported to station pose; LED/panel/leak apply still design_placeholder",
                }
            )

    for sid in ("A", "B", "C"):
        for n in (1, 2):
            pose = dict(STATION_POSE_WORLD[sid])
            target_pose = [pose["x"], pose["y"], pose["z"], 0.0, 0.0, pose["yaw"]]
            ent = f"t02c1_tmp_occ_{sid}_{n}"
            # Occlusion pose pending pilot confirmation — do not claim UNKNOWN GT
            occ_pose = [pose["x"], pose["y"] - 0.35, 0.45, 0.0, 0.0, 0.0]
            intent = {
                "leds": ["green", "green", "green"],
                "panel": "lit",
                "leak": "fixed_fail" if sid == "B" else "none",
            }
            cases.append(
                {
                    "case_id": f"pilot_hard_{sid}_{n}",
                    "station_id": sid,
                    "distance_m": 1.2,
                    "robot_pose_world": pose,
                    "pose_frame_id": "world",
                    "usage": USAGE_PILOT,
                    "execution_ready": False,
                    "target_states_intent": intent,
                    "scene_recipe": {
                        "kind": "partial_occlusion",
                        "seed": 400 + ord(sid) + n,
                        "temp_entity": ent,
                        "occlusion_pose_status": "pending_pilot_confirm",
                        "state_apply_status": "design_placeholder",
                        "note": "occlusion may leave reviewable UNKNOWN later — not auto GT",
                    },
                    "entity_allowlist": [ROBOT_ENTITY, ent],
                    "scene_ops": [
                        {
                            "op": "set_pose",
                            "name": ROBOT_ENTITY,
                            "pose": target_pose,
                            "previous_pose": list(home_pose),
                        },
                        {
                            "op": "spawn",
                            "name": ent,
                            "xml": _visible_box_sdf(ent),
                            "pose": occ_pose,
                        },
                    ],
                    "checklist_ids": [f"PILOT-H-{sid}"],
                    "hard_case": ["occlusion"],
                    "notes": "pending_pilot_confirm occlusion pose; review stays unreviewed; no UNKNOWN auto-GT",
                }
            )

    return {
        "session_id": "pilot_dev_design_only",
        "batch_id": "t02_pilot_dev_design_12",
        "usage": USAGE_PILOT,
        "execution_ready": False,
        "cases": cases,
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="T02 collect (default dry-run, no ROS)")
    ap.add_argument("--plan", type=Path)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--execute", action="store_true")
    ap.add_argument("--adapter", choices=("none", "fake", "live"), default="none")
    ap.add_argument("--source-registry", type=Path, help="explicit known-source JSON registry")
    ap.add_argument("--emit-example-pilot-plan", type=Path)
    ap.add_argument("--emit-pilot-c2a-plan", type=Path, help="emit C2A pilot_dev 12-case plan (no ROS)")
    args = ap.parse_args(list(argv) if argv is not None else None)

    if args.emit_example_pilot_plan:
        p = args.emit_example_pilot_plan
        if p.exists():
            print(f"REFUSE exists: {p}", file=sys.stderr)
            return 2
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(example_pilot_plan(), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"wrote {p}")
        return 0

    if args.emit_pilot_c2a_plan:
        # Local import of plan builder only — t02_ros_adapter import must not init ROS.
        import t02_ros_adapter as ros_ad  # noqa: WPS433

        p = args.emit_pilot_c2a_plan
        if p.exists():
            print(f"REFUSE exists: {p}", file=sys.stderr)
            return 2
        p.parent.mkdir(parents=True, exist_ok=True)
        plan_c2a = ros_ad.build_pilot_c2a_plan()
        p.write_text(json.dumps(plan_c2a, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"wrote {p}")
        return 0

    if not args.plan or not args.out:
        print("--plan and --out required unless emit-*-plan", file=sys.stderr)
        return 2
    plan = json.loads(args.plan.read_text(encoding="utf-8"))
    registry = load_source_registry(args.source_registry) if args.source_registry else None

    if not args.execute:
        if "rclpy" in sys.modules:
            print("REFUSE: rclpy already imported in dry-run process", file=sys.stderr)
            return 2
        report = dry_run(plan, args.out, source_registry=registry)
        print(
            json.dumps(
                {
                    "mode": "dry_run",
                    "ok": report["ok"],
                    "execution_readiness": report["execution_readiness"]["status"],
                    "out": str(args.out),
                },
                ensure_ascii=False,
            )
        )
        return 0 if report["ok"] else 1

    if args.adapter == "live":
        # Explicit live entry only — imports ROS inside adapter.
        import t02_ros_adapter as ros_ad  # noqa: WPS433

        try:
            ros_ad.refuse_if_formal(plan)
        except PermissionError as exc:
            print(f"REFUSE: {exc}", file=sys.stderr)
            return 2
        summary = ros_ad.run_live_batch(plan, args.out, source_registry=registry)
        print(
            json.dumps(
                {
                    "mode": "execute_live",
                    "ok": summary.get("ok"),
                    "status": summary.get("status"),
                    "formal_test_eligible": False,
                    "real_capture_pass": False,
                    "capture_source": "live_ros",
                },
                ensure_ascii=False,
            )
        )
        return 0 if summary.get("ok") else 1

    if args.adapter != "fake":
        print("REFUSE: use --adapter fake|live", file=sys.stderr)
        return 2
    if plan.get("usage") == USAGE_FORMAL:
        print("REFUSE: fake cannot execute formal_test_candidate", file=sys.stderr)
        return 2

    cam = FakeCamera()
    adapter = FakeSceneAdapter()
    # Seed robot entity for set_pose in plans
    adapter.entities[ROBOT_ENTITY] = {"xml": "<robot/>", "pose": [0.9, 3.0, 0.05, 0.0, 0.0, 0.0]}
    buf = BoundedFrameBuffer(maxlen=8)

    def grab(deadline: float) -> FrameCandidate:
        return cam.grab(deadline, time.monotonic)

    summary = run_batch(
        plan,
        args.out,
        adapter=adapter,
        buffer=buf,
        grab=grab,
        capture_source="fake",
        source_registry=registry,
    )
    print(
        json.dumps(
            {
                "mode": "execute_fake",
                "ok": summary["ok"],
                "status": summary["status"],
                "formal_test_eligible": False,
                "capture_source": "fake",
            },
            ensure_ascii=False,
        )
    )
    return 0 if summary["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
