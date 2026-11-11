#!/usr/bin/env python3
"""M1-T02-C2A-R1: live ROS/Gazebo adapter for t02_collect_raw (dev only).

Importing this module MUST NOT init rclpy or touch ROS services.
Only explicit live entry (LiveRosBundle / run_live_batch) initializes ROS.

Constraints:
- offline mock tests only; do not start/connect live ROS/Gazebo here
- live usage locked to pilot_dev; formal_test_candidate refused
- real adapter present ≠ real-capture PASS
- restore sources are session-controlled; plan cannot self-verify
"""

from __future__ import annotations

import hashlib
import json
import math
import sys
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Protocol, Sequence, Tuple

import numpy as np

REPO = Path(__file__).resolve().parents[1]
SCRIPTS = Path(__file__).resolve().parent
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import t02_collect_raw as core  # noqa: E402

ROBOT_ENTITY = core.ROBOT_ENTITY
USAGE_PILOT = core.USAGE_PILOT
USAGE_FORMAL = core.USAGE_FORMAL
STATION_POSE_WORLD = core.STATION_POSE_WORLD

IMAGE_TOPIC = "/camera/image_raw"
CLOCK_TOPIC = "/clock"
EXPECTED_FRAME_ID = "camera_link"
EXPECTED_WH = (640, 480)
POSE_POS_TOL_M = 0.05  # collect-tool verify only — not Nav2 threshold
POSE_RPY_TOL_RAD = 0.08
TMP_OCC_PREFIX = "t02c2a_tmp_occ_"
CAMERA_HEIGHT_M = 0.12
CAMERA_OFFSET_X_M = 0.215
CAMERA_HFOV_RAD = 1.047  # from robot_v0.gazebo.xacro
OCC_BOX_SIZE = (0.12, 0.08, 0.22)  # partial cover in camera FOV
APPROVED_VISUAL_PREFIXES = (
    "cabinet_a_led_",
    "cabinet_a_panel",
    "cabinet_b_led_",
    "cabinet_b_panel",
    "cabinet_c_led_",
    "cabinet_c_panel",
)

CommandRunner = Callable[[Sequence[str], float], "CmdResult"]


class Existence(Enum):
    PRESENT = "present"
    ABSENT = "absent"
    QUERY_FAILED = "query_failed"


class EntityQueryError(RuntimeError):
    """Query failed — must NOT be treated as ABSENT."""


class UncertainOpError(RuntimeError):
    """Service timeout / unknown outcome — op may have already mutated the scene."""

    def __init__(self, message: str, *, uncertain: bool = True) -> None:
        super().__init__(message)
        self.uncertain = uncertain


class ClockInvalidError(RuntimeError):
    pass


@dataclass
class CmdResult:
    returncode: int
    stdout: str = ""
    stderr: str = ""
    timed_out: bool = False


@dataclass
class GazeboCapabilities:
    spawn_entity: bool = False
    delete_entity: bool = False
    get_model_list: bool = False
    get_entity_state: bool = False
    set_entity_state: bool = False
    gz_model_pose_candidate: bool = False  # gz model -h only — not yet verified
    gz_model_pose_verified: bool = False  # set only after successful structured read
    notes: List[str] = field(default_factory=list)

    def can_read_pose(self) -> bool:
        return self.get_entity_state or self.gz_model_pose_verified

    def can_set_pose(self) -> bool:
        # set via gz allowed as candidate path only after read verified OR entity_state
        return self.set_entity_state or self.gz_model_pose_verified

    def can_spawn_delete(self) -> bool:
        return self.spawn_entity and self.delete_entity

    def summary(self) -> Dict[str, Any]:
        return {
            "spawn_entity": self.spawn_entity,
            "delete_entity": self.delete_entity,
            "get_model_list": self.get_model_list,
            "get_entity_state": self.get_entity_state,
            "set_entity_state": self.set_entity_state,
            "gz_model_pose_candidate": self.gz_model_pose_candidate,
            "gz_model_pose_verified": self.gz_model_pose_verified,
            "can_read_pose": self.can_read_pose(),
            "can_set_pose": self.can_set_pose(),
            "can_spawn_delete": self.can_spawn_delete(),
            "notes": list(self.notes),
        }


@dataclass
class RestoreSource:
    """Controlled restore origin — plan cannot self-verify this."""

    entity_name: str
    source_kind: str  # world_sdf | session_spawn
    source_path: Optional[str]
    content_hash: str
    xml: str
    pose: List[float]
    verified: bool = False
    world_hash: Optional[str] = None


def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def quat_to_rpy(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """Extract full roll, pitch, yaw from quaternion."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return quat_to_rpy(x, y, z, w)[2]


def _angle_diff(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def pose6_close(
    a: Sequence[float],
    b: Sequence[float],
    pos_tol: float = POSE_POS_TOL_M,
    rpy_tol: float = POSE_RPY_TOL_RAD,
) -> bool:
    if len(a) < 6 or len(b) < 6:
        return False
    try:
        af = [float(x) for x in a[:6]]
        bf = [float(x) for x in b[:6]]
    except (TypeError, ValueError):
        return False
    if any(not math.isfinite(v) for v in af + bf):
        return False
    if any(abs(af[i] - bf[i]) > pos_tol for i in range(3)):
        return False
    return all(_angle_diff(af[i], bf[i]) <= rpy_tol for i in range(3, 6))


def parse_gz_pose_stdout(stdout: str) -> List[float]:
    """Strict CLI pose parse: exactly six finite numbers as the entire token stream."""
    tokens = stdout.replace(",", " ").split()
    if len(tokens) != 6:
        raise RuntimeError(f"gz_pose_strict_parse_expected_6_tokens_got_{len(tokens)}")
    vals: List[float] = []
    for t in tokens:
        try:
            v = float(t)
        except ValueError as exc:
            raise RuntimeError(f"gz_pose_strict_parse_non_float:{t}") from exc
        if not math.isfinite(v):
            raise RuntimeError(f"gz_pose_strict_parse_non_finite:{t}")
        vals.append(v)
    return vals


def occlusion_pose_for_station(station_id: str) -> List[float]:
    """Place occluder on camera→target ray (partial FOV cover). Not shared world-y offset."""
    geom = compute_occlusion_geometry(station_id)
    return list(geom["occluder_pose"])


def compute_occlusion_geometry(station_id: str) -> Dict[str, Any]:
    """Offline FOV geometry using camera h=0.12, hfov, robot yaw, panel/led heights."""
    import apply_vision_scene_gt as avg  # noqa: WPS433

    rp = STATION_POSE_WORLD[station_id]
    yaw = float(rp["yaw"])
    fx, fy = math.cos(yaw), math.sin(yaw)
    # camera world position (approx): base + forward bumper offset + height
    cam = [
        float(rp["x"]) + fx * CAMERA_OFFSET_X_M,
        float(rp["y"]) + fy * CAMERA_OFFSET_X_M,
        CAMERA_HEIGHT_M,
    ]
    panel = list(avg.STATION_POSE[station_id]["panel"][:3])
    # aim at panel center; occluder fractionally along ray so it sits between cam and target
    t = 0.55
    occ_xyz = [cam[i] + t * (panel[i] - cam[i]) for i in range(3)]
    # slight lateral offset for partial (not full) cover
    lx, ly = -fy, fx
    occ_xyz[0] += lx * 0.04
    occ_xyz[1] += ly * 0.04
    occ_pose = [occ_xyz[0], occ_xyz[1], occ_xyz[2], 0.0, 0.0, yaw]
    sx, sy, sz = OCC_BOX_SIZE
    return {
        "camera_xyz": cam,
        "target_panel_xyz": panel,
        "occluder_pose": occ_pose,
        "occluder_size": [sx, sy, sz],
        "hfov_rad": CAMERA_HFOV_RAD,
        "station_yaw": yaw,
    }


def project_point_to_image(cam_xyz: Sequence[float], yaw: float, pt: Sequence[float], img_w: int = 640, img_h: int = 480) -> Optional[Tuple[float, float]]:
    """Simple body-forward pinhole; returns pixel or None if behind camera."""
    # transform point into camera frame (x forward, y left, z up) approx
    dx = float(pt[0]) - float(cam_xyz[0])
    dy = float(pt[1]) - float(cam_xyz[1])
    dz = float(pt[2]) - float(cam_xyz[2])
    # rotate by -yaw into robot forward frame
    cf = math.cos(-yaw)
    sf = math.sin(-yaw)
    x_f = cf * dx - sf * dy
    y_l = sf * dx + cf * dy
    if x_f <= 1e-6:
        return None
    fx = (img_w / 2.0) / math.tan(CAMERA_HFOV_RAD / 2.0)
    fy = fx  # square pixels assumption for offline check
    u = img_w / 2.0 - fx * (y_l / x_f)
    v = img_h / 2.0 - fy * (dz / x_f)
    return u, v


def occlusion_fov_overlap_report(station_id: str) -> Dict[str, Any]:
    """Check occluder partially overlaps target projection while leaving visible region."""
    import apply_vision_scene_gt as avg  # noqa: WPS433

    g = compute_occlusion_geometry(station_id)
    cam, yaw = g["camera_xyz"], g["station_yaw"]
    panel = g["target_panel_xyz"]
    led = list(avg.STATION_POSE[station_id]["leds"][1][:3])  # middle LED
    occ = g["occluder_pose"][:3]
    sx, sy, sz = g["occluder_size"]
    # sample corners of panel (approx 0.30 x 0.15) and occluder box
    panel_pts = [
        [panel[0] + dx, panel[1], panel[2] + dz]
        for dx in (-0.12, 0.12)
        for dz in (-0.05, 0.05)
    ]
    occ_pts = [
        [occ[0] + dx, occ[1] + dy, occ[2] + dz]
        for dx in (-sx / 2, sx / 2)
        for dy in (-sy / 2, sy / 2)
        for dz in (-sz / 2, sz / 2)
    ]
    panel_uv = [project_point_to_image(cam, yaw, p) for p in panel_pts]
    occ_uv = [project_point_to_image(cam, yaw, p) for p in occ_pts]
    panel_uv = [p for p in panel_uv if p is not None]
    occ_uv = [p for p in occ_uv if p is not None]
    if not panel_uv or not occ_uv:
        return {"ok": False, "reason": "projection_failed", "station_id": station_id}

    def _bbox(pts):
        us = [p[0] for p in pts]
        vs = [p[1] for p in pts]
        return min(us), min(vs), max(us), max(vs)

    pb = _bbox(panel_uv)
    ob = _bbox(occ_uv)

    def _area(b):
        return max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])

    inter = (
        max(pb[0], ob[0]),
        max(pb[1], ob[1]),
        min(pb[2], ob[2]),
        min(pb[3], ob[3]),
    )
    inter_a = _area(inter)
    panel_a = _area(pb)
    overlap_frac = inter_a / panel_a if panel_a > 1e-6 else 0.0
    # partial: some overlap but not full cover
    ok = 0.05 <= overlap_frac <= 0.85
    return {
        "ok": ok,
        "station_id": station_id,
        "overlap_frac": overlap_frac,
        "panel_bbox": pb,
        "occ_bbox": ob,
        "led_uv": project_point_to_image(cam, yaw, led),
        "note": "offline FOV partial-overlap check; real UNKNOWN effect pending pilot",
    }


def station_target_pose(station_id: str) -> List[float]:
    p = STATION_POSE_WORLD[station_id]
    return [float(p["x"]), float(p["y"]), float(p["z"]), 0.0, 0.0, float(p["yaw"])]


def is_tmp_occlusion_name(name: str) -> bool:
    return name.startswith("t02c1_tmp_occ_") or name.startswith(TMP_OCC_PREFIX) or name.startswith("t02c2a_tmp_")


def is_approved_visual_name(name: str) -> bool:
    return any(name.startswith(p) or name == p.rstrip("_") for p in APPROVED_VISUAL_PREFIXES) or name in {
        "cabinet_a_panel",
        "cabinet_b_panel",
        "cabinet_c_panel",
    }


def default_allowlist_for_case(case: Dict[str, Any]) -> List[str]:
    allow = [ROBOT_ENTITY]
    for op in case.get("scene_ops") or []:
        n = op.get("name")
        if isinstance(n, str) and n not in allow:
            allow.append(n)
    return allow


# --- approved visual XML (reuse apply_vision_scene_gt generators; no world edit) ---


def approved_visual_xml(name: str, state: str) -> str:
    """Build XML via approved generators only. Import is local to avoid ROS."""
    sys.path.insert(0, str(SCRIPTS))
    import apply_vision_scene_gt as avg  # noqa: WPS433

    zero = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    if "_led_" in name:
        return avg.led_model_xml(name, zero, state)
    if name.endswith("_panel") or "_panel" in name:
        return avg.panel_model_xml(name, zero, state)
    raise ValueError(f"no approved generator for visual entity: {name}")


def visible_box_sdf(name: str) -> str:
    return core._visible_box_sdf(name)


# --- subprocess command runner (param array + timeout; no stdout parsing for task API) ---


def run_cmd_argv(argv: Sequence[str], timeout_s: float = 5.0) -> CmdResult:
    import subprocess

    try:
        cp = subprocess.run(
            list(argv),
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )
        return CmdResult(cp.returncode, cp.stdout or "", cp.stderr or "", timed_out=False)
    except subprocess.TimeoutExpired as exc:
        out = exc.stdout.decode() if isinstance(exc.stdout, (bytes, bytearray)) else (exc.stdout or "")
        err = exc.stderr.decode() if isinstance(exc.stderr, (bytes, bytearray)) else (exc.stderr or "")
        return CmdResult(124, out, err, timed_out=True)


# --- clock / camera (injectable for offline tests) ---


@dataclass
class ClockState:
    """Persistent invalidation until explicit epoch reset (new capture cycle)."""

    stamp_sec: Optional[float] = None
    recv_mono: Optional[float] = None
    last_stamp_sec: Optional[float] = None
    epoch: int = 0
    invalid: bool = False
    invalid_reason: Optional[str] = None

    def update(self, stamp_sec: float, recv_mono: float) -> None:
        stamp_sec = float(stamp_sec)
        recv_mono = float(recv_mono)
        if self.invalid:
            # Keep invalid for this epoch; still track last seen stamp for diagnostics.
            self.last_stamp_sec = stamp_sec
            self.recv_mono = recv_mono
            return
        if self.last_stamp_sec is not None and stamp_sec < self.last_stamp_sec:
            self.invalid = True
            self.invalid_reason = "ros_clock_went_backwards"
            self.stamp_sec = None
            self.recv_mono = recv_mono
            self.last_stamp_sec = stamp_sec
            return
        if not math.isfinite(stamp_sec) or stamp_sec <= 0:
            self.invalid = True
            self.invalid_reason = f"ros_clock_invalid:{stamp_sec}"
            self.stamp_sec = None
            self.recv_mono = recv_mono
            self.last_stamp_sec = stamp_sec
            return
        self.stamp_sec = stamp_sec
        self.recv_mono = recv_mono
        self.last_stamp_sec = stamp_sec

    def reset_epoch(self) -> int:
        """Only call at explicit new capture cycle start."""
        self.epoch += 1
        self.invalid = False
        self.invalid_reason = None
        self.stamp_sec = None
        self.recv_mono = None
        self.last_stamp_sec = None
        return self.epoch

    def validate_fresh(self, *, max_age_mono: float = 2.0, now_mono: Optional[float] = None) -> float:
        if self.invalid:
            raise ClockInvalidError(self.invalid_reason or "ros_clock_epoch_invalid")
        if self.stamp_sec is None or self.recv_mono is None:
            raise ClockInvalidError("ros_clock_missing")
        if not math.isfinite(self.stamp_sec) or self.stamp_sec <= 0:
            raise ClockInvalidError(f"ros_clock_invalid:{self.stamp_sec}")
        now = time.monotonic() if now_mono is None else now_mono
        if now - self.recv_mono > max_age_mono:
            raise ClockInvalidError("ros_clock_stale")
        return float(self.stamp_sec)


@dataclass
class ImageFrame:
    bgr: np.ndarray
    stamp_sec: float
    frame_id: str
    seq: int
    recv_mono: float


class FrameRing:
    def __init__(self, maxlen: int = 8) -> None:
        self.maxlen = maxlen
        self._items: List[ImageFrame] = []
        self.seq = 0
        self.last_returned_seq: Optional[int] = None

    def __len__(self) -> int:
        return len(self._items)

    def clear(self) -> None:
        self._items.clear()
        self.last_returned_seq = None

    def push(self, fr: ImageFrame) -> None:
        self._items.append(fr)
        if len(self._items) > self.maxlen:
            self._items = self._items[-self.maxlen :]

    def newest(self) -> Optional[ImageFrame]:
        return self._items[-1] if self._items else None

    def newest_unreturned(self) -> Optional[ImageFrame]:
        fr = self.newest()
        if fr is None:
            return None
        if self.last_returned_seq is not None and fr.seq == self.last_returned_seq:
            return None
        return fr

    def mark_returned(self, fr: ImageFrame) -> None:
        self.last_returned_seq = fr.seq


def validate_bgr_frame(bgr: np.ndarray, frame_id: str) -> None:
    if frame_id != EXPECTED_FRAME_ID:
        raise ValueError(f"unexpected_image_frame_id:{frame_id}")
    if bgr is None or not isinstance(bgr, np.ndarray):
        raise ValueError("image_not_ndarray")
    if bgr.dtype != np.uint8:
        raise ValueError(f"image_dtype:{bgr.dtype}")
    if bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError(f"image_shape_not_bgr:{getattr(bgr, 'shape', None)}")
    h, w = bgr.shape[:2]
    if (w, h) != EXPECTED_WH:
        raise ValueError(f"image_wh_expected_640x480_got_{w}x{h}")


# --- service future helper (mockable) ---


@dataclass
class ServiceCallOutcome:
    done: bool
    success: Optional[bool] = None
    message: str = ""
    exception: Optional[BaseException] = None
    timed_out: bool = False
    result_obj: Any = None


class ServiceClientLike(Protocol):
    def wait_for_service(self, timeout_sec: float = 1.0) -> bool: ...

    def call_async(self, request: Any) -> Any: ...


def interpret_future(
    *,
    done: bool,
    result: Any,
    exc: Optional[BaseException],
    timed_out: bool,
) -> ServiceCallOutcome:
    if timed_out or not done:
        return ServiceCallOutcome(done=False, timed_out=True, message="service_timeout_uncertain")
    if exc is not None:
        return ServiceCallOutcome(done=True, exception=exc, message=str(exc))
    if result is None:
        return ServiceCallOutcome(done=True, success=False, message="null_result")
    success = bool(getattr(result, "success", True))
    msg = str(getattr(result, "status_message", "") or "")
    return ServiceCallOutcome(done=True, success=success, message=msg, result_obj=result)


def raise_from_outcome(op: str, name: str, out: ServiceCallOutcome) -> None:
    if out.timed_out or not out.done:
        raise UncertainOpError(f"{op}:{name}:timeout_result_uncertain")
    if out.exception is not None:
        raise UncertainOpError(f"{op}:{name}:exception:{out.exception}")
    if out.success is False:
        raise RuntimeError(f"{op}:{name}:success=false:{out.message}")


# --- Gazebo scene adapter (dependency-injected; no ROS import at construct for mocks) ---


@dataclass
class GazeboSceneAdapter:
    """SceneAdapter compatible with t02_collect_raw, with live safety contracts."""

    allowlist: Optional[frozenset] = None
    caps: GazeboCapabilities = field(default_factory=GazeboCapabilities)
    restore_sources: Dict[str, RestoreSource] = field(default_factory=dict)
    ops_log: List[Dict[str, Any]] = field(default_factory=list)
    # Injected backends — timeout_s is last arg where applicable
    query_fn: Optional[Callable[[str], Existence]] = None
    get_pose_fn: Optional[Callable[[str], List[float]]] = None
    set_pose_fn: Optional[Callable[..., None]] = None
    spawn_fn: Optional[Callable[..., None]] = None
    delete_fn: Optional[Callable[..., None]] = None
    cmd_runner: CommandRunner = run_cmd_argv
    verify_after: bool = True
    pose_tol_m: float = POSE_POS_TOL_M
    rpy_tol_rad: float = POSE_RPY_TOL_RAD
    # Live bundle hooks (optional)
    clock_state: Optional[ClockState] = None
    frames: Optional[FrameRing] = None

    def set_allowlist(self, names: Sequence[str]) -> None:
        self.allowlist = frozenset(names)

    def begin_capture_cycle(self) -> None:
        """Explicit new capture cycle: reset clock epoch + clear frame candidates."""
        if self.clock_state is not None:
            self.clock_state.reset_epoch()
        if self.frames is not None:
            self.frames.clear()

    def clear_frame_candidates(self) -> None:
        if self.frames is not None:
            self.frames.clear()

    def _check_allow(self, name: str) -> None:
        if self.allowlist is None:
            raise PermissionError("allowlist not set")
        if name not in self.allowlist:
            raise PermissionError(f"entity not in plan allowlist: {name}")
        if name != ROBOT_ENTITY and not is_tmp_occlusion_name(name) and not is_approved_visual_name(name):
            raise PermissionError(f"entity name not in structural allow policy: {name}")

    def register_world_sdf_source(
        self,
        *,
        name: str,
        xml: str,
        world_path: str,
        world_hash: str,
        pose: Sequence[float],
        clean_world_attested: bool,
    ) -> RestoreSource:
        """Register initial world_sdf source. verified only with explicit clean-world attestation.

        C2A-R1 implements the contract + offline tests; does NOT claim field attestation done.
        """
        if not clean_world_attested:
            raise PermissionError(
                "world_sdf source cannot be verified without clean_world_attested "
                "(requires later controlled pilot session proof)"
            )
        if not world_hash or not str(world_path).strip() or not xml.strip():
            raise ValueError("world_sdf registration requires world_path, world_hash, xml")
        src = RestoreSource(
            entity_name=name,
            source_kind="world_sdf",
            source_path=world_path,
            content_hash=sha256_text(xml),
            xml=xml,
            pose=list(pose),
            verified=True,
            world_hash=world_hash,
        )
        self.restore_sources[name] = src
        return src

    def query_entity(self, name: str) -> Existence:
        if self.query_fn is None:
            return Existence.QUERY_FAILED
        try:
            return self.query_fn(name)
        except Exception as exc:  # noqa: BLE001
            raise EntityQueryError(f"query_failed:{name}:{exc}") from exc

    def entity_exists(self, name: str) -> bool:
        st = self.query_entity(name)
        if st is Existence.QUERY_FAILED:
            raise EntityQueryError(f"entity_query_failed:{name}")
        return st is Existence.PRESENT

    def get_pose(self, name: str) -> List[float]:
        if self.get_pose_fn is None:
            raise RuntimeError("get_pose backend unavailable")
        return list(self.get_pose_fn(name))

    def _call_delete_backend(self, name: str, timeout_s: float) -> None:
        assert self.delete_fn is not None
        try:
            self.delete_fn(name, timeout_s)
        except TypeError:
            self.delete_fn(name)

    def _call_spawn_backend(self, name: str, xml: str, pose: Sequence[float], timeout_s: float) -> None:
        assert self.spawn_fn is not None
        try:
            self.spawn_fn(name, xml, pose, timeout_s)
        except TypeError:
            self.spawn_fn(name, xml, pose)

    def _call_set_pose_backend(self, name: str, pose: Sequence[float], timeout_s: float) -> None:
        assert self.set_pose_fn is not None
        try:
            self.set_pose_fn(name, pose, timeout_s)
        except TypeError:
            self.set_pose_fn(name, pose)

    def delete(self, name: str, timeout_s: float = 5.0) -> None:
        self._check_allow(name)
        if self.delete_fn is None:
            raise RuntimeError("delete backend unavailable")
        # Pop current controlled source after successful delete (replaced by later spawn)
        prior = self.restore_sources.get(name)
        self.ops_log.append({"op": "delete", "name": name, "timeout_s": timeout_s})
        self._call_delete_backend(name, timeout_s)
        if name in self.restore_sources:
            del self.restore_sources[name]
        if self.verify_after:
            st = self.query_entity(name)
            if st is Existence.QUERY_FAILED:
                raise UncertainOpError(f"delete:{name}:post_query_failed")
            if st is Existence.PRESENT:
                raise RuntimeError(f"delete:{name}:still_present_after_delete")
        _ = prior  # prior XML lives on restore stack via inject

    def _register_session_spawn(self, name: str, xml: str, pose: Sequence[float]) -> None:
        self.restore_sources[name] = RestoreSource(
            entity_name=name,
            source_kind="session_spawn",
            source_path=None,
            content_hash=sha256_text(xml),
            xml=xml,
            pose=list(pose),
            verified=True,
        )

    def spawn(self, name: str, xml: str, pose: Sequence[float], timeout_s: float = 5.0) -> None:
        self._check_allow(name)
        if self.spawn_fn is None:
            raise RuntimeError("spawn backend unavailable")
        st = self.query_entity(name)
        if st is Existence.QUERY_FAILED:
            raise EntityQueryError(f"spawn:{name}:pre_query_failed_refuse")
        if st is Existence.PRESENT:
            raise RuntimeError(f"spawn refuses overwrite existing entity: {name}")
        self.ops_log.append({"op": "spawn", "name": name, "pose": list(pose), "timeout_s": timeout_s})
        self._call_spawn_backend(name, xml, pose, timeout_s)
        self._register_session_spawn(name, xml, pose)
        if self.verify_after:
            st2 = self.query_entity(name)
            if st2 is Existence.QUERY_FAILED:
                raise UncertainOpError(f"spawn:{name}:post_query_failed")
            if st2 is not Existence.PRESENT:
                raise RuntimeError(f"spawn:{name}:not_present_after_spawn")
            if self.get_pose_fn is not None:
                got = self.get_pose(name)
                if not pose6_close(got, pose, self.pose_tol_m, self.rpy_tol_rad):
                    raise RuntimeError(f"spawn:{name}:pose_mismatch got={got} want={list(pose)}")

    def spawn_restoring(
        self,
        name: str,
        xml: str,
        pose: Sequence[float],
        controlled: Optional[Dict[str, Any]] = None,
        timeout_s: float = 5.0,
    ) -> None:
        """Restore spawn then reinstate controlled source record (not plan self-verify)."""
        self.spawn(name, xml, pose, timeout_s=timeout_s)
        if not controlled:
            return
        if not controlled.get("verified"):
            # Do not mark verified from untrusted plan metadata
            return
        kind = controlled.get("source_kind")
        if kind not in {"world_sdf", "session_spawn"}:
            return
        if controlled.get("content_hash") != sha256_text(xml):
            raise RuntimeError(f"restore:{name}:content_hash_mismatch_on_reinstate")
        actual_pose = self.get_pose(name) if self.get_pose_fn is not None else list(pose)
        self.restore_sources[name] = RestoreSource(
            entity_name=name,
            source_kind=str(kind),
            source_path=controlled.get("source_path"),
            content_hash=str(controlled.get("content_hash")),
            xml=xml,
            pose=list(actual_pose),
            verified=True,
            world_hash=controlled.get("world_hash"),
        )

    def set_pose(self, name: str, pose: Sequence[float], timeout_s: float = 5.0) -> None:
        self._check_allow(name)
        if self.set_pose_fn is None:
            raise RuntimeError("set_pose backend unavailable")
        st = self.query_entity(name)
        if st is Existence.QUERY_FAILED:
            raise EntityQueryError(f"set_pose:{name}:query_failed")
        if st is Existence.ABSENT:
            raise KeyError(f"set_pose: unknown entity {name}")
        self.ops_log.append({"op": "set_pose", "name": name, "pose": list(pose), "timeout_s": timeout_s})
        self._call_set_pose_backend(name, pose, timeout_s)
        if self.verify_after:
            got = self.get_pose(name)
            if not pose6_close(got, pose, self.pose_tol_m, self.rpy_tol_rad):
                raise RuntimeError(f"set_pose:{name}:readback_mismatch got={got} want={list(pose)}")

    def resolve_restore_source_for_delete(self, name: str, plan_previous: Optional[Dict[str, Any]] = None) -> RestoreSource:
        """Only session_spawn / attested world_sdf registry. Plan previous never verifies."""
        _ = plan_previous  # declaration only — ignored for trust
        src = self.restore_sources.get(name)
        if src is None or not src.verified:
            raise RuntimeError(
                f"delete:{name}:restore_source_unverifiable — refuse modify "
                "(no trusted session/world registry; will not trust plan previous)"
            )
        actual_pose = self.get_pose(name)
        return RestoreSource(
            entity_name=src.entity_name,
            source_kind=src.source_kind,
            source_path=src.source_path,
            content_hash=src.content_hash,
            xml=src.xml,
            pose=list(actual_pose),  # always actual query — never plan previous.pose
            verified=True,
            world_hash=src.world_hash,
        )

    def inject_actual_snapshots(self, ops: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Replace design previous_* with actual snapshots / trusted registry sources."""
        out: List[Dict[str, Any]] = []
        for op in ops:
            op2 = dict(op)
            kind = op2["op"]
            name = op2["name"]
            self._check_allow(name)
            if kind == "set_pose":
                op2["previous_pose"] = list(self.get_pose(name))
                op2["previous_pose_source"] = "actual_world_snapshot"
            elif kind == "delete":
                src = self.resolve_restore_source_for_delete(name, op2.get("previous"))
                op2["previous"] = {
                    "xml": src.xml,
                    "pose": list(src.pose),
                    "source_kind": src.source_kind,
                    "source_path": src.source_path,
                    "content_hash": src.content_hash,
                    "world_hash": src.world_hash,
                    "verified": True,
                }
            out.append(op2)
        return out


# --- Live bundle (ROS import only here) ---


class LiveRosBundle:
    """One rclpy context + one node: camera, /clock, Gazebo clients."""

    def __init__(
        self,
        *,
        image_topic: str = IMAGE_TOPIC,
        clock_topic: str = CLOCK_TOPIC,
        cmd_runner: CommandRunner = run_cmd_argv,
        node_name: str = "t02_c2a_live",
    ) -> None:
        import rclpy
        from cv_bridge import CvBridge
        from gazebo_msgs.srv import DeleteEntity, GetModelList, SpawnEntity
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image

        self._rclpy = rclpy
        self._owns_context = False
        if not rclpy.ok():
            rclpy.init()
            self._owns_context = True
        self.node = Node(node_name)
        self.bridge = CvBridge()
        self.clock_state = ClockState()
        self.frames = FrameRing(maxlen=8)
        self.cmd_runner = cmd_runner
        self.caps = GazeboCapabilities()
        self._closed = False

        def on_clock(msg: Clock) -> None:
            stamp = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
            self.clock_state.update(stamp, time.monotonic())

        def on_image(msg: Image) -> None:
            try:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception:  # noqa: BLE001
                return
            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            self.frames.seq += 1
            fr = ImageFrame(
                bgr=bgr,
                stamp_sec=stamp,
                frame_id=str(msg.header.frame_id or ""),
                seq=self.frames.seq,
                recv_mono=time.monotonic(),
            )
            try:
                validate_bgr_frame(fr.bgr, fr.frame_id)
            except ValueError:
                return
            # Do NOT attach case_id / token here — association only at select time
            self.frames.push(fr)

        self.node.create_subscription(Clock, clock_topic, on_clock, qos_profile_sensor_data)
        self.node.create_subscription(Image, image_topic, on_image, qos_profile_sensor_data)

        self._spawn = self.node.create_client(SpawnEntity, "/spawn_entity")
        self._delete = self.node.create_client(DeleteEntity, "/delete_entity")
        self._models = self.node.create_client(GetModelList, "/get_model_list")
        self._get_state = None
        self._set_state = None
        try:
            from gazebo_msgs.srv import GetEntityState, SetEntityState

            self._get_state = self.node.create_client(GetEntityState, "/get_entity_state")
            self._set_state = self.node.create_client(SetEntityState, "/set_entity_state")
        except Exception as exc:  # noqa: BLE001
            self.caps.notes.append(f"entity_state_srv_import_failed:{exc}")

        self.adapter = self._build_adapter()
        self.adapter.clock_state = self.clock_state
        self.adapter.frames = self.frames

    def _spin_once(self, timeout_sec: float) -> None:
        self._rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def _call_service(self, client, request, timeout_s: float) -> ServiceCallOutcome:
        """Honor a single total deadline for wait + future completion."""
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        remain = deadline - time.monotonic()
        if remain <= 0:
            return ServiceCallOutcome(done=False, timed_out=True, message="service_deadline_exceeded")
        if not client.wait_for_service(timeout_sec=remain):
            return ServiceCallOutcome(done=False, timed_out=True, message="service_unavailable")
        remain = deadline - time.monotonic()
        if remain <= 0:
            return ServiceCallOutcome(done=False, timed_out=True, message="service_deadline_exceeded")
        fut = client.call_async(request)
        self._rclpy.spin_until_future_complete(self.node, fut, timeout_sec=remain)
        if not fut.done():
            return ServiceCallOutcome(done=False, timed_out=True, message="service_timeout_uncertain")
        try:
            result = fut.result()
        except Exception as exc:  # noqa: BLE001
            return ServiceCallOutcome(done=True, exception=exc, message=str(exc))
        return interpret_future(done=True, result=result, exc=None, timed_out=False)

    def probe_capabilities(self, timeout_s: float = 2.0) -> GazeboCapabilities:
        caps = GazeboCapabilities()
        caps.spawn_entity = bool(self._spawn.wait_for_service(timeout_sec=timeout_s))
        caps.delete_entity = bool(self._delete.wait_for_service(timeout_sec=timeout_s))
        caps.get_model_list = bool(self._models.wait_for_service(timeout_sec=timeout_s))
        if self._get_state is not None:
            caps.get_entity_state = bool(self._get_state.wait_for_service(timeout_sec=timeout_s))
        if self._set_state is not None:
            caps.set_entity_state = bool(self._set_state.wait_for_service(timeout_sec=timeout_s))
        probe = self.cmd_runner(["gz", "model", "-h"], 2.0)
        caps.gz_model_pose_candidate = probe.returncode == 0 and not probe.timed_out
        caps.gz_model_pose_verified = False  # only after successful structured read
        if not caps.get_entity_state and caps.gz_model_pose_candidate:
            caps.notes.append(
                "GetEntityState unavailable — gz model is a capability *candidate* until read verified"
            )
        if not caps.set_entity_state and caps.gz_model_pose_candidate:
            caps.notes.append("SetEntityState unavailable — gz model set candidate pending read verify")
        if not caps.can_spawn_delete():
            caps.notes.append("spawn/delete services missing — cannot run scene ops")
        self.caps = caps
        self.adapter.caps = caps
        return caps

    def _gz_get_pose(self, name: str, timeout_s: float = 3.0) -> List[float]:
        r = self.cmd_runner(["gz", "model", "-m", name, "-p"], timeout_s)
        if r.timed_out:
            raise UncertainOpError(f"gz_model_get_pose:{name}:timeout")
        if r.returncode != 0:
            raise RuntimeError(f"gz_model_get_pose:{name}:rc={r.returncode}")
        pose = parse_gz_pose_stdout(r.stdout.strip())
        self.caps.gz_model_pose_verified = True
        self.adapter.caps.gz_model_pose_verified = True
        return pose

    def _gz_set_pose(self, name: str, pose: Sequence[float], timeout_s: float = 5.0) -> None:
        vals = [float(v) for v in pose[:6]]
        if any(not math.isfinite(v) for v in vals):
            raise RuntimeError(f"gz_model_set_pose:{name}:non_finite")
        x, y, z, roll, pitch, yaw = vals
        # Prefer full RPY flags; refuse silent drop of non-zero roll/pitch if flags unsupported.
        argv = [
            "gz",
            "model",
            "-m",
            name,
            "-x",
            str(x),
            "-y",
            str(y),
            "-z",
            str(z),
            "-R",
            str(roll),
            "-P",
            str(pitch),
            "-Y",
            str(yaw),
        ]
        r = self.cmd_runner(argv, timeout_s)
        if r.timed_out:
            raise UncertainOpError(f"gz_model_set_pose:{name}:timeout_uncertain")
        if r.returncode != 0:
            # Fallback path: only if roll/pitch ~ 0; otherwise refuse
            if abs(roll) > 1e-9 or abs(pitch) > 1e-9:
                raise RuntimeError(
                    f"gz_model_set_pose:{name}:nonzero_roll_pitch_refused_without_RPY_support rc={r.returncode}"
                )
            argv2 = ["gz", "model", "-m", name, "-x", str(x), "-y", str(y), "-z", str(z), "-Y", str(yaw)]
            r2 = self.cmd_runner(argv2, timeout_s)
            if r2.timed_out:
                raise UncertainOpError(f"gz_model_set_pose:{name}:timeout_uncertain")
            if r2.returncode != 0:
                raise RuntimeError(f"gz_model_set_pose:{name}:rc={r2.returncode}")

    def _build_adapter(self) -> GazeboSceneAdapter:
        from geometry_msgs.msg import Pose

        def query(name: str) -> Existence:
            if self.caps.get_model_list or self._models.wait_for_service(timeout_sec=0.1):
                from gazebo_msgs.srv import GetModelList

                out = self._call_service(self._models, GetModelList.Request(), 5.0)
                if out.timed_out or out.exception or out.result_obj is None:
                    return Existence.QUERY_FAILED
                if out.success is False:
                    return Existence.QUERY_FAILED
                names = set(getattr(out.result_obj, "model_names", []) or [])
                return Existence.PRESENT if name in names else Existence.ABSENT
            try:
                self.get_pose_world(name)
                return Existence.PRESENT
            except UncertainOpError:
                return Existence.QUERY_FAILED
            except Exception:
                return Existence.QUERY_FAILED

        def get_pose(name: str) -> List[float]:
            return self.get_pose_world(name)

        def set_pose(name: str, pose: Sequence[float], timeout_s: float = 5.0) -> None:
            if self.caps.set_entity_state and self._set_state is not None:
                from gazebo_msgs.msg import EntityState
                from gazebo_msgs.srv import SetEntityState

                req = SetEntityState.Request()
                st = EntityState()
                st.name = name
                st.pose = _pose_msg(Pose, pose)
                st.reference_frame = "world"
                req.state = st
                out = self._call_service(self._set_state, req, timeout_s)
                raise_from_outcome("set_entity_state", name, out)
                return
            if self.caps.gz_model_pose_verified or self.caps.gz_model_pose_candidate:
                if not self.caps.gz_model_pose_verified:
                    raise RuntimeError("gz_model_set_pose refused until pose read verified")
                self._gz_set_pose(name, pose, timeout_s)
                return
            raise RuntimeError("no set_pose capability")

        def spawn(name: str, xml: str, pose: Sequence[float], timeout_s: float = 5.0) -> None:
            from gazebo_msgs.srv import SpawnEntity

            req = SpawnEntity.Request()
            req.name = name
            req.xml = xml
            req.reference_frame = "world"
            req.initial_pose = _pose_msg(Pose, pose)
            out = self._call_service(self._spawn, req, timeout_s)
            raise_from_outcome("spawn", name, out)

        def delete(name: str, timeout_s: float = 5.0) -> None:
            from gazebo_msgs.srv import DeleteEntity

            req = DeleteEntity.Request()
            req.name = name
            out = self._call_service(self._delete, req, timeout_s)
            if out.timed_out or out.exception:
                raise_from_outcome("delete", name, out)

        return GazeboSceneAdapter(
            query_fn=query,
            get_pose_fn=get_pose,
            set_pose_fn=set_pose,
            spawn_fn=spawn,
            delete_fn=delete,
            cmd_runner=self.cmd_runner,
            caps=self.caps,
            clock_state=self.clock_state,
            frames=self.frames,
        )

    def get_pose_world(self, name: str, timeout_s: float = 5.0) -> List[float]:
        if self.caps.get_entity_state and self._get_state is not None:
            from gazebo_msgs.srv import GetEntityState

            req = GetEntityState.Request()
            req.name = name
            req.reference_frame = "world"
            out = self._call_service(self._get_state, req, timeout_s)
            raise_from_outcome("get_entity_state", name, out)
            pose = out.result_obj.state.pose
            roll, pitch, yaw = quat_to_rpy(
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            )
            return [pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw]
        if self.caps.gz_model_pose_candidate or self.caps.gz_model_pose_verified:
            return self._gz_get_pose(name, timeout_s)
        raise RuntimeError("no get_pose capability")

    def ros_stamp_now(self) -> float:
        self._spin_once(0.05)
        return self.clock_state.validate_fresh(now_mono=time.monotonic())

    def grab(self, deadline_mono: float) -> core.FrameCandidate:
        """Real grab: deadline-bound, clock/epoch checked, no duplicate seq, no forged token."""
        while True:
            now = time.monotonic()
            if now >= deadline_mono:
                raise TimeoutError("grab_deadline_exceeded")
            try:
                self.clock_state.validate_fresh(now_mono=now)
            except ClockInvalidError:
                self.frames.clear()
                raise
            remain = deadline_mono - time.monotonic()
            if remain <= 0:
                raise TimeoutError("grab_deadline_exceeded")
            self._spin_once(min(0.05, remain))
            try:
                self.clock_state.validate_fresh(now_mono=time.monotonic())
            except ClockInvalidError:
                self.frames.clear()
                raise
            fr = self.frames.newest_unreturned()
            if fr is None:
                continue
            validate_bgr_frame(fr.bgr, fr.frame_id)
            self.frames.mark_returned(fr)
            return core.FrameCandidate(
                recv_mono=fr.recv_mono,
                image_stamp_sec=fr.stamp_sec,
                image_frame_id=fr.frame_id,
                case_id=None,
                scene_op_done_token=None,
                bgr=fr.bgr.copy(),
                seq=fr.seq,
            )

    def clear_frame_candidates(self) -> None:
        self.frames.clear()

    def begin_capture_cycle(self) -> None:
        self.clock_state.reset_epoch()
        self.frames.clear()

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.node.destroy_node()
        finally:
            if self._owns_context and self._rclpy.ok():
                self._rclpy.shutdown()

    def __enter__(self) -> "LiveRosBundle":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()


def _is_float_token(s: str) -> bool:
    try:
        float(s)
        return True
    except ValueError:
        return False


def _pose_msg(Pose, pose: Sequence[float]):
    x, y, z, roll, pitch, yaw = [float(v) for v in pose[:6]]
    msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = x, y, z
    qx, qy, qz, qw = rpy_to_quat(roll, pitch, yaw)
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = qx, qy, qz, qw
    return msg


# --- pilot plan (C2A-R1) ---


def visual_entity_names(station_id: str) -> Dict[str, Any]:
    prefix = f"cabinet_{station_id.lower()}"
    return {
        "leds": [f"{prefix}_led_1", f"{prefix}_led_2", f"{prefix}_led_3"],
        "panel": f"{prefix}_panel",
    }


def led_panel_state_ops(station_id: str, states: Dict[str, Any]) -> Tuple[List[Dict[str, Any]], List[str]]:
    """Actual delete+spawn ops via approved generators. B leak untouched."""
    import apply_vision_scene_gt as avg  # noqa: WPS433

    names = visual_entity_names(station_id)
    poses = avg.STATION_POSE[station_id]
    ops: List[Dict[str, Any]] = []
    allow: List[str] = []
    for i, led_name in enumerate(names["leds"]):
        state = states["leds"][i]
        xml = approved_visual_xml(led_name, state)
        pose = list(poses["leds"][i])
        # previous is declaration only — live inject replaces from controlled registry
        ops.append(
            {
                "op": "delete",
                "name": led_name,
                "previous": {
                    "xml": "<untrusted_plan_declaration/>",
                    "pose": list(pose),
                    "note": "plan previous is NOT a trusted restore source",
                },
                "timeout_s": 5.0,
            }
        )
        ops.append({"op": "spawn", "name": led_name, "xml": xml, "pose": pose, "timeout_s": 5.0})
        allow.append(led_name)
    panel_name = names["panel"]
    panel_xml = approved_visual_xml(panel_name, states["panel"])
    panel_pose = list(poses["panel"])
    ops.append(
        {
            "op": "delete",
            "name": panel_name,
            "previous": {
                "xml": "<untrusted_plan_declaration/>",
                "pose": list(panel_pose),
                "note": "plan previous is NOT a trusted restore source",
            },
            "timeout_s": 5.0,
        }
    )
    ops.append({"op": "spawn", "name": panel_name, "xml": panel_xml, "pose": panel_pose, "timeout_s": 5.0})
    allow.append(panel_name)
    return ops, allow


def build_pilot_c2a_plan() -> Dict[str, Any]:
    """12 pilot_dev cases with real LED/panel delete+spawn ops + station FOV occlusions."""
    cases: List[Dict[str, Any]] = []
    design_home = [0.9, 3.0, 0.05, 0.0, 0.0, 0.0]
    resolved_states = {
        "A": {"leds": ["green", "green", "red"], "panel": "lit", "leak": "none"},
        "B": {"leds": ["green", "red", "green"], "panel": "dark", "leak": "fixed_fail"},
        "C": {"leds": ["green", "off", "green"], "panel": "lit", "leak": "none"},
    }

    def _base_ops(sid: str, states: Dict[str, Any]) -> Tuple[List[Dict[str, Any]], List[str]]:
        target = station_target_pose(sid)
        state_ops, visual_allow = led_panel_state_ops(sid, states)
        ops = [
            {
                "op": "set_pose",
                "name": ROBOT_ENTITY,
                "pose": target,
                "previous_pose": list(design_home),
                "previous_pose_source": "design_placeholder_replaced_at_live",
                "timeout_s": 5.0,
            },
            *state_ops,
        ]
        allow = [ROBOT_ENTITY, *visual_allow]
        return ops, allow

    for sid in ("A", "B", "C"):
        for n in (1, 2):
            intent = dict(resolved_states[sid])
            ops, allow = _base_ops(sid, intent)
            cases.append(
                {
                    "case_id": f"pilot_normal_{sid}_{n}",
                    "station_id": sid,
                    "distance_m": 1.2,
                    "robot_pose_world": dict(STATION_POSE_WORLD[sid]),
                    "pose_frame_id": "world",
                    "usage": USAGE_PILOT,
                    "execution_ready": False,
                    "target_states_intent": intent,
                    "resolved_states": dict(intent),
                    "review": {"status": "unreviewed", "states": None, "note": "physical intent ≠ review GT"},
                    "scene_recipe": {
                        "kind": "normal",
                        "seed": 500 + ord(sid) + n,
                        "state_apply_status": "ops_include_led_panel_delete_spawn",
                        "state_apply_method": "approved_visual_generator_delete_spawn",
                        "field_capability_verified": False,
                        "leak_policy": "B_fixed_fail_untouched",
                        "note": "ops apply LED/panel; restore needs controlled registry; not capture-ready",
                    },
                    "entity_allowlist": allow,
                    "scene_ops": ops,
                    "checklist_ids": [f"PILOT-N-{sid}"],
                    "hard_case": [],
                    "notes": "LED/panel delete+spawn in plan; B leak not modified",
                }
            )

    for sid in ("A", "B", "C"):
        for n in (1, 2):
            intent = dict(resolved_states[sid])
            intent["leds"] = ["green", "green", "green"]
            ops, allow = _base_ops(sid, intent)
            ent = f"{TMP_OCC_PREFIX}{sid}_{n}"
            geom = compute_occlusion_geometry(sid)
            sx, sy, sz = geom["occluder_size"]
            ops = list(ops) + [
                {
                    "op": "spawn",
                    "name": ent,
                    "xml": core._visible_box_sdf(ent, size=(sx, sy, sz)),
                    "pose": list(geom["occluder_pose"]),
                    "timeout_s": 5.0,
                }
            ]
            allow = list(allow) + [ent]
            cases.append(
                {
                    "case_id": f"pilot_hard_{sid}_{n}",
                    "station_id": sid,
                    "distance_m": 1.2,
                    "robot_pose_world": dict(STATION_POSE_WORLD[sid]),
                    "pose_frame_id": "world",
                    "usage": USAGE_PILOT,
                    "execution_ready": False,
                    "target_states_intent": intent,
                    "resolved_states": dict(intent),
                    "review": {
                        "status": "unreviewed",
                        "states": None,
                        "note": "occlusion effect 待真实试采确认 — do not auto UNKNOWN",
                    },
                    "scene_recipe": {
                        "kind": "partial_occlusion",
                        "seed": 600 + ord(sid) + n,
                        "temp_entity": ent,
                        "occlusion_pose_status": "camera_ray_partial_fov",
                        "occlusion_effect_status": "pending_live_pilot_confirm",
                        "state_apply_status": "ops_include_led_panel_delete_spawn",
                        "field_capability_verified": False,
                        "leak_policy": "B_fixed_fail_untouched",
                        "note": "FOV-partial occluder; effect 待真实试采确认",
                    },
                    "entity_allowlist": allow,
                    "scene_ops": ops,
                    "checklist_ids": [f"PILOT-H-{sid}"],
                    "hard_case": ["occlusion"],
                    "notes": "occlusion effect 待真实试采确认; review stays unreviewed",
                }
            )

    return {
        "session_id": "pilot_dev_c2a_r1_offline",
        "batch_id": "t02_pilot_dev_c2a_r1_12",
        "usage": USAGE_PILOT,
        "execution_ready": False,
        "capture_source_intended": "live_ros",
        "formal_test_eligible": False,
        "cases": cases,
        "notes": [
            "usage permanently pilot_dev — never promote to formal Test",
            "LED/panel ops present; restore requires controlled world/session registry",
            "field_capability_verified=false — not real-capture ready",
            "C2A-R1 does not claim clean-world field attestation",
        ],
    }


def static_check_pilot_plan(plan: Dict[str, Any]) -> List[str]:
    """LED/panel ops + FOV occlusion sanity (no ROS)."""
    errs: List[str] = []
    if plan.get("usage") != USAGE_PILOT:
        errs.append("usage_must_be_pilot_dev")
    if len(plan.get("cases") or []) > 12:
        errs.append("more_than_12_cases")
    normals = [c for c in plan["cases"] if c["case_id"].startswith("pilot_normal_")]
    hards = [c for c in plan["cases"] if c["case_id"].startswith("pilot_hard_")]
    if len(normals) != 6 or len(hards) != 6:
        errs.append(f"expected_6_normal_6_hard_got_{len(normals)}_{len(hards)}")

    for c in normals + hards:
        sid = c["station_id"]
        names = visual_entity_names(sid)
        resolved = c.get("resolved_states") or {}
        ops = c.get("scene_ops") or []
        allow = set(c.get("entity_allowlist") or [])
        if ROBOT_ENTITY not in allow:
            errs.append(f"{c['case_id']}:robot_missing_allowlist")
        # must include LED/panel delete+spawn, not only set_pose
        for led_i, led_name in enumerate(names["leds"]):
            if led_name not in allow:
                errs.append(f"{c['case_id']}:missing_allow_{led_name}")
            dels = [o for o in ops if o["op"] == "delete" and o["name"] == led_name]
            spawns = [o for o in ops if o["op"] == "spawn" and o["name"] == led_name]
            if not dels or not spawns:
                errs.append(f"{c['case_id']}:missing_led_ops:{led_name}")
            else:
                want = approved_visual_xml(led_name, resolved["leds"][led_i])
                if spawns[0]["xml"] != want:
                    errs.append(f"{c['case_id']}:led_xml_mismatch:{led_name}")
        panel = names["panel"]
        if panel not in allow:
            errs.append(f"{c['case_id']}:missing_allow_{panel}")
        pd = [o for o in ops if o["op"] == "delete" and o["name"] == panel]
        ps = [o for o in ops if o["op"] == "spawn" and o["name"] == panel]
        if not pd or not ps:
            errs.append(f"{c['case_id']}:missing_panel_ops")
        else:
            want_p = approved_visual_xml(panel, resolved["panel"])
            if ps[0]["xml"] != want_p:
                errs.append(f"{c['case_id']}:panel_xml_mismatch")
        if c.get("scene_recipe", {}).get("state_apply_status") == "design_placeholder":
            errs.append(f"{c['case_id']}:state_apply_still_design_placeholder")
        if c.get("scene_recipe", {}).get("state_apply_status") == "resolved_in_plan":
            # must not stop at status-only without ops
            if not any(o["op"] == "delete" and is_approved_visual_name(o["name"]) for o in ops):
                errs.append(f"{c['case_id']}:resolved_in_plan_without_visual_ops")
        if resolved.get("leak") == "fixed_fail" and sid != "B":
            errs.append(f"{c['case_id']}:unexpected_fixed_fail")
        if sid == "B" and resolved.get("leak") != "fixed_fail":
            errs.append(f"{c['case_id']}:B_leak_must_remain_fixed_fail")
        if "UNKNOWN" in json.dumps(c.get("target_states_intent")):
            errs.append(f"{c['case_id']}:UNKNOWN_in_intent")
        if c.get("review", {}).get("states") is not None:
            errs.append(f"{c['case_id']}:review_states_must_be_null")

    for c in hards:
        sid = c["station_id"]
        spawn = [o for o in c["scene_ops"] if o["op"] == "spawn" and is_tmp_occlusion_name(o["name"])][0]
        exp = occlusion_pose_for_station(sid)
        if not pose6_close(spawn["pose"], exp, pos_tol=0.05, rpy_tol=0.1):
            errs.append(f"{c['case_id']}:occlusion_pose_mismatch")
        rep = occlusion_fov_overlap_report(sid)
        if not rep.get("ok"):
            errs.append(f"{c['case_id']}:fov_overlap_fail:{rep}")
        if spawn["name"] not in c["entity_allowlist"]:
            errs.append(f"{c['case_id']}:occ_not_in_allowlist")
    return errs


# --- live entry (refuses formal; finally releases) ---


def refuse_if_formal(plan: Dict[str, Any]) -> None:
    if plan.get("usage") == USAGE_FORMAL or any(
        c.get("usage") == USAGE_FORMAL for c in (plan.get("cases") or [])
    ):
        raise PermissionError("live_adapter_refuses_formal_test_candidate — requires separate authorization")


def run_live_batch(
    plan: Dict[str, Any],
    out_dir: Path,
    *,
    source_registry: Optional[Dict[str, Any]] = None,
    settle_s: float = 0.05,
    wait_timeout_s: float = 2.0,
) -> Dict[str, Any]:
    """Explicit live entry — imports/inits ROS. Not used by offline tests."""
    refuse_if_formal(plan)
    if plan.get("usage") != USAGE_PILOT:
        raise PermissionError("live_adapter_only_pilot_dev")
    bundle: Optional[LiveRosBundle] = None
    try:
        bundle = LiveRosBundle()
        caps = bundle.probe_capabilities()
        if not caps.can_spawn_delete() or not caps.can_set_pose() or not caps.can_read_pose():
            raise RuntimeError(f"capability_preflight_failed:{caps.summary()}")

        def grab(deadline: float) -> core.FrameCandidate:
            return bundle.grab(deadline)

        def mono() -> float:
            return time.monotonic()

        def ros_stamp() -> float:
            # re-validate every call (per case selection path)
            return bundle.ros_stamp_now()

        def sleep_fn(s: float) -> None:
            time.sleep(s)

        adapter = bundle.adapter
        # begin_capture_cycle already bound on adapter (clock epoch + frames)

        summary = core.run_batch(
            plan,
            out_dir,
            adapter=adapter,
            buffer=core.BoundedFrameBuffer(maxlen=8),
            grab=grab,
            mono=mono,
            ros_stamp_now=ros_stamp,
            sleep_fn=sleep_fn,
            settle_s=settle_s,
            wait_timeout_s=wait_timeout_s,
            capture_source="live_ros",
            source_registry=source_registry,
        )
        summary["gazebo_capabilities"] = caps.summary()
        summary["formal_test_eligible"] = False
        summary["real_capture_pass"] = False
        summary["note"] = "live path executed — this task still does not claim real-capture PASS"
        return summary
    finally:
        if bundle is not None:
            bundle.close()


def draft_live_session_commands() -> Dict[str, List[str]]:
    """Draft only — do not execute in C2A."""
    world = "$HOME/inspection-robot/ros2_ws/src/simulation_worlds/worlds/test_room.world"
    return {
        "start": [
            f"ros2 launch simulation_worlds gazebo_robot_v0.launch.py world:={world}",
            "python3 scripts/t02_collect_raw.py --emit-pilot-c2a-plan bags/.../plans/pilot_c2a_12.json",
            "python3 scripts/t02_collect_raw.py --plan bags/.../plans/pilot_c2a_12.json "
            "--out bags/.../live_out --execute --adapter live --source-registry bags/.../source_registry.json",
        ],
        "stop": [
            "Ctrl-C collector",
            "Ctrl-C launch / quit Gazebo",
        ],
        "notes": [
            "Authorize separately before running; C2A did not start ROS/Gazebo",
            "pilot_dev only; formal refused",
        ],
    }
