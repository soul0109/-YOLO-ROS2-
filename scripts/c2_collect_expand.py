#!/usr/bin/env python3
"""C2 expand collection → bags/vision_5b_c2/ (no YOLO / no formal test freeze / no 90%).

Hard constraints:
  - default keep history; overwrite batch dirs only with --force
  - synthetic sidecar: parent_frame_id / transform_type / synthetic=true
  - aug suggested_split = train; never pad test with aug
  - box tiers: boxes_untrusted | boxes_candidate (≈1.2m fixed window) |
    boxes_review_passed (C2.1 human list only). Legacy boxes_trusted → candidate.
  - coverage: distance/normal/rare = raw only; exclude manifests/excluded_frames.json
  - UNKNOWN/hard split raw vs synthetic; never claim trainable/90%
"""

from __future__ import annotations

import argparse
import json
import random
import subprocess
import sys
import time
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from capture_vision_station_frames import STATION_POSE  # noqa: E402
from c1_collect_min import (  # noqa: E402
    DEFAULT_RULES,
    FORBIDDEN_CLASS_TOKENS,
    LABEL_VERSION,
    _apply_seed,
    _aug_from_source,
    _base_meta,
    _detections_from_rules,
    _load_rules,
    _prepare_batch_dir,
    _write_frame,
)

REPO = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = REPO / 'bags' / 'vision_5b_c2'
DET_WHITELIST = frozenset({'indicator_light', 'panel', 'inspection_region'})
TRUSTED_DISTANCE_M = 1.2
# Checklist IDs that may ONLY be credited from raw (non-synthetic) frames
RAW_ONLY_CHECKLIST = frozenset(
    {
        'N-A1',
        'N-B1',
        'N-C1',
        'N-MIX',
        'D-10',
        'D-15',
        'S-OFF',
        'S-RED',
        'S-YEL',
        'S-DARK',
        'S-LIT',
        'S-LEAK',
        'S-NOLK',
    }
)
# May be credited from raw and/or synthetic (reported separately)
SYNTH_OK_CHECKLIST = frozenset(
    {'U-PNL', 'U-LEAK', 'U-LED', 'U-OCC', 'H-OVR', 'H-OCC', 'H-BLUR', 'H-MISS'}
)

# Cabinet face y for distance parking
_FACE_Y = {'A': 0.55, 'B': 0.55, 'C': 5.45}


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def pose_at_distance(station: str, distance_m: float) -> dict[str, float]:
    base = STATION_POSE[station]
    if station in ('A', 'B'):
        y = _FACE_Y[station] + distance_m
    else:
        y = _FACE_Y[station] - distance_m
    return {
        'x': float(base['x']),
        'y': float(y),
        'z': float(base['z']),
        'yaw': float(base['yaw']),
    }


def _teleport_pose(pose: dict[str, float]) -> None:
    cmd = [
        'gz',
        'model',
        '-m',
        'robot_v0',
        '-x',
        str(pose['x']),
        '-y',
        str(pose['y']),
        '-z',
        str(pose['z']),
        '-Y',
        str(pose['yaw']),
    ]
    subprocess.run(cmd, check=True, capture_output=True, text=True)


def is_candidate_distance(distance_m: float) -> bool:
    """True iff distance is the 1.2 m calibration distance for roi_rules."""
    return abs(float(distance_m) - TRUSTED_DISTANCE_M) < 1e-6


# Legacy alias (read path / older call sites).
boxes_trusted_for_distance = is_candidate_distance


def load_excluded(root: Path) -> dict[str, Any]:
    path = root / 'manifests' / 'excluded_frames.json'
    if not path.is_file():
        return {'schema': 'vision_5b_excluded_frames_v1', 'frames': []}
    return json.loads(path.read_text(encoding='utf-8'))


def excluded_ids(root: Path) -> set[str]:
    data = load_excluded(root)
    return {str(f['image_id']) for f in data.get('frames') or [] if f.get('image_id')}


def load_review_passed(root: Path) -> set[str]:
    """C2.1 human-passed ids from txt or json under manifests/."""
    ids: set[str] = set()
    txt = root / 'manifests' / 'review_passed_image_ids.txt'
    js = root / 'manifests' / 'review_passed_image_ids.json'
    if txt.is_file():
        for line in txt.read_text(encoding='utf-8').splitlines():
            s = line.strip()
            if s and not s.startswith('#'):
                ids.add(s)
    if js.is_file():
        data = json.loads(js.read_text(encoding='utf-8'))
        if isinstance(data, list):
            ids.update(str(x) for x in data)
        else:
            ids.update(str(x) for x in data.get('image_ids') or [])
    return ids


def meta_boxes_untrusted(meta: dict[str, Any]) -> bool:
    if meta.get('boxes_untrusted') is True:
        return True
    if meta.get('boxes_untrusted') is False:
        return False
    # Infer from distance if unset
    return not is_candidate_distance(float(meta.get('distance_m', TRUSTED_DISTANCE_M)))


def meta_boxes_candidate(meta: dict[str, Any]) -> bool:
    """1.2 m fixed-window candidate (≠ final GT). Reads boxes_candidate or legacy boxes_trusted."""
    if meta_boxes_untrusted(meta):
        return False
    if 'boxes_candidate' in meta:
        return bool(meta['boxes_candidate'])
    if 'boxes_trusted' in meta:  # legacy one-version compat
        return bool(meta['boxes_trusted'])
    return is_candidate_distance(float(meta.get('distance_m', TRUSTED_DISTANCE_M)))


def meta_boxes_review_passed(meta: dict[str, Any], passed_ids: set[str] | None = None) -> bool:
    if meta.get('boxes_review_passed') is True:
        return True
    if passed_ids is not None and meta.get('image_id') in passed_ids:
        return True
    return False


def apply_box_tier_fields(
    meta: dict[str, Any],
    *,
    distance_m: float,
    review_passed_ids: set[str] | None = None,
) -> None:
    """Write canonical tier flags; keep legacy boxes_trusted == boxes_candidate for one version."""
    untrusted = not is_candidate_distance(distance_m)
    candidate = not untrusted
    iid = meta.get('image_id')
    passed = bool(
        review_passed_ids
        and iid in review_passed_ids
        and candidate
        and not untrusted
    )
    meta['boxes_untrusted'] = untrusted
    meta['boxes_candidate'] = candidate
    meta['boxes_review_passed'] = passed
    # Legacy compat write (one version)
    meta['boxes_trusted'] = candidate


def detections_for_distance(
    rules: dict[str, Any],
    distance_m: float,
    *,
    omit: set[str] | None = None,
) -> tuple[list[dict], bool]:
    """Return (detections, boxes_candidate).

    roi_rules.yaml is calibrated at 1.2 m only. Non-1.2 frames MUST set
    boxes_untrusted=True — never silently treat 1.2 windows as YOLO GT.
    """
    dets = _detections_from_rules(rules, omit=omit)
    candidate = is_candidate_distance(distance_m)
    return dets, candidate


class CameraSession:
    """Persistent /camera/image_raw subscriber (avoids rclpy init per frame)."""

    def __init__(self) -> None:
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.node import Node
        from sensor_msgs.msg import Image

        self._rclpy = rclpy
        rclpy.init()
        self.node = Node('c2_collect_cam')
        self.bridge = CvBridge()
        self._img: np.ndarray | None = None
        self._stamp: dict[str, Any] | None = None

        def cb(msg: Image) -> None:
            self._img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._stamp = {
                'sec': int(msg.header.stamp.sec),
                'nanosec': int(msg.header.stamp.nanosec),
            }

        self.node.create_subscription(Image, '/camera/image_raw', cb, 10)

    def grab(self, settle_s: float = 0.85, timeout_s: float = 8.0) -> dict[str, Any]:
        self._img = None
        t_settle = time.time() + settle_s
        while time.time() < t_settle:
            self._rclpy.spin_once(self.node, timeout_sec=0.05)
            self._img = None
        deadline = time.time() + timeout_s
        while time.time() < deadline and self._img is None:
            self._rclpy.spin_once(self.node, timeout_sec=0.05)
        if self._img is None:
            raise RuntimeError('timeout waiting /camera/image_raw')
        return {'img': self._img.copy(), 'stamp': dict(self._stamp or {})}

    def close(self) -> None:
        self.node.destroy_node()
        self._rclpy.shutdown()


def _checklist_raw(sid: str, distance_m: float, states: dict[str, Any]) -> list[str]:
    tags: list[str] = ['N-MIX']
    if abs(distance_m - 1.2) < 1e-6:
        tags.append({'A': 'N-A1', 'B': 'N-B1', 'C': 'N-C1'}[sid])
    elif abs(distance_m - 1.0) < 1e-6:
        tags.append('D-10')
    elif abs(distance_m - 1.5) < 1e-6:
        tags.append('D-15')
    leds = states.get('leds') or []
    if 'off' in leds:
        tags.append('S-OFF')
    if 'red' in leds:
        tags.append('S-RED')
    if 'yellow' in leds:
        tags.append('S-YEL')
    if states.get('panel') == 'dark':
        tags.append('S-DARK')
    if states.get('panel') == 'lit':
        tags.append('S-LIT')
    if states.get('leak') == 'fixed_fail':
        tags.append('S-LEAK')
    if states.get('leak') == 'none':
        tags.append('S-NOLK')
    return sorted(set(tags))


def collect_live_plan(
    root: Path,
    batch_id: str,
    plan: list[tuple[str, int, float]],
    rules: dict[str, Any],
    cam: CameraSession,
    *,
    force: bool,
    settle_s: float = 0.85,
    append: bool = False,
) -> dict[str, Any]:
    batch_dir = root / 'batches' / batch_id
    if append:
        batch_dir.mkdir(parents=True, exist_ok=True)
        (batch_dir / 'images').mkdir(parents=True, exist_ok=True)
        man_path = batch_dir / 'batch_manifest.json'
        frames = (
            list(json.loads(man_path.read_text(encoding='utf-8')).get('image_ids', []))
            if man_path.is_file()
            else []
        )
        seq = len(frames)
    else:
        _prepare_batch_dir(batch_dir, force=force)
        frames = []
        seq = 0
    last_seed: int | None = None
    gt: dict[str, Any] | None = None

    for sid, seed, dist in plan:
        if seed != last_seed:
            gt_path = batch_dir / f'gt_seed{seed}.json'
            last_err: Exception | None = None
            for attempt in range(1, 5):
                try:
                    gt = _apply_seed(seed, gt_path)
                    last_err = None
                    break
                except Exception as exc:  # noqa: BLE001
                    last_err = exc
                    print(
                        f'  apply_seed={seed} failed attempt {attempt}/4: {exc}; '
                        f'sleep {3 * attempt}s',
                        flush=True,
                    )
                    time.sleep(3 * attempt)
            if last_err is not None:
                raise last_err
            last_seed = seed
        assert gt is not None
        pose = pose_at_distance(sid, dist)
        _teleport_pose(pose)
        grabbed = cam.grab(settle_s=settle_s)
        st = gt['stations'][sid]
        states = {'leds': list(st['leds']), 'panel': st['panel'], 'leak': st['leak']}
        seq += 1
        image_id = f'{batch_id}_{sid}_d{dist:.1f}_s{seed}_{seq:04d}'.replace('.', 'p')
        dets, candidate = detections_for_distance(rules, dist)
        meta = _base_meta(
            station_id=sid,
            seed=seed,
            distance_m=float(dist),
            pose=pose,
            stamp=grabbed.get('stamp'),
            gt_scene=states,
            states=states,
            detections=list(dets),
            checklist_ids=_checklist_raw(sid, dist, states),
            hard_case=['none'],
            unknown_case_id=None,
            notes=f'c2 live seed={seed} dist={dist}',
            split=None,
        )
        meta['synthetic'] = False
        meta['parent_frame_id'] = None
        meta['transform_type'] = None
        meta['annotator'] = 'c2_collect_expand'
        meta['image_id'] = image_id
        apply_box_tier_fields(meta, distance_m=float(dist))
        if not candidate:
            meta['notes'] += (
                '; boxes_untrusted: roi_rules@1.2m reused for display only — '
                'NOT YOLO GT (not boxes_candidate)'
            )
        _write_frame(batch_dir, batch_id=batch_id, image_id=image_id, bgr=grabbed['img'], meta=meta)
        frames.append(image_id)
        print(f'  [{batch_id}] {image_id} candidate={candidate} {states}', flush=True)

    man = {
        'schema': 'vision_5b_batch_manifest_v1',
        'batch_id': batch_id,
        'split': None,
        'suggested_split': 'train',
        'n_frames': len(frames),
        'image_ids': frames,
        'label_version': LABEL_VERSION,
        'note': (
            'C2 unfrozen raw; not formal test. '
            'boxes_candidate only at distance_m≈1.2; else boxes_untrusted. '
            'boxes_review_passed only after C2.1 human list.'
        ),
    }
    (batch_dir / 'batch_manifest.json').write_text(
        json.dumps(man, indent=2, sort_keys=True) + '\n', encoding='utf-8'
    )
    return man


class ParentPicker:
    """Round-robin / seeded sampling across many parents per station."""

    def __init__(
        self,
        sources: list[tuple[Path, dict[str, Any]]],
        *,
        rng: random.Random,
    ) -> None:
        self._by_st: dict[str, list[tuple[Path, dict[str, Any]]]] = defaultdict(list)
        for png, meta in sources:
            self._by_st[meta['station_id']].append((png, meta))
        for sid in self._by_st:
            self._by_st[sid].sort(key=lambda x: x[1].get('image_id') or '')
            rng.shuffle(self._by_st[sid])
        self._idx: dict[str, int] = defaultdict(int)
        self._rng = rng

    def pick(
        self,
        station: str,
        pred,
        *,
        require_pred: bool = False,
        kind: str = '',
    ) -> tuple[Path, dict[str, Any]]:
        pool = list(self._by_st.get(station) or [])
        if not pool:
            raise RuntimeError(f'no source frames for station {station}')
        matched = [(p, m) for p, m in pool if pred(m)]
        if require_pred and not matched:
            raise RuntimeError(
                f'no parent for kind={kind} station={station} matching predicate '
                f'(refuse fallback to arbitrary frame)'
            )
        use = matched if matched else pool
        i = self._idx[station] % len(use)
        self._idx[station] += 1
        return use[i]


def _aug_extra(
    batch_dir: Path,
    batch_id: str,
    src_png: Path,
    src_meta: dict[str, Any],
    rules: dict[str, Any],
    *,
    image_id: str,
    kind: str,
) -> None:
    im = cv2.imread(str(src_png))
    if im is None:
        raise FileNotFoundError(src_png)
    # Aug parents must be boxes_candidate (1.2m) and not excluded
    if meta_boxes_untrusted(src_meta) or not meta_boxes_candidate(src_meta):
        raise RuntimeError(
            f'refuse aug from non-candidate parent {src_meta.get("image_id")} '
            f'dist={src_meta.get("distance_m")}'
        )
    rois = rules['rois']
    states = dict(src_meta['states'])
    gt_scene = dict(src_meta.get('gt_scene') or states)
    checklist = list(src_meta.get('checklist_ids') or [])
    hard = ['none']
    unknown_id = None
    parent_id = src_meta.get('image_id')
    notes = f'aug from {parent_id}'
    omit: set[str] = set()
    transform_type = kind

    if kind == 'U_led':
        y0, y1 = rois['led_row_y']
        for x0, x1 in rois['led_slots_x']:
            im[y0:y1, x0:x1] = (35, 45, 50)
        states['leds'] = ['UNKNOWN', 'UNKNOWN', 'UNKNOWN']
        unknown_id = 'U_led_ambiguous'
        checklist = list(set(checklist + ['U-LED']))
        notes += '; LED mid/weak → UNKNOWN (not green)'
    elif kind == 'U_occ':
        y0, y1 = rois['led_row_y']
        x0, x1 = rois['led_slots_x'][1]
        im[y0:y1, x0:x1] = 0
        px0, py0, px1, py1 = rois['panel']
        im[py0:py1, (px0 + px1) // 2 - 20 : (px0 + px1) // 2 + 20] = 0
        states['leds'] = list(states['leds'])
        states['leds'][1] = 'UNKNOWN'
        states['panel'] = 'UNKNOWN'
        unknown_id = 'U_occ_partial'
        hard = ['occlusion']
        checklist = list(set(checklist + ['U-OCC', 'U-PNL', 'U-LED', 'H-OCC']))
        notes += '; partial occlude → UNKNOWN'
    else:
        _aug_from_source(
            batch_dir, batch_id, src_png, src_meta, rules, image_id=image_id, kind=kind
        )
        jp = batch_dir / 'images' / f'{image_id}.json'
        meta = json.loads(jp.read_text(encoding='utf-8'))
        meta['annotator'] = 'c2_collect_expand'
        meta['suggested_split'] = 'train'
        meta['image_id'] = image_id
        apply_box_tier_fields(meta, distance_m=float(meta.get('distance_m', 1.2)))
        # Aug geometry inherits parent candidate window; not auto review_passed
        meta['boxes_review_passed'] = False
        if not meta.get('parent_frame_id'):
            raise RuntimeError(f'{image_id}: missing parent_frame_id after aug')
        if not meta.get('transform_type'):
            raise RuntimeError(f'{image_id}: missing transform_type after aug')
        if meta.get('synthetic') is not True:
            raise RuntimeError(f'{image_id}: synthetic must be true')
        jp.write_text(json.dumps(meta, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        return

    dets, _ = detections_for_distance(rules, float(src_meta.get('distance_m', 1.2)), omit=omit)
    meta = _base_meta(
        station_id=src_meta['station_id'],
        seed=src_meta.get('seed'),
        distance_m=float(src_meta.get('distance_m', 1.2)),
        pose=dict(src_meta['robot_pose']),
        stamp=src_meta.get('image_stamp'),
        gt_scene=gt_scene,
        states=states,
        detections=dets,
        checklist_ids=sorted(checklist),
        hard_case=hard,
        unknown_case_id=unknown_id,
        notes=notes,
        split=None,
    )
    meta['synthetic'] = True
    meta['parent_frame_id'] = parent_id
    meta['transform_type'] = transform_type
    meta['annotator'] = 'c2_collect_expand'
    meta['suggested_split'] = 'train'
    meta['image_id'] = image_id
    apply_box_tier_fields(meta, distance_m=float(src_meta.get('distance_m', 1.2)))
    meta['boxes_review_passed'] = False
    _write_frame(batch_dir, batch_id=batch_id, image_id=image_id, bgr=im, meta=meta)


def collect_aug_expand(
    root: Path,
    batch_id: str,
    rules: dict[str, Any],
    *,
    force: bool,
    seed: int = 42,
) -> dict[str, Any]:
    batch_dir = root / 'batches' / batch_id
    _prepare_batch_dir(batch_dir, force=force)
    frames: list[str] = []
    sources: list[tuple[Path, dict[str, Any]]] = []
    for bdir in sorted((root / 'batches').glob('c2_*')):
        if bdir.name == batch_id or 'aug' in bdir.name:
            continue
        for jp in sorted((bdir / 'images').glob('*.json')):
            meta = json.loads(jp.read_text(encoding='utf-8'))
            if meta.get('synthetic') or meta.get('unknown_case_id'):
                continue
            # Only boxes_candidate 1.2m parents; skip excluded
            if meta_boxes_untrusted(meta) or not meta_boxes_candidate(meta):
                continue
            png = bdir / meta['file_name']
            if png.is_file():
                sources.append((png, meta))

    excl = excluded_ids(root)
    sources = [(p, m) for p, m in sources if m.get('image_id') not in excl]
    rng = random.Random(seed)
    picker = ParentPicker(sources, rng=rng)

    plan: list[tuple[str, str, Any, bool]] = []
    # (kind, station, pred, require_pred)
    for sid in ('A', 'B', 'C'):
        for _ in range(4):
            plan.append(('U_panel', sid, lambda m: True, False))
        for _ in range(3):
            plan.append(('H_OVR', sid, lambda m: True, False))
            plan.append(('H_BLUR', sid, lambda m: True, False))
            plan.append(('H_OCC', sid, lambda m: True, False))
            plan.append(('H_MISS', sid, lambda m: True, False))
        for _ in range(3):
            plan.append(('U_led', sid, lambda m: True, False))
            plan.append(('U_occ', sid, lambda m: True, False))
    for _ in range(8):
        plan.append(
            (
                'U_leak',
                'B',
                lambda m: m.get('states', {}).get('leak') == 'fixed_fail',
                True,  # must not fall back
            )
        )

    parent_counts: Counter[str] = Counter()
    for i, (kind, sid, pred, require_pred) in enumerate(plan, start=1):
        png, meta = picker.pick(sid, pred, require_pred=require_pred, kind=kind)
        parent_counts[meta['image_id']] += 1
        image_id = f'{batch_id}_{kind}_{sid}_{i:04d}'
        _aug_extra(batch_dir, batch_id, png, meta, rules, image_id=image_id, kind=kind)
        frames.append(image_id)
        if i % 20 == 0:
            print(f'  [{batch_id}] {i}/{len(plan)} last={image_id}', flush=True)

    man = {
        'schema': 'vision_5b_batch_manifest_v1',
        'batch_id': batch_id,
        'split': None,
        'suggested_split': 'train',
        'n_frames': len(frames),
        'image_ids': frames,
        'label_version': LABEL_VERSION,
        'parent_frame_histogram': dict(parent_counts),
        'n_unique_parents': len(parent_counts),
        'note': (
            'C2 synthetic/aug v2: suggested_split=train; parents=trusted 1.2m only; '
            'U_leak requires leak=fixed_fail source (no fallback). '
            'Formal test = independent raw only.'
        ),
    }
    (batch_dir / 'batch_manifest.json').write_text(
        json.dumps(man, indent=2, sort_keys=True) + '\n', encoding='utf-8'
    )
    print(
        f'  [{batch_id}] parents unique={len(parent_counts)} '
        f'top={parent_counts.most_common(5)}',
        flush=True,
    )
    return man


def _iter_metas(root: Path) -> list[dict[str, Any]]:
    out = []
    for jp in sorted(root.glob('batches/*/images/*.json')):
        m = json.loads(jp.read_text(encoding='utf-8'))
        m['_json_path'] = str(jp)
        out.append(m)
    return out


def coverage_gaps(
    metas: list[dict[str, Any]],
    *,
    excluded: set[str] | None = None,
) -> dict[str, Any]:
    """Split raw vs synthetic; distance/normal/rare only credit raw.

    Frames in excluded_frames.json are kept on disk but not credited.
    """
    excl = excluded or set()
    raw_tags: Counter[str] = Counter()
    syn_tags: Counter[str] = Counter()
    for m in metas:
        if m.get('image_id') in excl:
            continue
        tags = m.get('checklist_ids') or []
        if m.get('synthetic'):
            for t in tags:
                syn_tags[t] += 1
        else:
            for t in tags:
                raw_tags[t] += 1

    minima = {
        'N-A1': 20,
        'N-B1': 30,
        'N-C1': 20,
        'N-MIX': 40,
        'D-10': 15,
        'D-15': 15,
        'S-OFF': 20,
        'S-RED': 20,
        'S-YEL': 20,
        'S-DARK': 30,
        'S-LIT': 30,
        'S-LEAK': 30,
        'S-NOLK': 30,
        'U-PNL': 10,
        'U-LEAK': 10,
        'U-LED': 10,
        'U-OCC': 5,
        'H-OVR': 10,
        'H-OCC': 10,
        'H-BLUR': 10,
        'H-MISS': 10,
    }
    gaps = []
    for kid, need in minima.items():
        rh = int(raw_tags.get(kid, 0))
        sh = int(syn_tags.get(kid, 0))
        if kid in RAW_ONLY_CHECKLIST:
            credited = rh
            credit_policy = 'raw_only'
            ok = rh >= need
            gap = max(0, need - rh)
        else:
            credited = rh + sh
            credit_policy = 'raw_plus_synthetic'
            ok = credited >= need
            gap = max(0, need - credited)
        gaps.append(
            {
                'id': kid,
                'raw_have': rh,
                'synthetic_have': sh,
                'credited': credited,
                'credit_policy': credit_policy,
                'need': need,
                'gap': gap,
                'ok': ok,
            }
        )

    raw_n = sum(1 for m in metas if not m.get('synthetic'))
    syn_n = sum(1 for m in metas if m.get('synthetic'))
    credited_n = sum(1 for m in metas if m.get('image_id') not in excl)
    candidate_n = sum(
        1
        for m in metas
        if not m.get('synthetic')
        and m.get('image_id') not in excl
        and meta_boxes_candidate(m)
    )
    untrusted_n = sum(
        1 for m in metas if not m.get('synthetic') and meta_boxes_untrusted(m)
    )
    review_passed_n = sum(
        1
        for m in metas
        if m.get('image_id') not in excl and meta_boxes_review_passed(m)
    )

    return {
        'schema': 'vision_5b_c2_coverage_gaps_v3',
        'excluded_n': len(excl),
        'excluded_ids': sorted(excl),
        'credit_policy_notes': {
            'distance_normal_rare': 'raw_have only; excluded_frames not credited',
            'unknown_hard': 'raw_have + synthetic_have; excluded not credited',
            'boxes': (
                'boxes_untrusted=不可训; boxes_candidate=1.2固定窗候选≠终检; '
                'boxes_review_passed=仅C2.1人工清单'
            ),
        },
        'checklist_gaps': gaps,
        'gaps_remaining': sum(1 for g in gaps if not g['ok']),
        'spec6_rollup': {
            'total_images_on_disk': len(metas),
            'total_images_credited': credited_n,
            'raw_images': raw_n,
            'synthetic_images': syn_n,
            'raw_boxes_candidate': candidate_n,
            'raw_boxes_untrusted': untrusted_n,
            'boxes_review_passed': review_passed_n,
            'spec6_total_suggest': 300,
            'note': '§6 建议对照，非覆盖达标/可训练/90% 宣称；总量含 excluded 原图证据',
        },
    }


def spot_check_c2(root: Path, rules: dict[str, Any]) -> dict[str, Any]:
    rows = []
    issues: list[str] = []
    meta_ok = True
    class_ok = True
    off_ok = True
    box_ok = True
    prov_ok = True
    split_ok = True
    img_ok = True
    parent_ok = True
    excl = excluded_ids(root)
    required = {
        'schema',
        'image_id',
        'file_name',
        'batch_id',
        'station_id',
        'distance_m',
        'robot_pose',
        'states',
        'detections',
        'checklist_ids',
        'hard_case',
        'label_version',
    }
    parents = {m['image_id']: m for m in _iter_metas(root)}
    overlay_dir = root / 'manifests' / 'spot_overlays'
    overlay_dir.mkdir(parents=True, exist_ok=True)

    all_jp = sorted(root.glob('batches/*/images/*.json'))
    raw_jp = [p for p in all_jp if 'aug' not in str(p)]
    aug_jp = [p for p in all_jp if 'aug' in str(p)]
    sample = raw_jp[:: max(1, len(raw_jp) // 8)][:8] + aug_jp[:: max(1, max(1, len(aug_jp)) // 8)][
        :8
    ]

    w = int(rules.get('image_size', [640, 480])[0])
    h = int(rules.get('image_size', [640, 480])[1])

    for jp in all_jp:
        meta = json.loads(jp.read_text(encoding='utf-8'))
        missing = sorted(required - set(meta))
        if missing:
            meta_ok = False
            issues.append(f"{meta.get('image_id')}: missing {missing}")

        png = jp.parent / (jp.stem + '.png')
        im = cv2.imread(str(png)) if png.is_file() else None
        if im is None:
            img_ok = False
            issues.append(f"{meta.get('image_id')}: image missing or undecodable")

        for d in meta.get('detections') or []:
            cn = str(d.get('class_name', ''))
            if cn not in DET_WHITELIST:
                class_ok = False
                issues.append(f"{meta.get('image_id')}: class not in whitelist: {cn}")
            if cn.lower() in FORBIDDEN_CLASS_TOKENS or 'green' in cn.lower() or 'fail' in cn.lower():
                class_ok = False
                issues.append(f"{meta.get('image_id')}: forbidden class token: {cn}")
            box = d.get('bbox_xyxy') or []
            if len(box) != 4:
                box_ok = False
                issues.append(f"{meta.get('image_id')}: bad box len")
            else:
                x1, y1, x2, y2 = map(int, box)
                if not (0 <= x1 < x2 <= w and 0 <= y1 < y2 <= h):
                    box_ok = False
                    issues.append(f"{meta.get('image_id')}: illegal xyxy {box}")

        # Non-1.2 must be flagged untrusted (legacy silent reuse = FAIL)
        dist = float(meta.get('distance_m', 1.2))
        if not meta.get('synthetic') and not is_candidate_distance(dist):
            if meta.get('boxes_untrusted') is not True:
                box_ok = False
                issues.append(
                    f"{meta.get('image_id')}: dist={dist} missing boxes_untrusted=true "
                    '(silent 1.2-window reuse forbidden)'
                )

        if meta.get('synthetic'):
            pid = meta.get('parent_frame_id')
            tt = meta.get('transform_type')
            if not pid or not tt or meta.get('synthetic') is not True:
                prov_ok = False
                issues.append(f"{meta.get('image_id')}: synthetic missing provenance")
            if pid not in parents:
                parent_ok = False
                issues.append(f"{meta.get('image_id')}: parent_frame_id missing in dataset: {pid}")
            else:
                parent = parents[pid]
                if parent.get('synthetic'):
                    parent_ok = False
                    issues.append(f"{meta.get('image_id')}: parent is synthetic")
                if abs(float(parent.get('distance_m', 1.2)) - float(meta.get('distance_m', 1.2))) > 1e-6:
                    parent_ok = False
                    issues.append(f"{meta.get('image_id')}: distance≠parent")
                if parent.get('station_id') != meta.get('station_id'):
                    parent_ok = False
                    issues.append(f"{meta.get('image_id')}: station≠parent")
            if meta.get('suggested_split') == 'test':
                split_ok = False
                issues.append(f"{meta.get('image_id')}: aug must not be test")
            bmp = root / 'batches' / meta['batch_id'] / 'batch_manifest.json'
            if bmp.is_file():
                if json.loads(bmp.read_text(encoding='utf-8')).get('suggested_split') == 'test':
                    split_ok = False
                    issues.append(f"{meta.get('image_id')}: aug batch suggested_split=test")

        row = {
            'image_id': meta.get('image_id'),
            'batch_id': meta.get('batch_id'),
            'station_id': meta.get('station_id'),
            'distance_m': meta.get('distance_m'),
            'seed': meta.get('seed'),
            'states': meta.get('states'),
            'synthetic': meta.get('synthetic'),
            'parent_frame_id': meta.get('parent_frame_id'),
            'transform_type': meta.get('transform_type'),
            'boxes_untrusted': meta_boxes_untrusted(meta),
            'boxes_candidate': meta_boxes_candidate(meta),
            'boxes_review_passed': meta.get('boxes_review_passed'),
            'unknown_case_id': meta.get('unknown_case_id'),
            'hard_case': meta.get('hard_case'),
            'n_det': len(meta.get('detections') or []),
            'missing_fields': missing,
            'png_bytes': png.stat().st_size if png.is_file() else 0,
            'excluded': meta.get('image_id') in excl,
        }

        # C-off: check EVERY slot labeled off (not only slot0 / led_1)
        # Excluded frames still checked for evidence but don't fail the round gate alone
        if (
            meta.get('station_id') == 'C'
            and im is not None
            and meta.get('unknown_case_id') is None
            and 'green' not in (meta.get('notes') or '')
            and meta.get('image_id') not in excl
        ):
            leds = (meta.get('states') or {}).get('leds') or []
            # face_pos_y: image slot i ↔ model leds[i]
            slot_map = rules.get('led_image_to_model', {}).get('face_pos_y', [0, 1, 2])
            y0, y1 = rules['rois']['led_row_y']
            for slot_i, model_i in enumerate(slot_map):
                if model_i >= len(leds) or leds[model_i] != 'off':
                    continue
                x0, x1 = rules['rois']['led_slots_x'][slot_i]
                patch = im[y0:y1, x0:x1]
                mean_g = float(patch[:, :, 1].mean())
                key = f'c_slot{slot_i}_mean_g'
                row[key] = round(mean_g, 1)
                if mean_g > 120:
                    off_ok = False
                    row['off_vs_green'] = 'FAIL_bright'
                    issues.append(
                        f"{meta.get('image_id')}: off slot{slot_i}≈green mean_g={mean_g:.1f}"
                    )
                elif 'off_vs_green' not in row:
                    row['off_vs_green'] = 'OK'
        rows.append(row)

    for jp in sample:
        meta = json.loads(jp.read_text(encoding='utf-8'))
        im = cv2.imread(str(jp.parent / (jp.stem + '.png')))
        if im is None:
            continue
        for d in meta.get('detections') or []:
            x1, y1, x2, y2 = map(int, d['bbox_xyxy'])
            color = (0, 0, 255) if meta_boxes_untrusted(meta) else (0, 255, 255)
            cv2.rectangle(im, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                im,
                d['class_name'][:12],
                (x1, max(12, y1 - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                color,
                1,
                cv2.LINE_AA,
            )
        if meta_boxes_untrusted(meta):
            flag = 'UNTRUSTED'
        elif meta.get('boxes_review_passed'):
            flag = 'REVIEW_OK'
        elif meta_boxes_candidate(meta):
            flag = 'CANDIDATE'
        else:
            flag = '?'
        cv2.putText(
            im,
            f"{meta.get('station_id')} d={meta.get('distance_m')} {flag}",
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (50, 220, 50),
            1,
            cv2.LINE_AA,
        )
        cv2.imwrite(str(overlay_dir / f"{meta['image_id']}_spot.png"), im)

    summary = {
        'schema': 'vision_5b_c2_spot_check_v2',
        'n_frames': len(rows),
        'n_overlays': len(list(overlay_dir.glob('*.png'))),
        'meta_fields_ok': meta_ok,
        'image_decodable_ok': img_ok,
        'class_whitelist_ok': class_ok,
        'bbox_xyxy_ok': box_ok,
        'synthetic_provenance_ok': prov_ok,
        'parent_exists_ok': parent_ok,
        'aug_not_test_ok': split_ok,
        'c_off_not_bright_green': off_ok,
        'issues_head': issues[:40],
        'n_issues': len(issues),
        'ok': all(
            [
                meta_ok,
                img_ok,
                class_ok,
                box_ok,
                prov_ok,
                parent_ok,
                split_ok,
                off_ok,
            ]
        ),
        'rows': rows,
    }
    man_dir = root / 'manifests'
    man_dir.mkdir(parents=True, exist_ok=True)
    (man_dir / 'spot_check.json').write_text(
        json.dumps(summary, indent=2, sort_keys=True) + '\n', encoding='utf-8'
    )
    lines = [
        '# C2 抽查表',
        '',
        f"- n_frames: {summary['n_frames']}",
        f"- ok: {summary['ok']}",
        f"- image_decodable_ok: {img_ok}",
        f"- class_whitelist_ok: {class_ok}",
        f"- bbox_xyxy_ok / boxes_untrusted flag: {box_ok}",
        f"- synthetic_provenance_ok: {prov_ok}",
        f"- parent_exists_ok: {parent_ok}",
        f"- c_off_not_bright_green (all off slots): {off_ok}",
        '',
        '| image_id | batch | st | dist | untrusted | synth | parent | off |',
        '|---|---|---|---|---|---|---|---|',
    ]
    for r in rows[:100]:
        lines.append(
            f"| {r['image_id']} | {r['batch_id']} | {r['station_id']} | {r.get('distance_m')} | "
            f"{r.get('boxes_untrusted')} | {r.get('synthetic')} | {r.get('parent_frame_id')} | "
            f"{r.get('off_vs_green', '-')} |"
        )
    (man_dir / 'spot_check.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return summary


def write_coverage_md(root: Path, cov: dict[str, Any], spot: dict[str, Any]) -> None:
    lines = [
        '# C2 覆盖缺口表（全量 · 分口径）',
        '',
        f"- 磁盘总图: **{cov['spec6_rollup']['total_images_on_disk']}** "
        f"(credited={cov['spec6_rollup']['total_images_credited']}, "
        f"excluded={cov.get('excluded_n', 0)}; "
        f"raw={cov['spec6_rollup']['raw_images']}, "
        f"syn={cov['spec6_rollup']['synthetic_images']})",
        f"- boxes_candidate(有效): **{cov['spec6_rollup'].get('raw_boxes_candidate')}** · "
        f"review_passed: **{cov['spec6_rollup'].get('boxes_review_passed')}**",
        f"- 缺口行数: **{cov['gaps_remaining']}** / {len(cov['checklist_gaps'])}",
        f"- spot_check: **{'PASS' if spot.get('ok') else 'FAIL'}**",
        f"- **声明：不得宣称覆盖达标 / 可训练 / 90%**",
        '',
        '| ID | raw_have | synthetic_have | credited | policy | need | gap | ok |',
        '|---|---:|---:|---:|---|---:|---:|:---:|',
    ]
    for g in cov['checklist_gaps']:
        lines.append(
            f"| {g['id']} | {g['raw_have']} | {g['synthetic_have']} | {g['credited']} | "
            f"{g['credit_policy']} | {g['need']} | {g['gap']} | {'✓' if g['ok'] else '·'} |"
        )
    lines += [
        '',
        '## §6 rollup（建议对照，非宣称）',
        '```json',
        json.dumps(cov['spec6_rollup'], indent=2, ensure_ascii=False),
        '```',
        '',
    ]
    (root / 'manifests' / 'coverage_gaps.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    (root / 'manifests' / 'coverage_gaps.json').write_text(
        json.dumps(cov, indent=2, ensure_ascii=False) + '\n', encoding='utf-8'
    )


def write_overview(root: Path, batch_mans: list[dict[str, Any]], *, round_id: str) -> None:
    """Overview = this round only (listed batch_mans)."""
    total = sum(m['n_frames'] for m in batch_mans)
    overview = {
        'schema': 'vision_5b_c2_overview_v2',
        'root': str(root),
        'round_id': round_id,
        'scope': 'this_round_batches_only',
        'n_batches': len(batch_mans),
        'n_frames': total,
        'batches': batch_mans,
        'claims': {
            'yolo_trained': False,
            'formal_test_frozen': False,
            'dataset_ready_90pct': False,
            'coverage_达标': False,
            'trainable_as_yolo_gt': False,
        },
        'splits': {
            'note': (
                'C2: batches unfrozen. Aug → suggested_split=train. '
                'Formal test = independent raw only; never pad with aug. '
                'boxes_untrusted=不可训; boxes_candidate≠终检; '
                'boxes_review_passed only after C2.1 list.'
            ),
            'suggested': {m['batch_id']: m.get('suggested_split') for m in batch_mans},
        },
        'label_version': LABEL_VERSION,
        'created_at': _now_iso(),
    }
    man_dir = root / 'manifests'
    man_dir.mkdir(parents=True, exist_ok=True)
    (man_dir / 'c2_overview.json').write_text(
        json.dumps(overview, indent=2, sort_keys=True) + '\n', encoding='utf-8'
    )
    (root / 'splits' / 'README.txt').parent.mkdir(parents=True, exist_ok=True)
    (root / 'splits' / 'README.txt').write_text(
        'C2: no formal test freeze.\n'
        'Aug/synthetic → same split as parent (suggested train).\n'
        'Formal test: independent raw batches only; never pad with aug.\n'
        'boxes_untrusted | boxes_candidate(~1.2m) | boxes_review_passed(C2.1).\n'
        'Legacy boxes_trusted == boxes_candidate (compat read/write one version).\n',
        encoding='utf-8',
    )


def migrate_box_tier_sidecars(root: Path) -> dict[str, int]:
    """Rewrite tier fields on all sidecars; apply C2.1 review_passed list if present."""
    passed = load_review_passed(root)
    n = 0
    n_passed = 0
    for jp in sorted(root.glob('batches/*/images/*.json')):
        meta = json.loads(jp.read_text(encoding='utf-8'))
        dist = float(meta.get('distance_m', 1.2))
        apply_box_tier_fields(meta, distance_m=dist, review_passed_ids=passed)
        if meta.get('synthetic'):
            # Aug never auto review_passed unless explicitly listed
            if meta.get('image_id') not in passed:
                meta['boxes_review_passed'] = False
        if meta.get('boxes_untrusted') and 'boxes_untrusted:' not in (meta.get('notes') or ''):
            meta['notes'] = (meta.get('notes') or '') + (
                '; boxes_untrusted: roi_rules@1.2m display only — not boxes_candidate'
            )
        if meta.get('boxes_review_passed'):
            n_passed += 1
        jp.write_text(json.dumps(meta, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        n += 1
    return {
        'migrated_sidecars': n,
        'boxes_review_passed_marked': n_passed,
        'review_list_size': len(passed),
    }


# Back-compat name used by older CLI docs
annotate_untrusted_existing = migrate_box_tier_sidecars


def _c_off_slot_means(im: np.ndarray, rules: dict[str, Any], leds: list[str]) -> list[dict[str, Any]]:
    """Per image-slot mean_g for model leds labeled off (C / face_pos_y)."""
    slot_map = rules.get('led_image_to_model', {}).get('face_pos_y', [0, 1, 2])
    y0, y1 = rules['rois']['led_row_y']
    out = []
    for slot_i, model_i in enumerate(slot_map):
        if model_i >= len(leds) or leds[model_i] != 'off':
            continue
        x0, x1 = rules['rois']['led_slots_x'][slot_i]
        mean_g = float(im[y0:y1, x0:x1][:, :, 1].mean())
        out.append({'slot': slot_i, 'model_i': model_i, 'mean_g': round(mean_g, 1), 'ok': mean_g <= 120})
    return out


def update_excluded_after_coff_replace(
    root: Path,
    *,
    old_id: str,
    new_id: str,
    verify: dict[str, Any],
) -> None:
    path = root / 'manifests' / 'excluded_frames.json'
    data = load_excluded(root)
    frames = list(data.get('frames') or [])
    for f in frames:
        if f.get('image_id') == old_id:
            f['recollect_needed'] = False
            f['superseded_by'] = new_id
            f['status'] = 'superseded_kept_on_disk'
            f['recollect_note'] = (
                f'replaced by {new_id} (C2.2); keep evidence; do not use as aug parent'
            )
        if f.get('image_id') == 'c2_20261011_b4_aug_v2_H_MISS_C_0060':
            f['recollect_needed'] = False
            f['status'] = 'derived_from_superseded_parent'
            f['recollect_note'] = (
                'parent superseded; do not use old parent for aug; frame stays excluded'
            )
    data['frames'] = frames
    data['updated_at'] = _now_iso()
    data['c22_coff_replace'] = {
        'old_image_id': old_id,
        'new_image_id': new_id,
        'verify': verify,
        'note': 'old ids remain excluded from credit; new id is credited raw',
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')


def c22_fill_gaps(
    root: Path,
    rules: dict[str, Any],
    *,
    settle_s: float = 0.85,
    run_id: str = '',
) -> dict[str, Any]:
    """C2.2: new batch_ids only — fill credited-raw gaps; no --force on prior evidence."""
    rid = run_id or datetime.now().strftime('%Y%m%d')
    cam = CameraSession()
    batch_mans: list[dict[str, Any]] = []
    try:
        # --- NA/NB at 1.2m ---
        # N-A1 +4, N-B1 +8 (fresh seeds, avoid relying on aug)
        plan_ab: list[tuple[str, int, float]] = []
        for seed in (101, 102, 103, 104):
            plan_ab.append(('A', seed, 1.2))
        for seed in (101, 102, 103, 104, 105, 106, 107, 108):
            plan_ab.append(('B', seed, 1.2))
        b_ab = f'c2_{rid}_c22_na_nb'
        if (root / 'batches' / b_ab).exists():
            b_ab = f'{b_ab}_{datetime.now().strftime("%H%M%S")}'
        print(f'C2.2 LIVE {b_ab} n={len(plan_ab)} ...', flush=True)
        batch_mans.append(
            collect_live_plan(
                root, b_ab, plan_ab, rules, cam, force=False, settle_s=settle_s
            )
        )

        # --- distance +6 each ---
        plan_d: list[tuple[str, int, float]] = []
        for dist in (1.0, 1.5):
            for seed in (101, 102):
                for sid in ('A', 'B', 'C'):
                    plan_d.append((sid, seed, dist))
        b_dist = f'c2_{rid}_c22_dist'
        if (root / 'batches' / b_dist).exists():
            b_dist = f'{b_dist}_{datetime.now().strftime("%H%M%S")}'
        print(f'C2.2 LIVE {b_dist} n={len(plan_d)} ...', flush=True)
        batch_mans.append(
            collect_live_plan(
                root, b_dist, plan_d, rules, cam, force=False, settle_s=settle_s
            )
        )

        # --- replace excluded C seed8 off ---
        b_coff = f'c2_{rid}_c22_coff_s8'
        if (root / 'batches' / b_coff).exists():
            b_coff = f'{b_coff}_{datetime.now().strftime("%H%M%S")}'
        print(f'C2.2 LIVE {b_coff} C seed=8 replace ...', flush=True)
        man_c = collect_live_plan(
            root,
            b_coff,
            [('C', 8, 1.2)],
            rules,
            cam,
            force=False,
            settle_s=max(1.0, settle_s),
        )
        batch_mans.append(man_c)
        new_id = man_c['image_ids'][0]
        jp = root / 'batches' / b_coff / 'images' / f'{new_id}.json'
        meta = json.loads(jp.read_text(encoding='utf-8'))
        im = cv2.imread(str(root / 'batches' / b_coff / meta['file_name']))
        if im is None:
            raise RuntimeError(f'C2.2 coff replace: undecodable {new_id}')
        verify_slots = _c_off_slot_means(im, rules, list((meta.get('states') or {}).get('leds') or []))
        verify = {
            'image_id': new_id,
            'leds': (meta.get('states') or {}).get('leds'),
            'slots': verify_slots,
            'ok': bool(verify_slots) and all(s['ok'] for s in verify_slots),
        }
        (root / 'batches' / b_coff / 'coff_verify.json').write_text(
            json.dumps(verify, indent=2, ensure_ascii=False) + '\n', encoding='utf-8'
        )
        if not verify['ok']:
            print(f'C2.2 WARNING: C s8 replace camera check FAILED: {verify}', flush=True)
        else:
            update_excluded_after_coff_replace(
                root,
                old_id='c2_20261011_b3_rare_C_d1p2_s8_0009',
                new_id=new_id,
                verify=verify,
            )
            print(f'C2.2 coff replace OK → {new_id}; excluded updated', flush=True)

        # tiers + coverage (no aug)
        migrate_box_tier_sidecars(root)
        all_mans: list[dict[str, Any]] = []
        for mp in sorted((root / 'batches').glob('*/batch_manifest.json')):
            all_mans.append(json.loads(mp.read_text(encoding='utf-8')))
        write_overview(root, all_mans, round_id=f'{rid}_c22')
        # also write this-round-only overview
        round_path = root / 'manifests' / 'c22_overview.json'
        round_path.write_text(
            json.dumps(
                {
                    'schema': 'vision_5b_c22_overview_v1',
                    'round': 'C2.2',
                    'new_batches': batch_mans,
                    'n_new_frames': sum(m['n_frames'] for m in batch_mans),
                    'coff_verify': verify,
                    'claims': {
                        'yolo_trained': False,
                        'formal_test_frozen': False,
                        'dataset_ready_90pct': False,
                        'trainable_pack': False,
                    },
                    'created_at': _now_iso(),
                },
                indent=2,
                ensure_ascii=False,
            )
            + '\n',
            encoding='utf-8',
        )
        metas = _iter_metas(root)
        excl = excluded_ids(root)
        cov = coverage_gaps(metas, excluded=excl)
        spot = spot_check_c2(root, rules)
        write_coverage_md(root, cov, spot)
        return {
            'new_batches': {m['batch_id']: m['n_frames'] for m in batch_mans},
            'coff_verify': verify,
            'coverage': {
                'on_disk': cov['spec6_rollup']['total_images_on_disk'],
                'credited': cov['spec6_rollup']['total_images_credited'],
                'gaps_remaining': cov['gaps_remaining'],
                'priority': [
                    g
                    for g in cov['checklist_gaps']
                    if g['id'] in ('N-A1', 'N-B1', 'D-10', 'D-15')
                ],
            },
            'spot_ok': spot['ok'],
            'review_passed': cov['spec6_rollup'].get('boxes_review_passed'),
        }
    finally:
        cam.close()


def c21_export_candidate_overlays(root: Path, rules: dict[str, Any]) -> dict[str, Any]:
    """Export overlays for every boxes_candidate raw frame (excl. excluded_frames)."""
    excl = excluded_ids(root)
    out_dir = root / 'manifests' / 'c21_candidate_overlays'
    out_dir.mkdir(parents=True, exist_ok=True)
    ids: list[str] = []
    skipped_excl: list[str] = []
    for jp in sorted(root.glob('batches/*/images/*.json')):
        if 'aug' in jp.parts:
            continue
        meta = json.loads(jp.read_text(encoding='utf-8'))
        if meta.get('synthetic'):
            continue
        iid = meta.get('image_id')
        if iid in excl:
            skipped_excl.append(iid)
            continue
        if not meta_boxes_candidate(meta):
            continue
        png = jp.parent / (jp.stem + '.png')
        im = cv2.imread(str(png))
        if im is None:
            continue
        for d in meta.get('detections') or []:
            x1, y1, x2, y2 = map(int, d['bbox_xyxy'])
            cv2.rectangle(im, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(
                im,
                d['class_name'][:14],
                (x1, max(12, y1 - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )
        cv2.putText(
            im,
            f"{meta.get('station_id')} d={meta.get('distance_m')} seed={meta.get('seed')}",
            (8, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (40, 220, 40),
            1,
            cv2.LINE_AA,
        )
        cv2.imwrite(str(out_dir / f'{iid}.png'), im)
        ids.append(iid)
    index = {
        'schema': 'vision_5b_c21_candidate_overlay_index_v1',
        'n_overlays': len(ids),
        'skipped_excluded': skipped_excl,
        'image_ids': ids,
        'overlay_dir': str(out_dir),
        'note': (
            'C2.1: human review these overlays; write review_passed_image_ids.txt; '
            'then --c21-apply-review. Unpassed stay boxes_candidate only.'
        ),
    }
    (root / 'manifests' / 'c21_candidate_index.json').write_text(
        json.dumps(index, indent=2, ensure_ascii=False) + '\n', encoding='utf-8'
    )
    return index


def repair_batch_manifest(root: Path, batch_id: str) -> dict[str, Any]:
    bdir = root / 'batches' / batch_id
    ids = sorted(p.stem for p in (bdir / 'images').glob('*.png'))
    man = {
        'schema': 'vision_5b_batch_manifest_v1',
        'batch_id': batch_id,
        'split': None,
        'suggested_split': 'train',
        'n_frames': len(ids),
        'image_ids': ids,
        'label_version': LABEL_VERSION,
        'note': 'repaired from images/*.png after interrupted collect; not formal test',
        'repaired_at': _now_iso(),
    }
    (bdir / 'batch_manifest.json').write_text(
        json.dumps(man, indent=2, sort_keys=True) + '\n', encoding='utf-8'
    )
    return man


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('-o', '--root', type=Path, default=DEFAULT_ROOT)
    p.add_argument('--rules', type=Path, default=DEFAULT_RULES)
    p.add_argument('--force', action='store_true', help='Allow replacing batch dirs')
    p.add_argument('--settle', type=float, default=0.85)
    p.add_argument('--spot-check-only', action='store_true')
    p.add_argument('--aug-only', action='store_true')
    p.add_argument(
        '--annotate-untrusted',
        action='store_true',
        help='Legacy alias for --migrate-box-tiers',
    )
    p.add_argument(
        '--migrate-box-tiers',
        action='store_true',
        help='Rewrite boxes_untrusted/candidate/review_passed (+ legacy boxes_trusted)',
    )
    p.add_argument(
        '--c21-export-overlays',
        action='store_true',
        help='Export overlays for all boxes_candidate raw frames (C2.1)',
    )
    p.add_argument(
        '--c21-apply-review',
        action='store_true',
        help='Apply manifests/review_passed_image_ids.* onto sidecars + refresh coverage',
    )
    p.add_argument(
        '--c22-fill-gaps',
        action='store_true',
        help='C2.2: new batch_ids fill N-A1/N-B1/D-10/D-15 + replace C s8; no --force wipe',
    )
    p.add_argument('--repair-manifest', default='', help='Rebuild batch_manifest from pngs')
    p.add_argument('--aug-batch-id', default='', help='New aug batch_id (default c2_<day>_b4_aug_v2)')
    p.add_argument('--aug-seed', type=int, default=42)
    p.add_argument('--append-batch', default='')
    p.add_argument('--run-id', default='')
    args = p.parse_args(argv)
    root: Path = args.root
    rules = _load_rules(args.rules)
    rid = args.run_id or datetime.now().strftime('%Y%m%d')

    if args.repair_manifest:
        man = repair_batch_manifest(root, args.repair_manifest)
        print(json.dumps({'repaired': man['batch_id'], 'n': man['n_frames']}, indent=2))
        return 0

    if args.c22_fill_gaps:
        if args.force:
            print('C2.2 refuses --force (must not wipe prior evidence)', file=sys.stderr)
            return 3
        r = c22_fill_gaps(root, rules, settle_s=args.settle, run_id=rid)
        print(json.dumps(r, indent=2, ensure_ascii=False))
        return 0 if r.get('coff_verify', {}).get('ok') and r.get('spot_ok') else 2

    if args.c21_export_overlays:
        r = c21_export_candidate_overlays(root, rules)
        print(json.dumps(r, indent=2, ensure_ascii=False))
        return 0

    if args.c21_apply_review or args.migrate_box_tiers or (
        args.annotate_untrusted and not args.aug_only and not args.spot_check_only
    ):
        r = migrate_box_tier_sidecars(root)
        metas = _iter_metas(root)
        excl = excluded_ids(root)
        cov = coverage_gaps(metas, excluded=excl)
        spot = spot_check_c2(root, rules)
        write_coverage_md(root, cov, spot)
        # refresh overview n_frames from disk
        batch_mans = []
        for mp in sorted((root / 'batches').glob('*/batch_manifest.json')):
            batch_mans.append(json.loads(mp.read_text(encoding='utf-8')))
        write_overview(root, batch_mans, round_id=rid)
        print(
            json.dumps(
                {
                    **r,
                    'disk_n': len(metas),
                    'overview_n': sum(m['n_frames'] for m in batch_mans),
                    'coverage_on_disk': cov['spec6_rollup']['total_images_on_disk'],
                    'coverage_credited': cov['spec6_rollup']['total_images_credited'],
                    'spot_ok': spot['ok'],
                    'gaps_remaining': cov['gaps_remaining'],
                },
                indent=2,
                ensure_ascii=False,
            )
        )
        if args.c21_apply_review or args.migrate_box_tiers or args.annotate_untrusted:
            if not args.aug_only and not args.spot_check_only:
                return 0 if spot['ok'] else 1

    if args.annotate_untrusted and (args.aug_only or args.spot_check_only):
        print(json.dumps(migrate_box_tier_sidecars(root), indent=2))

    if args.spot_check_only:
        metas = _iter_metas(root)
        excl = excluded_ids(root)
        cov = coverage_gaps(metas, excluded=excl)
        spot = spot_check_c2(root, rules)
        write_coverage_md(root, cov, spot)
        print(
            json.dumps(
                {
                    'ok': spot['ok'],
                    'n': spot['n_frames'],
                    'on_disk': cov['spec6_rollup']['total_images_on_disk'],
                    'credited': cov['spec6_rollup']['total_images_credited'],
                    'gaps_remaining': cov['gaps_remaining'],
                    'issues': spot['issues_head'][:12],
                },
                indent=2,
                ensure_ascii=False,
            )
        )
        return 0 if spot['ok'] else 1
    root.mkdir(parents=True, exist_ok=True)
    batch_mans: list[dict[str, Any]] = []
    cam: CameraSession | None = None

    try:
        if args.append_batch:
            cam = CameraSession()
            seeds_left = [3, 17, 21, 8, 11, 19, 27]
            plan = [('A', s, 1.2) for s in seeds_left]
            bdir = root / 'batches' / args.append_batch
            have_seeds = set()
            if bdir.is_dir():
                for jp in (bdir / 'images').glob('*.json'):
                    m = json.loads(jp.read_text(encoding='utf-8'))
                    if m.get('station_id') == 'A' and m.get('seed') is not None:
                        have_seeds.add(int(m['seed']))
            plan = [x for x in plan if x[1] not in have_seeds]
            print(f'APPEND {args.append_batch} n={len(plan)} ...', flush=True)
            if plan:
                collect_live_plan(
                    root,
                    args.append_batch,
                    plan,
                    rules,
                    cam,
                    force=False,
                    settle_s=args.settle,
                    append=True,
                )
            cam.close()
            cam = None
            args.aug_only = True

        if not args.aug_only:
            if not args.force:
                # refuse accidental wipe of existing C2 root batches
                existing = list((root / 'batches').glob('c2_*')) if (root / 'batches').exists() else []
                if existing:
                    print(
                        f'refusing full live collect: {len(existing)} batches exist under {root}; '
                        'use --aug-only / --append-batch / new --run-id, or --force to wipe',
                        file=sys.stderr,
                    )
                    return 3
            cam = CameraSession()
            seeds_b1 = [42, 100, 7, 13, 55, 3, 17, 21, 33, 99]
            plan_b1 = [(s, seed, 1.2) for seed in seeds_b1 for s in ('A', 'B', 'C')]
            b1 = f'c2_{rid}_b1_d1p2'
            print(f'LIVE {b1} n={len(plan_b1)} ...', flush=True)
            batch_mans.append(
                collect_live_plan(
                    root, b1, plan_b1, rules, cam, force=args.force, settle_s=args.settle
                )
            )
            plan_b2 = [
                (s, seed, d)
                for d in (1.0, 1.5)
                for seed in (42, 7, 55)
                for s in ('A', 'B', 'C')
            ]
            b2 = f'c2_{rid}_b2_dist'
            print(f'LIVE {b2} n={len(plan_b2)} ...', flush=True)
            batch_mans.append(
                collect_live_plan(
                    root, b2, plan_b2, rules, cam, force=args.force, settle_s=args.settle
                )
            )
            seeds_c = list(range(0, 24))
            seeds_ab = [42, 7, 13, 55, 99, 3, 17, 21, 8, 11, 19, 27]
            plan_b3: list[tuple[str, int, float]] = [('C', s, 1.2) for s in seeds_c]
            plan_b3 += [('B', s, 1.2) for s in seeds_ab]
            plan_b3 += [('A', s, 1.2) for s in seeds_ab]
            b3 = f'c2_{rid}_b3_rare'
            print(f'LIVE {b3} n={len(plan_b3)} ...', flush=True)
            batch_mans.append(
                collect_live_plan(
                    root,
                    b3,
                    plan_b3,
                    rules,
                    cam,
                    force=args.force,
                    settle_s=max(0.7, args.settle - 0.1),
                )
            )

        # Aug with NEW batch_id by default (never clobber raw; force only for that aug dir)
        b4 = args.aug_batch_id or f'c2_{rid}_b4_aug_v2'
        print(f'AUG {b4} ...', flush=True)
        # load existing raw manifests for overview round
        for name in sorted(
            p.name for p in (root / 'batches').glob('c2_*') if 'aug' not in p.name
        ):
            mp = root / 'batches' / name / 'batch_manifest.json'
            if mp.is_file():
                batch_mans.append(json.loads(mp.read_text(encoding='utf-8')))
        # force=True only for the new aug batch dir if it already exists from a prior v2 attempt
        aug_force = args.force or (root / 'batches' / b4).exists()
        if (root / 'batches' / b4).exists() and not args.force:
            # unique suffix instead of force-wipe
            b4 = f'{b4}_{datetime.now().strftime("%H%M%S")}'
            print(f'AUG dir exists; writing new id {b4} (no --force)', flush=True)
            aug_force = False
        batch_mans.append(
            collect_aug_expand(root, b4, rules, force=aug_force, seed=args.aug_seed)
        )

        migrate_box_tier_sidecars(root)
        write_overview(root, batch_mans, round_id=rid)
        metas = _iter_metas(root)
        excl = excluded_ids(root)
        cov = coverage_gaps(metas, excluded=excl)
        spot = spot_check_c2(root, rules)
        write_coverage_md(root, cov, spot)

        print(
            json.dumps(
                {
                    'ok': spot['ok'],
                    'root': str(root),
                    'n_frames': len(metas),
                    'batches': {m['batch_id']: m['n_frames'] for m in batch_mans},
                    'gaps_remaining': cov['gaps_remaining'],
                    'claims_forbidden': True,
                    'issues_head': spot['issues_head'][:10],
                },
                indent=2,
                ensure_ascii=False,
            )
        )
        return 0 if spot['ok'] else 2
    finally:
        if cam is not None:
            cam.close()


if __name__ == '__main__':
    raise SystemExit(main())
