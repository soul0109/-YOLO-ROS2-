#!/usr/bin/env python3
"""Hook-1: Train-1 YOLO best.pt → ROI Decision → E1 events (dev demo).

Real detector boxes only. Missing expected classes → UNKNOWN / miss record.
Never silently falls back to fixed_window while claiming detector success.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from e1_emit_events import (  # noqa: E402
    EventStore,
    copy_station_image,
    decision_to_event,
    load_patrol_poses,
    validate_event,
)
from roi_decision import (  # noqa: E402
    DEFAULT_RULES,
    Decision,
    RoiRequest,
    _rollup_leds,
    decide_one,
    led_slot_order,
    load_rules,
    rules_version,
)

REPO = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = REPO / 'bags' / 'vision_5b_c2'
DEFAULT_WEIGHTS = REPO / 'bags' / 'vision_5b_c2' / 'train1' / 'weights' / 'best.pt'
DEFAULT_OUT = REPO / 'bags' / 'vision_train1_e1_demo'
DEFAULT_ROUTE = (
    REPO
    / 'ros2_ws'
    / 'src'
    / 'inspection_mission'
    / 'config'
    / 'patrol_route.yaml'
)

CLASS_NAMES = ('indicator_light', 'panel', 'inspection_region')


def find_meta(root: Path, image_id: str) -> Path:
    matches = list((root / 'batches').rglob(f'{image_id}.json'))
    if not matches:
        raise FileNotFoundError(image_id)
    return matches[0]


def load_frame(meta_path: Path) -> tuple[np.ndarray, dict[str, Any], Path]:
    meta = json.loads(meta_path.read_text(encoding='utf-8'))
    png = meta_path.with_suffix('.png')
    if not png.is_file():
        # vision_5b uses file_name relative to batch dir
        fn = meta.get('file_name') or meta.get('image')
        if fn:
            cand = meta_path.parent.parent / fn if not Path(fn).is_absolute() else Path(fn)
            if not cand.is_file():
                cand = meta_path.parent / Path(fn).name
            png = cand
    bgr = cv2.imread(str(png))
    if bgr is None:
        raise FileNotFoundError(png)
    return bgr, meta, png


def run_yolo(weights: Path, bgr: np.ndarray, conf: float) -> list[dict[str, Any]]:
    from ultralytics import YOLO

    model = YOLO(str(weights))
    # BGR→RGB for ultralytics
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    res = model.predict(source=rgb, conf=conf, verbose=False)[0]
    out: list[dict[str, Any]] = []
    if res.boxes is None or len(res.boxes) == 0:
        return out
    names = res.names or {}
    for box in res.boxes:
        cls_id = int(box.cls.item())
        name = str(names.get(cls_id, cls_id))
        if name not in CLASS_NAMES:
            continue
        xyxy = [int(round(float(x))) for x in box.xyxy[0].tolist()]
        score = float(box.conf.item())
        out.append({'class_name': name, 'bbox_xyxy': xyxy, 'confidence': score})
    return out


def _iou(a: list[int], b: list[int]) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    area_a = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    area_b = max(0, bx2 - bx1) * max(0, by2 - by1)
    return inter / max(1.0, area_a + area_b - inter)


def associate_led_slots(
    lights: list[dict[str, Any]], rules: dict[str, Any]
) -> list[dict[str, Any] | None]:
    """Map up to 3 indicator_light boxes to image-left→right LED slots.

    Prefer calibrated 1.2m reference-slot IoU when it fills all 3 slots
    (no 1.2m regression). Otherwise use detector row-band + x-sort so
    1.0/1.5m frames are not hard-gated by fixed led_slots_x/led_row_y.
    """
    assigned: list[dict[str, Any] | None] = [None, None, None]
    if not lights:
        return assigned

    def _cxcy(d: dict[str, Any]) -> tuple[float, float]:
        x0, y0, x1, y1 = d['bbox_xyxy']
        return (float(x0 + x1) / 2.0, float(y0 + y1) / 2.0)

    assoc = rules.get('led_association') or {}
    band_half = float(assoc.get('row_band_half_px', 40))
    min_xsort = int(assoc.get('min_xsort_dets', 3))
    ref_iou_min = float(assoc.get('ref_slot_iou_min', 0.02))

    rois = rules.get('rois') or {}
    slots_x = rois.get('led_slots_x')
    led_row = rois.get('led_row_y')

    def _iou_assign() -> list[dict[str, Any] | None]:
        out: list[dict[str, Any] | None] = [None, None, None]
        if not slots_x or not led_row:
            return out
        y0, y1 = led_row
        slot_boxes = [[int(x0), int(y0), int(x1), int(y1)] for x0, x1 in slots_x]
        remaining = sorted(lights, key=lambda d: _cxcy(d)[0])
        used_slots: set[int] = set()
        for det in remaining:
            best_i, best_score = None, -1e9
            cx = _cxcy(det)[0]
            for i, sb in enumerate(slot_boxes):
                if i in used_slots:
                    continue
                iou = _iou(det['bbox_xyxy'], sb)
                scx = (sb[0] + sb[2]) / 2.0
                dist = abs(cx - scx)
                score = iou * 10.0 - dist / 100.0
                if score > best_score:
                    best_score, best_i = score, i
            if best_i is None:
                continue
            # Require weak geometric support so far-away 1.0m boxes don't force-map.
            sb = slot_boxes[best_i]
            iou = _iou(det['bbox_xyxy'], sb)
            scx = (sb[0] + sb[2]) / 2.0
            if iou < ref_iou_min and abs(cx - scx) > 80:
                continue
            used_slots.add(best_i)
            out[best_i] = det
        return out

    iou_assigned = _iou_assign()
    if all(s is not None for s in iou_assigned):
        return iou_assigned

    # Distance-robust path: row band around median cy + x-sort → slots 0..2.
    cys = [_cxcy(d)[1] for d in lights]
    median_cy = float(sorted(cys)[len(cys) // 2])
    in_band = [d for d in lights if abs(_cxcy(d)[1] - median_cy) <= band_half]
    if len(in_band) < min_xsort:
        in_band = [d for d in lights if abs(_cxcy(d)[1] - median_cy) <= band_half * 1.75]
    pool = in_band if in_band else list(lights)
    ordered = sorted(pool, key=lambda d: _cxcy(d)[0])
    if len(ordered) > 3:
        ordered = sorted(ordered, key=lambda d: -float(d.get('confidence') or 0.0))[:3]
        ordered = sorted(ordered, key=lambda d: _cxcy(d)[0])
    for i, det in enumerate(ordered[:3]):
        assigned[i] = det
    # If IoU filled some slots and x-sort filled others inconsistently, prefer
    # x-sort wholesale for non-1.2m (already in this branch).
    return assigned


def decisions_from_yolo(
    bgr: np.ndarray,
    meta: dict[str, Any],
    rules: dict[str, Any],
    dets: list[dict[str, Any]],
    *,
    model_version: str,
) -> tuple[list[Decision], dict[str, Any]]:
    """Build station Decisions from real YOLO boxes. Miss → UNKNOWN."""
    by_cls: dict[str, list[dict[str, Any]]] = {c: [] for c in CLASS_NAMES}
    for d in dets:
        by_cls[d['class_name']].append(d)
    for c in CLASS_NAMES:
        by_cls[c].sort(key=lambda x: -x['confidence'])

    miss: dict[str, Any] = {'model_version': model_version, 'missed': []}
    decisions: list[Decision] = []
    ver = rules_version(rules)

    # LEDs: need association to 3 slots; any miss → UNKNOWN rollup.
    slots = associate_led_slots(by_cls['indicator_light'], rules)
    order = led_slot_order(float(meta['robot_pose']['yaw']), rules)
    slot_states: list[str] = []
    slot_details: list[dict[str, Any]] = []
    led_confs: list[float] = []
    for i, det in enumerate(slots):
        if det is None:
            slot_states.append('UNKNOWN')
            slot_details.append({'reason': 'yolo_miss_led_slot', 'slot_i': i})
            miss['missed'].append(f'indicator_light_slot_{i}')
            continue
        d = decide_one(
            bgr,
            RoiRequest(
                roi_source='detector_box',
                object_type='indicator_light',
                bbox_xyxy=tuple(det['bbox_xyxy']),  # type: ignore[arg-type]
                confidence=det['confidence'],
            ),
            rules,
        )
        slot_states.append(str(d.measurements.get('led_state', 'UNKNOWN')))
        slot_details.append(
            {k: v for k, v in d.measurements.items() if k not in ('bbox_xyxy', 'roi_source')}
        )
        led_confs.append(float(det['confidence']))
    led_dec = _rollup_leds(slot_states, slot_details, order, rules, 'detector_box')
    if led_confs:
        led_dec.confidence = float(sum(led_confs) / len(led_confs))
    decisions.append(led_dec)

    # Panel / leak: top-1 box or UNKNOWN miss (no fixed_window fallback).
    for obj, key in (('panel', 'panel'), ('inspection_region', 'inspection_region')):
        cands = by_cls[key]
        if not cands:
            miss['missed'].append(key)
            decisions.append(
                Decision(
                    object_type=obj,
                    verdict='UNKNOWN',
                    reason=f'yolo_miss_{key}',
                    roi_source='detector_box',
                    roi_rules_version=ver,
                    measurements={'miss': True, 'model_version': model_version},
                    confidence=None,
                )
            )
            continue
        det = cands[0]
        decisions.append(
            decide_one(
                bgr,
                RoiRequest(
                    roi_source='detector_box',
                    object_type=obj,  # type: ignore[arg-type]
                    bbox_xyxy=tuple(det['bbox_xyxy']),  # type: ignore[arg-type]
                    confidence=det['confidence'],
                ),
                rules,
            )
        )

    return decisions, miss


def pick_demo_ids(root: Path) -> list[str]:
    """Prefer A/B/C covering PASS/FAIL/UNKNOWN from review_passed."""
    man = root / 'manifests'
    ids = [
        l.strip()
        for l in (man / 'review_passed_image_ids.txt').read_text().splitlines()
        if l.strip() and not l.startswith('#')
    ]
    chosen: list[str] = []
    # Heuristic picks known-good examples
    prefs = [
        'c2_20261011_s0_bal_A_d1p2_s201_0001',  # A
        'c2_20261011_s0_bal_B_d1p2_s201_0002',  # B
        'c2_20261011_s0_bal_C_d1p2_s201_0003',  # C
        'c2_20261011_s0_unk_A_U_led_0002',  # UNKNOWN leds
        'c2_20261011_s0_unk_B_U_leak_0007',  # UNKNOWN leak
        'c2_20261011_b1_d1p2_B_d1p2_s42_0002',  # B FAIL-ish
    ]
    idset = set(ids)
    for p in prefs:
        if p in idset:
            chosen.append(p)
    # ensure A/B/C
    have = set()
    for iid in chosen:
        m = json.loads(find_meta(root, iid).read_text(encoding='utf-8'))
        have.add(m['station_id'])
    for sid in ('A', 'B', 'C'):
        if sid in have:
            continue
        for iid in ids:
            m = json.loads(find_meta(root, iid).read_text(encoding='utf-8'))
            if m['station_id'] == sid:
                chosen.append(iid)
                break
    return chosen


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    ap.add_argument('--weights', type=Path, default=DEFAULT_WEIGHTS)
    ap.add_argument('--out', type=Path, default=DEFAULT_OUT)
    ap.add_argument('--rules', type=Path, default=DEFAULT_RULES)
    ap.add_argument('--route', type=Path, default=DEFAULT_ROUTE)
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--mission-id', default='mission_hook1_train1')
    ap.add_argument('--image-ids', nargs='*', default=None)
    ap.add_argument('--include-pass', action='store_true', default=True)
    args = ap.parse_args()

    if not args.weights.is_file():
        raise SystemExit(f'missing Train-1 weights: {args.weights}')

    # model_version = path + sha256 short
    import hashlib

    sha = hashlib.sha256(args.weights.read_bytes()).hexdigest()[:12]
    model_version = f'train1_dev:{args.weights.name}:{sha}'

    rules = load_rules(args.rules)
    poses = load_patrol_poses(args.route)
    out: Path = args.out
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    store = EventStore(out / 'events.jsonl', dedup_window_sec=2.0)

    image_ids = args.image_ids or pick_demo_ids(args.root)
    summary: list[dict[str, Any]] = []

    for image_id in image_ids:
        meta_path = find_meta(args.root, image_id)
        bgr, meta, png = load_frame(meta_path)
        station_id = str(meta['station_id'])
        dets = run_yolo(args.weights, bgr, args.conf)
        decisions, miss = decisions_from_yolo(
            bgr, meta, rules, dets, model_version=model_version
        )

        # annotate preview
        vis = bgr.copy()
        for d in dets:
            x1, y1, x2, y2 = d['bbox_xyxy']
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(
                vis,
                f"{d['class_name'][:4]}:{d['confidence']:.2f}",
                (x1, max(12, y1 - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )
        prev = out / 'previews' / f'{image_id}_yolo.png'
        prev.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(prev), vis)

        rel_name = f'{image_id}.png'
        dest = copy_station_image(png, out / 'images', rel_name)
        image_path = f'images/{dest.name}'
        pose = poses.get(station_id) or meta.get('robot_pose')
        created = []
        for d in decisions:
            if d.verdict == 'PASS' and not args.include_pass:
                continue
            ev = decision_to_event(
                d,
                mission_id=args.mission_id,
                station_id=station_id,
                station_pose=pose,
                frame_id='map',
                image_path=image_path,
                image_stamp=meta.get('image_stamp'),
                source='hook1_yolo_roi_e1',
                amcl_error_m=None,
            )
            ev['model_version'] = model_version
            ev['detector'] = {
                'weights': str(args.weights),
                'n_dets': len(dets),
                'miss': miss,
            }
            # forbid claiming fixed_window success
            if ev.get('roi_source') != 'detector_box':
                raise RuntimeError(f'expected detector_box, got {ev.get("roi_source")}')
            action, kept = store.try_add(ev)
            if action == 'created':
                errs = validate_event(kept)
                if errs:
                    raise RuntimeError(errs)
                created.append(kept)

        summary.append(
            {
                'image_id': image_id,
                'station_id': station_id,
                'n_dets': len(dets),
                'missed': miss['missed'],
                'verdicts': {d.object_type: d.verdict for d in decisions},
                'n_events_created': len(created),
                'preview': str(prev.relative_to(REPO)),
                'sample_event_id': created[0]['event_id'] if created else None,
            }
        )
        print(
            f'{image_id} st={station_id} dets={len(dets)} '
            f'verdicts={summary[-1]["verdicts"]} miss={miss["missed"]}'
        )

    store.flush()
    (out / 'hook1_summary.json').write_text(
        json.dumps(
            {
                'schema': 'hook1_yolo_roi_e1_v1',
                'model_version': model_version,
                'weights': str(args.weights),
                'note': 'dev_functional_demo_not_formal_5b_test',
                'frames': summary,
                'events_path': str((out / 'events.jsonl').relative_to(REPO)),
            },
            indent=2,
            ensure_ascii=False,
        )
        + '\n',
        encoding='utf-8',
    )
    print(f'events → {out / "events.jsonl"}')
    print(f'model_version={model_version}')
    # coverage check
    stations = {f['station_id'] for f in summary}
    verdicts = set()
    for f in summary:
        verdicts.update(f['verdicts'].values())
    print(f'stations={sorted(stations)} verdicts_seen={sorted(verdicts)}')
    if not {'A', 'B', 'C'} <= stations:
        print('WARN: missing A/B/C coverage', file=sys.stderr)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
