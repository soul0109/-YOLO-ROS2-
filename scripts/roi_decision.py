#!/usr/bin/env python3
"""I1 unified ROI entry: fixed_window | detector_box → Decision.

Pixel ROIs live here (and in roi_rules.yaml). Event layer must only consume
Decision — never read fixed pixel coordinates. Fake detector_box fixture ≠ YOLO.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal

import cv2
import numpy as np

from roi_judge_d1 import (  # noqa: WPS433 — same scripts/ dir
    DEFAULT_RULES,
    _clip_box,
    classify_leak,
    classify_led,
    classify_panel,
    led_slot_order,
    load_rules,
)

RoiSource = Literal['fixed_window', 'detector_box']
ObjectType = Literal['indicator_light', 'panel', 'inspection_region']

VERDICTS = frozenset({'PASS', 'FAIL', 'UNKNOWN'})


@dataclass
class Decision:
    """Structured judgment for the event layer (thin; not a ROIProvider hierarchy).

    roi_rules_version: copy of yaml `schema` (protocol), not a content hash.
    confidence: detector/fixture box score only; never drives PASS|FAIL|UNKNOWN.
    Verdict comes solely from ROI classify_* rules.
    """

    object_type: str
    verdict: str
    reason: str
    roi_source: str
    roi_rules_version: str
    measurements: dict[str, Any] = field(default_factory=dict)
    confidence: float | None = None  # detector_box score; null if ROI has no score

    def __post_init__(self) -> None:
        if self.verdict not in VERDICTS:
            raise ValueError(f'bad verdict: {self.verdict}')
        if self.roi_source not in ('fixed_window', 'detector_box'):
            raise ValueError(f'bad roi_source: {self.roi_source}')


@dataclass(frozen=True)
class RoiRequest:
    """Caller-supplied ROI. detector_box brings bbox; fixed_window names a rules role."""

    roi_source: RoiSource
    object_type: ObjectType
    bbox_xyxy: tuple[int, int, int, int] | None = None
    fixed_role: str | None = None  # panel | leak_floor | led_row
    led_slot_i: int | None = None
    # Detector-side score only (fixture may set e.g. 0.42). Not decision_confidence.
    confidence: float | None = None
    fixture_note: str | None = None


def rules_version(rules: dict[str, Any]) -> str:
    """Protocol version = yaml schema. Content tracked by git on roi_rules.yaml."""
    return str(rules.get('schema', 'roi_rules_v1'))


def resolve_bbox(req: RoiRequest, rules: dict[str, Any]) -> list[int]:
    """Only ROI entry resolves pixel boxes from rules or detector output."""
    if req.roi_source == 'detector_box':
        if not req.bbox_xyxy:
            raise ValueError('detector_box requires bbox_xyxy')
        return [int(x) for x in req.bbox_xyxy]

    rois = rules['rois']
    if req.fixed_role == 'panel':
        return list(rois['panel'])
    if req.fixed_role == 'leak_floor':
        return list(rois['leak_floor'])
    if req.fixed_role == 'led_slot':
        if req.led_slot_i is None:
            raise ValueError('led_slot requires led_slot_i')
        y0, y1 = rois['led_row_y']
        x0, x1 = rois['led_slots_x'][req.led_slot_i]
        return [int(x0), int(y0), int(x1), int(y1)]
    raise ValueError(f'unsupported fixed_role: {req.fixed_role}')


def _led_verdict(state: str, insp: dict[str, Any]) -> str:
    if state == 'UNKNOWN':
        return 'UNKNOWN'
    return 'PASS' if state == insp['led_pass_color'] else 'FAIL'


def _panel_verdict(state: str, insp: dict[str, Any]) -> str:
    if state == 'UNKNOWN':
        return 'UNKNOWN'
    return 'PASS' if state == insp['panel_pass_state'] else 'FAIL'


def _leak_verdict(state: str, insp: dict[str, Any]) -> str:
    if state == 'UNKNOWN':
        return 'UNKNOWN'
    return 'PASS' if state == insp['leak_pass_state'] else 'FAIL'


# Degenerate crop (illegal / empty-ish) must not become PASS.
_MIN_ROI_PIXELS = 64


def decide_one(bgr: np.ndarray, req: RoiRequest, rules: dict[str, Any]) -> Decision:
    """Unified judge: crop by resolved bbox → classify → Decision."""
    box = resolve_bbox(req, rules)
    patch = _clip_box(bgr, box)
    ver = rules_version(rules)
    insp = rules['inspection']
    meas: dict[str, Any] = {
        'bbox_xyxy': box,
        'roi_source': req.roi_source,
    }
    if req.fixture_note:
        meas['bbox_note'] = req.fixture_note

    ph, pw = (patch.shape[0], patch.shape[1]) if patch.size else (0, 0)
    if ph * pw < _MIN_ROI_PIXELS:
        return Decision(
            object_type=req.object_type,
            verdict='UNKNOWN',
            reason='roi_degenerate_or_empty',
            roi_source=req.roi_source,
            roi_rules_version=ver,
            measurements={**meas, 'patch_hw': [ph, pw]},
            confidence=req.confidence,
        )

    if req.object_type == 'panel':
        state, detail = classify_panel(patch, rules['panel'])
        verd = _panel_verdict(state, insp)
        if verd == 'PASS':
            reason = 'panel_lit'
        elif verd == 'FAIL':
            reason = 'panel_dark'
        else:
            reason = str(detail.get('reason', 'panel_unknown'))
        return Decision(
            object_type='panel',
            verdict=verd,
            reason=reason,
            roi_source=req.roi_source,
            roi_rules_version=ver,
            measurements={**meas, 'panel': state, **detail},
            confidence=req.confidence,
        )

    if req.object_type == 'inspection_region':
        state, detail = classify_leak(patch, rules['leak'])
        verd = _leak_verdict(state, insp)
        if verd == 'PASS':
            reason = 'leak_none'
        elif verd == 'FAIL':
            reason = 'leak_fixed_fail'
        else:
            reason = str(detail.get('reason', 'leak_unknown'))
        return Decision(
            object_type='inspection_region',
            verdict=verd,
            reason=reason,
            roi_source=req.roi_source,
            roi_rules_version=ver,
            measurements={**meas, 'leak': state, **detail},
            confidence=req.confidence,
        )

    if req.object_type == 'indicator_light':
        state, detail = classify_led(patch, rules['led'])
        verd = _led_verdict(state, insp)
        if verd == 'PASS':
            reason = 'led_green'
        elif state == 'UNKNOWN':
            reason = str(detail.get('reason', 'led_unknown'))
        else:
            reason = f'led_non_green:{state}'
        return Decision(
            object_type='indicator_light',
            verdict=verd,
            reason=reason,
            roi_source=req.roi_source,
            roi_rules_version=ver,
            measurements={**meas, 'led_state': state, **detail},
            confidence=req.confidence,
        )

    raise ValueError(f'unsupported object_type: {req.object_type}')


def _rollup_leds(
    slot_states: list[str],
    slot_details: list[dict[str, Any]],
    order: list[int],
    rules: dict[str, Any],
    roi_source: RoiSource,
) -> Decision:
    """Three slots → one indicator_light Decision (station rollup)."""
    led_cfg = rules['led']
    # Relative brightness belt (same as roi_judge_d1).
    rel_max = float(led_cfg.get('rel_to_row_max', 0.0) or 0.0)
    means = [float(d.get('mean_vmax', d.get('v90', 0.0))) for d in slot_details]
    row_max = max(means) if means else 0.0
    states = list(slot_states)
    if rel_max > 0 and row_max > 0:
        for i, mv in enumerate(means):
            ratio = mv / row_max
            slot_details[i]['rel_to_row_max'] = round(ratio, 3)
            if states[i] != 'off' and ratio <= rel_max:
                slot_details[i]['rel_off_override'] = True
                states[i] = 'off'

    model_leds: list[str | None] = [None, None, None]
    debug_slots = []
    for slot_i, (state, detail) in enumerate(zip(states, slot_details)):
        model_i = order[slot_i]
        model_leds[model_i] = state
        debug_slots.append(
            {
                'image_slot': slot_i,
                'model_index': model_i,
                'state': state,
                'detail': detail,
            }
        )
    leds = [model_leds[i] or 'UNKNOWN' for i in range(3)]
    insp = rules['inspection']
    verd_list = [_led_verdict(s, insp) for s in leds]
    if 'UNKNOWN' in verd_list:
        roll = 'UNKNOWN'
        reason = 'led_unknown_in_slots'
    elif 'FAIL' in verd_list:
        roll = 'FAIL'
        reason = f'led_non_green:{",".join(leds)}'
    else:
        roll = 'PASS'
        reason = 'all_leds_green'
    return Decision(
        object_type='indicator_light',
        verdict=roll,
        reason=reason,
        roi_source=roi_source,
        roi_rules_version=rules_version(rules),
        measurements={'leds': leds, 'led_slots': debug_slots},
        confidence=None,
    )


def decisions_fixed_window(
    bgr: np.ndarray,
    meta: dict[str, Any],
    rules: dict[str, Any],
    *,
    include_leak: bool = True,
) -> list[Decision]:
    """Station-level Decisions from yaml fixed windows (reads rois only here)."""
    out: list[Decision] = []
    order = led_slot_order(float(meta['robot_pose']['yaw']), rules)
    slot_states: list[str] = []
    slot_details: list[dict[str, Any]] = []
    for slot_i in range(3):
        req = RoiRequest(
            roi_source='fixed_window',
            object_type='indicator_light',
            fixed_role='led_slot',
            led_slot_i=slot_i,
        )
        d = decide_one(bgr, req, rules)
        slot_states.append(str(d.measurements.get('led_state', 'UNKNOWN')))
        slot_details.append(
            {k: v for k, v in d.measurements.items() if k not in ('bbox_xyxy', 'roi_source')}
        )
    out.append(_rollup_leds(slot_states, slot_details, order, rules, 'fixed_window'))

    out.append(
        decide_one(
            bgr,
            RoiRequest(
                roi_source='fixed_window',
                object_type='panel',
                fixed_role='panel',
            ),
            rules,
        )
    )
    if include_leak:
        out.append(
            decide_one(
                bgr,
                RoiRequest(
                    roi_source='fixed_window',
                    object_type='inspection_region',
                    fixed_role='leak_floor',
                ),
                rules,
            )
        )
    return out


def fixture_leak_detector_box(rules: dict[str, Any]) -> RoiRequest:
    """Fake detector output: bbox equals rules leak_floor (≠ trained YOLO).

    confidence=0.42 is a placeholder detector score only — verdict is from ROI rules.
    """
    box = tuple(int(x) for x in rules['rois']['leak_floor'])
    return RoiRequest(
        roi_source='detector_box',
        object_type='inspection_region',
        bbox_xyxy=box,  # type: ignore[arg-type]
        confidence=0.42,
        fixture_note=(
            'I1 fixture: synthetic detector_box ≠ real YOLO / trained detector; '
            'confidence is box score only, not verdict'
        ),
    )


def decision_detector_box(
    bgr: np.ndarray,
    req: RoiRequest,
    rules: dict[str, Any],
) -> Decision:
    if req.roi_source != 'detector_box':
        raise ValueError('decision_detector_box requires roi_source=detector_box')
    return decide_one(bgr, req, rules)


def decisions_for_station(
    bgr: np.ndarray,
    meta: dict[str, Any],
    rules: dict[str, Any],
    *,
    use_fixed_window: bool = True,
    use_detector_box: bool = False,
) -> list[Decision]:
    """Compose station Decisions. Leak from detector_box when that path is on."""
    decisions: list[Decision] = []
    if use_fixed_window:
        # When detector_box also on, skip fixed leak so E1 gets detector_box event.
        decisions.extend(
            decisions_fixed_window(
                bgr, meta, rules, include_leak=not use_detector_box
            )
        )
    if use_detector_box:
        decisions.append(
            decision_detector_box(bgr, fixture_leak_detector_box(rules), rules)
        )
    return decisions


def load_bgr_and_meta(meta_path: Path) -> tuple[np.ndarray, dict[str, Any]]:
    meta = __import__('json').loads(meta_path.read_text(encoding='utf-8'))
    img_path = meta_path.parent / meta['image']
    bgr = cv2.imread(str(img_path))
    if bgr is None:
        raise FileNotFoundError(img_path)
    return bgr, meta


__all__ = [
    'DEFAULT_RULES',
    'Decision',
    'RoiRequest',
    'decide_one',
    'decision_detector_box',
    'decisions_fixed_window',
    'decisions_for_station',
    'fixture_leak_detector_box',
    'load_bgr_and_meta',
    'load_rules',
    'resolve_bbox',
    'rules_version',
]
