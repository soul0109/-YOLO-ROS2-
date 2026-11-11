"""MVP-R3: same-frame YOLO + ROI station inspection (reuse Hook-1 contract).

Does not invent LED numbering from image left/right alone:
- image slots = left→right in the frame
- device/model leds[] via roi_rules led_image_to_model (face_neg_y for A)
"""

from __future__ import annotations

import hashlib
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

REPO = Path.home() / 'inspection-robot'
SCRIPTS = REPO / 'scripts'
DEFAULT_WEIGHTS = REPO / 'bags' / 'vision_5b_c2' / 'train1' / 'weights' / 'best.pt'
DEFAULT_RULES = (
    REPO / 'ros2_ws' / 'src' / 'vision_perception' / 'config' / 'roi_rules.yaml'
)


def _ensure_scripts_path() -> None:
    s = str(SCRIPTS)
    if s not in sys.path:
        sys.path.insert(0, s)


@dataclass
class InspectResult:
    ok: bool
    error_code: str = 'NONE'
    message: str = ''
    inspection_status: str = ''
    station_result_path: str = ''
    annotated_path: str = ''
    model_version: str = ''
    payload: dict = field(default_factory=dict)


def _station_rollup(verdicts: list[str]) -> str:
    """Station-level inspection rollup.

    Priority for known legal inputs: FAIL > UNKNOWN > PASS.
    Empty list → UNKNOWN. Illegal/blank values cannot yield PASS;
    if any FAIL is present they do not mask it.
    """
    if not verdicts:
        return 'UNKNOWN'
    legal = []
    has_illegal = False
    for raw in verdicts:
        v = str(raw or '').strip()
        if v in ('PASS', 'FAIL', 'UNKNOWN'):
            legal.append(v)
        else:
            has_illegal = True
    if any(v == 'FAIL' for v in legal):
        return 'FAIL'
    if has_illegal or not legal or any(v == 'UNKNOWN' for v in legal):
        return 'UNKNOWN'
    return 'PASS'


def _model_version(weights: Path) -> str:
    sha = hashlib.sha256(weights.read_bytes()).hexdigest()[:12]
    return f'train1_dev:{weights.name}:{sha}'


def inspect_station_image(
    *,
    screenshot_path: Path,
    mission_id: str,
    station_id: str,
    image_stamp: str,
    robot_yaw: float,
    weights: Path = DEFAULT_WEIGHTS,
    rules_path: Path = DEFAULT_RULES,
    conf: float = 0.25,
    write_annotated: bool = True,
) -> InspectResult:
    """Run Hook-1 YOLO→ROI on the saved screenshot bytes (same frame)."""
    _ensure_scripts_path()
    import cv2
    from hook1_yolo_roi_e1 import (  # noqa: WPS433
        decisions_from_yolo,
        run_yolo,
    )
    from roi_decision import load_rules, rules_version  # noqa: WPS433

    if not weights.is_file():
        return InspectResult(
            ok=False,
            error_code='vision_model_missing',
            message=f'missing YOLO weights: {weights}',
        )
    if not screenshot_path.is_file():
        return InspectResult(
            ok=False,
            error_code='vision_image_missing',
            message=f'missing screenshot: {screenshot_path}',
        )

    bgr = cv2.imread(str(screenshot_path))
    if bgr is None:
        return InspectResult(
            ok=False,
            error_code='vision_image_read_failed',
            message=f'cv2.imread failed: {screenshot_path}',
        )

    try:
        rules = load_rules(rules_path)
        model_version = _model_version(weights)
        dets = run_yolo(weights, bgr, conf)
        meta = {
            'station_id': station_id,
            'robot_pose': {'yaw': float(robot_yaw)},
            'image_stamp': image_stamp,
        }
        decisions, miss = decisions_from_yolo(
            bgr, meta, rules, dets, model_version=model_version,
        )
    except Exception as exc:
        return InspectResult(
            ok=False,
            error_code='vision_infer_failed',
            message=f'YOLO/ROI failed: {exc}',
        )

    by_type = {d.object_type: d for d in decisions}
    led = by_type.get('indicator_light')
    panel = by_type.get('panel')
    leak = by_type.get('inspection_region')

    mapping = rules.get('led_image_to_model', {})
    face = 'face_neg_y' if float(robot_yaw) < 0 else 'face_pos_y'
    led_payload: dict[str, Any] = {
        'verdict': led.verdict if led else 'UNKNOWN',
        'reason': led.reason if led else 'missing_led_decision',
        'model_states': (led.measurements.get('leds') if led else None),
        'image_slots': (led.measurements.get('led_slots') if led else None),
        'slot_contract': {
            'image_order': 'left_to_right',
            'device_index': 'model leds[0..2] = device led_1..led_3',
            'led_image_to_model_key': face,
            'led_image_to_model': list(mapping.get(face, [])),
            'note': (
                'image_slot is left→right in the frame; device/model index uses '
                'roi_rules led_image_to_model. Do not invent world LED ids from '
                'image left/right alone.'
            ),
        },
    }
    panel_payload = {
        'verdict': panel.verdict if panel else 'UNKNOWN',
        'reason': panel.reason if panel else 'missing_panel_decision',
        'state': (panel.measurements.get('panel') if panel else None),
        'detail': (
            {k: v for k, v in panel.measurements.items()
             if k not in ('bbox_xyxy', 'roi_source')}
            if panel else {}
        ),
    }
    leak_payload = {
        'verdict': leak.verdict if leak else 'UNKNOWN',
        'reason': leak.reason if leak else 'missing_leak_decision',
        'state': (leak.measurements.get('leak') if leak else None),
        'detail': (
            {k: v for k, v in leak.measurements.items()
             if k not in ('bbox_xyxy', 'roi_source')}
            if leak else {}
        ),
    }

    inspection_status = _station_rollup([
        led_payload['verdict'],
        panel_payload['verdict'],
        leak_payload['verdict'],
    ])

    annotated_path = ''
    if write_annotated:
        vis = bgr.copy()
        for d in dets:
            x1, y1, x2, y2 = d['bbox_xyxy']
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 2)
            cv2.putText(
                vis,
                f"{d['class_name'][:8]}:{d['confidence']:.2f}",
                (x1, max(12, y1 - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )
        ann = screenshot_path.with_name(screenshot_path.stem + '_annotated.png')
        if cv2.imwrite(str(ann), vis):
            annotated_path = str(ann)

    result_path = screenshot_path.with_name(
        screenshot_path.stem + '_station_result.json',
    )
    payload = {
        'schema': 'mvp_r3_station_result_v1',
        'mission_id': mission_id,
        'station_id': station_id,
        'image_stamp': image_stamp,
        'screenshot_path': str(screenshot_path),
        'annotated_path': annotated_path,
        'model_version': model_version,
        'roi_rules_version': rules_version(rules),
        'roi_rules_path': str(rules_path),
        'weights_path': str(weights),
        'inspection_status': inspection_status,
        'execution_note': (
            'execution_status COMPLETED means nav+capture+vision ran; '
            'inspection_status is PASS|FAIL|UNKNOWN for the station. '
            'Missing detections stay UNKNOWN (never default PASS). '
            'GT is not used for visual verdicts.'
        ),
        'objects': {
            'leds': led_payload,
            'panel': panel_payload,
            'leak': leak_payload,
        },
        'detections': dets,
        'missed': miss.get('missed', []),
        'robot_yaw_used_for_led_map': float(robot_yaw),
    }
    try:
        result_path.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False) + '\n',
            encoding='utf-8',
        )
    except Exception as exc:
        return InspectResult(
            ok=False,
            error_code='vision_result_save_failed',
            message=f'station result save failed: {exc}',
            model_version=model_version,
        )

    return InspectResult(
        ok=True,
        error_code='NONE',
        message=f'station inspection {inspection_status}',
        inspection_status=inspection_status,
        station_result_path=str(result_path),
        annotated_path=annotated_path,
        model_version=model_version,
        payload=payload,
    )
