#!/usr/bin/env python3
"""I1 demo: fixed_window and detector_box share roi_decision → Decision → E1 events.

No YOLO train, no Web, no 90% claim. Event layer never reads pixel ROIs.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from e1_emit_events import (  # noqa: E402
    DEFAULT_ROUTE,
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
    decide_one,
    decisions_fixed_window,
    fixture_leak_detector_box,
    load_bgr_and_meta,
    load_rules,
)

REPO = Path(__file__).resolve().parents[1]
DEFAULT_D1 = REPO / 'bags' / 'vision_d1_seed42'
DEFAULT_OUT = REPO / 'bags' / 'vision_i1_demo'

DECISION_KEYS = (
    'object_type',
    'verdict',
    'reason',
    'roi_source',
    'roi_rules_version',
    'measurements',
    'confidence',
)


def _as_dict(d: Decision) -> dict:
    return {
        'object_type': d.object_type,
        'verdict': d.verdict,
        'reason': d.reason,
        'roi_source': d.roi_source,
        'roi_rules_version': d.roi_rules_version,
        'measurements': d.measurements,
        'confidence': d.confidence,
    }


def run_self_check(args: argparse.Namespace) -> int:
    rules = load_rules(args.rules)
    meta_path = Path(args.d1_dir) / 'stationB_seed42.json'
    bgr, meta = load_bgr_and_meta(meta_path)

    fw = [
        d
        for d in decisions_fixed_window(bgr, meta, rules)
        if d.object_type == 'inspection_region'
    ][0]
    db = decide_one(bgr, fixture_leak_detector_box(rules), rules)

    assert set(_as_dict(fw)) == set(DECISION_KEYS)
    assert set(_as_dict(db)) == set(DECISION_KEYS)
    assert fw.roi_source == 'fixed_window'
    assert db.roi_source == 'detector_box'
    assert fw.object_type == db.object_type == 'inspection_region'
    assert fw.verdict == db.verdict == 'FAIL', (fw.verdict, db.verdict)
    assert fw.reason == db.reason == 'leak_fixed_fail'
    assert 'bbox_xyxy' in fw.measurements and 'bbox_xyxy' in db.measurements
    # confidence is detector score only — must not imply verdict
    assert db.confidence == 0.42
    assert fw.confidence is None
    assert db.roi_rules_version == rules.get('schema') == 'roi_rules_v1'

    # Far / empty crop must not become PASS (bbox actually cropped).
    far = decide_one(
        bgr,
        RoiRequest(
            roi_source='detector_box',
            object_type='inspection_region',
            bbox_xyxy=(0, 0, 2, 2),
            confidence=0.99,
            fixture_note='far/empty box probe',
        ),
        rules,
    )
    assert far.verdict != 'PASS', far
    assert far.confidence == 0.99  # score kept; verdict still from ROI on crop

    out = Path(args.output)
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)
    poses = load_patrol_poses(args.route)
    store = EventStore(out / 'events.jsonl', dedup_window_sec=60.0)

    src_img = meta_path.parent / meta['image']
    dest = copy_station_image(src_img, out / 'images', 'stationB_seed42.png')
    image_path = f'images/{dest.name}'
    pose = poses['B']

    # Emit via detector_box path only (proves E1 eats unified Decision).
    ev = decision_to_event(
        db,
        mission_id='mission_i1_detector_box',
        station_id='B',
        station_pose=pose,
        frame_id='map',
        image_path=image_path,
        image_stamp=meta.get('image_stamp'),
        source='i1_roi_entry_demo',
        amcl_error_m=None,
    )
    # Persist path markers explicitly on the event.
    ev['roi_source'] = db.roi_source
    ev['source'] = 'i1_roi_entry_demo'
    action, kept = store.try_add(ev)
    assert action == 'created', action
    store.flush()

    errs = validate_event(kept)
    assert not errs, errs
    assert kept['roi_source'] == 'detector_box'
    assert kept['source'] == 'i1_roi_entry_demo'
    assert kept['verdict'] == 'FAIL'
    assert (out / kept['image_path']).stat().st_size > 0

    # Event layer must not need rois: smoke that Decision→event never opened yaml rois.
    # (Structural: decision_to_event signature has no rules/rois.)

    sample = {
        'fixed_window_decision': _as_dict(fw),
        'detector_box_decision': _as_dict(db),
        'detector_box_event': kept,
        'note': (
            'I1: same Decision shape; event from detector_box path. '
            'Fixture ≠ real YOLO. Not 90%.'
        ),
    }
    (out / 'i1_sample.json').write_text(
        json.dumps(sample, indent=2, sort_keys=True) + '\n', encoding='utf-8'
    )

    print('I1_SELF_CHECK_OK')
    print(f'  fixed_window: {fw.object_type} {fw.verdict} {fw.reason}')
    print(f'  detector_box: {db.object_type} {db.verdict} {db.reason} '
          f'conf={db.confidence}')
    print(f'  event_id={kept["event_id"]} roi_source={kept["roi_source"]} '
          f'source={kept["source"]}')
    print(f'  wrote {out / "events.jsonl"} and {out / "i1_sample.json"}')
    print('  sample_event=', json.dumps(kept, ensure_ascii=False, sort_keys=True))
    return 0


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--rules', type=Path, default=DEFAULT_RULES)
    p.add_argument('--d1-dir', type=Path, default=DEFAULT_D1)
    p.add_argument('--route', type=Path, default=DEFAULT_ROUTE)
    p.add_argument('-o', '--output', type=Path, default=DEFAULT_OUT)
    p.add_argument('--self-check', action='store_true')
    args = p.parse_args(argv)
    if args.self_check:
        return run_self_check(args)
    # default: same as self-check
    args.self_check = True
    return run_self_check(args)


if __name__ == '__main__':
    sys.exit(main())
