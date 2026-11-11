#!/usr/bin/env python3
"""Hash-gated formal vision Test runner (B2).

One-shot evaluation on a frozen test root. Refuses to run if candidate /
ROI / freeze hashes mismatch, or if freeze marker is absent (unless --dry-run).

Dry-run on soft/dev data MUST set report_tag=DRY_RUN_NOT_TEST.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import subprocess
import sys
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np

SCRIPTS = Path(__file__).resolve().parent
REPO = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))

from hook1_yolo_roi_e1 import (  # noqa: E402
    CLASS_NAMES,
    associate_led_slots,
    decisions_from_yolo,
    run_yolo,
)
from roi_decision import load_rules, rules_version  # noqa: E402

DEFAULT_CANDIDATE = (
    REPO / 'models' / 'candidates' / 'inspection_train1_20261012.candidate.json'
)


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def env_info() -> dict[str, Any]:
    info: dict[str, Any] = {
        'hostname': platform.node(),
        'python': sys.version.split()[0],
        'platform': platform.platform(),
        'cwd': str(Path.cwd()),
        'repo': str(REPO),
    }
    try:
        info['git_head'] = subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=REPO, text=True
        ).strip()
    except Exception as exc:  # noqa: BLE001
        info['git_head_error'] = str(exc)
    try:
        import ultralytics

        info['ultralytics'] = ultralytics.__version__
    except Exception as exc:  # noqa: BLE001
        info['ultralytics_error'] = str(exc)
    try:
        import torch

        info['torch'] = torch.__version__
        info['cuda_available'] = bool(torch.cuda.is_available())
    except Exception as exc:  # noqa: BLE001
        info['torch_error'] = str(exc)
    return info


def gate_candidate(cand: dict[str, Any], *, dry_run: bool) -> dict[str, Any]:
    wpath = REPO / cand['weights']['path']
    rpath = REPO / cand['roi_rules']['path']
    issues: list[str] = []
    if not wpath.is_file():
        issues.append(f'missing weights: {wpath}')
    else:
        got = sha256_file(wpath)
        if got != cand['weights']['sha256']:
            issues.append(
                f'weights sha mismatch: got={got} expect={cand["weights"]["sha256"]}'
            )
    if not rpath.is_file():
        issues.append(f'missing roi_rules: {rpath}')
    else:
        got = sha256_file(rpath)
        if got != cand['roi_rules']['sha256']:
            issues.append(
                f'roi_rules sha mismatch: got={got} expect={cand["roi_rules"]["sha256"]}'
            )
    if not dry_run:
        ft = cand.get('formal_test') or {}
        if not ft.get('frozen'):
            issues.append('candidate.formal_test.frozen is false; refuse formal run')
        freeze_path = ft.get('freeze_manifest')
        if freeze_path:
            fp = REPO / freeze_path if not Path(freeze_path).is_absolute() else Path(freeze_path)
            if not fp.is_file():
                issues.append(f'missing freeze manifest: {fp}')
            else:
                freeze = load_json(fp)
                expect = ft.get('freeze_sha256')
                if expect and sha256_file(fp) != expect:
                    issues.append('freeze manifest sha mismatch vs candidate')
                if not freeze.get('frozen'):
                    issues.append('freeze manifest frozen=false')
    return {'ok': not issues, 'issues': issues}


def iter_frame_metas(root: Path, batch_ids: list[str] | None) -> list[tuple[Path, dict[str, Any]]]:
    out: list[tuple[Path, dict[str, Any]]] = []
    batches_root = root / 'batches'
    if not batches_root.is_dir():
        raise FileNotFoundError(f'no batches/ under {root}')
    preferred = sorted(batches_root.glob('*/states/*.json'))
    search = preferred if preferred else sorted(batches_root.rglob('*.json'))
    seen: set[str] = set()
    for meta_path in search:
        if meta_path.name in ('batch_manifest.json', 'gt.json', 'status.json'):
            continue
        if meta_path.name.startswith('gt_seed'):
            continue
        try:
            meta = load_json(meta_path)
        except Exception:
            continue
        iid = meta.get('image_id')
        if not iid or 'station_id' not in meta:
            continue
        if iid in seen:
            continue
        if batch_ids is not None and meta.get('batch_id') not in batch_ids:
            continue
        seen.add(str(iid))
        out.append((meta_path, meta))
    return out


def load_bgr(meta_path: Path, meta: dict[str, Any]) -> np.ndarray:
    # states/<id>.json → images/<id>.png ; or sidecar next to png
    candidates: list[Path] = []
    fn = meta.get('file_name') or meta.get('image')
    if fn:
        # states/ → batch dir
        candidates.append(meta_path.parent.parent / fn)
        candidates.append(meta_path.parent / Path(fn).name)
    candidates.append(meta_path.with_suffix('.png'))
    if meta_path.parent.name == 'states':
        candidates.append(meta_path.parent.parent / 'images' / f"{meta.get('image_id')}.png")
    png = next((p for p in candidates if p.is_file()), None)
    if png is None:
        raise FileNotFoundError(f"image for {meta.get('image_id')} from {meta_path}")
    bgr = cv2.imread(str(png))
    if bgr is None:
        raise FileNotFoundError(png)
    return bgr


def gt_object_states(meta: dict[str, Any]) -> dict[str, Any]:
    states = meta.get('states') or meta.get('gt_scene') or meta.get('gt') or {}
    leds = list(states.get('leds') or [])
    while len(leds) < 3:
        leds.append('UNKNOWN')
    return {
        'leds': [str(x) for x in leds[:3]],
        'panel': str(states.get('panel', 'UNKNOWN')),
        'leak': str(states.get('leak', 'UNKNOWN')),
    }


def pred_object_states(
    decisions: list[Any], meta: dict[str, Any], rules: dict[str, Any]
) -> dict[str, Any]:
    """Map Decision list → per-object state labels for agreement."""
    from roi_judge_d1 import classify_leak, classify_led, classify_panel  # local

    # decisions_from_yolo returns rollup Decision for leds + panel + leak objects.
    # We need per-slot LED states from measurements when available.
    leds = ['UNKNOWN', 'UNKNOWN', 'UNKNOWN']
    panel = 'UNKNOWN'
    leak = 'UNKNOWN'
    for d in decisions:
        ot = d.object_type
        m = d.measurements or {}
        if ot == 'indicator_light':
            # Prefer per-slot states if present
            slot_states = m.get('slot_states') or m.get('led_states')
            if isinstance(slot_states, list) and len(slot_states) >= 3:
                leds = [str(x) for x in slot_states[:3]]
            elif 'led_state' in m:
                # single — leave UNKNOWN slots (miss already encoded)
                pass
            # recover from nested details
            details = m.get('slot_details') or []
            if details and len(details) >= 3:
                recovered = []
                for det in details[:3]:
                    if isinstance(det, dict) and 'led_state' in det:
                        recovered.append(str(det['led_state']))
                    elif isinstance(det, dict) and det.get('reason', '').startswith('yolo_miss'):
                        recovered.append('UNKNOWN')
                    else:
                        recovered.append('UNKNOWN')
                if any(x != 'UNKNOWN' for x in recovered) or all(
                    isinstance(det, dict) and 'reason' in det for det in details[:3]
                ):
                    leds = recovered
        elif ot == 'panel':
            panel = str(m.get('panel_state') or ('UNKNOWN' if d.verdict == 'UNKNOWN' else d.verdict))
            if panel in ('PASS', 'FAIL'):
                # map verdict back if state missing
                panel = 'lit' if d.verdict == 'PASS' else ('dark' if d.verdict == 'FAIL' else 'UNKNOWN')
            if 'panel_state' in m:
                panel = str(m['panel_state'])
        elif ot == 'inspection_region':
            if 'leak_state' in m:
                leak = str(m['leak_state'])
            elif d.verdict == 'PASS':
                leak = 'none'
            elif d.verdict == 'FAIL':
                leak = 'fixed_fail'
            else:
                leak = 'UNKNOWN'
    # Fallback: if LED rollup stored states differently, try slot_details on led decision
    for d in decisions:
        if d.object_type != 'indicator_light':
            continue
        m = d.measurements or {}
        if m.get('slot_states'):
            leds = [str(x) for x in m['slot_states'][:3]]
        elif m.get('per_slot'):
            leds = [str(x.get('led_state', 'UNKNOWN')) for x in m['per_slot'][:3]]
    _ = (classify_leak, classify_led, classify_panel, rules, meta)  # lint keep
    return {'leds': leds, 'panel': panel, 'leak': leak}


def extract_pred_states_from_yolo_path(
    bgr: np.ndarray,
    meta: dict[str, Any],
    rules: dict[str, Any],
    dets: list[dict[str, Any]],
    model_version: str,
) -> tuple[dict[str, Any], list[Any], dict[str, Any]]:
    """Replicate Runtime-like path but return per-slot LED states explicitly."""
    from roi_decision import Decision, RoiRequest, decide_one, led_slot_order, _rollup_leds

    by_cls: dict[str, list[dict[str, Any]]] = {c: [] for c in CLASS_NAMES}
    for d in dets:
        by_cls[d['class_name']].append(d)
    for c in CLASS_NAMES:
        by_cls[c].sort(key=lambda x: -x['confidence'])

    miss: dict[str, Any] = {'model_version': model_version, 'missed': []}
    slots = associate_led_slots(by_cls['indicator_light'], rules)
    order = led_slot_order(float(meta['robot_pose']['yaw']), rules)
    slot_states: list[str] = []
    slot_details: list[dict[str, Any]] = []
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
        slot_details.append(dict(d.measurements))

    led_dec = _rollup_leds(slot_states, slot_details, order, rules, 'detector_box')
    led_dec.measurements['slot_states'] = list(slot_states)
    decisions: list[Any] = [led_dec]

    panel_state = 'UNKNOWN'
    panel_cands = by_cls['panel']
    if not panel_cands:
        miss['missed'].append('panel')
        decisions.append(
            Decision(
                object_type='panel',
                verdict='UNKNOWN',
                reason='yolo_miss_panel',
                roi_source='detector_box',
                roi_rules_version=rules_version(rules),
                measurements={'miss': True, 'panel_state': 'UNKNOWN'},
            )
        )
    else:
        det = panel_cands[0]
        d = decide_one(
            bgr,
            RoiRequest(
                roi_source='detector_box',
                object_type='panel',
                bbox_xyxy=tuple(det['bbox_xyxy']),  # type: ignore[arg-type]
                confidence=det['confidence'],
            ),
            rules,
        )
        panel_state = str(d.measurements.get('panel_state', 'UNKNOWN'))
        d.measurements['panel_state'] = panel_state
        decisions.append(d)

    leak_state = 'UNKNOWN'
    leak_cands = by_cls['inspection_region']
    if not leak_cands:
        miss['missed'].append('inspection_region')
        decisions.append(
            Decision(
                object_type='inspection_region',
                verdict='UNKNOWN',
                reason='yolo_miss_inspection_region',
                roi_source='detector_box',
                roi_rules_version=rules_version(rules),
                measurements={'miss': True, 'leak_state': 'UNKNOWN'},
            )
        )
    else:
        det = leak_cands[0]
        d = decide_one(
            bgr,
            RoiRequest(
                roi_source='detector_box',
                object_type='inspection_region',
                bbox_xyxy=tuple(det['bbox_xyxy']),  # type: ignore[arg-type]
                confidence=det['confidence'],
            ),
            rules,
        )
        leak_state = str(d.measurements.get('leak_state', 'UNKNOWN'))
        d.measurements['leak_state'] = leak_state
        decisions.append(d)

    pred = {'leds': slot_states, 'panel': panel_state, 'leak': leak_state}
    return pred, decisions, miss


def station_rollup(states: dict[str, Any]) -> str:
    """PASS if all components pass; UNKNOWN if any UNKNOWN; else FAIL."""
    leds = states['leds']
    panel = states['panel']
    leak = states['leak']
    parts = []
    if any(x == 'UNKNOWN' for x in leds):
        parts.append('UNKNOWN')
    elif all(x == 'green' for x in leds):
        parts.append('PASS')
    else:
        parts.append('FAIL')
    if panel == 'UNKNOWN':
        parts.append('UNKNOWN')
    elif panel == 'lit':
        parts.append('PASS')
    else:
        parts.append('FAIL')
    if leak == 'UNKNOWN':
        parts.append('UNKNOWN')
    elif leak == 'none':
        parts.append('PASS')
    else:
        parts.append('FAIL')
    if 'UNKNOWN' in parts:
        return 'UNKNOWN'
    if 'FAIL' in parts:
        return 'FAIL'
    return 'PASS'


def agreement_units(
    gt: dict[str, Any], pred: dict[str, Any]
) -> list[tuple[str, str, str, bool]]:
    """Per-object units: 3 LED slots + panel + leak. UNKNOWN-in-denominator."""
    units: list[tuple[str, str, str, bool]] = []
    for i in range(3):
        g, p = gt['leds'][i], pred['leds'][i]
        units.append((f'led_{i}', g, p, g == p))
    units.append(('panel', gt['panel'], pred['panel'], gt['panel'] == pred['panel']))
    units.append(('leak', gt['leak'], pred['leak'], gt['leak'] == pred['leak']))
    return units


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--candidate', type=Path, default=DEFAULT_CANDIDATE)
    ap.add_argument('--root', type=Path, required=True, help='dataset root with batches/')
    ap.add_argument('--out', type=Path, required=True, help='report output directory')
    ap.add_argument('--dry-run', action='store_true', help='dev data only; tag DRY_RUN_NOT_TEST')
    ap.add_argument('--conf', type=float, default=None)
    ap.add_argument(
        '--batch-ids-file',
        type=Path,
        default=None,
        help='test_batches.txt; required for formal run',
    )
    ap.add_argument('--limit', type=int, default=0, help='optional frame cap (debug)')
    args = ap.parse_args(argv)

    cand = load_json(args.candidate)
    gate = gate_candidate(cand, dry_run=args.dry_run)
    if not gate['ok']:
        print('REFUSE:', *gate['issues'], sep='\n  ', file=sys.stderr)
        return 2

    if not args.dry_run:
        if args.batch_ids_file is None:
            ft = cand.get('formal_test') or {}
            tb = ft.get('test_batches_file')
            if tb:
                args.batch_ids_file = REPO / tb if not Path(tb).is_absolute() else Path(tb)
        if args.batch_ids_file is None or not args.batch_ids_file.is_file():
            print('REFUSE: formal run requires frozen test_batches.txt', file=sys.stderr)
            return 2

    batch_ids: list[str] | None = None
    if args.batch_ids_file and args.batch_ids_file.is_file():
        batch_ids = [
            ln.strip()
            for ln in args.batch_ids_file.read_text(encoding='utf-8').splitlines()
            if ln.strip() and not ln.strip().startswith('#')
        ]

    weights = REPO / cand['weights']['path']
    rules_path = REPO / cand['roi_rules']['path']
    rules = load_rules(rules_path)
    conf = float(args.conf if args.conf is not None else cand['inference']['conf'])
    model_version = f"{cand['candidate_id']}:{cand['weights']['sha256'][:12]}"

    frames = iter_frame_metas(args.root, batch_ids)
    if args.limit and args.limit > 0:
        frames = frames[: args.limit]
    if not frames:
        print('REFUSE: zero frames selected', file=sys.stderr)
        return 2

    args.out.mkdir(parents=True, exist_ok=True)
    per_frame: list[dict[str, Any]] = []
    unit_ok = 0
    unit_n = 0
    by_object = Counter()
    by_object_ok = Counter()
    confusion: dict[str, Counter] = defaultdict(Counter)
    failures: list[dict[str, Any]] = []
    station_ok = 0
    station_n = 0
    unknown_gt_n = 0
    unknown_gt_ok = 0

    for meta_path, meta in frames:
        # Skip synthetic on formal path
        if not args.dry_run and meta.get('synthetic') is True:
            failures.append(
                {
                    'image_id': meta.get('image_id'),
                    'error': 'synthetic_frame_in_formal_test',
                }
            )
            continue
        try:
            bgr = load_bgr(meta_path, meta)
            if 'robot_pose' not in meta:
                raise KeyError('robot_pose')
            dets = run_yolo(weights, bgr, conf)
            pred, decisions, miss = extract_pred_states_from_yolo_path(
                bgr, meta, rules, dets, model_version
            )
            gt = gt_object_states(meta)
            units = agreement_units(gt, pred)
            frame_fail = []
            for name, g, p, ok in units:
                unit_n += 1
                by_object[name] += 1
                confusion[name][f'{g}->{p}'] += 1
                if g == 'UNKNOWN':
                    unknown_gt_n += 1
                    if ok:
                        unknown_gt_ok += 1
                if ok:
                    unit_ok += 1
                    by_object_ok[name] += 1
                else:
                    frame_fail.append({'object': name, 'gt': g, 'pred': p})
            g_roll = station_rollup(gt)
            p_roll = station_rollup(pred)
            station_n += 1
            if g_roll == p_roll:
                station_ok += 1
            else:
                frame_fail.append({'object': 'station_rollup', 'gt': g_roll, 'pred': p_roll})
            rec = {
                'image_id': meta.get('image_id'),
                'batch_id': meta.get('batch_id'),
                'station_id': meta.get('station_id'),
                'distance_m': meta.get('distance_m'),
                'gt': gt,
                'pred': pred,
                'gt_rollup': g_roll,
                'pred_rollup': p_roll,
                'miss': miss,
                'n_dets': len(dets),
                'failures': frame_fail,
            }
            per_frame.append(rec)
            if frame_fail:
                failures.append(rec)
        except Exception as exc:  # noqa: BLE001
            failures.append({'image_id': meta.get('image_id'), 'error': str(exc)})

    report = {
        'schema': 'formal_vision_test_report_v1',
        'report_tag': 'DRY_RUN_NOT_TEST' if args.dry_run else 'FORMAL_TEST',
        'created_at': datetime.now(timezone.utc).isoformat(),
        'candidate_id': cand['candidate_id'],
        'candidate_path': str(args.candidate),
        'weights_sha256': cand['weights']['sha256'],
        'roi_rules_sha256': cand['roi_rules']['sha256'],
        'code_head_at_candidate': cand.get('code_head'),
        'root': str(args.root),
        'batch_ids': batch_ids,
        'n_frames': len(per_frame),
        'inference_conf': conf,
        'metrics': {
            'per_object_agreement': (unit_ok / unit_n) if unit_n else None,
            'per_object_correct': unit_ok,
            'per_object_total': unit_n,
            'station_rollup_agreement': (station_ok / station_n) if station_n else None,
            'station_correct': station_ok,
            'station_total': station_n,
            'unknown_gt_agreement': (unknown_gt_ok / unknown_gt_n) if unknown_gt_n else None,
            'unknown_gt_correct': unknown_gt_ok,
            'unknown_gt_total': unknown_gt_n,
            'by_object': {
                k: {
                    'correct': by_object_ok[k],
                    'total': by_object[k],
                    'agreement': (by_object_ok[k] / by_object[k]) if by_object[k] else None,
                }
                for k in sorted(by_object)
            },
        },
        'denominator_rules': {
            'unit': 'per image: led_0,led_1,led_2,panel,leak',
            'unknown_in_denominator': True,
            'unknown_correct_only_if_pred_unknown': True,
            'miss_counts_as_error': True,
            'synthetic_forbidden_in_formal': True,
        },
        'confusion': {k: dict(v) for k, v in confusion.items()},
        'n_failures': len(failures),
        'environment': env_info(),
        'gate': gate,
        'note': (
            'DRY_RUN_NOT_TEST — must not be cited as formal 5B gate'
            if args.dry_run
            else 'Formal frozen test run; do not retune on this set'
        ),
    }

    (args.out / 'report.json').write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8'
    )
    (args.out / 'per_frame.jsonl').write_text(
        ''.join(json.dumps(r, ensure_ascii=False) + '\n' for r in per_frame),
        encoding='utf-8',
    )
    (args.out / 'failures.json').write_text(
        json.dumps(failures, indent=2, ensure_ascii=False) + '\n', encoding='utf-8'
    )
    md = [
        f"# Vision Test Report ({report['report_tag']})",
        '',
        f"- created: `{report['created_at']}`",
        f"- candidate: `{report['candidate_id']}`",
        f"- frames: **{report['n_frames']}**",
        f"- per-object agreement: **{report['metrics']['per_object_agreement']}** "
        f"({unit_ok}/{unit_n})",
        f"- station rollup agreement: **{report['metrics']['station_rollup_agreement']}** "
        f"({station_ok}/{station_n})",
        f"- UNKNOWN GT agreement: **{report['metrics']['unknown_gt_agreement']}** "
        f"({unknown_gt_ok}/{unknown_gt_n})",
        f"- failures listed: {report['n_failures']}",
        '',
        '## By object',
        '',
    ]
    for k, v in report['metrics']['by_object'].items():
        md.append(f"- `{k}`: {v['agreement']} ({v['correct']}/{v['total']})")
    md.append('')
    md.append(f"## Note\n\n{report['note']}\n")
    (args.out / 'report.md').write_text('\n'.join(md), encoding='utf-8')

    print(json.dumps({
        'report_tag': report['report_tag'],
        'n_frames': report['n_frames'],
        'per_object_agreement': report['metrics']['per_object_agreement'],
        'station_rollup_agreement': report['metrics']['station_rollup_agreement'],
        'out': str(args.out),
    }, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
