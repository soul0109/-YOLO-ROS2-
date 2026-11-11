#!/usr/bin/env python3
"""Hash-gated formal vision Test runner (B2 / B2-fix).

Uses Runtime `inspect_station_image` for predictions (model-order LEDs,
measurements['panel']/['leak']). Refuses if candidate/ROI/freeze hashes
mismatch. Sanity gate marks INVALID_REPORT on harness-class bugs.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import shutil
import subprocess
import sys
import tempfile
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

SCRIPTS = Path(__file__).resolve().parent
REPO = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(REPO / 'ros2_ws' / 'src' / 'inspection_mission'))

from inspection_mission.station_inspect import (  # noqa: E402
    inspect_station_image,
)

DEFAULT_CANDIDATE = (
    REPO / 'models' / 'candidates' / 'inspection_train1_20261012.candidate.json'
)
STATION_INSPECT_PATH = (
    REPO / 'ros2_ws' / 'src' / 'inspection_mission' / 'inspection_mission' / 'station_inspect.py'
)
ROI_DECISION_PATH = SCRIPTS / 'roi_decision.py'


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


def resolve_png(meta_path: Path, meta: dict[str, Any]) -> Path:
    candidates: list[Path] = []
    fn = meta.get('file_name') or meta.get('image')
    if fn:
        candidates.append(meta_path.parent.parent / fn)
        candidates.append(meta_path.parent / Path(fn).name)
    candidates.append(meta_path.with_suffix('.png'))
    if meta_path.parent.name == 'states':
        candidates.append(meta_path.parent.parent / 'images' / f"{meta.get('image_id')}.png")
    png = next((p for p in candidates if p.is_file()), None)
    if png is None:
        raise FileNotFoundError(f"image for {meta.get('image_id')} from {meta_path}")
    return png


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


def _panel_from_verdict(verdict: str) -> str:
    v = str(verdict or '').strip()
    if v == 'PASS':
        return 'lit'
    if v == 'FAIL':
        return 'dark'
    return 'UNKNOWN'


def _leak_from_verdict(verdict: str) -> str:
    v = str(verdict or '').strip()
    if v == 'PASS':
        return 'none'
    if v == 'FAIL':
        return 'fixed_fail'
    return 'UNKNOWN'


def states_from_inspect_payload(payload: dict[str, Any]) -> dict[str, Any]:
    """Extract model-order LED / panel / leak states from Runtime payload."""
    objs = payload.get('objects') or {}
    led_obj = objs.get('leds') or {}
    panel_obj = objs.get('panel') or {}
    leak_obj = objs.get('leak') or {}

    leds_raw = led_obj.get('model_states')
    if not isinstance(leds_raw, list) or len(leds_raw) < 3:
        # Fallback: rebuild from image_slots via model_index (still Runtime fields)
        slots = led_obj.get('image_slots') or []
        leds = ['UNKNOWN', 'UNKNOWN', 'UNKNOWN']
        for slot in slots:
            if not isinstance(slot, dict):
                continue
            mi = slot.get('model_index')
            if mi is None:
                continue
            mi = int(mi)
            if 0 <= mi < 3:
                leds[mi] = str(slot.get('state', 'UNKNOWN'))
    else:
        leds = [str(x) for x in leds_raw[:3]]

    panel = panel_obj.get('state')
    if panel is None:
        panel = _panel_from_verdict(panel_obj.get('verdict', 'UNKNOWN'))
    else:
        panel = str(panel)

    leak = leak_obj.get('state')
    if leak is None:
        leak = _leak_from_verdict(leak_obj.get('verdict', 'UNKNOWN'))
    else:
        leak = str(leak)

    return {
        'leds': leds,
        'panel': panel,
        'leak': leak,
        'slot_contract': led_obj.get('slot_contract'),
        'inspection_status': payload.get('inspection_status'),
    }


def predict_via_runtime(
    *,
    png: Path,
    station_id: str,
    robot_yaw: float,
    weights: Path,
    rules_path: Path,
    conf: float,
    image_stamp: str = '0',
    mission_id: str = 'formal_harness',
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Call Runtime inspect_station_image on a temp copy (no pollution of bags/)."""
    with tempfile.TemporaryDirectory(prefix='formal_vision_') as td:
        tmp = Path(td) / 'shot.png'
        shutil.copy2(png, tmp)
        result = inspect_station_image(
            screenshot_path=tmp,
            mission_id=mission_id,
            station_id=station_id,
            image_stamp=image_stamp,
            robot_yaw=float(robot_yaw),
            weights=weights,
            rules_path=rules_path,
            conf=float(conf),
            write_annotated=False,
        )
        if not result.ok:
            raise RuntimeError(
                f'inspect_station_image failed: {result.error_code}: {result.message}'
            )
        payload = dict(result.payload)
    return states_from_inspect_payload(payload), payload


def station_rollup(states: dict[str, Any]) -> str:
    """Match Runtime priority: FAIL > UNKNOWN > PASS on component verdicts."""
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
    if 'FAIL' in parts:
        return 'FAIL'
    if 'UNKNOWN' in parts:
        return 'UNKNOWN'
    return 'PASS'


def agreement_units(
    gt: dict[str, Any], pred: dict[str, Any]
) -> list[tuple[str, str, str, bool]]:
    units: list[tuple[str, str, str, bool]] = []
    for i in range(3):
        g, p = gt['leds'][i], pred['leds'][i]
        units.append((f'led_{i}', g, p, g == p))
    units.append(('panel', gt['panel'], pred['panel'], gt['panel'] == pred['panel']))
    units.append(('leak', gt['leak'], pred['leak'], gt['leak'] == pred['leak']))
    return units


def sanity_gate(
    report: dict[str, Any],
    per_frame: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Detect harness-class bugs. Returns {ok, issues, report_tag_override?}."""
    issues: list[str] = []
    by_object = (report.get('metrics') or {}).get('by_object') or {}
    confusion = report.get('confusion') or {}

    for name, stats in by_object.items():
        total = int(stats.get('total') or 0)
        correct = int(stats.get('correct') or 0)
        if total > 0 and correct == 0:
            issues.append(f'{name}: correct==0 with total={total}')

    # All predictions UNKNOWN while non-UNKNOWN GT exists (per object class)
    if per_frame:
        for obj in ('led_0', 'led_1', 'led_2', 'panel', 'leak'):
            preds: set[str] = set()
            gts: set[str] = set()
            for rec in per_frame:
                gt = rec.get('gt') or {}
                pred = rec.get('pred') or {}
                if obj.startswith('led_'):
                    idx = int(obj.split('_')[1])
                    g = (gt.get('leds') or ['UNKNOWN'] * 3)[idx]
                    p = (pred.get('leds') or ['UNKNOWN'] * 3)[idx]
                else:
                    g = gt.get(obj, 'UNKNOWN')
                    p = pred.get(obj, 'UNKNOWN')
                gts.add(str(g))
                preds.add(str(p))
            if gts - {'UNKNOWN'} and preds == {'UNKNOWN'}:
                issues.append(
                    f'{obj}: all predictions UNKNOWN while non-UNKNOWN GT exists'
                )
    else:
        for name, conf in confusion.items():
            preds: set[str] = set()
            has_non_unk_gt = False
            for key in conf:
                if '->' not in key:
                    continue
                g, p = key.split('->', 1)
                preds.add(p)
                if g != 'UNKNOWN':
                    has_non_unk_gt = True
            if has_non_unk_gt and preds == {'UNKNOWN'}:
                issues.append(
                    f'{name}: all predictions UNKNOWN while non-UNKNOWN GT exists'
                )

    metrics = report.get('metrics') or {}
    unk_agr = metrics.get('unknown_gt_agreement')
    unk_total = int(metrics.get('unknown_gt_total') or 0)
    if unk_agr is not None and float(unk_agr) >= 1.0 - 1e-12 and unk_total > 0:
        non_ok = 0
        non_n = 0
        for conf in confusion.values():
            for key, n in conf.items():
                if '->' not in key:
                    continue
                g, p = key.split('->', 1)
                if g == 'UNKNOWN':
                    continue
                non_n += int(n)
                if g == p:
                    non_ok += int(n)
        if non_n > 0 and (non_ok / non_n) < 0.5:
            issues.append(
                f'UNKNOWN GT agreement=100% but non-UNKNOWN accuracy='
                f'{non_ok / non_n:.3f} < 0.5 (n={non_n})'
            )

    ok = not issues
    return {
        'ok': ok,
        'issues': issues,
        'report_validity': 'VALID' if ok else 'INVALID_REPORT',
    }


def code_hashes() -> dict[str, str]:
    return {
        'harness_sha256': sha256_file(Path(__file__).resolve()),
        'station_inspect_sha256': sha256_file(STATION_INSPECT_PATH),
        'roi_decision_sha256': sha256_file(ROI_DECISION_PATH),
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--candidate', type=Path, default=DEFAULT_CANDIDATE)
    ap.add_argument('--root', type=Path, default=None, help='dataset root with batches/')
    ap.add_argument('--out', type=Path, default=None, help='report output directory')
    ap.add_argument('--dry-run', action='store_true', help='dev data only; tag DRY_RUN_NOT_TEST')
    ap.add_argument('--conf', type=float, default=None)
    ap.add_argument('--batch-ids-file', type=Path, default=None)
    ap.add_argument('--limit', type=int, default=0)
    ap.add_argument(
        '--report-tag',
        default=None,
        help='override report_tag (e.g. FORMAL_TEST_v2)',
    )
    ap.add_argument(
        '--sanity-check-report',
        type=Path,
        default=None,
        help='only run sanity_gate on an existing report.json (no inference)',
    )
    args = ap.parse_args(argv)

    if args.sanity_check_report is not None:
        report = load_json(args.sanity_check_report)
        gate = sanity_gate(report)
        print(json.dumps(gate, indent=2, ensure_ascii=False))
        return 0 if gate['ok'] else 3

    if args.root is None or args.out is None:
        print('REFUSE: --root and --out required for evaluation runs', file=sys.stderr)
        return 2

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
    conf = float(args.conf if args.conf is not None else cand['inference']['conf'])

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
    by_object: Counter = Counter()
    by_object_ok: Counter = Counter()
    confusion: dict[str, Counter] = defaultdict(Counter)
    failures: list[dict[str, Any]] = []
    station_ok = 0
    station_n = 0
    unknown_gt_n = 0
    unknown_gt_ok = 0

    for meta_path, meta in frames:
        if not args.dry_run and meta.get('synthetic') is True:
            failures.append(
                {
                    'image_id': meta.get('image_id'),
                    'error': 'synthetic_frame_in_formal_test',
                }
            )
            continue
        try:
            png = resolve_png(meta_path, meta)
            pose = meta.get('robot_pose') or {}
            if 'yaw' not in pose:
                raise KeyError('robot_pose.yaw')
            stamp = meta.get('image_stamp')
            if isinstance(stamp, dict):
                image_stamp = f"{stamp.get('sec', 0)}.{int(stamp.get('nanosec', 0)):09d}"
            else:
                image_stamp = str(stamp or meta.get('captured_at') or '0')
            pred, payload = predict_via_runtime(
                png=png,
                station_id=str(meta['station_id']),
                robot_yaw=float(pose['yaw']),
                weights=weights,
                rules_path=rules_path,
                conf=conf,
                image_stamp=image_stamp,
                mission_id=f"formal_{meta.get('batch_id', 'x')}",
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
            p_roll = str(pred.get('inspection_status') or station_rollup(pred))
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
                'gt': {k: gt[k] for k in ('leds', 'panel', 'leak')},
                'pred': {
                    'leds': pred['leds'],
                    'panel': pred['panel'],
                    'leak': pred['leak'],
                },
                'slot_contract': pred.get('slot_contract'),
                'gt_rollup': g_roll,
                'pred_rollup': p_roll,
                'missed': payload.get('missed', []),
                'n_dets': len(payload.get('detections') or []),
                'failures': frame_fail,
            }
            per_frame.append(rec)
            if frame_fail:
                failures.append(rec)
        except Exception as exc:  # noqa: BLE001
            failures.append({'image_id': meta.get('image_id'), 'error': str(exc)})

    if args.report_tag:
        report_tag = args.report_tag
    elif args.dry_run:
        report_tag = 'DRY_RUN_NOT_TEST'
    else:
        report_tag = 'FORMAL_TEST'

    hashes = code_hashes()
    env = env_info()
    report: dict[str, Any] = {
        'schema': 'formal_vision_test_report_v1',
        'report_tag': report_tag,
        'created_at': datetime.now(timezone.utc).isoformat(),
        'candidate_id': cand['candidate_id'],
        'candidate_path': str(args.candidate),
        'weights_sha256': cand['weights']['sha256'],
        'roi_rules_sha256': cand['roi_rules']['sha256'],
        'code_head_at_candidate': cand.get('code_head'),
        'code_head_at_run': env.get('git_head'),
        'harness_sha256': hashes['harness_sha256'],
        'station_inspect_sha256': hashes['station_inspect_sha256'],
        'roi_decision_sha256': hashes['roi_decision_sha256'],
        'predictor': 'inspection_mission.station_inspect.inspect_station_image',
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
            'unit': 'per image: led_0,led_1,led_2,panel,leak (model-order LEDs)',
            'unknown_in_denominator': True,
            'unknown_correct_only_if_pred_unknown': True,
            'miss_counts_as_error': True,
            'synthetic_forbidden_in_formal': True,
        },
        'confusion': {k: dict(v) for k, v in confusion.items()},
        'n_failures': len(failures),
        'environment': env,
        'gate': gate,
        'note': (
            'DRY_RUN_NOT_TEST — must not be cited as formal 5B gate'
            if args.dry_run
            else 'Formal frozen test run via Runtime inspect_station_image; do not retune'
        ),
    }

    sgate = sanity_gate(report, per_frame)
    report['sanity_gate'] = sgate
    if not sgate['ok']:
        report['report_validity'] = 'INVALID_REPORT'
        report['report_tag'] = 'INVALID_REPORT'
    else:
        report['report_validity'] = 'VALID'

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
        f"- validity: `{report['report_validity']}`",
        f"- created: `{report['created_at']}`",
        f"- candidate: `{report['candidate_id']}`",
        f"- predictor: `{report['predictor']}`",
        f"- frames: **{report['n_frames']}**",
        f"- per-object agreement: **{report['metrics']['per_object_agreement']}** "
        f"({unit_ok}/{unit_n})",
        f"- station rollup agreement: **{report['metrics']['station_rollup_agreement']}** "
        f"({station_ok}/{station_n})",
        f"- UNKNOWN GT agreement: **{report['metrics']['unknown_gt_agreement']}** "
        f"({unknown_gt_ok}/{unknown_gt_n})",
        f"- failures listed: {report['n_failures']}",
        f"- harness_sha256: `{hashes['harness_sha256']}`",
        f"- station_inspect_sha256: `{hashes['station_inspect_sha256']}`",
        f"- roi_decision_sha256: `{hashes['roi_decision_sha256']}`",
        f"- code_head_at_run: `{env.get('git_head')}`",
        '',
        '## By object',
        '',
    ]
    for k, v in report['metrics']['by_object'].items():
        md.append(f"- `{k}`: {v['agreement']} ({v['correct']}/{v['total']})")
    if sgate['issues']:
        md.append('\n## Sanity gate issues\n')
        for issue in sgate['issues']:
            md.append(f'- {issue}')
    md.append('')
    md.append(f"## Note\n\n{report['note']}\n")
    (args.out / 'report.md').write_text('\n'.join(md), encoding='utf-8')

    print(
        json.dumps(
            {
                'report_tag': report['report_tag'],
                'report_validity': report['report_validity'],
                'n_frames': report['n_frames'],
                'per_object_agreement': report['metrics']['per_object_agreement'],
                'station_rollup_agreement': report['metrics']['station_rollup_agreement'],
                'sanity_ok': sgate['ok'],
                'sanity_issues': sgate['issues'],
                'out': str(args.out),
            },
            indent=2,
        )
    )
    return 0 if sgate['ok'] else 3


if __name__ == '__main__':
    raise SystemExit(main())
