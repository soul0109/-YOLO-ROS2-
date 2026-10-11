#!/usr/bin/env python3
"""C1 minimal vision_5b collection — pipeline + label discipline smoke.

~30–50 frames, 2–3 batch_ids. No YOLO train, no formal test freeze, no 90%.
Requires Gazebo+robot for --live; --aug-only builds UNKNOWN/hard from live batches.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from capture_vision_station_frames import STATION_POSE, _grab_bgr, _teleport  # noqa: E402
from c_off_verify import _spawn_c_led1  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = REPO / 'bags' / 'vision_5b_c1'
DEFAULT_RULES = (
    REPO / 'ros2_ws' / 'src' / 'vision_perception' / 'config' / 'roi_rules.yaml'
)
LABEL_VERSION = 'd2-v0.1'
FORBIDDEN_CLASS_TOKENS = frozenset(
    {
        'pass',
        'fail',
        'unknown',
        'green',
        'red',
        'yellow',
        'off',
        'lit',
        'dark',
        'fixed_fail',
        'none',
    }
)


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _load_rules(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _detections_from_rules(rules: dict[str, Any], *, omit: set[str] | None = None) -> list[dict]:
    omit = omit or set()
    rois = rules['rois']
    dets: list[dict] = []
    if 'indicator_light' not in omit:
        y0, y1 = rois['led_row_y']
        for x0, x1 in rois['led_slots_x']:
            dets.append(
                {
                    'class_name': 'indicator_light',
                    'bbox_xyxy': [int(x0), int(y0), int(x1), int(y1)],
                }
            )
    if 'panel' not in omit:
        dets.append({'class_name': 'panel', 'bbox_xyxy': [int(x) for x in rois['panel']]})
    if 'inspection_region' not in omit:
        dets.append(
            {
                'class_name': 'inspection_region',
                'bbox_xyxy': [int(x) for x in rois['leak_floor']],
            }
        )
    return dets


def _verdict_gt(states: dict[str, Any]) -> dict[str, str]:
    leds = states['leds']
    if 'UNKNOWN' in leds:
        led_v = 'UNKNOWN'
    elif any(x != 'green' for x in leds):
        led_v = 'FAIL'
    else:
        led_v = 'PASS'
    panel = states['panel']
    if panel == 'UNKNOWN':
        panel_v = 'UNKNOWN'
    elif panel == 'lit':
        panel_v = 'PASS'
    else:
        panel_v = 'FAIL'
    leak = states['leak']
    if leak == 'UNKNOWN':
        leak_v = 'UNKNOWN'
    elif leak == 'none':
        leak_v = 'PASS'
    else:
        leak_v = 'FAIL'
    return {'leds': led_v, 'panel': panel_v, 'leak': leak_v}


def _assert_detections_clean(dets: list[dict]) -> None:
    for d in dets:
        cn = str(d.get('class_name', '')).lower()
        if cn in FORBIDDEN_CLASS_TOKENS or any(t in cn for t in ('fail', 'pass', 'green')):
            raise ValueError(f'state word in class_name: {d}')


def _write_frame(
    batch_dir: Path,
    *,
    batch_id: str,
    image_id: str,
    bgr: np.ndarray,
    meta: dict[str, Any],
) -> Path:
    img_dir = batch_dir / 'images'
    img_dir.mkdir(parents=True, exist_ok=True)
    png = img_dir / f'{image_id}.png'
    js = img_dir / f'{image_id}.json'
    cv2.imwrite(str(png), bgr)
    meta = dict(meta)
    meta['file_name'] = f'images/{png.name}'
    meta['image_id'] = image_id
    meta['batch_id'] = batch_id
    _assert_detections_clean(meta.get('detections') or [])
    js.write_text(json.dumps(meta, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    return png


def _base_meta(
    *,
    station_id: str,
    seed: int | None,
    distance_m: float,
    pose: dict[str, float],
    stamp: dict[str, Any] | None,
    gt_scene: dict[str, Any],
    states: dict[str, Any],
    detections: list[dict],
    checklist_ids: list[str],
    hard_case: list[str],
    unknown_case_id: str | None,
    notes: str,
    split: str | None = None,
) -> dict[str, Any]:
    return {
        'schema': 'vision_5b_frame_v1',
        'split': split,
        'station_id': station_id,
        'distance_m': distance_m,
        'robot_pose': pose,
        'frame_id': 'map',
        'image_stamp': stamp,
        'captured_at': _now_iso(),
        'seed': seed,
        'gt_scene': gt_scene,
        'states': states,
        'verdict_gt': _verdict_gt(states),
        'detections': detections,
        'checklist_ids': checklist_ids,
        'hard_case': hard_case,
        'unknown_case_id': unknown_case_id,
        'notes': notes,
        'annotator': 'c1_collect_min',
        'label_version': LABEL_VERSION,
    }


def _apply_seed(seed: int, gt_out: Path) -> dict[str, Any]:
    cmd = [
        sys.executable,
        str(REPO / 'scripts' / 'apply_vision_scene_gt.py'),
        '--seed',
        str(seed),
        '--apply',
        '-o',
        str(gt_out),
    ]
    subprocess.run(cmd, check=True)
    return json.loads(gt_out.read_text(encoding='utf-8'))


def _capture_station(sid: str) -> dict[str, Any]:
    _teleport(sid)
    return _grab_bgr()


def _prepare_batch_dir(batch_dir: Path, *, force: bool) -> None:
    """Refuse to wipe history unless --force (C2 guardrail)."""
    if batch_dir.exists():
        if not force:
            raise SystemExit(
                f'{batch_dir} exists; pass --force to replace, '
                'or use a new run_id / bags/vision_5b_c2/'
            )
        shutil.rmtree(batch_dir)
    batch_dir.mkdir(parents=True)


def collect_live_batch(
    root: Path,
    batch_id: str,
    seeds: list[int],
    rules: dict[str, Any],
    *,
    include_c_green: bool,
    force: bool,
) -> dict[str, Any]:
    batch_dir = root / 'batches' / batch_id
    _prepare_batch_dir(batch_dir, force=force)
    frames: list[str] = []
    seq = 0
    dets_full = _detections_from_rules(rules)

    for seed in seeds:
        gt = _apply_seed(seed, batch_dir / f'gt_seed{seed}.json')
        for sid in ('A', 'B', 'C'):
            seq += 1
            grabbed = _capture_station(sid)
            st = gt['stations'][sid]
            states = {'leds': list(st['leds']), 'panel': st['panel'], 'leak': st['leak']}
            image_id = f'{batch_id}_{sid}_{seq:04d}'
            check = []
            if sid == 'A':
                check.append('N-A1')
            if sid == 'B':
                check.extend(['N-B1', 'S-LEAK'])
            if sid == 'C':
                check.append('N-C1')
                if 'off' in states['leds']:
                    check.append('S-OFF')
            if states['panel'] == 'dark':
                check.append('S-DARK')
            if states['panel'] == 'lit':
                check.append('S-LIT')
            meta = _base_meta(
                station_id=sid,
                seed=seed,
                distance_m=1.2,
                pose=dict(STATION_POSE[sid]),
                stamp=grabbed.get('stamp'),
                gt_scene=states,
                states=states,
                detections=list(dets_full),
                checklist_ids=check,
                hard_case=['none'],
                unknown_case_id=None,
                notes=f'live seed={seed}',
                split=None,
            )
            _write_frame(
                batch_dir,
                batch_id=batch_id,
                image_id=image_id,
                bgr=grabbed['img'],
                meta=meta,
            )
            frames.append(image_id)
            print(f'  [{batch_id}] {image_id} seed={seed} {sid} {states}', flush=True)

        if include_c_green and seed == seeds[0]:
            # Same pose C: force led_1 green for off/green contrast (C-off).
            st_c = gt['stations']['C']
            if st_c['leds'][0] == 'off':
                _spawn_c_led1('green')
                time.sleep(0.2)
                grabbed = _capture_station('C')
                seq += 1
                image_id = f'{batch_id}_C_green_{seq:04d}'
                states = {
                    'leds': ['green', st_c['leds'][1], st_c['leds'][2]],
                    'panel': st_c['panel'],
                    'leak': st_c['leak'],
                }
                meta = _base_meta(
                    station_id='C',
                    seed=seed,
                    distance_m=1.2,
                    pose=dict(STATION_POSE['C']),
                    stamp=grabbed.get('stamp'),
                    gt_scene={
                        'leds': list(st_c['leds']),
                        'panel': st_c['panel'],
                        'leak': st_c['leak'],
                    },
                    states=states,
                    detections=list(dets_full),
                    checklist_ids=['N-C1', 'S-OFF'],
                    hard_case=['none'],
                    unknown_case_id=None,
                    notes='C-off contrast: cabinet_c_led_1 forced green; gt_scene keeps seed off',
                    split=None,
                )
                _write_frame(
                    batch_dir,
                    batch_id=batch_id,
                    image_id=image_id,
                    bgr=grabbed['img'],
                    meta=meta,
                )
                frames.append(image_id)
                print(f'  [{batch_id}] {image_id} C green contrast', flush=True)
                _spawn_c_led1('off')

    man = {
        'schema': 'vision_5b_batch_manifest_v1',
        'batch_id': batch_id,
        'split': None,
        'suggested_split': 'train',
        'n_frames': len(frames),
        'image_ids': frames,
        'seeds': seeds,
        'label_version': LABEL_VERSION,
        'note': 'C1 unfrozen; whole batch → one split later',
    }
    (batch_dir / 'batch_manifest.json').write_text(
        json.dumps(man, indent=2, sort_keys=True) + '\n', encoding='utf-8'
    )
    return man


def _aug_from_source(
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
    rois = rules['rois']
    states = dict(src_meta['states'])
    gt_scene = dict(src_meta.get('gt_scene') or states)
    checklist = list(src_meta.get('checklist_ids') or [])
    hard = ['none']
    unknown_id = None
    parent_id = src_meta.get('image_id')
    transform_type = kind
    notes = f'aug from {parent_id}'
    omit: set[str] = set()

    if kind == 'U_panel':
        x0, y0, x1, y1 = rois['panel']
        im[y0:y1, x0:x1] = (80, 80, 80)
        states['panel'] = 'UNKNOWN'
        unknown_id = 'U_panel_ambiguous'
        checklist = list(set(checklist + ['U-PNL']))
        notes += '; panel mid-gray UNKNOWN'
    elif kind == 'U_leak':
        x0, y0, x1, y1 = rois['leak_floor']
        im[y0:y1, x0:x1] = (160, 160, 160)
        im[y0 + 30 : y0 + 50, x0 + 50 : x0 + 74] = (200, 220, 0)
        states['leak'] = 'UNKNOWN'
        unknown_id = 'U_leak_ambiguous'
        checklist = list(set(checklist + ['U-LEAK']))
        notes += '; leak mid-band UNKNOWN'
    elif kind == 'H_OVR':
        im = cv2.convertScaleAbs(im, alpha=2.2, beta=40)
        hard = ['overexpose']
        checklist = list(set(checklist + ['H-OVR']))
        notes += '; synthetic overexpose'
    elif kind == 'H_OCC':
        x0, y0, x1, y1 = rois['panel']
        im[y0:y1, x0:x1] = (20, 20, 20)
        states['panel'] = 'UNKNOWN'
        unknown_id = 'U_panel_occluded'
        hard = ['occlusion']
        checklist = list(set(checklist + ['H-OCC', 'U-PNL']))
        notes += '; panel occluded → UNKNOWN (not FAIL)'
    elif kind == 'H_BLUR':
        im = cv2.GaussianBlur(im, (21, 21), 8)
        if states['panel'] != 'UNKNOWN':
            states['panel'] = 'UNKNOWN'
            unknown_id = 'U_panel_blur'
        hard = ['blur']
        checklist = list(set(checklist + ['H-BLUR', 'U-PNL']))
        notes += '; blur → panel UNKNOWN'
    elif kind == 'H_MISS':
        # Pipeline fixture only — NOT a real YOLO miss.
        omit.add('panel')
        hard = ['crop']
        checklist = list(set(checklist + ['H-MISS']))
        transform_type = 'simulated_missing_detection'
        notes += (
            '; simulated_missing_detection (pipeline case) ≠ real YOLO miss; '
            'e2e must not default PASS'
        )
    else:
        raise ValueError(kind)

    dets = _detections_from_rules(rules, omit=omit)
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
    # Provenance (C2+ required; C1 also writes when regenerating aug).
    meta['synthetic'] = True
    meta['parent_frame_id'] = parent_id
    meta['transform_type'] = transform_type
    if kind == 'H_MISS':
        meta['missing_detections'] = ['panel']
        meta['e2e_note'] = (
            'simulated_missing_detection≠real_YOLO_miss; '
            'missing_detection≠PASS; panel state still labeled for audit'
        )
    _write_frame(batch_dir, batch_id=batch_id, image_id=image_id, bgr=im, meta=meta)


def collect_aug_batch(
    root: Path, batch_id: str, rules: dict[str, Any], *, force: bool
) -> dict[str, Any]:
    batch_dir = root / 'batches' / batch_id
    _prepare_batch_dir(batch_dir, force=force)
    frames: list[str] = []

    # Prefer live batch sources under root
    sources: list[tuple[Path, dict[str, Any]]] = []
    for bdir in sorted((root / 'batches').glob('c1_*')):
        if bdir.name == batch_id:
            continue
        for jp in sorted((bdir / 'images').glob('*.json')):
            meta = json.loads(jp.read_text(encoding='utf-8'))
            if meta.get('unknown_case_id') or meta.get('notes', '').startswith('aug'):
                continue
            png = bdir / meta['file_name']
            if png.is_file():
                sources.append((png, meta))

    def pick(station: str, pred) -> tuple[Path, dict[str, Any]]:
        for png, meta in sources:
            if meta['station_id'] == station and pred(meta):
                return png, meta
        for png, meta in sources:
            if meta['station_id'] == station:
                return png, meta
        raise RuntimeError(f'no source for station {station}')

    plan = [
        ('U_panel', 'A', lambda m: True),
        ('U_panel', 'B', lambda m: True),
        ('U_leak', 'B', lambda m: m['states']['leak'] == 'fixed_fail'),
        ('U_leak', 'B', lambda m: m['states']['leak'] == 'fixed_fail'),
        ('H_OVR', 'B', lambda m: True),
        ('H_OVR', 'C', lambda m: True),
        ('H_OCC', 'A', lambda m: True),
        ('H_OCC', 'B', lambda m: True),
        ('H_BLUR', 'B', lambda m: True),
        ('H_BLUR', 'C', lambda m: True),
        ('H_MISS', 'B', lambda m: True),
        ('H_MISS', 'A', lambda m: True),
        ('H_OVR', 'A', lambda m: True),
        ('U_panel', 'C', lambda m: True),
        ('H_BLUR', 'A', lambda m: True),
    ]

    for i, (kind, sid, pred) in enumerate(plan, start=1):
        png, meta = pick(sid, pred)
        image_id = f'{batch_id}_{kind}_{sid}_{i:04d}'
        _aug_from_source(
            batch_dir, batch_id, png, meta, rules, image_id=image_id, kind=kind
        )
        frames.append(image_id)
        print(f'  [{batch_id}] {image_id} from {meta.get("image_id")}', flush=True)

    man = {
        'schema': 'vision_5b_batch_manifest_v1',
        'batch_id': batch_id,
        'split': None,
        # Must match parent live batches — never suggest val/test (leakage).
        'suggested_split': 'train',
        'n_frames': len(frames),
        'image_ids': frames,
        'label_version': LABEL_VERSION,
        'note': (
            'Synthetic/aug batch: same suggested_split as parents (train). '
            'Formal test = independent raw batches only; never pad test with aug.'
        ),
    }
    (batch_dir / 'batch_manifest.json').write_text(
        json.dumps(man, indent=2, sort_keys=True) + '\n', encoding='utf-8'
    )
    return man


def spot_check(root: Path) -> dict[str, Any]:
    rows = []
    class_ok = True
    off_ok = True
    meta_ok = True
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
    for jp in sorted(root.glob('batches/*/images/*.json')):
        meta = json.loads(jp.read_text(encoding='utf-8'))
        missing = sorted(required - set(meta))
        if missing:
            meta_ok = False
        for d in meta.get('detections') or []:
            cn = str(d.get('class_name', '')).lower()
            if cn in FORBIDDEN_CLASS_TOKENS or 'green' in cn or 'fail' in cn:
                class_ok = False
        png = jp.parent / (jp.stem + '.png')
        row = {
            'image_id': meta.get('image_id'),
            'batch_id': meta.get('batch_id'),
            'station_id': meta.get('station_id'),
            'seed': meta.get('seed'),
            'states': meta.get('states'),
            'unknown_case_id': meta.get('unknown_case_id'),
            'hard_case': meta.get('hard_case'),
            'n_det': len(meta.get('detections') or []),
            'missing_fields': missing,
            'png_bytes': png.stat().st_size if png.is_file() else 0,
        }
        # off≠green: C with off in states — mean green channel in left LED slot not saturated
        if meta.get('station_id') == 'C' and 'off' in (meta.get('states') or {}).get(
            'leds', []
        ):
            if meta.get('unknown_case_id') is None and 'green' not in (
                meta.get('notes') or ''
            ):
                rules = _load_rules(DEFAULT_RULES)
                im = cv2.imread(str(png))
                if im is not None:
                    y0, y1 = rules['rois']['led_row_y']
                    x0, x1 = rules['rois']['led_slots_x'][0]
                    # C face +Y: slot0 ≈ led_1
                    if meta['states']['leds'][0] == 'off':
                        patch = im[y0:y1, x0:x1]
                        mean_g = float(patch[:, :, 1].mean())
                        row['c_slot0_mean_g'] = round(mean_g, 1)
                        if mean_g > 120:
                            off_ok = False
                            row['off_vs_green'] = 'FAIL_bright'
                        else:
                            row['off_vs_green'] = 'OK'
        rows.append(row)

    summary = {
        'schema': 'vision_5b_c1_spot_check_v1',
        'n_frames': len(rows),
        'meta_fields_ok': meta_ok,
        'class_name_no_state_words': class_ok,
        'c_off_not_bright_green': off_ok,
        'rows': rows,
    }
    out = root / 'manifests' / 'spot_check.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n', encoding='utf-8')

    # Human table
    lines = [
        '# C1 抽查表',
        '',
        f'- n_frames: {len(rows)}',
        f'- meta_fields_ok: {meta_ok}',
        f'- class_name_no_state_words: {class_ok}',
        f'- c_off_not_bright_green: {off_ok}',
        '',
        '| image_id | batch | station | seed | hard/unknown | dets | off_check |',
        '|---|---|---|---|---|---|---|',
    ]
    for r in rows:
        lines.append(
            f"| {r['image_id']} | {r['batch_id']} | {r['station_id']} | {r.get('seed')} | "
            f"{r.get('hard_case')}/{r.get('unknown_case_id')} | {r['n_det']} | "
            f"{r.get('off_vs_green', '-')} |"
        )
    (root / 'manifests' / 'spot_check.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return summary


def write_overview(root: Path, batch_mans: list[dict[str, Any]]) -> None:
    total = sum(m['n_frames'] for m in batch_mans)
    overview = {
        'schema': 'vision_5b_c1_overview_v1',
        'root': str(root),
        'n_batches': len(batch_mans),
        'n_frames': total,
        'batches': batch_mans,
        'splits': {
            'note': (
                'C1: all batches unfrozen (split=null). '
                'Aug/synthetic must share suggested_split with parents (train). '
                'Formal test = raw capture batches only.'
            ),
            'suggested': {m['batch_id']: m.get('suggested_split') for m in batch_mans},
        },
        'label_version': LABEL_VERSION,
        'created_at': _now_iso(),
    }
    man_dir = root / 'manifests'
    man_dir.mkdir(parents=True, exist_ok=True)
    (man_dir / 'c1_overview.json').write_text(
        json.dumps(overview, indent=2, sort_keys=True) + '\n', encoding='utf-8'
    )
    (root / 'splits' / 'README.txt').parent.mkdir(parents=True, exist_ok=True)
    (root / 'splits' / 'README.txt').write_text(
        'C1: no formal test freeze.\n'
        'Aug/synthetic frames → same split as parent_frame_id (suggested train).\n'
        'Formal test: independent raw batches only; never pad with aug.\n'
        'Do not randomly split neighbor frames across splits.\n',
        encoding='utf-8',
    )


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('-o', '--root', type=Path, default=DEFAULT_ROOT)
    p.add_argument('--rules', type=Path, default=DEFAULT_RULES)
    p.add_argument('--live', action='store_true', help='apply+capture via Gazebo')
    p.add_argument('--aug-only', action='store_true', help='only build aug batch from existing')
    p.add_argument(
        '--force',
        action='store_true',
        help='Allow replacing existing batch dirs (default: refuse; keep history)',
    )
    p.add_argument(
        '--patch-suggested-splits',
        action='store_true',
        help='Rewrite batch_manifest/overview suggested_split (aug→train); no recapture',
    )
    p.add_argument('--spot-check-only', action='store_true')
    args = p.parse_args(argv)
    root = args.root
    rules = _load_rules(args.rules)

    if args.spot_check_only:
        s = spot_check(root)
        print(
            f"SPOT n={s['n_frames']} meta_ok={s['meta_fields_ok']} "
            f"class_ok={s['class_name_no_state_words']} off_ok={s['c_off_not_bright_green']}"
        )
        return 0 if s['meta_fields_ok'] and s['class_name_no_state_words'] and s[
            'c_off_not_bright_green'
        ] else 1

    if args.patch_suggested_splits:
        mans = []
        for mp in sorted((root / 'batches').glob('*/batch_manifest.json')):
            man = json.loads(mp.read_text(encoding='utf-8'))
            if 'aug' in man.get('batch_id', ''):
                man['suggested_split'] = 'train'
                man['note'] = (
                    'Synthetic/aug batch: same suggested_split as parents (train). '
                    'Formal test = independent raw batches only; never pad test with aug.'
                )
                mp.write_text(
                    json.dumps(man, indent=2, sort_keys=True) + '\n', encoding='utf-8'
                )
            mans.append(man)
        write_overview(root, mans)
        print('patched suggested_split: aug→train; overview regenerated')
        return 0

    batch_mans: list[dict[str, Any]] = []
    day = datetime.now().strftime('%Y%m%d')

    if not args.aug_only:
        if not args.live:
            print('pass --live to capture (Gazebo required), or --aug-only', file=sys.stderr)
            return 2
        b1 = f'c1_{day}_b1_s42_100'
        b2 = f'c1_{day}_b2_s7_13_55'
        print(f'LIVE batch {b1} ...', flush=True)
        batch_mans.append(
            collect_live_batch(
                root, b1, [42, 100], rules, include_c_green=True, force=args.force
            )
        )
        print(f'LIVE batch {b2} ...', flush=True)
        batch_mans.append(
            collect_live_batch(
                root, b2, [7, 13, 55], rules, include_c_green=False, force=args.force
            )
        )

    # Always (re)build aug from whatever live batches exist
    b3 = f'c1_{day}_b3_aug'
    live_batches = [
        d.name
        for d in sorted((root / 'batches').glob('c1_*'))
        if 'aug' not in d.name and d.is_dir()
    ]
    if not live_batches and args.aug_only:
        print('no live batches to aug from', file=sys.stderr)
        return 1
    print(f'AUG batch {b3} ...', flush=True)
    batch_mans_existing = []
    for name in live_batches:
        mp = root / 'batches' / name / 'batch_manifest.json'
        if mp.is_file():
            batch_mans_existing.append(json.loads(mp.read_text(encoding='utf-8')))
    aug_man = collect_aug_batch(root, b3, rules, force=args.force)
    if args.aug_only:
        batch_mans = batch_mans_existing + [aug_man]
    else:
        batch_mans.append(aug_man)

    write_overview(root, batch_mans)
    summary = spot_check(root)
    print(
        f"C1_DONE root={root} frames={summary['n_frames']} "
        f"meta_ok={summary['meta_fields_ok']} class_ok={summary['class_name_no_state_words']} "
        f"off_ok={summary['c_off_not_bright_green']}"
    )
    ok = (
        summary['meta_fields_ok']
        and summary['class_name_no_state_words']
        and summary['c_off_not_bright_green']
        and summary['n_frames'] >= 30
    )
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
