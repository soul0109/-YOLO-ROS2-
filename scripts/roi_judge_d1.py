#!/usr/bin/env python3
"""R1 offline ROI judge on D1 station frames.

Reads roi_rules.yaml + station*.png/json, emits per-object status and
PASS|FAIL|UNKNOWN inspection fields. No training, no Nav2, no 90% claim.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

DEFAULT_RULES = (
    Path.home()
    / 'inspection-robot'
    / 'ros2_ws'
    / 'src'
    / 'vision_perception'
    / 'config'
    / 'roi_rules.yaml'
)


def load_rules(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _clip_box(im: np.ndarray, box: list[int]) -> np.ndarray:
    h, w = im.shape[:2]
    x0, y0, x1, y1 = box
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(w, x1), min(h, y1)
    return im[y0:y1, x0:x1]


def classify_led(patch_bgr: np.ndarray, cfg: dict[str, Any]) -> tuple[str, dict[str, Any]]:
    if patch_bgr.size == 0:
        return 'UNKNOWN', {'reason': 'empty_roi'}

    vmax = patch_bgr.max(axis=2)
    v90 = float(np.percentile(vmax, 90))
    mean_vmax = float(vmax.mean())
    off = cfg['off']
    if v90 <= off['v90_max'] and mean_vmax <= off['mean_vmax_max']:
        return 'off', {'v90': v90, 'mean_vmax': mean_vmax}

    bright = vmax >= cfg['bright_vmax_min']
    if int(bright.sum()) < cfg['min_bright_pixels']:
        return 'UNKNOWN', {
            'reason': 'too_few_bright_pixels',
            'bright': int(bright.sum()),
            'v90': v90,
        }

    sel = patch_bgr[bright]
    b, g, r = [float(x) for x in sel.mean(axis=0)]
    detail = {'bgr': [round(b, 1), round(g, 1), round(r, 1)], 'v90': round(v90, 1)}

    red = cfg['red']
    if r >= red['r_min'] and (r - g) >= red['r_over_g'] and (r - b) >= red['r_over_b']:
        return 'red', detail

    yellow = cfg['yellow']
    if (
        r >= yellow['r_min']
        and g >= yellow['g_min']
        and b <= yellow['b_max']
        and abs(r - g) <= yellow['rg_abs_max']
    ):
        return 'yellow', detail

    green = cfg['green']
    if g >= green['g_min'] and (g - r) >= green['g_over_r'] and (g - b) >= green['g_over_b']:
        return 'green', detail

    return 'UNKNOWN', {**detail, 'reason': 'U_led_no_color'}


def classify_panel(patch_bgr: np.ndarray, cfg: dict[str, Any]) -> tuple[str, dict[str, Any]]:
    if patch_bgr.size == 0:
        return 'UNKNOWN', {'reason': 'empty_roi'}
    hsv = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2HSV)
    v_mean = float(hsv[:, :, 2].mean())
    detail = {'v_mean': round(v_mean, 1)}
    if v_mean >= cfg['lit_v_min']:
        return 'lit', detail
    if v_mean <= cfg['dark_v_max']:
        return 'dark', detail
    return 'UNKNOWN', {**detail, 'reason': 'U_panel_ambiguous'}


def classify_leak(patch_bgr: np.ndarray, cfg: dict[str, Any]) -> tuple[str, dict[str, Any]]:
    if patch_bgr.size == 0:
        return 'UNKNOWN', {'reason': 'empty_roi'}
    hsv = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2HSV)
    low = np.array(cfg['hsv_low'], dtype=np.uint8)
    high = np.array(cfg['hsv_high'], dtype=np.uint8)
    mask = cv2.inRange(hsv, low, high)
    frac = float(mask.mean() / 255.0)
    detail = {'cyan_frac': round(frac, 4)}
    if frac >= cfg['fail_frac_min']:
        return 'fixed_fail', detail
    if frac <= cfg['none_frac_max']:
        return 'none', detail
    return 'UNKNOWN', {**detail, 'reason': 'U_leak_ambiguous'}


def led_slot_order(yaw: float, rules: dict[str, Any]) -> list[int]:
    mapping = rules['led_image_to_model']
    if yaw < 0:
        return list(mapping['face_neg_y'])
    return list(mapping['face_pos_y'])


def judge_image(
    bgr: np.ndarray,
    meta: dict[str, Any],
    rules: dict[str, Any],
) -> dict[str, Any]:
    rois = rules['rois']
    y0, y1 = rois['led_row_y']
    order = led_slot_order(float(meta['robot_pose']['yaw']), rules)

    led_states: list[str | None] = [None, None, None]
    led_debug: list[dict[str, Any]] = []
    for slot_i, (x0, x1) in enumerate(rois['led_slots_x']):
        patch = bgr[y0:y1, x0:x1]
        state, detail = classify_led(patch, rules['led'])
        model_i = order[slot_i]
        led_states[model_i] = state
        led_debug.append(
            {
                'image_slot': slot_i,
                'model_index': model_i,
                'state': state,
                'detail': detail,
            }
        )

    panel_state, panel_detail = classify_panel(
        _clip_box(bgr, rois['panel']), rules['panel']
    )
    leak_state, leak_detail = classify_leak(
        _clip_box(bgr, rois['leak_floor']), rules['leak']
    )

    insp = rules['inspection']
    def obj_verdict(kind: str, state: str) -> str:
        if state == 'UNKNOWN':
            return 'UNKNOWN'
        if kind == 'led':
            return 'PASS' if state == insp['led_pass_color'] else 'FAIL'
        if kind == 'panel':
            return 'PASS' if state == insp['panel_pass_state'] else 'FAIL'
        if kind == 'leak':
            return 'PASS' if state == insp['leak_pass_state'] else 'FAIL'
        return 'UNKNOWN'

    leds = [led_states[i] or 'UNKNOWN' for i in range(3)]
    led_verdicts = [obj_verdict('led', s) for s in leds]
    # Station LED rollup: UNKNOWN if any UNKNOWN; else FAIL if any FAIL; else PASS
    if 'UNKNOWN' in led_verdicts:
        led_roll = 'UNKNOWN'
    elif 'FAIL' in led_verdicts:
        led_roll = 'FAIL'
    else:
        led_roll = 'PASS'

    return {
        'schema': 'roi_judge_d1_v1',
        'station_id': meta.get('station_id'),
        'seed': meta.get('seed'),
        'objects': {
            'leds': leds,
            'panel': panel_state,
            'leak': leak_state,
        },
        'verdict': {
            'leds': led_roll,
            'panel': obj_verdict('panel', panel_state),
            'leak': obj_verdict('leak', leak_state),
        },
        'debug': {
            'led_slots': led_debug,
            'panel': panel_detail,
            'leak': leak_detail,
            'led_image_to_model': order,
        },
    }


def compare_gt(result: dict[str, Any], gt: dict[str, Any]) -> dict[str, Any]:
    obj = result['objects']
    checks = {
        'leds': obj['leds'] == gt['leds'],
        'panel': obj['panel'] == gt['panel'],
        'leak': obj['leak'] == gt['leak'],
    }
    return {
        'match': checks,
        'all_match': all(checks.values()),
        'gt': gt,
    }


def make_unknown_demos(src_dir: Path, out_dir: Path, rules: dict[str, Any]) -> list[Path]:
    """Synthesize 2 UNKNOWN fixtures from D1 frames (not a test framework)."""
    out_dir.mkdir(parents=True, exist_ok=True)
    paths: list[Path] = []

    # U_panel_ambiguous: mid gray panel on A
    a_png = src_dir / 'stationA_seed42.png'
    a_json = src_dir / 'stationA_seed42.json'
    if a_png.is_file():
        im = cv2.imread(str(a_png))
        meta = json.loads(a_json.read_text(encoding='utf-8'))
        x0, y0, x1, y1 = rules['rois']['panel']
        im[y0:y1, x0:x1] = (80, 80, 80)
        p_img = out_dir / 'U_panel_ambiguous.png'
        p_meta = out_dir / 'U_panel_ambiguous.json'
        cv2.imwrite(str(p_img), im)
        meta = dict(meta)
        meta['station_id'] = 'U_panel'
        meta['gt'] = {'leds': meta['gt']['leds'], 'panel': 'UNKNOWN', 'leak': 'none'}
        meta['image'] = p_img.name
        p_meta.write_text(json.dumps(meta, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        paths.append(p_meta)

    # U_leak_ambiguous: wipe B leak, paint a small cyan blotch → frac in (none, fail)
    b_png = src_dir / 'stationB_seed42.png'
    b_json = src_dir / 'stationB_seed42.json'
    if b_png.is_file():
        im = cv2.imread(str(b_png))
        meta = json.loads(b_json.read_text(encoding='utf-8'))
        x0, y0, x1, y1 = rules['rois']['leak_floor']
        im[y0:y1, x0:x1] = (160, 160, 160)
        # ~480 px / (160*100) ≈ 0.03 → UNKNOWN band
        im[y0 + 30 : y0 + 50, x0 + 50 : x0 + 74] = (200, 220, 0)
        p_img = out_dir / 'U_leak_ambiguous.png'
        p_meta = out_dir / 'U_leak_ambiguous.json'
        cv2.imwrite(str(p_img), im)
        meta = dict(meta)
        meta['station_id'] = 'U_leak'
        meta['gt'] = {
            'leds': meta['gt']['leds'],
            'panel': meta['gt']['panel'],
            'leak': 'UNKNOWN',
        }
        meta['image'] = p_img.name
        p_meta.write_text(json.dumps(meta, indent=2, sort_keys=True) + '\n', encoding='utf-8')
        paths.append(p_meta)

    return paths


def judge_one(meta_path: Path, rules: dict[str, Any]) -> dict[str, Any]:
    meta = json.loads(meta_path.read_text(encoding='utf-8'))
    img_path = meta_path.parent / meta['image']
    bgr = cv2.imread(str(img_path))
    if bgr is None:
        raise FileNotFoundError(img_path)
    result = judge_image(bgr, meta, rules)
    gt = meta.get('gt', {})
    cmp_ = compare_gt(result, gt)
    # UNKNOWN fixtures: only require the targeted field to be UNKNOWN.
    if gt.get('panel') == 'UNKNOWN':
        cmp_['match'] = {
            'leds': True,
            'panel': result['objects']['panel'] == 'UNKNOWN',
            'leak': True,
        }
        cmp_['all_match'] = cmp_['match']['panel']
    if gt.get('leak') == 'UNKNOWN':
        cmp_['match'] = {
            'leds': True,
            'panel': True,
            'leak': result['objects']['leak'] == 'UNKNOWN',
        }
        cmp_['all_match'] = cmp_['match']['leak']
    result['gt_compare'] = cmp_
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--rules',
        type=Path,
        default=DEFAULT_RULES,
        help='roi_rules.yaml path',
    )
    parser.add_argument(
        '--d1-dir',
        type=Path,
        default=Path.home() / 'inspection-robot' / 'bags' / 'vision_d1_seed42',
    )
    parser.add_argument(
        '-o',
        '--output',
        type=Path,
        default=None,
        help='Write summary JSON (default: <d1-dir>/roi_judge_summary.json)',
    )
    parser.add_argument(
        '--unknown-demo',
        action='store_true',
        help='Also synthesize U_panel_ambiguous / U_leak_ambiguous and judge them',
    )
    args = parser.parse_args(argv)

    rules = load_rules(args.rules)
    metas = sorted(args.d1_dir.glob('station*_seed*.json'))
    if not metas:
        print(f'no station*.json under {args.d1_dir}', file=sys.stderr)
        return 1

    results = []
    for mp in metas:
        results.append(judge_one(mp, rules))

    if args.unknown_demo:
        demo_dir = args.d1_dir / 'unknown_demo'
        for mp in make_unknown_demos(args.d1_dir, demo_dir, rules):
            results.append(judge_one(mp, rules))

    summary = {
        'schema': 'roi_judge_d1_summary_v1',
        'rules': str(args.rules),
        'd1_dir': str(args.d1_dir),
        'frames': results,
        'note': (
            'Offline ROI smoke only. C-station GT led off may mismatch if '
            'camera still shows green bloom — judge reports image truth.'
        ),
    }
    out = args.output or (args.d1_dir / 'roi_judge_summary.json')
    out.write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n', encoding='utf-8')

    for r in results:
        sid = r.get('station_id')
        obj = r['objects']
        ok = r['gt_compare']['all_match']
        print(
            f"{sid}: leds={obj['leds']} panel={obj['panel']} leak={obj['leak']} "
            f"verdict={r['verdict']} gt_match={ok}"
        )
    print(f'wrote {out}')
    # Exit 0 even with known C mismatch — print mismatches clearly.
    return 0


if __name__ == '__main__':
    sys.exit(main())
