#!/usr/bin/env python3
"""M1-T01 six-frame Hook-1 regression (opt-in model run).

Does NOT call hook1 main / E1 self-check / event writers.
Imports only helper functions: find_meta, load_frame, run_yolo, decisions_from_yolo.

Pure-logic path (no model): always available via unittest.
Model path: set M1_T01_RUN_MODEL=1 explicitly.

  # logic-only (default unittest discover safe — model skipped)
  PYTHONDONTWRITEBYTECODE=1 python3 -m unittest tests.test_m1_t01_hook1_regression -v

  # six-frame model regression
  M1_T01_RUN_MODEL=1 PYTHONDONTWRITEBYTECODE=1 \\
    YOLO_CONFIG_DIR=bags/m1_t01_<ts>/ultralytics_cfg \\
    python3 -m unittest tests.TestM1T01Hook1ModelRegression -v
"""

from __future__ import annotations

import copy
import hashlib
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Any

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'scripts'))

from roi_decision import DEFAULT_RULES, _rollup_leds, load_rules  # noqa: E402

WEIGHTS = REPO / 'bags' / 'vision_5b_c2' / 'train1' / 'weights' / 'best.pt'
EXPECTED_WEIGHTS_SHA256 = (
    'a1d2507a6fe5fc9bae12e85f28fc3e876477b010b88640bb251d198ee023fed4'
)
SUMMARY = REPO / 'bags' / 'vision_train1_e1_demo' / 'hook1_summary.json'
EVENTS = REPO / 'bags' / 'vision_train1_e1_demo' / 'events.jsonl'
ROOT = REPO / 'bags' / 'vision_5b_c2'

HIST_ID = 'c2_20261011_s0_unk_B_U_leak_0007'
ORDER_NEG_Y = [2, 1, 0]


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def _frame_ids() -> list[str]:
    data = json.loads(SUMMARY.read_text(encoding='utf-8'))
    return [f['image_id'] for f in data['frames']]


def _hist_event_led() -> dict[str, Any]:
    for line in EVENTS.read_text(encoding='utf-8').splitlines():
        e = json.loads(line)
        if (
            e.get('image_path', '').endswith(f'{HIST_ID}.png')
            and e.get('object_type') == 'indicator_light'
        ):
            return e
    raise AssertionError(f'historical LED event not found for {HIST_ID}')


class TestM1T01Hook1LogicOnly(unittest.TestCase):
    """Always-on: reconstruct historical miss without loading YOLO."""

    def test_weights_sha_matches_frozen(self):
        self.assertTrue(WEIGHTS.is_file(), f'missing weights {WEIGHTS}')
        self.assertEqual(_sha256(WEIGHTS), EXPECTED_WEIGHTS_SHA256)

    def test_historical_miss_rollup_from_event_inputs(self):
        """Fixed-detection regression: feed historical slot details into _rollup_leds.

        Distinguishes from live model regression: boxes/states come from events.jsonl
        shape of the known bug, not from current YOLO.
        """
        ev = _hist_event_led()
        slots = ev['measurements']['led_slots']
        # Rebuild pre-override inputs: miss slot was UNKNOWN before buggy rewrite.
        states: list[str] = []
        details: list[dict[str, Any]] = []
        for s in sorted(slots, key=lambda x: x['image_slot']):
            detail = copy.deepcopy(s['detail'])
            detail.pop('rel_to_row_max', None)
            detail.pop('rel_off_override', None)
            if detail.get('reason') == 'yolo_miss_led_slot':
                states.append('UNKNOWN')
            else:
                states.append(str(detail.get('led_state', s['state'])))
            details.append(detail)
        rules = load_rules(DEFAULT_RULES) if DEFAULT_RULES.exists() else {
            'schema': 'roi_rules_v1',
            'led': {'rel_to_row_max': 0.40},
            'inspection': {'led_pass_color': 'green'},
        }
        # Ensure led/inspection keys present even if yaml load path differs
        if 'led' not in rules:
            rules = {
                'schema': rules.get('schema', 'roi_rules_v1'),
                'led': {'rel_to_row_max': 0.40},
                'inspection': {'led_pass_color': 'green'},
            }
        dec = _rollup_leds(states, details, ORDER_NEG_Y, rules, 'detector_box')
        # V2-01: known red + miss → component FAIL; miss slot stays UNKNOWN.
        self.assertEqual(dec.verdict, 'FAIL', msg=dec)
        self.assertEqual(dec.measurements['leds'], ['UNKNOWN', 'red', 'green'])
        miss_detail = next(
            s['detail']
            for s in dec.measurements['led_slots']
            if s['detail'].get('reason') == 'yolo_miss_led_slot'
        )
        self.assertNotIn('rel_off_override', miss_detail)
        self.assertNotIn('rel_to_row_max', miss_detail)


@unittest.skipUnless(
    os.environ.get('M1_T01_RUN_MODEL') == '1',
    'opt-in only: export M1_T01_RUN_MODEL=1 to run six-frame model regression',
)
class TestM1T01Hook1ModelRegression(unittest.TestCase):
    """Live Train-1 inference on the six Hook-1 demo frames."""

    @classmethod
    def setUpClass(cls) -> None:
        # Redirect ultralytics settings/cache away from shared user config.
        evid = os.environ.get('M1_T01_EVID_DIR')
        if evid:
            cfg = Path(evid) / 'ultralytics_cfg'
        else:
            cfg = Path(tempfile.mkdtemp(prefix='m1_t01_ultra_'))
        cfg.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault('YOLO_CONFIG_DIR', str(cfg))
        os.environ.setdefault('ULTRALYTICS_OFFLINE', '1')

        import hook1_yolo_roi_e1 as hook1  # noqa: WPS433

        cls.hook1 = hook1
        cls.rules = load_rules(DEFAULT_RULES)
        if _sha256(WEIGHTS) != EXPECTED_WEIGHTS_SHA256:
            raise unittest.SkipTest('weights sha mismatch; refuse to run model regression')
        cls.results: list[dict[str, Any]] = []
        for image_id in _frame_ids():
            meta_path = hook1.find_meta(ROOT, image_id)
            bgr, meta, _png = hook1.load_frame(meta_path)
            dets = hook1.run_yolo(WEIGHTS, bgr, conf=0.25)
            decisions, miss = hook1.decisions_from_yolo(
                bgr,
                meta,
                cls.rules,
                dets,
                model_version='train1_dev:best.pt:a1d2507a6fe5',
            )
            by_obj = {d.object_type: d for d in decisions}
            cls.results.append(
                {
                    'image_id': image_id,
                    'station_id': meta.get('station_id'),
                    'n_dets': len(dets),
                    'dets': dets,
                    'missed': list(miss.get('missed', [])),
                    'verdicts': {
                        k: by_obj[k].verdict
                        for k in ('indicator_light', 'panel', 'inspection_region')
                        if k in by_obj
                    },
                    'led': {
                        'verdict': by_obj['indicator_light'].verdict,
                        'reason': by_obj['indicator_light'].reason,
                        'leds': by_obj['indicator_light'].measurements.get('leds'),
                        'led_slots': by_obj['indicator_light'].measurements.get(
                            'led_slots'
                        ),
                    },
                    'panel': {
                        'verdict': by_obj['panel'].verdict,
                        'reason': by_obj['panel'].reason,
                    },
                    'inspection_region': {
                        'verdict': by_obj['inspection_region'].verdict,
                        'reason': by_obj['inspection_region'].reason,
                    },
                }
            )

    def test_six_frames_ran(self):
        self.assertEqual(len(self.results), 6)

    def test_historical_counterexample_if_miss_reproduced(self):
        row = next(r for r in self.results if r['image_id'] == HIST_ID)
        if 'indicator_light_slot_2' in row['missed']:
            # V2-01: known red + miss → component FAIL; miss slot stays UNKNOWN.
            self.assertEqual(row['verdicts']['indicator_light'], 'FAIL')
            leds = row['led']['leds']
            self.assertIn('UNKNOWN', leds)
            self.assertIn('red', leds)
            miss_slots = [
                s
                for s in row['led']['led_slots']
                if s['detail'].get('reason') == 'yolo_miss_led_slot'
            ]
            self.assertTrue(miss_slots)
            for s in miss_slots:
                self.assertEqual(s['state'], 'UNKNOWN')
                self.assertNotIn('rel_off_override', s['detail'])
                self.assertNotIn('rel_to_row_max', s['detail'])
        else:
            # Environment may not reproduce historical miss — do not fake dets.
            self.skipTest(
                'live model did not miss indicator_light_slot_2; '
                'historical e2e acceptance deferred to architect '
                f'(n_dets={row["n_dets"]}, missed={row["missed"]}, '
                f'led={row["led"]["verdict"]})'
            )

    def test_abc_balance_led_fail_not_pass(self):
        summary = {f['image_id']: f for f in json.loads(SUMMARY.read_text())['frames']}
        for image_id in (
            'c2_20261011_s0_bal_A_d1p2_s201_0001',
            'c2_20261011_s0_bal_B_d1p2_s201_0002',
            'c2_20261011_s0_bal_C_d1p2_s201_0003',
        ):
            hist = summary[image_id]['verdicts']['indicator_light']
            live = next(r for r in self.results if r['image_id'] == image_id)
            if hist == 'FAIL':
                self.assertNotEqual(
                    live['verdicts']['indicator_light'],
                    'PASS',
                    msg=f'{image_id} must not flip known FAIL→PASS',
                )


def dump_model_snapshot(out_path: Path) -> dict[str, Any]:
    """Helper for evidence capture (importable; not an automatic unittest)."""
    os.environ['M1_T01_RUN_MODEL'] = '1'
    # Reuse class setup without asserting
    import hook1_yolo_roi_e1 as hook1

    rules = load_rules(DEFAULT_RULES)
    assert _sha256(WEIGHTS) == EXPECTED_WEIGHTS_SHA256
    rows = []
    for image_id in _frame_ids():
        meta_path = hook1.find_meta(ROOT, image_id)
        bgr, meta, _png = hook1.load_frame(meta_path)
        dets = hook1.run_yolo(WEIGHTS, bgr, conf=0.25)
        decisions, miss = hook1.decisions_from_yolo(
            bgr,
            meta,
            rules,
            dets,
            model_version='train1_dev:best.pt:a1d2507a6fe5',
        )
        by_obj = {d.object_type: d for d in decisions}
        rows.append(
            {
                'image_id': image_id,
                'station_id': meta.get('station_id'),
                'n_dets': len(dets),
                'dets': dets,
                'missed': list(miss.get('missed', [])),
                'verdicts': {k: by_obj[k].verdict for k in by_obj},
                'reasons': {k: by_obj[k].reason for k in by_obj},
                'led_measurements': by_obj['indicator_light'].measurements,
                'panel_measurements': {
                    k: by_obj['panel'].measurements.get(k)
                    for k in ('panel', 'v_mean', 'bbox_xyxy', 'reason')
                    if k in by_obj['panel'].measurements
                    or k == 'bbox_xyxy'
                },
                'leak_measurements': {
                    k: by_obj['inspection_region'].measurements.get(k)
                    for k in ('leak', 'cyan_frac', 'bbox_xyxy', 'reason')
                },
            }
        )
    payload = {
        'kind': 'm1_t01_six_frame_snapshot',
        'weights': str(WEIGHTS.relative_to(REPO)),
        'weights_sha256': EXPECTED_WEIGHTS_SHA256,
        'frames': rows,
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')
    return payload


if __name__ == '__main__':
    if os.environ.get('M1_T01_DUMP_SNAPSHOT'):
        out = Path(os.environ['M1_T01_DUMP_SNAPSHOT'])
        dump_model_snapshot(out)
        print(f'wrote {out}')
    else:
        unittest.main()
