#!/usr/bin/env python3
"""M1-T01 pure-logic regression for roi_decision._rollup_leds.

No YOLO / ultralytics import. Run:
  PYTHONDONTWRITEBYTECODE=1 python3 -m unittest tests.test_roi_decision_rollup -v
"""

from __future__ import annotations

import copy
import math
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'scripts'))

from roi_decision import _rollup_leds  # noqa: E402

# Minimal rules: keep rel belt + FAIL>UNKNOWN>PASS inspection mapping.
RULES = {
    'schema': 'roi_rules_v1',
    'led': {'rel_to_row_max': 0.40},
    'inspection': {'led_pass_color': 'green'},
}

# Two existing image→model mappings from roi_rules.yaml
ORDER_NEG_Y = [2, 1, 0]  # face_neg_y (A/B)
ORDER_POS_Y = [0, 1, 2]  # face_pos_y (C)


def _det(state: str, *, mean_vmax=None, v90=None, extra=None) -> tuple[str, dict]:
    d: dict = {'led_state': state}
    if mean_vmax is not None:
        d['mean_vmax'] = mean_vmax
    if v90 is not None:
        d['v90'] = v90
    if extra:
        d.update(extra)
    return state, d


def _miss(slot_i: int = 2) -> tuple[str, dict]:
    return 'UNKNOWN', {'reason': 'yolo_miss_led_slot', 'slot_i': slot_i}


def _rollup(states_details, order=ORDER_NEG_Y, rules=None):
    states = [s for s, _ in states_details]
    details = [copy.deepcopy(d) for _, d in states_details]
    return _rollup_leds(states, details, order, rules or RULES, 'detector_box'), details


class TestRollupMissingVsZero(unittest.TestCase):
    def test_01_green_red_miss_keeps_fail_and_red(self):
        """Known red + miss: component FAIL; miss stays UNKNOWN (never off).

        V2-01: confirmed FAIL must not be masked by slot UNKNOWN. Historical
        miss→0.0→rel_off coercion protection still required.
        """
        dec, details = _rollup(
            [
                _det('green', mean_vmax=191.6, v90=255.0),
                _det('red', mean_vmax=197.5, v90=255.0),
                _miss(2),
            ]
        )
        self.assertEqual(dec.verdict, 'FAIL')
        self.assertEqual(dec.reason, 'led_non_green:UNKNOWN,red,green;incomplete')
        # face_neg_y: image [green, red, UNKNOWN] → model [UNKNOWN, red, green]
        self.assertEqual(dec.measurements['leds'], ['UNKNOWN', 'red', 'green'])
        miss = details[2]
        self.assertNotIn('rel_to_row_max', miss)
        self.assertNotIn('rel_off_override', miss)
        self.assertEqual(miss.get('reason'), 'yolo_miss_led_slot')
        # red detail preserved on image slot 1 / model index 1
        slots = dec.measurements['led_slots']
        self.assertEqual(slots[1]['state'], 'red')
        self.assertEqual(slots[1]['detail'].get('led_state'), 'red')

    def test_02_three_green_pass(self):
        dec, details = _rollup(
            [
                _det('green', mean_vmax=180.0, v90=255.0),
                _det('green', mean_vmax=185.0, v90=255.0),
                _det('green', mean_vmax=190.0, v90=255.0),
            ]
        )
        self.assertEqual(dec.verdict, 'PASS')
        self.assertEqual(dec.reason, 'all_leds_green')
        self.assertEqual(dec.measurements['leds'], ['green', 'green', 'green'])
        for d in details:
            self.assertIn('rel_to_row_max', d)
            self.assertNotIn('rel_off_override', d)

    def test_03_red_no_unknown_fail(self):
        dec, _ = _rollup(
            [
                _det('green', mean_vmax=180.0, v90=255.0),
                _det('red', mean_vmax=190.0, v90=255.0),
                _det('green', mean_vmax=185.0, v90=255.0),
            ]
        )
        self.assertEqual(dec.verdict, 'FAIL')
        self.assertIn('red', dec.measurements['leds'])
        self.assertNotIn('UNKNOWN', dec.measurements['leds'])

    def test_04_measured_off_fail(self):
        dec, details = _rollup(
            [
                _det('off', mean_vmax=10.0, v90=20.0),
                _det('green', mean_vmax=180.0, v90=255.0),
                _det('green', mean_vmax=185.0, v90=255.0),
            ]
        )
        self.assertEqual(dec.verdict, 'FAIL')
        self.assertEqual(dec.measurements['leds'][2], 'off')  # face_neg_y model[2]=slot0
        # known off may still get ratio; must not be treated as missing
        self.assertIn('rel_to_row_max', details[0])

    def test_05_all_three_miss_unknown_no_ratio(self):
        dec, details = _rollup([_miss(0), _miss(1), _miss(2)])
        self.assertEqual(dec.verdict, 'UNKNOWN')
        self.assertEqual(dec.measurements['leds'], ['UNKNOWN', 'UNKNOWN', 'UNKNOWN'])
        for d in details:
            self.assertNotIn('rel_to_row_max', d)
            self.assertNotIn('rel_off_override', d)

    def test_06_unknown_with_measurement_not_forced_off(self):
        """Bright-but-no-color UNKNOWN below rel belt must stay UNKNOWN."""
        dec, details = _rollup(
            [
                _det('green', mean_vmax=200.0, v90=255.0),
                _det(
                    'UNKNOWN',
                    mean_vmax=20.0,
                    v90=30.0,
                    extra={'reason': 'led_no_color'},
                ),
                _det('green', mean_vmax=190.0, v90=255.0),
            ]
        )
        self.assertEqual(dec.verdict, 'UNKNOWN')
        self.assertEqual(details[1].get('led_state', 'UNKNOWN'), 'UNKNOWN')
        # may record ratio for measured UNKNOWN, but never override to off
        self.assertNotIn('rel_off_override', details[1])
        slots = {s['image_slot']: s for s in dec.measurements['led_slots']}
        self.assertEqual(slots[1]['state'], 'UNKNOWN')

    def test_07_mean_vmax_zero_keeps_zero_not_v90(self):
        """Explicit mean_vmax=0.0 is a measurement; must not fall back to v90."""
        dec, details = _rollup(
            [
                _det('off', mean_vmax=0.0, v90=255.0),
                _det('green', mean_vmax=180.0, v90=255.0),
                _det('green', mean_vmax=190.0, v90=255.0),
            ]
        )
        self.assertEqual(details[0]['mean_vmax'], 0.0)
        # ratio must be based on 0.0 / row_max, not 255
        self.assertEqual(details[0].get('rel_to_row_max'), 0.0)
        self.assertEqual(dec.verdict, 'FAIL')

    def test_08_v90_compat_and_both_missing(self):
        # only v90 present → compat path; dim known-green becomes off via belt
        dec_v90, details_v90 = _rollup(
            [
                _det('green', v90=200.0),
                _det('green', v90=210.0),
                _det('green', v90=5.0),
            ]
        )
        self.assertIn('rel_to_row_max', details_v90[2])
        self.assertLessEqual(details_v90[2]['rel_to_row_max'], 0.40)
        self.assertEqual(details_v90[2].get('rel_off_override'), True)
        self.assertEqual(dec_v90.verdict, 'FAIL')

        # both brightness fields absent: must not invent 0.0 / force off
        dec_miss, details_miss = _rollup(
            [
                _det('green', mean_vmax=180.0, v90=255.0),
                ('green', {'led_state': 'green'}),  # no mean_vmax, no v90
                _det('green', mean_vmax=190.0, v90=255.0),
            ]
        )
        self.assertNotIn('rel_to_row_max', details_miss[1])
        self.assertNotIn('rel_off_override', details_miss[1])
        # state stays green (no fake off); rollup still PASS if all green
        self.assertEqual(dec_miss.verdict, 'PASS')

    def test_09_all_valid_brightness_zero_no_div_zero(self):
        dec, details = _rollup(
            [
                _det('off', mean_vmax=0.0, v90=0.0),
                _det('off', mean_vmax=0.0, v90=0.0),
                _det('off', mean_vmax=0.0, v90=0.0),
            ]
        )
        self.assertEqual(dec.verdict, 'FAIL')
        for d in details:
            self.assertNotIn('rel_to_row_max', d)
            self.assertNotIn('rel_off_override', d)

    def test_10_invalid_or_missing_does_not_pollute_row_max(self):
        # NaN / inf / None must not become row_max or crash
        dec, details = _rollup(
            [
                ('green', {'led_state': 'green', 'mean_vmax': float('nan')}),
                _det('red', mean_vmax=50.0, v90=60.0),
                ('UNKNOWN', {'reason': 'yolo_miss_led_slot', 'slot_i': 2}),
            ]
        )
        # row_max should be 50 from red only; green NaN skipped
        self.assertNotIn('rel_to_row_max', details[0])
        self.assertNotIn('rel_to_row_max', details[2])
        self.assertEqual(details[1].get('rel_to_row_max'), 1.0)
        # red present → component FAIL; miss slot still UNKNOWN (not off)
        self.assertEqual(dec.verdict, 'FAIL')
        self.assertEqual(dec.measurements['leds'][0], 'UNKNOWN')
        self.assertNotIn('rel_off_override', details[2])
        # inf sibling must not poison
        _, details_inf = _rollup(
            [
                ('green', {'led_state': 'green', 'mean_vmax': float('inf')}),
                _det('green', mean_vmax=100.0, v90=120.0),
                _det('green', mean_vmax=90.0, v90=110.0),
            ]
        )
        self.assertNotIn('rel_to_row_max', details_inf[0])
        self.assertEqual(details_inf[1].get('rel_to_row_max'), 1.0)

    def test_11_both_slot_mappings(self):
        triple = [
            _det('green', mean_vmax=180.0, v90=255.0),
            _det('red', mean_vmax=190.0, v90=255.0),
            _miss(2),
        ]
        dec_neg, _ = _rollup(triple, order=ORDER_NEG_Y)
        # image [G,R,U] → model leds[order]: [U, R, G]
        self.assertEqual(dec_neg.measurements['leds'], ['UNKNOWN', 'red', 'green'])
        slots_neg = dec_neg.measurements['led_slots']
        self.assertEqual(
            [(s['image_slot'], s['model_index'], s['state']) for s in slots_neg],
            [(0, 2, 'green'), (1, 1, 'red'), (2, 0, 'UNKNOWN')],
        )

        dec_pos, _ = _rollup(triple, order=ORDER_POS_Y)
        # image [G,R,U] → model [G, R, U]
        self.assertEqual(dec_pos.measurements['leds'], ['green', 'red', 'UNKNOWN'])
        slots_pos = dec_pos.measurements['led_slots']
        self.assertEqual(
            [(s['image_slot'], s['model_index'], s['state']) for s in slots_pos],
            [(0, 0, 'green'), (1, 1, 'red'), (2, 2, 'UNKNOWN')],
        )
        # confirmed red → FAIL; mapping/miss protections unchanged
        self.assertEqual(dec_neg.verdict, 'FAIL')
        self.assertEqual(dec_pos.verdict, 'FAIL')

    def test_12_yellow_or_off_with_miss_is_fail(self):
        for bad in ('yellow', 'off'):
            with self.subTest(bad=bad):
                dec, details = _rollup(
                    [
                        _det('green', mean_vmax=180.0, v90=255.0),
                        _det(bad, mean_vmax=40.0 if bad == 'off' else 190.0, v90=255.0),
                        _miss(2),
                    ]
                )
                self.assertEqual(dec.verdict, 'FAIL')
                self.assertIn(bad, dec.measurements['leds'])
                self.assertIn('UNKNOWN', dec.measurements['leds'])
                self.assertNotIn('rel_off_override', details[2])
                self.assertEqual(details[2].get('reason'), 'yolo_miss_led_slot')

    def test_historical_counterexample_event_shape(self):
        """Deterministic reconstruction of c2_20261011_s0_unk_B_U_leak_0007 LED rollup."""
        dec, details = _rollup(
            [
                _det('green', mean_vmax=191.6, v90=255.0, extra={'bgr': [0.1, 254.5, 0.1]}),
                _det('red', mean_vmax=197.5, v90=255.0, extra={'bgr': [0.1, 0.1, 254.5]}),
                _miss(2),
            ],
            order=ORDER_NEG_Y,
        )
        self.assertEqual(dec.verdict, 'FAIL')
        self.assertEqual(dec.measurements['leds'], ['UNKNOWN', 'red', 'green'])
        self.assertNotIn('rel_off_override', details[2])
        self.assertNotIn('rel_to_row_max', details[2])
        self.assertTrue(math.isfinite(details[0]['rel_to_row_max']))
        self.assertTrue(math.isfinite(details[1]['rel_to_row_max']))


if __name__ == '__main__':
    unittest.main()
