#!/usr/bin/env python3
"""V2-01: station / cross-layer inspection rollup (FAIL > UNKNOWN > PASS).

Uses real functions via PYTHONPATH. No YOLO weights / Gazebo.
Run:
  PYTHONDONTWRITEBYTECODE=1 python3 -m unittest tests.test_v2_01_station_rollup -v
"""

from __future__ import annotations

import itertools
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'ros2_ws' / 'src' / 'inspection_mission'))
sys.path.insert(0, str(REPO / 'scripts'))

from inspection_mission.station_inspect import _station_rollup  # noqa: E402
from inspection_mission.route_plan import aggregate_inspection_status  # noqa: E402
from roi_decision import _rollup_leds  # noqa: E402


RULES = {
    'schema': 'roi_rules_v1',
    'led': {'rel_to_row_max': 0.40},
    'inspection': {'led_pass_color': 'green'},
}
ORDER_NEG_Y = [2, 1, 0]


class TestStationRollupPriority(unittest.TestCase):
    def test_all_pass(self) -> None:
        self.assertEqual(_station_rollup(['PASS', 'PASS', 'PASS']), 'PASS')

    def test_unknown_without_fail(self) -> None:
        self.assertEqual(_station_rollup(['UNKNOWN', 'PASS', 'PASS']), 'UNKNOWN')
        self.assertEqual(_station_rollup(['PASS', 'UNKNOWN', 'PASS']), 'UNKNOWN')
        self.assertEqual(_station_rollup(['PASS', 'PASS', 'UNKNOWN']), 'UNKNOWN')

    def test_fail_unknown_pass_all_permutations(self) -> None:
        for perm in itertools.permutations(['FAIL', 'UNKNOWN', 'PASS']):
            with self.subTest(perm=perm):
                self.assertEqual(_station_rollup(list(perm)), 'FAIL')

    def test_empty_list_unknown(self) -> None:
        self.assertEqual(_station_rollup([]), 'UNKNOWN')

    def test_illegal_or_blank_without_fail_is_unknown(self) -> None:
        self.assertEqual(_station_rollup(['PASS', '', 'PASS']), 'UNKNOWN')
        self.assertEqual(_station_rollup(['PASS', 'bogus', 'PASS']), 'UNKNOWN')
        self.assertEqual(_station_rollup([None]), 'UNKNOWN')  # type: ignore[list-item]
        self.assertEqual(_station_rollup(['UNKNOWN', 'nope']), 'UNKNOWN')

    def test_illegal_does_not_mask_fail(self) -> None:
        self.assertEqual(_station_rollup(['FAIL', '', 'PASS']), 'FAIL')
        self.assertEqual(_station_rollup(['bogus', 'FAIL', 'UNKNOWN']), 'FAIL')


class TestBStationCounterexample(unittest.TestCase):
    def test_leds_unknown_panel_pass_leak_fail(self) -> None:
        """Archive shape: leds=UNKNOWN, panel=PASS, leak=FAIL → station FAIL."""
        self.assertEqual(
            _station_rollup(['UNKNOWN', 'PASS', 'FAIL']), 'FAIL',
        )

    def test_only_b_fail_keeps_mission_fail(self) -> None:
        """Mission must not rely on A/C faults to surface B's known FAIL."""
        station_b = _station_rollup(['UNKNOWN', 'PASS', 'FAIL'])
        self.assertEqual(station_b, 'FAIL')
        self.assertEqual(
            aggregate_inspection_status(['PASS', station_b, 'PASS']), 'FAIL',
        )
        self.assertEqual(
            aggregate_inspection_status(['PASS', 'PASS', 'PASS']), 'PASS',
        )


class TestLedComponentMixedFault(unittest.TestCase):
    def test_green_red_miss_component_fail_keeps_miss(self) -> None:
        states = ['green', 'red', 'UNKNOWN']
        details = [
            {'led_state': 'green', 'mean_vmax': 191.6, 'v90': 255.0},
            {'led_state': 'red', 'mean_vmax': 197.5, 'v90': 255.0},
            {'reason': 'yolo_miss_led_slot', 'slot_i': 2},
        ]
        dec = _rollup_leds(states, details, ORDER_NEG_Y, RULES, 'detector_box')
        self.assertEqual(dec.verdict, 'FAIL')
        self.assertEqual(dec.measurements['leds'], ['UNKNOWN', 'red', 'green'])
        self.assertIn('incomplete', dec.reason)
        self.assertNotIn('rel_off_override', details[2])
        self.assertNotIn('rel_to_row_max', details[2])


if __name__ == '__main__':
    unittest.main()
