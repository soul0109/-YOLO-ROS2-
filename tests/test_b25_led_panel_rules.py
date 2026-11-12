#!/usr/bin/env python3
"""B2.5 unit tests: distance-robust LED association + panel UNKNOWN uncertain.

No Gazebo. Does not tune on B2 formal test frames.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'scripts'))

from hook1_yolo_roi_e1 import associate_led_slots  # noqa: E402
from roi_judge_d1 import classify_panel, load_rules  # noqa: E402

V1 = REPO / 'ros2_ws' / 'src' / 'vision_perception' / 'config' / 'roi_rules.yaml'
V2 = REPO / 'ros2_ws' / 'src' / 'vision_perception' / 'config' / 'roi_rules_v2.yaml'


def _box(x0, y0, x1, y1, conf=0.9):
    return {'class_name': 'indicator_light', 'bbox_xyxy': [x0, y0, x1, y1], 'confidence': conf}


class TestLedAssociation(unittest.TestCase):
    def setUp(self) -> None:
        self.rules = yaml.safe_load(V2.read_text(encoding='utf-8'))

    def test_xsort_assigns_left_to_right_away_from_1p2_slots(self):
        # Simulate 1.0m: LEDs lower/larger, not overlapping 1.2m ref slots.
        lights = [
            _box(150, 60, 210, 120, 0.95),
            _box(280, 62, 340, 118, 0.93),
            _box(410, 58, 470, 122, 0.91),
        ]
        slots = associate_led_slots(lights, self.rules)
        self.assertEqual([s['bbox_xyxy'] for s in slots], [d['bbox_xyxy'] for d in lights])

    def test_1p2_calibrated_boxes_still_map(self):
        # Boxes near calibrated 1.2m slots must still assign 0,1,2.
        lights = [
            _box(182, 2, 229, 45, 0.99),
            _box(297, 2, 342, 45, 0.98),
            _box(410, 2, 457, 45, 0.97),
        ]
        slots = associate_led_slots(lights, self.rules)
        self.assertIsNotNone(slots[0])
        self.assertIsNotNone(slots[1])
        self.assertIsNotNone(slots[2])
        self.assertLess(slots[0]['bbox_xyxy'][0], slots[1]['bbox_xyxy'][0])
        self.assertLess(slots[1]['bbox_xyxy'][0], slots[2]['bbox_xyxy'][0])


class TestPanelUncertain(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = load_rules(V2)['panel']

    def test_normal_lit_and_dark_still_work(self):
        # Typical lit panel: high V but not near-white bloom, with chroma.
        lit = np.zeros((40, 80, 3), dtype=np.uint8)
        lit[:] = (180, 200, 40)  # BGR cyan-teal, V~200, sat high
        st, d = classify_panel(lit, self.cfg)
        self.assertEqual(st, 'lit', d)

        dark = np.zeros((40, 80, 3), dtype=np.uint8)
        dark[:] = (3, 3, 3)
        st, d = classify_panel(dark, self.cfg)
        self.assertEqual(st, 'dark', d)

    def test_overexpose_returns_unknown(self):
        bloom = np.full((40, 80, 3), 255, dtype=np.uint8)
        st, d = classify_panel(bloom, self.cfg)
        self.assertEqual(st, 'UNKNOWN', d)
        self.assertEqual(d.get('reason'), 'U_panel_overexpose')

    def test_flat_occlusion_returns_unknown(self):
        occ = np.full((40, 80, 3), 20, dtype=np.uint8)
        st, d = classify_panel(occ, self.cfg)
        self.assertEqual(st, 'UNKNOWN', d)
        self.assertEqual(d.get('reason'), 'U_panel_occluded')

    def test_heavy_crop_returns_unknown(self):
        patch = np.full((40, 80, 3), 30, dtype=np.uint8)
        st, d = classify_panel(patch, self.cfg, crop_outside_frac=0.4)
        self.assertEqual(st, 'UNKNOWN', d)
        self.assertEqual(d.get('reason'), 'U_panel_cropped')

    def test_v1_rules_without_uncertain_keep_legacy(self):
        cfg = load_rules(V1)['panel']
        bloom = np.full((40, 80, 3), 255, dtype=np.uint8)
        st, _ = classify_panel(bloom, cfg)
        # v1 has no uncertain block → still lit by v_mean threshold
        self.assertEqual(st, 'lit')


if __name__ == '__main__':
    unittest.main()
