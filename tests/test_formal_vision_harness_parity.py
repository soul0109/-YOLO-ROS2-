#!/usr/bin/env python3
"""Parity: Runtime inspect_station_image vs formal harness on verify_abc captures.

Also proves sanity_gate rejects the B2 INVALID harness report.
Offline only — no Gazebo/Nav2/Web.
"""

from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SCRIPTS = REPO / 'scripts'
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(REPO / 'ros2_ws' / 'src' / 'inspection_mission'))

from formal_vision_test_run import (  # noqa: E402
    predict_via_runtime,
    sanity_gate,
    sha256_file,
    states_from_inspect_payload,
)
from inspection_mission.station_inspect import inspect_station_image  # noqa: E402

CAPTURE_DIR = REPO / 'bags' / 'mission_captures' / 'verify_abc_20261111_155710'
WEIGHTS = REPO / 'models' / 'weights' / 'inspection_train1_20261012.pt'
RULES = REPO / 'ros2_ws' / 'src' / 'vision_perception' / 'config' / 'roi_rules.yaml'
OLD_REPORT = REPO / 'log' / 'b2_vision_test_20261111' / 'formal_test_run' / 'report.json'
CONF = 0.25
EXPECT_WEIGHT_SHA = 'a1d2507a6fe5fc9bae12e85f28fc3e876477b010b88640bb251d198ee023fed4'
EXPECT_ROI_SHA = '1e1f1fd289f49b34d23142882336f4746b4a31be6c907f5a8a66f80665434265'


def _station_pngs() -> list[tuple[str, Path, Path]]:
    """Return (station_id, png, station_result_json)."""
    out = []
    for sid in ('A', 'B', 'C'):
        results = sorted(CAPTURE_DIR.glob(f'{sid}_*_station_result.json'))
        if not results:
            raise FileNotFoundError(f'no station_result for {sid} under {CAPTURE_DIR}')
        result_path = results[0]
        payload = json.loads(result_path.read_text(encoding='utf-8'))
        png = Path(payload['screenshot_path'])
        if not png.is_file():
            # fallback next to result
            png = result_path.with_name(result_path.name.replace('_station_result.json', '.png'))
        out.append((sid, png, result_path))
    return out


class TestFormalVisionHarnessParity(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        assert WEIGHTS.is_file(), f'missing weights {WEIGHTS}'
        assert RULES.is_file(), f'missing rules {RULES}'
        assert CAPTURE_DIR.is_dir(), f'missing captures {CAPTURE_DIR}'
        wsha = sha256_file(WEIGHTS)
        rsha = sha256_file(RULES)
        if wsha != EXPECT_WEIGHT_SHA:
            raise unittest.SkipTest(
                f'weights sha changed: {wsha} (check conf/weights; do not retune)'
            )
        if rsha != EXPECT_ROI_SHA:
            raise unittest.SkipTest(
                f'ROI sha changed: {rsha} (check rules version; do not retune)'
            )

    def test_sanity_gate_blocks_old_b2_report(self) -> None:
        self.assertTrue(OLD_REPORT.is_file(), f'missing old report {OLD_REPORT}')
        report = json.loads(OLD_REPORT.read_text(encoding='utf-8'))
        gate = sanity_gate(report)
        self.assertFalse(gate['ok'], gate)
        self.assertEqual(gate['report_validity'], 'INVALID_REPORT')
        joined = ' | '.join(gate['issues'])
        self.assertTrue(
            any('correct==0' in i for i in gate['issues'])
            or 'non-UNKNOWN accuracy' in joined
            or 'all predictions UNKNOWN' in joined,
            gate['issues'],
        )

    def test_runtime_vs_harness_parity_on_verify_abc(self) -> None:
        for sid, png, result_path in _station_pngs():
            with self.subTest(station=sid):
                archived = json.loads(result_path.read_text(encoding='utf-8'))
                yaw = float(archived.get('robot_yaw_used_for_led_map', -1.5708))
                stamp = str(archived.get('image_stamp') or '0')

                # Direct Runtime call (temp copy inside predict_via_runtime)
                harness_states, harness_payload = predict_via_runtime(
                    png=png,
                    station_id=sid,
                    robot_yaw=yaw,
                    weights=WEIGHTS,
                    rules_path=RULES,
                    conf=CONF,
                    image_stamp=stamp,
                    mission_id=f'parity_{sid}',
                )

                # Second Runtime call for explicit compare
                import shutil
                import tempfile

                with tempfile.TemporaryDirectory() as td:
                    tmp = Path(td) / 'shot.png'
                    shutil.copy2(png, tmp)
                    again = inspect_station_image(
                        screenshot_path=tmp,
                        mission_id=f'parity2_{sid}',
                        station_id=sid,
                        image_stamp=stamp,
                        robot_yaw=yaw,
                        weights=WEIGHTS,
                        rules_path=RULES,
                        conf=CONF,
                        write_annotated=False,
                    )
                self.assertTrue(again.ok, again.message)
                again_states = states_from_inspect_payload(again.payload)

                self.assertEqual(
                    harness_states['leds'],
                    again_states['leds'],
                    msg=f'{sid} leds harness vs inspect_station',
                )
                self.assertEqual(
                    harness_states['panel'],
                    again_states['panel'],
                    msg=f'{sid} panel harness vs inspect_station',
                )
                self.assertEqual(
                    harness_states['leak'],
                    again_states['leak'],
                    msg=f'{sid} leak harness vs inspect_station',
                )

                # Also match archived mission capture results (same weights/ROI/conf)
                arch_states = states_from_inspect_payload(archived)
                self.assertEqual(
                    harness_states['leds'],
                    arch_states['leds'],
                    msg=(
                        f'{sid} leds vs archived station_result — '
                        'if fail, check conf/weights/rules hash first'
                    ),
                )
                self.assertEqual(
                    harness_states['panel'],
                    arch_states['panel'],
                    msg=f'{sid} panel vs archived',
                )
                self.assertEqual(
                    harness_states['leak'],
                    arch_states['leak'],
                    msg=f'{sid} leak vs archived',
                )
                self.assertIsNotNone(harness_states.get('slot_contract'))
                self.assertEqual(
                    (harness_payload.get('objects') or {}).get('leds', {}).get('slot_contract'),
                    archived['objects']['leds']['slot_contract'],
                )


if __name__ == '__main__':
    unittest.main()
