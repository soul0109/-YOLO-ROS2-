#!/usr/bin/env python3
"""MVP-R7 offline: route plan + multi-station archive append."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'ros2_ws' / 'src' / 'inspection_mission'))

from inspection_mission.mission_archive import MissionArchive  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from rclpy.task import Future  # noqa: E402

from inspection_mission.map_pose import pose_from_route_values  # noqa: E402
from inspection_mission.mission_runner import MissionRunner  # noqa: E402
from inspection_mission.route_plan import (  # noqa: E402
    ROUTE_A,
    ROUTE_ABC,
    aggregate_inspection_status,
    stations_for_route,
    transfer_through_world_poses,
    transfer_vias_before_station,
)
from inspection_mission.mission_state import MissionRuntimeState, ErrorCode  # noqa: E402


class _FakeThroughNav:
    def __init__(self) -> None:
        self.wait_through: bool | None = None
        self.send_calls = 0
        self.last_poses: list[PoseStamped] | None = None
        self.send_future: Future | None = None

    def wait_for_server(self, timeout_sec: float, *, through_poses: bool = False) -> bool:
        self.wait_through = bool(through_poses)
        return True

    def send_goal_async(self, pose, feedback_callback=None):
        raise AssertionError('B→C must not use NavigateToPose')

    def send_through_poses_async(self, poses, feedback_callback=None):
        self.send_calls += 1
        self.last_poses = list(poses)
        self.send_future = Future()
        return self.send_future

    def cancel_goal_async(self, goal_handle):
        return Future()

    def get_result_async(self, goal_handle):
        return Future()


class TestRoutePlan(unittest.TestCase):
    def test_stations_and_vias(self) -> None:
        self.assertEqual(stations_for_route(ROUTE_A), ['A'])
        self.assertEqual(stations_for_route(ROUTE_ABC), ['A', 'B', 'C'])
        route = {
            'inspection_order': ['A', 'B', 'C'],
            'stations': {
                'A': [3.15, 1.75, -1.57],
                'B': [8.45, 1.75, -1.57],
                'C': [4.95, 4.25, 1.57],
            },
            'transfers': {
                'B_to_C': {'via': [[8.6, 2.1, 1.57], [5.25, 3.0, 1.57]]},
            },
        }
        vias = transfer_vias_before_station(route, from_station='B', to_station='C')
        self.assertEqual(len(vias), 2)
        self.assertEqual(vias[0][0], 8.6)
        through = transfer_through_world_poses(
            route, from_station='B', to_station='C',
        )
        self.assertEqual(len(through), 3)
        self.assertEqual(through[0], [8.6, 2.1, 1.57])
        self.assertEqual(through[1], [5.25, 3.0, 1.57])
        self.assertEqual(through[2], [4.95, 4.25, 1.57])
        # A/B transfers without vias → empty → NavigateToPose
        self.assertEqual(
            transfer_through_world_poses(route, from_station='A', to_station='B'),
            [],
        )

    def test_b_to_c_through_poses_wiring(self) -> None:
        """B→C sends one ordered ThroughPoses list; one stamp; ends with C."""
        route = {
            'stations': {
                'B': [8.45, 1.75, -1.57079632679],
                'C': [5.25, 4.25, 1.57079632679],
            },
            'transfers': {
                'B_to_C': {
                    'via': [
                        [8.60, 2.10, 1.57079632679],
                        [8.45, 3.00, 1.57079632679],
                        [5.25, 3.00, 1.57079632679],
                    ],
                },
            },
        }
        origin = [-0.0134, 0.0519, 0.0]
        world = transfer_through_world_poses(
            route, from_station='B', to_station='C',
        )
        self.assertEqual(len(world), 4)
        self.assertEqual(world[-1], [5.25, 4.25, 1.57079632679])

        stamp = Time(sec=42, nanosec=7)
        poses = [
            pose_from_route_values(vals, origin, stamp=stamp) for vals in world
        ]
        self.assertEqual(len(poses), 4)
        self.assertTrue(all(p.header.stamp.sec == 42 for p in poses))
        self.assertTrue(all(p.header.stamp.nanosec == 7 for p in poses))
        # Final map pose equals converting C once with same stamp/origin.
        c_only = pose_from_route_values(route['stations']['C'], origin, stamp=stamp)
        self.assertAlmostEqual(poses[-1].pose.position.x, c_only.pose.position.x)
        self.assertAlmostEqual(poses[-1].pose.position.y, c_only.pose.position.y)

        nav = _FakeThroughNav()
        state = MissionRuntimeState()
        mid = 'wire_bc'
        gen = state.begin(mid)
        runner = MissionRunner(
            state=state,
            nav=nav,
            poses=poses,
            mission_id=mid,
            generation=gen,
            station_id='C',
            defer_success=True,
        )
        runner.start()
        self.assertTrue(nav.wait_through)
        self.assertEqual(nav.send_calls, 1)
        self.assertIsNotNone(nav.last_poses)
        self.assertEqual(len(nav.last_poses), 4)
        self.assertAlmostEqual(
            nav.last_poses[-1].pose.position.x, c_only.pose.position.x,
        )
        # Ordered: first via then C last (x decreases toward C map frame).
        xs = [p.pose.position.x for p in nav.last_poses]
        self.assertGreater(xs[0], xs[-1])

    def test_aggregate_fail_beats_unknown(self) -> None:
        self.assertEqual(
            aggregate_inspection_status(['PASS', 'UNKNOWN', 'FAIL']), 'FAIL',
        )
        self.assertEqual(aggregate_inspection_status(['PASS', 'UNKNOWN']), 'UNKNOWN')
        self.assertEqual(aggregate_inspection_status(['PASS', 'PASS']), 'PASS')
        self.assertEqual(aggregate_inspection_status([]), '')

    def test_validate_abc_route(self) -> None:
        st = MissionRuntimeState()
        ok, code, _ = st.validate_goal('m1', ROUTE_ABC)
        self.assertTrue(ok)
        ok2, code2, _ = st.validate_goal('m2', 'nope')
        self.assertFalse(ok2)
        self.assertEqual(code2, ErrorCode.UNSUPPORTED_ROUTE)


class TestMultiStationArchive(unittest.TestCase):
    def test_begin_persist_two_stations_finalize(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            db = Path(td) / 'm.sqlite3'
            arch = MissionArchive(db)
            self.assertTrue(arch.begin_mission(
                mission_id='abc1', route_id=ROUTE_ABC, started_at='t0',
            ).ok)
            self.assertTrue(arch.persist_station(
                mission_id='abc1',
                station={
                    'station_id': 'A',
                    'inspection_status': 'FAIL',
                    'screenshot_path': '/a.png',
                    'objects': {'leak': {'verdict': 'PASS'}},
                },
            ).ok)
            self.assertTrue(arch.persist_station(
                mission_id='abc1',
                station={
                    'station_id': 'B',
                    'inspection_status': 'PASS',
                    'screenshot_path': '/b.png',
                },
            ).ok)
            dup = arch.persist_station(
                mission_id='abc1',
                station={'station_id': 'A', 'inspection_status': 'PASS'},
            )
            self.assertFalse(dup.ok)
            self.assertEqual(dup.error_code, 'archive_duplicate_station')
            self.assertTrue(arch.finalize_mission(
                mission_id='abc1',
                execution_status='COMPLETED',
                error_code='NONE',
                message='done',
                ended_at='t1',
            ).ok)
            stations = arch.get_stations('abc1')
            self.assertEqual([s['station_id'] for s in stations], ['A', 'B'])
            self.assertEqual(arch.get_mission('abc1')['execution_status'], 'COMPLETED')
            arch.close()


if __name__ == '__main__':
    unittest.main()
