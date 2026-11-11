#!/usr/bin/env python3
"""MVP-R2 offline tests: settle/capture helpers + defer_success handoff.

No Gazebo. Requires ROS Humble Python libs + numpy/opencv for convert/save.
Run after: source /opt/ros/humble/setup.bash
  PYTHONPATH=ros2_ws/src/inspection_mission:$PYTHONPATH \\
    python3 -m unittest tests.test_mvp_r2_station_capture -v
"""

from __future__ import annotations

import json
import sys
import tempfile
import threading
import time
import unittest
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'ros2_ws' / 'src' / 'inspection_mission'))

from inspection_mission.mission_runner import MissionRunner  # noqa: E402
from inspection_mission.mission_state import (  # noqa: E402
    ErrorCode,
    ExecutionStatus,
    MissionRuntimeState,
    Phase,
    STATION_A,
)
from inspection_mission.station_capture import (  # noqa: E402
    CameraFrameBuffer,
    CaptureResult,
    OdomSettleMonitor,
    capture_station_frame,
    image_msg_to_bgr_array,
    save_bgr_png,
    save_capture_meta,
)


class FakeGoalHandle:
    def __init__(self, accepted: bool, nav: 'FakeNavAdapter') -> None:
        self.accepted = accepted
        self._nav = nav

    def cancel_goal_async(self):
        return self._nav.cancel_goal_async(self)

    def get_result_async(self):
        return self._nav.get_result_async(self)


class FakeNavAdapter:
    def __init__(self) -> None:
        self.server_available = True
        self.send_future: Future | None = None
        self.cancel_future: Future | None = None
        self.result_future: Future | None = None

    def wait_for_server(self, timeout_sec: float, *, through_poses: bool = False) -> bool:
        return bool(self.server_available)

    def send_goal_async(self, pose: PoseStamped, feedback_callback=None):
        self.send_future = Future()
        return self.send_future

    def cancel_goal_async(self, goal_handle):
        self.cancel_future = Future()
        return self.cancel_future

    def get_result_async(self, goal_handle):
        if self.result_future is None:
            self.result_future = Future()
        return self.result_future

    def complete_send(self, accepted: bool = True) -> FakeGoalHandle:
        assert self.send_future is not None
        handle = FakeGoalHandle(accepted, self)
        self.send_future.set_result(handle)
        return handle

    def complete_result(self, status: int) -> None:
        assert self.result_future is not None
        self.result_future.set_result(SimpleNamespace(status=status, result=None))


def _make_rgb8(width=4, height=3, stamp_sec=1.0, frame_id='camera_link') -> Image:
    msg = Image()
    msg.header.stamp.sec = int(stamp_sec)
    msg.header.stamp.nanosec = int((stamp_sec % 1) * 1e9)
    msg.header.frame_id = frame_id
    msg.height = height
    msg.width = width
    msg.encoding = 'rgb8'
    msg.step = width * 3
    # simple gradient
    data = bytearray()
    for y in range(height):
        for x in range(width):
            data.extend([x * 40 % 256, y * 50 % 256, 128])
    msg.data = bytes(data)
    return msg


class FakeCamera:
    def __init__(self) -> None:
        self._seq = 0
        self._latest = None
        self._lock = threading.Lock()

    def push(self, msg: Image) -> None:
        with self._lock:
            self._latest = msg
            self._seq += 1

    def watermark(self):
        with self._lock:
            stamp = 0.0
            if self._latest is not None:
                from inspection_mission.station_capture import stamp_msg_to_sec
                stamp = stamp_msg_to_sec(self._latest.header.stamp)
            return self._seq, stamp

    def wait_new_frame(self, *, after_seq, after_stamp_sec, timeout_sec, clock=time.monotonic,
                       should_abort=None, sleep_fn=time.sleep):
        # Reuse production logic via temporary CameraFrameBuffer-like API
        buf = CameraFrameBuffer.__new__(CameraFrameBuffer)
        buf._lock = self._lock
        buf._latest = None
        buf._seq = 0

        def snapshot():
            with self._lock:
                return self._seq, self._latest

        buf.snapshot = snapshot  # type: ignore
        return CameraFrameBuffer.wait_new_frame(
            buf,
            after_seq=after_seq,
            after_stamp_sec=after_stamp_sec,
            timeout_sec=timeout_sec,
            clock=clock,
            should_abort=should_abort,
            sleep_fn=sleep_fn,
        )


class FakeSettle:
    def __init__(self, err: str = '', on_settled=None) -> None:
        self.err = err
        self.calls = 0
        self.on_settled = on_settled

    def wait_settled(self, **kwargs):
        self.calls += 1
        if kwargs.get('should_abort') and kwargs['should_abort']():
            return 'canceled', None
        if not self.err and self.on_settled:
            self.on_settled()
        return self.err, (None if self.err else 1.0)


class TestImageConvertSave(unittest.TestCase):
    def test_rgb8_to_bgr_and_png(self) -> None:
        msg = _make_rgb8()
        bgr = image_msg_to_bgr_array(msg)
        self.assertEqual(bgr.shape, (3, 4, 3))
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 't.png'
            save_bgr_png(path, bgr)
            self.assertTrue(path.is_file())
            self.assertGreater(path.stat().st_size, 0)

    def test_unsupported_encoding(self) -> None:
        msg = _make_rgb8()
        msg.encoding = 'yuv422'
        with self.assertRaises(ValueError):
            image_msg_to_bgr_array(msg)


class TestCaptureOrchestration(unittest.TestCase):
    def test_happy_path_saves_png_json(self) -> None:
        cam = FakeCamera()
        # Pre-settle frame becomes post-settle watermark; first sleep pushes capture frame.
        cam.push(_make_rgb8(stamp_sec=1.0))
        settle = FakeSettle()
        pushed = {'done': False}

        def sleep_fn(_):
            if not pushed['done']:
                cam.push(_make_rgb8(stamp_sec=2.5, frame_id='camera_optical_frame'))
                pushed['done'] = True

        phases = []
        with tempfile.TemporaryDirectory() as td:
            result = capture_station_frame(
                camera=cam,
                settle=settle,
                output_dir=Path(td),
                mission_id='m_r2_1',
                settle_timeout_sec=1.0,
                settle_sec=0.0,
                frame_timeout_sec=1.0,
                sleep_fn=sleep_fn,
                on_phase=lambda p, m: phases.append(p),
                nav_goal={'station_id': 'A'},
            )
            self.assertTrue(result.ok, result.message)
            self.assertEqual(result.error_code, ErrorCode.NONE)
            self.assertEqual(result.station_id, STATION_A)
            self.assertEqual(result.frame_id, 'camera_optical_frame')
            self.assertTrue(Path(result.screenshot_path).is_file())
            self.assertTrue(Path(result.meta_path).is_file())
            meta = json.loads(Path(result.meta_path).read_text(encoding='utf-8'))
            self.assertEqual(meta['mission_id'], 'm_r2_1')
            self.assertEqual(meta['station_id'], 'A')
            self.assertIn('diagnostics', meta)
            self.assertEqual(meta['diagnostics']['nav_goal']['station_id'], 'A')
            self.assertIn(Phase.SETTLING.value, phases)
            self.assertIn(Phase.CAPTURING.value, phases)

    def test_no_new_frame(self) -> None:
        cam = FakeCamera()
        cam.push(_make_rgb8(stamp_sec=1.0))
        settle = FakeSettle()
        t = {'v': 0.0}

        def clock():
            t['v'] += 1.0
            return t['v']

        with tempfile.TemporaryDirectory() as td:
            result = capture_station_frame(
                camera=cam,
                settle=settle,
                output_dir=Path(td),
                mission_id='m_r2_nof',
                settle_timeout_sec=1.0,
                settle_sec=0.0,
                frame_timeout_sec=0.5,
                clock=clock,
                sleep_fn=lambda _: None,
            )
            self.assertFalse(result.ok)
            self.assertEqual(result.error_code, ErrorCode.NO_NEW_FRAME)

    def test_settle_timeout(self) -> None:
        cam = FakeCamera()
        settle = FakeSettle(err=ErrorCode.SETTLE_TIMEOUT)
        with tempfile.TemporaryDirectory() as td:
            result = capture_station_frame(
                camera=cam,
                settle=settle,
                output_dir=Path(td),
                mission_id='m_r2_settle',
            )
            self.assertFalse(result.ok)
            self.assertEqual(result.error_code, ErrorCode.SETTLE_TIMEOUT)


class TestDeferSuccessHandoff(unittest.TestCase):
    def test_nav_success_defers_terminal(self) -> None:
        state = MissionRuntimeState()
        gen = state.begin('m_defer')
        nav = FakeNavAdapter()
        arrived = threading.Event()
        finished = []
        runner = MissionRunner(
            state=state,
            nav=nav,
            pose=PoseStamped(),
            mission_id='m_defer',
            generation=gen,
            defer_success=True,
            nav_arrived_event=arrived,
            on_finished=finished.append,
            result_timeout_sec=30.0,
        )
        runner.start()
        nav.complete_send(True)
        nav.complete_result(GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(arrived.wait(1.0))
        self.assertTrue(runner.nav_arrived)
        self.assertEqual(finished, [])
        self.assertTrue(state.is_owner('m_defer', gen))
        self.assertFalse(state.terminal_published)

        # Runtime publishes terminal after capture
        terminal = state.try_publish_terminal(
            'm_defer',
            ExecutionStatus.COMPLETED.value,
            ErrorCode.NONE,
            'captured',
            release=True,
            generation=gen,
            screenshot_path='/tmp/a.png',
            image_stamp='2.500000',
            frame_id='camera_optical_frame',
            station_id=STATION_A,
        )
        self.assertIsNotNone(terminal)
        runner.mark_externally_finished()
        self.assertEqual(terminal.screenshot_path, '/tmp/a.png')
        self.assertIsNone(state.active_mission_id)

    def test_defer_false_still_completes_on_nav(self) -> None:
        """R1 semantics preserved when defer_success=False."""
        state = MissionRuntimeState()
        gen = state.begin('m_r1')
        nav = FakeNavAdapter()
        finished = []
        runner = MissionRunner(
            state=state,
            nav=nav,
            pose=PoseStamped(),
            mission_id='m_r1',
            generation=gen,
            defer_success=False,
            on_finished=finished.append,
        )
        runner.start()
        nav.complete_send(True)
        nav.complete_result(GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(len(finished), 1)
        self.assertEqual(finished[0].execution_status, ExecutionStatus.COMPLETED.value)


if __name__ == '__main__':
    unittest.main()
