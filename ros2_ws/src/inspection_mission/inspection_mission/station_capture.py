"""MVP-R2: settle + new camera frame + PNG/JSON save (A-station only)."""

from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Optional

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

from inspection_mission.mission_state import ErrorCode, STATION_A


@dataclass
class CaptureResult:
    ok: bool
    error_code: str = ErrorCode.NONE
    message: str = ''
    screenshot_path: str = ''
    image_stamp: str = ''
    frame_id: str = ''
    station_id: str = STATION_A
    meta_path: str = ''
    diagnostics: dict = field(default_factory=dict)


def stamp_msg_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_quat(z: float, w: float) -> float:
    return math.atan2(2.0 * z * w, 1.0 - 2.0 * z * z)


def pose_dict_from_odom(msg: Odometry, *, frame_label: str) -> dict:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return {
        'frame': frame_label,
        'x': float(p.x),
        'y': float(p.y),
        'z': float(p.z),
        'yaw': yaw_from_quat(float(q.z), float(q.w)),
        'stamp': stamp_msg_to_sec(msg.header.stamp),
        'child_frame_id': msg.child_frame_id or '',
        'header_frame_id': msg.header.frame_id or '',
    }


def image_msg_to_bgr_array(msg: Image):
    """Convert sensor_msgs/Image to HxWx3 uint8 BGR without cv_bridge."""
    import numpy as np

    if msg.width <= 0 or msg.height <= 0 or not msg.data:
        raise ValueError('empty image')
    encoding = (msg.encoding or '').lower()
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if encoding in ('rgb8', 'bgr8'):
        if step < w * 3:
            raise ValueError(f'bad step={step} for {encoding}')
        arr = np.ndarray((h, w, 3), dtype=np.uint8, buffer=buf, strides=(step, 3, 1)).copy()
        if encoding == 'rgb8':
            return arr[:, :, ::-1].copy()
        return arr
    if encoding in ('rgba8', 'bgra8'):
        if step < w * 4:
            raise ValueError(f'bad step={step} for {encoding}')
        arr = np.ndarray((h, w, 4), dtype=np.uint8, buffer=buf, strides=(step, 4, 1)).copy()
        if encoding == 'rgba8':
            return arr[:, :, 2::-1].copy()
        return arr[:, :, :3].copy()
    if encoding in ('mono8', '8uc1'):
        if step < w:
            raise ValueError(f'bad step={step} for {encoding}')
        gray = np.ndarray((h, w), dtype=np.uint8, buffer=buf, strides=(step, 1)).copy()
        return np.stack([gray, gray, gray], axis=-1)
    raise ValueError(f'unsupported encoding: {msg.encoding}')


def save_bgr_png(path: Path, bgr) -> None:
    import cv2

    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), bgr):
        raise OSError(f'cv2.imwrite failed: {path}')


def save_capture_meta(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + '\n', encoding='utf-8')


class CameraFrameBuffer:
    """Keeps latest /camera/image_raw and a monotonic seq for new-frame gating."""

    def __init__(self, node, topic: str = '/camera/image_raw', qos_depth: int = 5) -> None:
        self._lock = threading.Lock()
        self._latest: Optional[Image] = None
        self._seq = 0
        self._sub = node.create_subscription(
            Image, topic, self._on_image, qos_depth,
        )

    def _on_image(self, msg: Image) -> None:
        with self._lock:
            self._latest = msg
            self._seq += 1

    def snapshot(self) -> tuple[int, Optional[Image]]:
        with self._lock:
            return self._seq, self._latest

    def watermark(self) -> tuple[int, float]:
        """Return (seq, stamp_sec of latest or 0). Prefer call after settle."""
        with self._lock:
            seq = self._seq
            stamp = 0.0
            if self._latest is not None:
                stamp = stamp_msg_to_sec(self._latest.header.stamp)
            return seq, stamp

    def wait_new_frame(
        self,
        *,
        after_seq: int,
        after_stamp_sec: float,
        timeout_sec: float,
        clock: Callable[[], float] = time.monotonic,
        should_abort: Optional[Callable[[], bool]] = None,
        sleep_fn: Callable[[float], None] = time.sleep,
    ) -> tuple[Optional[Image], str]:
        """Wait for a frame newer than the given watermark.

        Returns (msg, error_code). error_code empty on success.
        """
        deadline = clock() + timeout_sec
        while clock() < deadline:
            if should_abort and should_abort():
                return None, 'canceled'
            seq, msg = self.snapshot()
            if msg is not None and seq > after_seq:
                stamp = stamp_msg_to_sec(msg.header.stamp)
                if stamp > after_stamp_sec or (
                    stamp == 0.0 and after_stamp_sec == 0.0 and seq > after_seq
                ):
                    if msg.width > 0 and msg.height > 0 and msg.data:
                        return msg, ''
            sleep_fn(0.05)
        return None, ErrorCode.NO_NEW_FRAME


class OdomSettleMonitor:
    """Stop-wait using /odom; requires fresh, continuously updating messages."""

    def __init__(self, node, topic: str = '/odom', qos_depth: int = 10) -> None:
        self._lock = threading.Lock()
        self._last: Optional[Odometry] = None
        self._seq = 0
        self._last_recv_mono = 0.0
        self._clock = time.monotonic
        self._sub = node.create_subscription(
            Odometry, topic, self._on_odom, qos_depth,
        )

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._last = msg
            self._seq += 1
            self._last_recv_mono = time.monotonic()

    def snapshot(self) -> tuple[int, Optional[Odometry], float]:
        with self._lock:
            return self._seq, self._last, self._last_recv_mono

    def wait_settled(
        self,
        *,
        timeout_sec: float,
        settle_sec: float = 1.0,
        v_gate: float = 0.05,
        w_gate: float = 0.08,
        stale_sec: float = 0.5,
        clock: Callable[[], float] = time.monotonic,
        should_abort: Optional[Callable[[], bool]] = None,
        sleep_fn: Callable[[float], None] = time.sleep,
    ) -> tuple[str, Optional[float]]:
        """Return (error_code_or_empty, settle_completed_mono).

        Fresh odom means: callbacks keep advancing seq, and last receive age
        stays within stale_sec while speed stays below gates for settle_sec.
        """
        deadline = clock() + timeout_sec
        below_since = None
        last_seq = -1
        last_seq_change = clock()
        saw_fresh = False
        while clock() < deadline:
            if should_abort and should_abort():
                return 'canceled', None
            now = clock()
            seq, msg, recv_mono = self.snapshot()
            if msg is None or (now - recv_mono) > stale_sec:
                below_since = None
                sleep_fn(0.05)
                continue
            if seq != last_seq:
                last_seq = seq
                last_seq_change = now
                saw_fresh = True
            elif (now - last_seq_change) > stale_sec:
                below_since = None
                sleep_fn(0.05)
                continue

            v = abs(float(msg.twist.twist.linear.x))
            w = abs(float(msg.twist.twist.angular.z))
            if v <= v_gate and w <= w_gate:
                if below_since is None:
                    below_since = now
                elif now - below_since >= settle_sec:
                    return '', now
            else:
                below_since = None
            sleep_fn(0.05)
        if not saw_fresh:
            return ErrorCode.ODOM_STALE, None
        # If stream went stale near the end, prefer odom_stale.
        now = clock()
        _seq, _msg, recv_mono = self.snapshot()
        if _msg is None or (now - recv_mono) > stale_sec:
            return ErrorCode.ODOM_STALE, None
        return ErrorCode.SETTLE_TIMEOUT, None


class PoseSnapshotter:
    """Latest /ground_truth and /amcl_pose for capture diagnostics only."""

    def __init__(
        self,
        node,
        *,
        gt_topic: str = '/ground_truth',
        amcl_topic: str = '/amcl_pose',
    ) -> None:
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.qos import qos_profile_sensor_data

        self._lock = threading.Lock()
        self._gt: Optional[Odometry] = None
        self._amcl: Optional[PoseWithCovarianceStamped] = None
        self._gt_sub = node.create_subscription(
            Odometry, gt_topic, self._on_gt, qos_profile_sensor_data,
        )
        self._amcl_sub = node.create_subscription(
            PoseWithCovarianceStamped, amcl_topic, self._on_amcl, 10,
        )

    def _on_gt(self, msg: Odometry) -> None:
        with self._lock:
            self._gt = msg

    def _on_amcl(self, msg) -> None:
        with self._lock:
            self._amcl = msg

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            gt = self._gt
            amcl = self._amcl
        out: dict[str, Any] = {
            'gt_world_pose': None,
            'amcl_pose': None,
        }
        if gt is not None:
            # gazebo_ros_p3d /ground_truth is world/odom-like; treat as world for diag.
            out['gt_world_pose'] = pose_dict_from_odom(gt, frame_label='world')
        if amcl is not None:
            p = amcl.pose.pose.position
            q = amcl.pose.pose.orientation
            out['amcl_pose'] = {
                'frame': 'map',
                'x': float(p.x),
                'y': float(p.y),
                'z': float(p.z),
                'yaw': yaw_from_quat(float(q.z), float(q.w)),
                'stamp': stamp_msg_to_sec(amcl.header.stamp),
                'header_frame_id': amcl.header.frame_id or '',
            }
        return out


def capture_station_frame(
    *,
    camera: CameraFrameBuffer,
    settle: OdomSettleMonitor,
    output_dir: Path,
    mission_id: str,
    station_id: str = STATION_A,
    route_id: str = 'test_room_A',
    settle_timeout_sec: float = 15.0,
    settle_sec: float = 1.0,
    frame_timeout_sec: float = 10.0,
    should_abort: Optional[Callable[[], bool]] = None,
    clock: Callable[[], float] = time.monotonic,
    sleep_fn: Callable[[float], None] = time.sleep,
    on_phase: Optional[Callable[[str, str], None]] = None,
    nav_goal: Optional[dict] = None,
    pose_snapshotter: Optional[PoseSnapshotter] = None,
    extra_diagnostics: Optional[dict] = None,
) -> CaptureResult:
    """Settle → reset frame watermark → wait new frame → PNG + JSON."""

    def phase(name: str, message: str) -> None:
        if on_phase:
            on_phase(name, message)

    phase('SETTLING', 'waiting for stop at station A (fresh odom required)')
    settle_err, settle_mono = settle.wait_settled(
        timeout_sec=settle_timeout_sec,
        settle_sec=settle_sec,
        clock=clock,
        should_abort=should_abort,
        sleep_fn=sleep_fn,
    )
    if settle_err == 'canceled':
        return CaptureResult(ok=False, error_code='canceled', message='canceled during settle')
    if settle_err:
        msg = (
            'odom stale or not continuously updating during settle'
            if settle_err == ErrorCode.ODOM_STALE
            else 'timed out waiting for robot to settle at station A'
        )
        return CaptureResult(
            ok=False,
            error_code=settle_err,
            message=msg,
            station_id=station_id,
        )

    # Watermark AFTER settle so capture uses a post-stop frame.
    after_seq, after_stamp_sec = camera.watermark()
    phase('CAPTURING', 'waiting for new camera frame after settle watermark')
    img, ferr = camera.wait_new_frame(
        after_seq=after_seq,
        after_stamp_sec=after_stamp_sec,
        timeout_sec=frame_timeout_sec,
        clock=clock,
        should_abort=should_abort,
        sleep_fn=sleep_fn,
    )
    if ferr == 'canceled':
        return CaptureResult(ok=False, error_code='canceled', message='canceled during capture')
    if img is None:
        return CaptureResult(
            ok=False,
            error_code=ferr or ErrorCode.NO_NEW_FRAME,
            message='no new camera frame after settle watermark',
            station_id=station_id,
        )

    poses = pose_snapshotter.snapshot() if pose_snapshotter is not None else {
        'gt_world_pose': None,
        'amcl_pose': None,
    }

    try:
        bgr = image_msg_to_bgr_array(img)
    except Exception as exc:
        return CaptureResult(
            ok=False,
            error_code=ErrorCode.IMAGE_CONVERT_FAILED,
            message=f'image convert failed: {exc}',
            station_id=station_id,
        )

    stamp_sec = stamp_msg_to_sec(img.header.stamp)
    stamp_label = f'{stamp_sec:.3f}'.replace('.', '_')
    out_dir = Path(output_dir) / mission_id
    png_path = out_dir / f'{station_id}_{stamp_label}.png'
    meta_path = out_dir / f'{station_id}_{stamp_label}.json'
    frame_id = img.header.frame_id or ''
    image_stamp = f'{stamp_sec:.6f}'

    diagnostics = {
        'nav_goal': nav_goal or {},
        'gt_world_pose': poses.get('gt_world_pose'),
        'amcl_pose': poses.get('amcl_pose'),
        'settle_completed_mono': settle_mono,
        'settle_completed_image_watermark_seq': after_seq,
        'settle_completed_image_watermark_stamp': after_stamp_sec,
        'image_stamp': image_stamp,
        'image_header_stamp': stamp_sec,
        'capture_note': (
            'GT is simulation diagnosis only; does not replace Nav2 control. '
            'AMCL≤0.10m is not a product capture hard gate here.'
        ),
    }
    if extra_diagnostics:
        diagnostics.update(extra_diagnostics)

    try:
        save_bgr_png(png_path, bgr)
        meta = {
            'mission_id': mission_id,
            'station_id': station_id,
            'route_id': route_id,
            'screenshot_path': str(png_path),
            'image_stamp': image_stamp,
            'frame_id': frame_id,
            'width': int(img.width),
            'height': int(img.height),
            'encoding': img.encoding,
            'diagnostics': diagnostics,
        }
        save_capture_meta(meta_path, meta)
    except Exception as exc:
        return CaptureResult(
            ok=False,
            error_code=ErrorCode.IMAGE_SAVE_FAILED,
            message=f'image/meta save failed: {exc}',
            station_id=station_id,
        )

    return CaptureResult(
        ok=True,
        error_code=ErrorCode.NONE,
        message='A-station capture saved',
        screenshot_path=str(png_path),
        image_stamp=image_stamp,
        frame_id=frame_id,
        station_id=station_id,
        meta_path=str(meta_path),
        diagnostics=diagnostics,
    )
