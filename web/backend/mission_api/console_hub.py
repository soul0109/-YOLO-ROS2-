"""Read-only runtime console telemetry inside the FastAPI ROS process.

Subscribes: /map, /amcl_pose, /plan, /camera/image_raw
Caches are bounded; slow HTTP clients get the latest snapshot only.
Does not publish cmd_vel, call Nav2 goals, or write the archive DB.
"""

from __future__ import annotations

import io
import math
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Optional

import yaml

from .map_coords import (
    map_xy_to_canvas,
    occupancy_meta_dict,
    path_points_map_to_canvas,
    world_xy_yaw_to_map,
    yaw_to_canvas_angle,
)

_REPO = Path(__file__).resolve().parents[3]
_DEFAULT_ROUTE = (
    _REPO / 'ros2_ws' / 'src' / 'inspection_mission' / 'config' / 'patrol_route.yaml'
)
_DEFAULT_MAP_YAML = (
    _REPO / 'ros2_ws' / 'src' / 'navigation_config' / 'maps' / 'test_room.yaml'
)

POSE_STALE_SEC = 2.0
PATH_STALE_SEC = 5.0
CAMERA_STALE_SEC = 2.0
MAP_STALE_SEC = 30.0
TRAJECTORY_MAX = 800
CAMERA_MIN_INTERVAL_SEC = 0.5  # ≤2 Hz encode
CAMERA_JPEG_QUALITY = 55
CAMERA_MAX_WIDTH = 320


def _wall_now() -> float:
    return time.time()


def _freshness(age_sec: Optional[float], limit_sec: float) -> dict[str, Any]:
    if age_sec is None:
        return {
            'fresh': False,
            'stale': True,
            'unknown': True,
            'age_sec': None,
            'limit_sec': limit_sec,
        }
    stale = age_sec > limit_sec
    return {
        'fresh': not stale,
        'stale': stale,
        'unknown': False,
        'age_sec': round(age_sec, 3),
        'limit_sec': limit_sec,
    }


def load_map_yaml_origin(path: Path = _DEFAULT_MAP_YAML) -> list[float]:
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    origin = data.get('origin') or [0.0, 0.0, 0.0]
    return [float(origin[0]), float(origin[1]), float(origin[2])]


def load_route_stations_map(
    route_path: Path = _DEFAULT_ROUTE,
    map_yaml: Path = _DEFAULT_MAP_YAML,
) -> dict[str, Any]:
    """ABC (and vias) in map frame for overlay — route world minus map origin once."""
    origin = load_map_yaml_origin(map_yaml)
    route = yaml.safe_load(route_path.read_text(encoding='utf-8')) or {}
    stations_out: dict[str, dict[str, float]] = {}
    for sid, vals in (route.get('stations') or {}).items():
        if not isinstance(vals, (list, tuple)) or len(vals) < 3:
            continue
        mx, my, myaw = world_xy_yaw_to_map(
            float(vals[0]), float(vals[1]), float(vals[2]), origin,
        )
        stations_out[str(sid)] = {'x': mx, 'y': my, 'yaw': myaw}
    vias_out: list[dict[str, float]] = []
    transfers = route.get('transfers') or {}
    for _key, block in transfers.items():
        for item in (block or {}).get('via') or []:
            if isinstance(item, (list, tuple)) and len(item) >= 3:
                mx, my, myaw = world_xy_yaw_to_map(
                    float(item[0]), float(item[1]), float(item[2]), origin,
                )
                vias_out.append({'x': mx, 'y': my, 'yaw': myaw})
    return {
        'frame_id': 'map',
        'origin_applied': origin,
        'inspection_order': list(route.get('inspection_order') or ['A', 'B', 'C']),
        'stations': stations_out,
        'vias': vias_out,
        'note': (
            'Route YAML is world calibration; origin subtracted once. '
            'amcl_pose/path must NOT subtract origin again.'
        ),
    }


class ConsoleHub:
    """Owns optional ROS subscriptions on the mission API node."""

    def __init__(
        self,
        *,
        map_topic: str = '/map',
        pose_topic: str = '/amcl_pose',
        plan_topic: str = '/plan',
        camera_topic: str = '/camera/image_raw',
        trajectory_max: int = TRAJECTORY_MAX,
    ) -> None:
        self.map_topic = map_topic
        self.pose_topic = pose_topic
        self.plan_topic = plan_topic
        self.camera_topic = camera_topic
        self.trajectory_max = trajectory_max

        self._lock = threading.RLock()
        self._started = False
        self._node = None
        self._subs: list[Any] = []

        self._map_meta: Optional[dict[str, Any]] = None
        self._map_data: Optional[list[int]] = None
        self._map_wall: Optional[float] = None

        self._pose: Optional[dict[str, Any]] = None
        self._pose_wall: Optional[float] = None

        self._path_pts: list[tuple[float, float]] = []
        self._path_frame: str = ''
        self._path_wall: Optional[float] = None

        self._traj: deque[tuple[float, float, float]] = deque(maxlen=trajectory_max)
        self._current_goal: Optional[dict[str, Any]] = None

        self._jpeg: Optional[bytes] = None
        self._jpeg_wall: Optional[float] = None
        self._cam_stamp: str = ''
        self._cam_encode_wall: float = 0.0
        self._cam_frame_id: str = ''

        self._route_overlay = load_route_stations_map()

    @property
    def started(self) -> bool:
        return self._started

    def attach_node(self, node: Any) -> None:
        """Subscribe using an existing rclpy Node (shared with Action bridge)."""
        if self._started:
            return
        from nav_msgs.msg import OccupancyGrid, Path
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from sensor_msgs.msg import Image
        from rclpy.qos import (
            QoSProfile,
            ReliabilityPolicy,
            DurabilityPolicy,
            HistoryPolicy,
        )

        self._node = node
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._subs = [
            node.create_subscription(
                OccupancyGrid, self.map_topic, self._on_map, map_qos,
            ),
            node.create_subscription(
                PoseWithCovarianceStamped, self.pose_topic, self._on_pose, 10,
            ),
            node.create_subscription(
                Path, self.plan_topic, self._on_plan, 10,
            ),
            node.create_subscription(
                Image, self.camera_topic, self._on_image, sensor_qos,
            ),
        ]
        self._started = True
        node.get_logger().info(
            f'ConsoleHub attached map={self.map_topic} pose={self.pose_topic} '
            f'plan={self.plan_topic} camera={self.camera_topic}',
        )

    def detach(self) -> None:
        with self._lock:
            self._started = False
            self._node = None
            self._subs = []

    def set_current_goal(self, goal: Optional[dict[str, Any]]) -> None:
        with self._lock:
            self._current_goal = dict(goal) if goal else None

    def clear_session_trajectory(self) -> None:
        with self._lock:
            self._traj.clear()

    def _on_map(self, msg) -> None:
        meta = occupancy_meta_dict(msg)
        # Bound: keep int8 list once; map rarely changes.
        data = list(msg.data)
        with self._lock:
            self._map_meta = meta
            self._map_data = data
            self._map_wall = _wall_now()

    def _on_pose(self, msg) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        wall = _wall_now()
        with self._lock:
            self._pose = {
                'frame_id': str(msg.header.frame_id or 'map'),
                'x': float(p.x),
                'y': float(p.y),
                'yaw': float(yaw),
                'stamp_sec': int(msg.header.stamp.sec),
                'stamp_nanosec': int(msg.header.stamp.nanosec),
            }
            self._pose_wall = wall
            # Session trajectory: skip near-duplicates to bound noise.
            if (
                not self._traj
                or abs(self._traj[-1][0] - p.x) > 0.03
                or abs(self._traj[-1][1] - p.y) > 0.03
            ):
                self._traj.append((float(p.x), float(p.y), wall))

    def _on_plan(self, msg) -> None:
        frame = str(msg.header.frame_id or '')
        pts: list[tuple[float, float]] = []
        for ps in msg.poses:
            pts.append((float(ps.pose.position.x), float(ps.pose.position.y)))
            if len(pts) >= 500:
                break
        with self._lock:
            self._path_pts = pts
            self._path_frame = frame
            self._path_wall = _wall_now()

    def _on_image(self, msg) -> None:
        now = _wall_now()
        with self._lock:
            if now - self._cam_encode_wall < CAMERA_MIN_INTERVAL_SEC and self._jpeg:
                return
            self._cam_encode_wall = now
        try:
            jpeg = _image_msg_to_jpeg(msg)
        except Exception:
            return
        with self._lock:
            self._jpeg = jpeg
            self._jpeg_wall = now
            self._cam_stamp = (
                f'{int(msg.header.stamp.sec)}.{int(msg.header.stamp.nanosec):09d}'
            )
            self._cam_frame_id = str(msg.header.frame_id or '')

    def get_map_payload(self) -> dict[str, Any]:
        now = _wall_now()
        with self._lock:
            age = None if self._map_wall is None else now - self._map_wall
            fresh = _freshness(age, MAP_STALE_SEC)
            if self._map_meta is None or self._map_data is None:
                return {
                    'available': False,
                    'freshness': fresh,
                    'meta': None,
                    'data': None,
                    'message': 'no OccupancyGrid received yet',
                }
            return {
                'available': True,
                'freshness': fresh,
                'meta': dict(self._map_meta),
                'data': list(self._map_data),
                'message': '',
            }

    def get_telemetry(self) -> dict[str, Any]:
        now = _wall_now()
        with self._lock:
            pose_age = None if self._pose_wall is None else now - self._pose_wall
            path_age = None if self._path_wall is None else now - self._path_wall
            cam_age = None if self._jpeg_wall is None else now - self._jpeg_wall
            map_age = None if self._map_wall is None else now - self._map_wall

            meta = self._map_meta
            canvas_pose = None
            canvas_path: list[list[float]] = []
            canvas_traj: list[list[float]] = []
            canvas_stations: dict[str, Any] = {}
            path_displayable = True
            path_note = ''

            if meta and self._pose:
                c = map_xy_to_canvas(
                    self._pose['x'], self._pose['y'],
                    origin_x=meta['origin']['x'],
                    origin_y=meta['origin']['y'],
                    resolution=meta['resolution'],
                    width=meta['width'],
                    height=meta['height'],
                )
                if c is not None:
                    canvas_pose = {
                        'x': c[0],
                        'y': c[1],
                        'yaw_canvas': yaw_to_canvas_angle(self._pose['yaw']),
                        'yaw_map': self._pose['yaw'],
                    }

            if meta:
                if self._path_frame and self._path_frame not in ('map', ''):
                    path_displayable = False
                    path_note = (
                        f'path frame_id={self._path_frame!r} is not map; '
                        'not converted (frozen map unchanged)'
                    )
                elif self._path_pts:
                    canvas_path = path_points_map_to_canvas(
                        self._path_pts,
                        origin_x=meta['origin']['x'],
                        origin_y=meta['origin']['y'],
                        resolution=meta['resolution'],
                        width=meta['width'],
                        height=meta['height'],
                    )
                for pt in self._traj:
                    c = map_xy_to_canvas(
                        pt[0], pt[1],
                        origin_x=meta['origin']['x'],
                        origin_y=meta['origin']['y'],
                        resolution=meta['resolution'],
                        width=meta['width'],
                        height=meta['height'],
                    )
                    if c is not None:
                        canvas_traj.append([c[0], c[1]])
                for sid, st in self._route_overlay['stations'].items():
                    c = map_xy_to_canvas(
                        st['x'], st['y'],
                        origin_x=meta['origin']['x'],
                        origin_y=meta['origin']['y'],
                        resolution=meta['resolution'],
                        width=meta['width'],
                        height=meta['height'],
                    )
                    canvas_stations[sid] = {
                        **st,
                        'canvas': None if c is None else {'x': c[0], 'y': c[1]},
                    }

            return {
                'hub_started': self._started,
                'server_time': now,
                'map': {
                    'available': self._map_meta is not None,
                    'freshness': _freshness(map_age, MAP_STALE_SEC),
                    'meta': dict(meta) if meta else None,
                },
                'pose': {
                    'available': self._pose is not None,
                    'freshness': _freshness(pose_age, POSE_STALE_SEC),
                    'map': dict(self._pose) if self._pose else None,
                    'canvas': canvas_pose,
                    'note': (
                        'amcl_pose is map-frame; origin not re-applied. '
                        'AMCL may be quiet while stationary — stale≠lost localization.'
                    ),
                },
                'path': {
                    'available': bool(self._path_pts),
                    'freshness': _freshness(path_age, PATH_STALE_SEC),
                    'frame_id': self._path_frame,
                    'displayable': path_displayable,
                    'note': path_note,
                    'map_points': (
                        [[x, y] for x, y in self._path_pts]
                        if path_displayable else []
                    ),
                    'canvas_points': canvas_path if path_displayable else [],
                },
                'trajectory': {
                    'count': len(self._traj),
                    'max': self.trajectory_max,
                    'canvas_points': canvas_traj,
                },
                'stations': {
                    'order': list(self._route_overlay['inspection_order']),
                    'map': self._route_overlay['stations'],
                    'canvas': canvas_stations,
                    'vias_map': self._route_overlay['vias'],
                    'origin_applied': self._route_overlay['origin_applied'],
                    'note': self._route_overlay['note'],
                },
                'goal': self._current_goal,
                'camera': {
                    'available': self._jpeg is not None,
                    'freshness': _freshness(cam_age, CAMERA_STALE_SEC),
                    'stamp': self._cam_stamp,
                    'frame_id': self._cam_frame_id,
                    'jpeg_url': '/console/camera.jpg',
                    'note': (
                        'Live preview only — not formal station evidence. '
                        'Use archived screenshot/annotated for inspection proof.'
                    ),
                },
            }

    def get_camera_jpeg(self) -> tuple[Optional[bytes], dict[str, Any]]:
        now = _wall_now()
        with self._lock:
            age = None if self._jpeg_wall is None else now - self._jpeg_wall
            meta = {
                'freshness': _freshness(age, CAMERA_STALE_SEC),
                'stamp': self._cam_stamp,
                'frame_id': self._cam_frame_id,
            }
            return self._jpeg, meta


def _image_msg_to_jpeg(msg) -> bytes:
    import numpy as np
    from PIL import Image

    h, w = int(msg.height), int(msg.width)
    enc = str(msg.encoding or '').lower()
    raw = bytes(msg.data)
    if enc in ('rgb8', 'bgr8'):
        arr = np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 3))
        if enc == 'bgr8':
            arr = arr[:, :, ::-1].copy()
    elif enc in ('rgba8', 'bgra8'):
        arr = np.frombuffer(raw, dtype=np.uint8).reshape((h, w, 4))[:, :, :3]
        if enc == 'bgra8':
            arr = arr[:, :, ::-1].copy()
    elif enc in ('mono8', '8uc1'):
        arr = np.frombuffer(raw, dtype=np.uint8).reshape((h, w))
    else:
        raise ValueError(f'unsupported image encoding: {enc}')

    im = Image.fromarray(arr)
    if im.width > CAMERA_MAX_WIDTH:
        nh = max(1, int(im.height * (CAMERA_MAX_WIDTH / im.width)))
        im = im.resize((CAMERA_MAX_WIDTH, nh))
    buf = io.BytesIO()
    im.save(buf, format='JPEG', quality=CAMERA_JPEG_QUALITY, optimize=True)
    return buf.getvalue()
