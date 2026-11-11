"""OccupancyGrid / map-frame helpers for the read-only console.

Rules:
- amcl_pose and Nav2 Path are already in ``map``; do NOT re-apply world→map origin.
- Route YAML stations are world-calibration values; subtract map yaml origin once.
- OccupancyGrid: data is row-major, row 0 = map -Y side of the grid origin cell strip;
  canvas usually flips Y so visual matches RViz (map +Y up).
"""

from __future__ import annotations

import math
from typing import Any, Iterable, Optional, Sequence


def world_xy_yaw_to_map(
    x: float,
    y: float,
    yaw: float,
    origin: Sequence[float],
) -> tuple[float, float, float]:
    """Convert route world [x,y,yaw] to map frame using OccupancyGrid/map yaml origin."""
    if len(origin) < 3:
        raise ValueError(f'invalid map origin: {origin}')
    ox, oy, oyaw = float(origin[0]), float(origin[1]), float(origin[2])
    return x - ox, y - oy, yaw - oyaw


def map_xy_to_grid_cell(
    x: float,
    y: float,
    *,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
) -> Optional[tuple[int, int]]:
    """Map-frame meters → OccupancyGrid cell (col=i, row=j). None if out of bounds."""
    if resolution <= 0 or width <= 0 or height <= 0:
        return None
    i = int(math.floor((x - origin_x) / resolution))
    j = int(math.floor((y - origin_y) / resolution))
    if i < 0 or j < 0 or i >= width or j >= height:
        return None
    return i, j


def grid_cell_to_canvas(
    i: int,
    j: int,
    *,
    height: int,
    flip_y: bool = True,
) -> tuple[int, int]:
    """Grid cell → canvas pixel (top-left origin). Default flip_y matches RViz."""
    cx = int(i)
    cy = int((height - 1 - j) if flip_y else j)
    return cx, cy


def map_xy_to_canvas(
    x: float,
    y: float,
    *,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
    flip_y: bool = True,
) -> Optional[tuple[float, float]]:
    """Map meters → canvas pixel coords (float, for drawing)."""
    if resolution <= 0 or width <= 0 or height <= 0:
        return None
    px = (x - origin_x) / resolution
    py_grid = (y - origin_y) / resolution
    if flip_y:
        py = (height - py_grid)
    else:
        py = py_grid
    if px < -1 or py < -1 or px > width + 1 or py > height + 1:
        return None
    return px, py


def yaw_to_canvas_angle(yaw: float, *, flip_y: bool = True) -> float:
    """Map-frame yaw (CCW from +X) → canvas draw angle.

    Canvas Y grows downward when flip_y drawing of the grid is used; negate yaw
    so the robot nose points correctly on the flipped canvas.
    """
    return -float(yaw) if flip_y else float(yaw)


def path_points_map_to_canvas(
    points: Iterable[tuple[float, float]],
    *,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
    flip_y: bool = True,
) -> list[list[float]]:
    out: list[list[float]] = []
    for x, y in points:
        c = map_xy_to_canvas(
            float(x), float(y),
            origin_x=origin_x, origin_y=origin_y,
            resolution=resolution, width=width, height=height,
            flip_y=flip_y,
        )
        if c is not None:
            out.append([c[0], c[1]])
    return out


def occupancy_meta_dict(msg: Any) -> dict[str, Any]:
    """Extract JSON-serializable OccupancyGrid meta (no raw data)."""
    info = msg.info
    origin = info.origin.position
    q = info.origin.orientation
    # yaw from quaternion (flat map → mostly z/w)
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return {
        'frame_id': str(msg.header.frame_id or ''),
        'resolution': float(info.resolution),
        'width': int(info.width),
        'height': int(info.height),
        'origin': {
            'x': float(origin.x),
            'y': float(origin.y),
            'yaw': float(yaw),
        },
        'stamp_sec': int(getattr(msg.header.stamp, 'sec', 0) or 0),
        'stamp_nanosec': int(getattr(msg.header.stamp, 'nanosec', 0) or 0),
    }
