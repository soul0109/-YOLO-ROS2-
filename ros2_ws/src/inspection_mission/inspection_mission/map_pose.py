"""Shared map-frame pose helpers for mission navigation.

Coordinate rule matches patrol_mission_node.pose(): route YAML values are in
the existing test_room world calibration; Nav2 goals use frame_id=map after
subtracting navigation_config maps/test_room.yaml origin.
"""

from __future__ import annotations

import math
from typing import Sequence

from geometry_msgs.msg import PoseStamped


def pose_from_route_values(
    values: Sequence[float],
    origin: Sequence[float],
    *,
    stamp=None,
) -> PoseStamped:
    """Convert a route [x, y, yaw] triple into a map-frame PoseStamped."""
    if len(values) != 3:
        raise ValueError(f'invalid route pose: {values}')
    if len(origin) < 3:
        raise ValueError(f'invalid map origin: {origin}')
    x, y, yaw = (float(v) for v in values)
    ox, oy, oyaw = float(origin[0]), float(origin[1]), float(origin[2])
    if not all(math.isfinite(v) for v in (x, y, yaw, ox, oy, oyaw)):
        raise ValueError(f'non-finite pose/origin: values={values} origin={origin}')

    msg = PoseStamped()
    msg.header.frame_id = 'map'
    if stamp is not None:
        msg.header.stamp = stamp
    msg.pose.position.x = x - ox
    msg.pose.position.y = y - oy
    msg.pose.orientation.z = math.sin((yaw - oyaw) / 2.0)
    msg.pose.orientation.w = math.cos((yaw - oyaw) / 2.0)
    return msg
