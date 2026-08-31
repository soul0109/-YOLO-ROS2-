"""共享坐标工具：地图 origin 读取与 world↔map 平面转换。"""

from __future__ import annotations

import math
from pathlib import Path

import yaml


def load_map_origin(map_yaml: Path) -> tuple[float, float, float]:
    data = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
    origin = data.get('origin', [0.0, 0.0, 0.0])
    return float(origin[0]), float(origin[1]), float(origin[2])


def world_to_map_xy(world_x: float, world_y: float, origin_x: float, origin_y: float) -> tuple[float, float]:
    """ROS map 元数据：world = origin + map_xy。"""
    return world_x - origin_x, world_y - origin_y


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
