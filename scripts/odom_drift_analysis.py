#!/usr/bin/env python3
"""
T3：对比 rosbag2 中 /odom 与 /ground_truth，输出漂移曲线与摘要。

用法：
  python3 scripts/odom_drift_analysis.py bags/odom_drift/run_YYYYMMDD_HHMMSS
  python3 scripts/odom_drift_analysis.py bags/odom_drift/run_YYYYMMDD_HHMMSS --out docs/assets
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def load_odometry_topic(bag_path: Path, topic: str) -> list[tuple[float, Odometry]]:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    out: list[tuple[float, Odometry]] = []
    while reader.has_next():
        name, data, t_ns = reader.read_next()
        if name != topic:
            continue
        msg = deserialize_message(data, Odometry)
        out.append((t_ns * 1e-9, msg))
    return out


def to_xy_yaw(series: list[tuple[float, Odometry]]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if not series:
        raise ValueError('空话题序列')
    times = np.array([t for t, _ in series], dtype=float)
    xs = np.array([m.pose.pose.position.x for _, m in series], dtype=float)
    ys = np.array([m.pose.pose.position.y for _, m in series], dtype=float)
    yaws = np.array([yaw_from_quat(m.pose.pose.orientation) for _, m in series], dtype=float)
    return times, xs, ys, yaws


def align_to_start(xs, ys, yaws):
    """减去首帧位姿：平移 + 旋转到起点朝向为 +X。"""
    x0, y0, yaw0 = xs[0], ys[0], yaws[0]
    cos_y = math.cos(-yaw0)
    sin_y = math.sin(-yaw0)
    dx = xs - x0
    dy = ys - y0
    x_rel = cos_y * dx - sin_y * dy
    y_rel = sin_y * dx + cos_y * dy
    yaw_rel = np.array([normalize_angle(y - yaw0) for y in yaws], dtype=float)
    return x_rel, y_rel, yaw_rel


def path_length(xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    """累积路程（米）。"""
    if len(xs) < 2:
        return np.zeros_like(xs)
    seg = np.sqrt(np.diff(xs) ** 2 + np.diff(ys) ** 2)
    return np.concatenate([[0.0], np.cumsum(seg)])


def sync_nearest(
    t_ref: np.ndarray,
    t_other: np.ndarray,
    arrays: list[np.ndarray],
    max_dt: float = 0.15,
) -> list[np.ndarray]:
    synced = []
    for arr in arrays:
        out = np.full_like(t_ref, np.nan, dtype=float)
        for i, tr in enumerate(t_ref):
            j = int(np.argmin(np.abs(t_other - tr)))
            if abs(t_other[j] - tr) <= max_dt:
                out[i] = arr[j]
        synced.append(out)
    return synced


def main() -> int:
    parser = argparse.ArgumentParser(description='T3 odom vs ground_truth 漂移分析')
    parser.add_argument('bag_path', type=Path, help='rosbag2 目录路径')
    parser.add_argument(
        '--out',
        type=Path,
        default=Path('docs/assets'),
        help='PNG 输出目录（默认 docs/assets）',
    )
    args = parser.parse_args()

    bag_path = args.bag_path.expanduser().resolve()
    if not bag_path.is_dir():
        print(f'错误: bag 目录不存在: {bag_path}', file=sys.stderr)
        return 1

    odom_series = load_odometry_topic(bag_path, '/odom')
    gt_series = load_odometry_topic(bag_path, '/ground_truth')
    if len(odom_series) < 10 or len(gt_series) < 10:
        print('错误: /odom 或 /ground_truth 消息过少，请重新录制', file=sys.stderr)
        return 1

    t_o, x_o, y_o, yaw_o = to_xy_yaw(odom_series)
    t_g, x_g, y_g, yaw_g = to_xy_yaw(gt_series)

    x_o, y_o, yaw_o = align_to_start(x_o, y_o, yaw_o)
    x_g, y_g, yaw_g = align_to_start(x_g, y_g, yaw_g)

    # 以 ground_truth 时间为参考对齐 odom
    yaw_o_sync = sync_nearest(t_g, t_o, [yaw_o])[0]
    x_o_sync, y_o_sync = sync_nearest(t_g, t_o, [x_o, y_o])

    valid = ~np.isnan(x_o_sync)
    t_plot = t_g[valid] - t_g[0]
    x_g_v, y_g_v = x_g[valid], y_g[valid]
    x_o_v, y_o_v = x_o_sync[valid], y_o_sync[valid]
    yaw_g_v, yaw_o_v = yaw_g[valid], yaw_o_sync[valid]

    pos_err = np.sqrt((x_o_v - x_g_v) ** 2 + (y_o_v - y_g_v) ** 2)
    s_g = path_length(x_g_v, y_g_v)
    s_o = path_length(x_o_v, y_o_v)
    heading_err_deg = np.degrees(np.array([normalize_angle(a - b) for a, b in zip(yaw_o_v, yaw_g_v)]))

    total_gt = float(s_g[-1]) if len(s_g) else 0.0
    total_odom = float(s_o[-1]) if len(s_o) else 0.0
    scale = (total_odom / total_gt) if total_gt > 1e-3 else float('nan')
    max_pos_err = float(np.max(pos_err)) if len(pos_err) else 0.0
    final_pos_err = float(pos_err[-1]) if len(pos_err) else 0.0
    max_heading = float(np.max(np.abs(heading_err_deg))) if len(heading_err_deg) else 0.0

    out_dir = args.out.expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = bag_path.name
    png_xy = out_dir / f'odom_drift_xy_{stem}.png'
    png_pos = out_dir / f'odom_drift_pos_{stem}.png'
    png_yaw = out_dir / f'odom_drift_yaw_{stem}.png'

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(x_g_v, y_g_v, 'g-', label='ground truth', linewidth=2)
    ax.plot(x_o_v, y_o_v, 'b--', label='odom', linewidth=1.5)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Trajectory (aligned to start pose)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(png_xy, dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(s_g[valid], pos_err, 'r-')
    ax.set_xlabel('distance along ground truth (m)')
    ax.set_ylabel('position error (m)')
    ax.set_title('Position error vs distance')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(png_pos, dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t_plot, heading_err_deg, 'm-')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('heading error (deg)')
    ax.set_title('Heading error vs time')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(png_yaw, dpi=150)
    plt.close(fig)

    print('========================================')
    print(f'  bag: {bag_path}')
    print(f'  真值路程: {total_gt:.2f} m | odom 路程: {total_odom:.2f} m')
    print(f'  尺度因子 (odom/GT): {scale:.4f}')
    print(f'  最大位置误差: {max_pos_err:.3f} m | 终点误差: {final_pos_err:.3f} m')
    print(f'  最大航向误差: {max_heading:.1f} deg')
    print(f'  图: {png_xy.name}, {png_pos.name}, {png_yaw.name}')
    print('========================================')

    if abs(scale - 1.0) > 0.03:
        print('摘要: 尺度因子偏离 1，优先检查 diff_drive 的 wheel_diameter / wheel_radius。')
    elif max_pos_err > 0.5:
        print('摘要: 累积漂移偏大，SLAM/AMCL 必须参与修正；记录本曲线作基线。')
    else:
        print('摘要: 漂移在可接受量级，可进入 SLAM；仍建议 AMCL 抑制长期累积。')

    return 0


if __name__ == '__main__':
    sys.exit(main())
