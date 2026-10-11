#!/usr/bin/env python3
"""D1: capture 1.2 m head-on frames at stations A/B/C with GT sidecars.

- Teleports robot_v0 via `gz model` (no Nav2).
- Saves PNG + JSON per station (station_id, seed, gt states, label classes).
- Does not train models or draw boxes (coarse label fields only).
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

# Same parking poses as patrol_route / test_room comments (~1.2 m to cabinet face).
STATION_POSE = {
    'A': {'x': 3.15, 'y': 1.75, 'z': 0.05, 'yaw': -math.pi / 2},
    'B': {'x': 8.45, 'y': 1.75, 'z': 0.05, 'yaw': -math.pi / 2},
    'C': {'x': 5.25, 'y': 4.25, 'z': 0.05, 'yaw': math.pi / 2},
}


def _load_gt(path: Path | None, seed: int) -> dict[str, Any]:
    if path is not None:
        return json.loads(path.read_text(encoding='utf-8'))
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from apply_vision_scene_gt import generate_gt  # noqa: WPS433

    return generate_gt(seed)


def _teleport(station: str) -> None:
    p = STATION_POSE[station]
    cmd = [
        'gz',
        'model',
        '-m',
        'robot_v0',
        '-x',
        str(p['x']),
        '-y',
        str(p['y']),
        '-z',
        str(p['z']),
        '-Y',
        str(p['yaw']),
    ]
    subprocess.run(cmd, check=True, capture_output=True, text=True)


def _grab_bgr(timeout_sec: float = 10.0):
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node('capture_vision_station_frames')
    bridge = CvBridge()
    box: dict[str, Any] = {'img': None, 'stamp': None}

    def cb(msg: Image) -> None:
        if box['img'] is not None:
            return
        box['img'] = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        box['stamp'] = {
            'sec': int(msg.header.stamp.sec),
            'nanosec': int(msg.header.stamp.nanosec),
        }
        box['frame_id'] = msg.header.frame_id

    node.create_subscription(Image, '/camera/image_raw', cb, 10)
    deadline = time.time() + timeout_sec
    # Discard a few frames after teleport so the camera settles.
    settle_until = time.time() + 1.2
    while time.time() < settle_until:
        rclpy.spin_once(node, timeout_sec=0.1)
        box['img'] = None
    while time.time() < deadline and box['img'] is None:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if box['img'] is None:
        raise RuntimeError('no /camera/image_raw within timeout; is Gazebo+robot running?')
    return box


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument(
        '--gt',
        type=Path,
        default=None,
        help='Existing gt.json (default: regenerate from --seed)',
    )
    parser.add_argument(
        '-o',
        '--out',
        type=Path,
        default=None,
        help='Output directory (default: bags/vision_d1_seed<seed>/)',
    )
    parser.add_argument(
        '--stations',
        default='A,B,C',
        help='Comma-separated station ids',
    )
    args = parser.parse_args(argv)

    out = args.out or (
        Path.home() / 'inspection-robot' / 'bags' / f'vision_d1_seed{args.seed}'
    )
    out.mkdir(parents=True, exist_ok=True)

    gt = _load_gt(args.gt, args.seed)
    if int(gt.get('seed', args.seed)) != args.seed:
        print(
            f'warning: gt seed {gt.get("seed")} != --seed {args.seed}',
            file=sys.stderr,
        )

    (out / 'gt.json').write_text(
        json.dumps(gt, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )

    import cv2

    stations = [s.strip().upper() for s in args.stations.split(',') if s.strip()]
    manifest: list[dict[str, Any]] = []

    for sid in stations:
        if sid not in STATION_POSE:
            raise SystemExit(f'unknown station {sid}')
        if sid not in gt['stations']:
            raise SystemExit(f'station {sid} missing from gt')

        print(f'capture {sid}: teleport + grab ...', flush=True)
        _teleport(sid)
        grabbed = _grab_bgr()
        stem = f'station{sid}_seed{args.seed}'
        png = out / f'{stem}.png'
        meta_path = out / f'{stem}.json'
        cv2.imwrite(str(png), grabbed['img'])

        st = gt['stations'][sid]
        meta = {
            'schema': 'vision_d1_frame_v1',
            'station_id': sid,
            'seed': args.seed,
            'image': png.name,
            'image_stamp': grabbed['stamp'],
            'frame_id': grabbed.get('frame_id'),
            'distance_m_nominal': 1.2,
            'robot_pose': STATION_POSE[sid],
            'gt': {
                'leds': st['leds'],
                'panel': st['panel'],
                'leak': st['leak'],
            },
            'label': {
                'classes': ['indicator_light', 'panel', 'inspection_region'],
                'note': (
                    'boxes not drawn in D1; status fields copy gt. '
                    'leds[i] = cabinet_*_led_{i+1} model order, '
                    'NOT necessarily left-to-right in image '
                    '(A/B face -Y so image-left ≈ led_3).'
                ),
                'boxes': [],
            },
        }
        meta_path.write_text(
            json.dumps(meta, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        print(f'  wrote {png} + {meta_path.name}', flush=True)
        manifest.append({'station_id': sid, 'image': png.name, 'meta': meta_path.name})

    man_path = out / 'manifest.json'
    man_path.write_text(
        json.dumps(
            {
                'schema': 'vision_d1_manifest_v1',
                'seed': args.seed,
                'frames': manifest,
            },
            indent=2,
            sort_keys=True,
        )
        + '\n',
        encoding='utf-8',
    )
    print(f'done -> {out} ({len(manifest)} frames)', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
