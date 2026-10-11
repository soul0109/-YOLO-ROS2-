#!/usr/bin/env python3
"""C-off data-quality check: off≠green on camera; green not demoted.

Primary fix is Gazebo material (explicit emissive 0 0 0 1). This script
captures same-pose off/on contrast + ROI measurements, then asserts.
Does not train YOLO, does not touch Web/Nav2, does not claim 90%.
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

sys.path.insert(0, str(Path(__file__).resolve().parent))
from apply_vision_scene_gt import LED_MAT, _material_xml, _rpy_to_quat  # noqa: E402
from roi_judge_d1 import DEFAULT_RULES, judge_one, load_rules  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
DEFAULT_OUT = REPO / 'bags' / 'vision_c_off'
C_LED1_POSE = (5.05, 5.44, 0.52, 1.5708, 0.0, 0.0)


def _spawn_c_led1(state: str) -> None:
    import rclpy
    from gazebo_msgs.srv import DeleteEntity, SpawnEntity
    from geometry_msgs.msg import Pose
    from rclpy.node import Node

    mat = LED_MAT[state]
    xml = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="cabinet_c_led_1">
    <pose>0 0 0 0 0 0</pose>
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><cylinder><radius>0.04</radius><length>0.02</length></cylinder></geometry>
        <material>{_material_xml(mat)}</material>
      </visual>
    </link>
  </model>
</sdf>"""
    rclpy.init()
    node = Node('c_off_spawn_led1')
    dcli = node.create_client(DeleteEntity, '/delete_entity')
    scli = node.create_client(SpawnEntity, '/spawn_entity')
    if not dcli.wait_for_service(timeout_sec=8.0) or not scli.wait_for_service(timeout_sec=8.0):
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError('Gazebo delete/spawn services unavailable')
    req = DeleteEntity.Request()
    req.name = 'cabinet_c_led_1'
    fut = dcli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    time.sleep(0.15)
    req = SpawnEntity.Request()
    req.name = 'cabinet_c_led_1'
    req.xml = xml
    req.reference_frame = 'world'
    x, y, z, roll, pitch, yaw = C_LED1_POSE
    req.initial_pose = Pose()
    req.initial_pose.position.x = float(x)
    req.initial_pose.position.y = float(y)
    req.initial_pose.position.z = float(z)
    qx, qy, qz, qw = _rpy_to_quat(roll, pitch, yaw)
    o = req.initial_pose.orientation
    o.x, o.y, o.z, o.w = qx, qy, qz, qw
    fut = scli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
    ok = fut.done() and fut.result() is not None and fut.result().success
    node.destroy_node()
    rclpy.shutdown()
    if not ok:
        raise RuntimeError(f'spawn cabinet_c_led_1 -> {state} failed')


def _grab_c(path: Path) -> dict[str, Any]:
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    subprocess.run(
        [
            'gz',
            'model',
            '-m',
            'robot_v0',
            '-x',
            '5.25',
            '-y',
            '4.25',
            '-z',
            '0.05',
            '-Y',
            str(math.pi / 2),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    rclpy.init()
    node = Node('c_off_grab')
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

    node.create_subscription(Image, '/camera/image_raw', cb, 10)
    settle = time.time() + 1.3
    while time.time() < settle:
        rclpy.spin_once(node, timeout_sec=0.05)
    box['img'] = None
    deadline = time.time() + 8.0
    while box['img'] is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    if box['img'] is None:
        raise RuntimeError('no camera frame')
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), box['img'])
    return box


def _slot_measurements(meta_path: Path, rules: dict[str, Any]) -> dict[str, Any]:
    result = judge_one(meta_path, rules)
    return {
        'station_id': result.get('station_id'),
        'objects': result['objects'],
        'verdict': result['verdict'],
        'gt_match': result['gt_compare']['all_match'],
        'led_slots': result['debug']['led_slots'],
    }


def capture_contrast(out: Path, seed: int) -> None:
    gt_path = out / 'gt.json'
    subprocess.run(
        [
            sys.executable,
            str(REPO / 'scripts' / 'apply_vision_scene_gt.py'),
            '--seed',
            str(seed),
            '--apply',
            '-o',
            str(gt_path),
        ],
        check=True,
    )
    subprocess.run(
        [
            sys.executable,
            str(REPO / 'scripts' / 'capture_vision_station_frames.py'),
            '--seed',
            str(seed),
            '--gt',
            str(gt_path),
            '-o',
            str(out),
        ],
        check=True,
    )
    # Same pose: force C led_1 green for on-contrast (overwrites only that LED).
    _spawn_c_led1('green')
    box = _grab_c(out / 'stationC_led1_green.png')
    meta = {
        'schema': 'vision_c_off_contrast_v1',
        'station_id': 'C',
        'seed': seed,
        'image': 'stationC_led1_green.png',
        'image_stamp': box['stamp'],
        'robot_pose': {'x': 5.25, 'y': 4.25, 'z': 0.05, 'yaw': math.pi / 2},
        'gt': {'leds': ['green', 'green', 'green'], 'panel': 'lit', 'leak': 'none'},
        'note': 'same pose as stationC_seed42; only cabinet_c_led_1 forced green',
    }
    (out / 'stationC_led1_green.json').write_text(
        json.dumps(meta, indent=2, sort_keys=True) + '\n', encoding='utf-8'
    )
    # Restore seed GT off on led_1 for a clean world leave-behind.
    _spawn_c_led1('off')


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('-o', '--output', type=Path, default=DEFAULT_OUT)
    p.add_argument('--rules', type=Path, default=DEFAULT_RULES)
    p.add_argument('--seed', type=int, default=42)
    p.add_argument(
        '--capture',
        action='store_true',
        help='apply GT + capture A/B/C + same-pose C led1 green contrast (needs Gazebo)',
    )
    args = p.parse_args(argv)
    out = args.output
    rules = load_rules(args.rules)

    if args.capture:
        capture_contrast(out, args.seed)

    c_meta = out / f'stationC_seed{args.seed}.json'
    if not c_meta.is_file():
        print(f'missing {c_meta}; run with --capture', file=sys.stderr)
        return 1

    meas: dict[str, Any] = {
        'schema': 'vision_c_off_measurements_v1',
        'rules': str(args.rules),
        'output': str(out),
        'note': (
            'Primary fix: explicit emissive 0 0 0 1 for off/dark. '
            'ROI rel_to_row_max is a belt. Not a 90% claim.'
        ),
        'stations': {},
    }
    for sid in ('A', 'B', 'C'):
        mp = out / f'station{sid}_seed{args.seed}.json'
        if mp.is_file():
            meas['stations'][sid] = _slot_measurements(mp, rules)

    green_meta = out / 'stationC_led1_green.json'
    if green_meta.is_file():
        meas['stations']['C_led1_green'] = _slot_measurements(green_meta, rules)

    (out / 'c_off_measurements.json').write_text(
        json.dumps(meas, indent=2, sort_keys=True) + '\n', encoding='utf-8'
    )

    c = meas['stations']['C']
    assert c['objects']['leds'][0] == 'off', c['objects']['leds']
    assert c['objects']['leds'][1] == 'green' and c['objects']['leds'][2] == 'green', c
    assert c['gt_match'] is True, c

    # Green not demoted on A/B (and on C siblings).
    a = meas['stations']['A']
    assert a['objects']['leds'] == ['green', 'green', 'yellow'], a['objects']['leds']
    assert a['gt_match'] is True, a
    b = meas['stations']['B']
    assert b['objects']['leds'] == ['red', 'red', 'green'], b['objects']['leds']
    assert b['gt_match'] is True, b

    if 'C_led1_green' in meas['stations']:
        g = meas['stations']['C_led1_green']
        assert g['objects']['leds'][0] == 'green', g['objects']['leds']
        off_mv = c['led_slots'][0]['detail']['mean_vmax']
        on_mv = g['led_slots'][0]['detail']['mean_vmax']
        assert on_mv > off_mv * 2, (off_mv, on_mv)
        meas['contrast'] = {
            'C_slot0_off_mean_vmax': off_mv,
            'C_slot0_green_mean_vmax': on_mv,
        }
        (out / 'c_off_measurements.json').write_text(
            json.dumps(meas, indent=2, sort_keys=True) + '\n', encoding='utf-8'
        )

    print('C_OFF_OK')
    print(f"  C leds={c['objects']['leds']} gt_match={c['gt_match']}")
    print(f"  A leds={a['objects']['leds']} B leds={b['objects']['leds']}")
    if 'contrast' in meas:
        print(f"  contrast slot0 mean_vmax off={meas['contrast']['C_slot0_off_mean_vmax']} "
              f"green={meas['contrast']['C_slot0_green_mean_vmax']}")
    print(f'  wrote {out / "c_off_measurements.json"}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
