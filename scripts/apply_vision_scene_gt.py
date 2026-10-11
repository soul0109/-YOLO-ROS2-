#!/usr/bin/env python3
"""W2: seed-controlled vision scene GT for test_room cabinets.

- Randomize leds[] / panel within enums; leak on B stays fixed_fail (not seeded).
- Same seed → identical gt.json.
- Optional: apply LED/panel materials in a running Gazebo (delete+spawn visuals).
- Does not touch Nav2, training, or cabinet_b_inspection_region pose.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from random import Random
from typing import Any

LED_STATES = ('green', 'red', 'yellow', 'off')
PANEL_STATES = ('lit', 'dark')
STATION_IDS = ('A', 'B', 'C')

# Visual poses after W1 layout fix (lights above, panel below). Leak untouched.
STATION_POSE: dict[str, dict[str, Any]] = {
    'A': {
        'leds': [
            (2.95, 0.560, 0.52, 1.5708, 0.0, 0.0),
            (3.15, 0.560, 0.52, 1.5708, 0.0, 0.0),
            (3.35, 0.560, 0.52, 1.5708, 0.0, 0.0),
        ],
        'panel': (3.15, 0.555, 0.32, 0.0, 0.0, 0.0),
        'leak': 'none',
    },
    'B': {
        'leds': [
            (8.25, 0.560, 0.52, 1.5708, 0.0, 0.0),
            (8.45, 0.560, 0.52, 1.5708, 0.0, 0.0),
            (8.65, 0.560, 0.52, 1.5708, 0.0, 0.0),
        ],
        'panel': (8.45, 0.555, 0.32, 0.0, 0.0, 0.0),
        'leak': 'fixed_fail',
    },
    'C': {
        'leds': [
            (5.05, 5.440, 0.52, 1.5708, 0.0, 0.0),
            (5.25, 5.440, 0.52, 1.5708, 0.0, 0.0),
            (5.45, 5.440, 0.52, 1.5708, 0.0, 0.0),
        ],
        'panel': (5.25, 5.445, 0.32, 0.0, 0.0, 0.0),
        'leak': 'none',
    },
}

LED_MAT = {
    'green': {
        'ambient': '0 0.8 0 1',
        'diffuse': '0 0.6 0 1',
        'emissive': '0 1 0 1',
    },
    'red': {
        'ambient': '0.8 0 0 1',
        'diffuse': '0.6 0 0 1',
        'emissive': '1 0 0 1',
    },
    'yellow': {
        'ambient': '0.8 0.8 0 1',
        'diffuse': '0.6 0.6 0 1',
        'emissive': '1 1 0 1',
    },
    'off': {
        # Near-black, no emissive — must read as unlit vs green/red/yellow.
        'ambient': '0.02 0.02 0.02 1',
        'diffuse': '0.04 0.04 0.04 1',
        'emissive': None,
    },
}

PANEL_MAT = {
    'lit': {
        'ambient': '0.15 0.45 0.55 1',
        'diffuse': '0.25 0.65 0.75 1',
        'emissive': '0.35 0.95 1.0 1',
    },
    'dark': {
        'ambient': '0.05 0.05 0.08 1',
        'diffuse': '0.08 0.08 0.12 1',
        'emissive': None,
    },
}

NON_GREEN = ('red', 'yellow', 'off')


def _pose_str(pose: tuple[float, ...]) -> str:
    return ' '.join(f'{v:.6g}' for v in pose)


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Return (x, y, z, w)."""
    import math

    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def _panel_set(stations: dict[str, dict[str, Any]]) -> set[str]:
    return {stations[s]['panel'] for s in STATION_IDS}


def generate_gt(seed: int) -> dict[str, Any]:
    """Draw station states; repair until constraints hold (deterministic on seed).

    Repair order (only runs when needed; each step may consume rng):
    1) ensure ≥1 non-green LED
    2) ensure ≥1 lit panel
    3) ensure ≥1 dark panel (if that removes the only lit, force lit on another)
    """
    rng = Random(seed)
    stations: dict[str, dict[str, Any]] = {}
    for sid in STATION_IDS:
        stations[sid] = {
            'leds': [rng.choice(LED_STATES) for _ in range(3)],
            'panel': rng.choice(PANEL_STATES),
            'leak': STATION_POSE[sid]['leak'],
        }

    led_slots = [(sid, i) for sid in STATION_IDS for i in range(3)]
    if not any(led != 'green' for sid in STATION_IDS for led in stations[sid]['leds']):
        sid, idx = rng.choice(led_slots)
        stations[sid]['leds'][idx] = rng.choice(NON_GREEN)

    if 'lit' not in _panel_set(stations):
        stations[rng.choice(STATION_IDS)]['panel'] = 'lit'

    if 'dark' not in _panel_set(stations):
        lit_ids = [sid for sid in STATION_IDS if stations[sid]['panel'] == 'lit']
        forced_dark = rng.choice(lit_ids)
        stations[forced_dark]['panel'] = 'dark'
        if 'lit' not in _panel_set(stations):
            others = [sid for sid in STATION_IDS if sid != forced_dark]
            stations[rng.choice(others)]['panel'] = 'lit'

    assert any(led != 'green' for sid in STATION_IDS for led in stations[sid]['leds'])
    assert 'lit' in _panel_set(stations) and 'dark' in _panel_set(stations)
    assert stations['B']['leak'] == 'fixed_fail'
    assert stations['A']['leak'] == 'none' and stations['C']['leak'] == 'none'

    return {
        'schema': 'vision_scene_gt_v1',
        'seed': seed,
        'leak_fixed_model': 'cabinet_b_inspection_region',
        'note': 'leak does not follow seed; only leds/panel are randomized',
        'constraints': {
            'led_enum': list(LED_STATES),
            'panel_enum': list(PANEL_STATES),
            'min_non_green_led': 1,
            'min_lit_panel': 1,
            'min_dark_panel': 1,
        },
        'stations': stations,
    }


def write_gt(path: Path, gt: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(gt, indent=2, sort_keys=True) + '\n'
    path.write_text(text, encoding='utf-8')


def _material_xml(mat: dict[str, Any]) -> str:
    parts = [
        f"<ambient>{mat['ambient']}</ambient>",
        f"<diffuse>{mat['diffuse']}</diffuse>",
    ]
    if mat.get('emissive'):
        parts.append(f"<emissive>{mat['emissive']}</emissive>")
    return ''.join(parts)


def led_model_xml(name: str, pose: tuple[float, ...], state: str) -> str:
    mat = LED_MAT[state]
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <pose>{_pose_str(pose)}</pose>
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><cylinder><radius>0.04</radius><length>0.02</length></cylinder></geometry>
        <material>{_material_xml(mat)}</material>
      </visual>
    </link>
  </model>
</sdf>
"""


def panel_model_xml(name: str, pose: tuple[float, ...], state: str) -> str:
    mat = PANEL_MAT[state]
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <pose>{_pose_str(pose)}</pose>
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>0.30 0.01 0.15</size></box></geometry>
        <material>{_material_xml(mat)}</material>
      </visual>
    </link>
  </model>
</sdf>
"""


def apply_to_gazebo(gt: dict[str, Any], timeout_sec: float = 30.0) -> None:
    """Replace LED/panel visuals in a running Gazebo. Leak model is never touched."""
    import rclpy
    from gazebo_msgs.srv import DeleteEntity, SpawnEntity
    from geometry_msgs.msg import Pose
    from rclpy.node import Node

    def log(msg: str) -> None:
        print(msg, flush=True)

    log('apply: waiting for /delete_entity and /spawn_entity ...')
    rclpy.init()
    node = Node('apply_vision_scene_gt')
    delete_cli = node.create_client(DeleteEntity, '/delete_entity')
    spawn_cli = node.create_client(SpawnEntity, '/spawn_entity')

    if not delete_cli.wait_for_service(timeout_sec=timeout_sec):
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError(
            'Gazebo /delete_entity not ready. Launch test_room first:\n'
            '  ros2 launch simulation_worlds gazebo_robot_v0.launch.py '
            'world:=$HOME/inspection-robot/ros2_ws/src/simulation_worlds/worlds/test_room.world'
        )
    if not spawn_cli.wait_for_service(timeout_sec=5.0):
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError('Gazebo /spawn_entity not ready')

    def call_delete(name: str) -> None:
        req = DeleteEntity.Request()
        req.name = name
        fut = delete_cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
        if not fut.done():
            raise RuntimeError(f'delete timeout: {name}')
        # Missing entity → success=False; OK to continue.

    def call_spawn(name: str, xml: str, pose: tuple[float, ...]) -> None:
        req = SpawnEntity.Request()
        req.name = name
        req.xml = xml
        req.robot_namespace = ''
        x, y, z, roll, pitch, yaw = pose
        req.initial_pose = Pose()
        req.initial_pose.position.x = float(x)
        req.initial_pose.position.y = float(y)
        req.initial_pose.position.z = float(z)
        qx, qy, qz, qw = _rpy_to_quat(roll, pitch, yaw)
        req.initial_pose.orientation.x = qx
        req.initial_pose.orientation.y = qy
        req.initial_pose.orientation.z = qz
        req.initial_pose.orientation.w = qw
        req.reference_frame = 'world'
        fut = spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
        if not fut.done() or fut.result() is None or not fut.result().success:
            msg = fut.result().status_message if fut.done() and fut.result() else 'timeout'
            raise RuntimeError(f'spawn failed for {name}: {msg}')

    jobs: list[tuple[str, str, tuple[float, ...], str]] = []
    # (kind, name, pose, state)
    for sid in STATION_IDS:
        st = gt['stations'][sid]
        prefix = f'cabinet_{sid.lower()}'
        for i, state in enumerate(st['leds'], start=1):
            jobs.append(('led', f'{prefix}_led_{i}', STATION_POSE[sid]['leds'][i - 1], state))
        jobs.append(('panel', f'{prefix}_panel', STATION_POSE[sid]['panel'], st['panel']))

    try:
        total = len(jobs)
        for n, (kind, name, pose, state) in enumerate(jobs, start=1):
            log(f'apply [{n}/{total}] {name} -> {state}')
            call_delete(name)
            time.sleep(0.05)
            if kind == 'led':
                xml = led_model_xml(name, (0, 0, 0, 0, 0, 0), state)
            else:
                xml = panel_model_xml(name, (0, 0, 0, 0, 0, 0), state)
            call_spawn(name, xml, pose)
            time.sleep(0.05)
        log('apply: skip cabinet_b_inspection_region (fixed leak)')

        # Verify all targets exist (Gazebo occasionally drops a late spawn under load).
        from gazebo_msgs.srv import GetModelList

        list_cli = node.create_client(GetModelList, '/get_model_list')
        if not list_cli.wait_for_service(timeout_sec=5.0):
            log('apply: warn /get_model_list unavailable; skip verify')
        else:
            job_by_name = {name: (kind, name, pose, state) for kind, name, pose, state in jobs}

            def _model_names() -> set[str]:
                fut = list_cli.call_async(GetModelList.Request())
                rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
                if not fut.done() or fut.result() is None:
                    return set()
                return set(fut.result().model_names)

            for attempt in range(1, 4):
                names = _model_names()
                missing = [name for name in job_by_name if name not in names]
                leak_missing = 'cabinet_b_inspection_region' not in names
                if not missing and not leak_missing:
                    log(f'apply: verified {total} LED/panel models present')
                    break
                if leak_missing:
                    log('apply: warn leak model missing (not respawned; reload world)')
                if not missing:
                    break
                log(f'apply: retry {attempt}/3 missing {missing}')
                for name in missing:
                    kind, _, pose, state = job_by_name[name]
                    call_delete(name)
                    time.sleep(0.08)
                    xml = (
                        led_model_xml(name, (0, 0, 0, 0, 0, 0), state)
                        if kind == 'led'
                        else panel_model_xml(name, (0, 0, 0, 0, 0, 0), state)
                    )
                    call_spawn(name, xml, pose)
                    time.sleep(0.08)
            else:
                names = _model_names()
                missing = [name for name in job_by_name if name not in names]
                if missing:
                    raise RuntimeError(f'apply incomplete, still missing: {missing}')
                log(f'apply: verified {total} LED/panel models present')
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, required=True, help='RNG seed (reproducible GT)')
    parser.add_argument(
        '-o',
        '--output',
        type=Path,
        default=Path.home() / 'inspection-robot' / 'bags' / 'vision_gt' / 'gt.json',
        help='gt.json output path',
    )
    parser.add_argument(
        '--gt-only',
        action='store_true',
        help='Only write gt.json (no Gazebo apply)',
    )
    parser.add_argument(
        '--apply',
        action='store_true',
        help='Apply LED/panel visuals to running Gazebo (keeps leak fixed)',
    )
    args = parser.parse_args(argv)

    if not args.gt_only and not args.apply:
        # Default: write GT; apply if requested separately for safety.
        args.gt_only = True

    gt = generate_gt(args.seed)
    write_gt(args.output, gt)
    print(f'wrote {args.output}')
    print(json.dumps(gt['stations'], indent=2, sort_keys=True))

    if args.apply:
        print('apply: starting (12 LED/panel models; may take ~30s) ...', flush=True)
        apply_to_gazebo(gt)
        print('applied LED/panel visuals; leak model left unchanged', flush=True)

    return 0


if __name__ == '__main__':
    sys.exit(main())
