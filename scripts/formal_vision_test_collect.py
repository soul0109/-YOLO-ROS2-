#!/usr/bin/env python3
"""B2 formal vision Test collector — Gazebo-only, raw frames, no Nav2, no aug.

Writes bags/vision_5b_test_<date>/batches/<batch_id>/{images,states} + manifests.
Uses apply_vision_scene_gt seeds for normal coverage; mid-band materials / temp
occlusion for UNKNOWN & hard cases (synthetic=false; not image aug).

Does NOT freeze test_batches.txt — call formal_vision_test_freeze.py after review.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import yaml

SCRIPTS = Path(__file__).resolve().parent
REPO = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))

from apply_vision_scene_gt import (  # noqa: E402
    STATION_POSE as CABINET_POSE,
    apply_to_gazebo,
    generate_gt,
    led_model_xml,
    panel_model_xml,
    write_gt,
)
from capture_vision_station_frames import _grab_bgr  # noqa: E402
from c2_collect_expand import pose_at_distance  # noqa: E402

LABEL_VERSION = 'formal_test_b2_v0.1'
SCHEMA = 'vision_5b_frame_v1'
DEFAULT_RULES = (
    REPO / 'ros2_ws' / 'src' / 'vision_perception' / 'config' / 'roi_rules.yaml'
)

# Mid-band / ambiguous materials (raw scene — not image aug).
PANEL_MID = {
    'ambient': '0.12 0.12 0.14 1',
    'diffuse': '0.18 0.18 0.22 1',
    'emissive': '0.08 0.10 0.12 1',  # target HSV V between dark_v_max and lit_v_min
}
LED_AMBIG = {
    'ambient': '0.35 0.10 0.35 1',
    'diffuse': '0.45 0.15 0.45 1',
    'emissive': '0.55 0.20 0.55 1',  # magenta — matches no R/G/Y/off rule
}
PANEL_BLOOM = {
    'ambient': '1 1 1 1',
    'diffuse': '1 1 1 1',
    'emissive': '1 1 1 1',
}
LED_BLOOM = {
    'ambient': '1 1 0.8 1',
    'diffuse': '1 1 0.8 1',
    'emissive': '1 1 0.6 1',
}


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _load_rules(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _detections_from_rules(rules: dict[str, Any], distance_m: float) -> list[dict]:
    rois = rules['rois']
    dets = [
        {
            'class_name': 'indicator_light',
            'bbox_xyxy': [int(x0), int(rois['led_row_y'][0]), int(x1), int(rois['led_row_y'][1])],
        }
        for x0, x1 in rois['led_slots_x']
    ]
    dets.append({'class_name': 'panel', 'bbox_xyxy': [int(x) for x in rois['panel']]})
    dets.append(
        {
            'class_name': 'inspection_region',
            'bbox_xyxy': [int(x) for x in rois['leak_floor']],
        }
    )
    # Non-1.2 boxes are display-only candidates for GT placement; mark untrusted.
    return dets


def _teleport(pose: dict[str, float]) -> None:
    cmd = [
        'gz',
        'model',
        '-m',
        'robot_v0',
        '-x',
        str(pose['x']),
        '-y',
        str(pose['y']),
        '-z',
        str(pose['z']),
        '-Y',
        str(pose['yaw']),
    ]
    subprocess.run(cmd, check=True, capture_output=True, text=True)


def _verdict_from_states(states: dict[str, Any]) -> dict[str, str]:
    leds = states['leds']
    if any(x == 'UNKNOWN' for x in leds):
        led_v = 'UNKNOWN'
    elif all(x == 'green' for x in leds):
        led_v = 'PASS'
    else:
        led_v = 'FAIL'
    panel = states['panel']
    panel_v = (
        'UNKNOWN'
        if panel == 'UNKNOWN'
        else ('PASS' if panel == 'lit' else 'FAIL')
    )
    leak = states['leak']
    leak_v = (
        'UNKNOWN'
        if leak == 'UNKNOWN'
        else ('PASS' if leak == 'none' else 'FAIL')
    )
    return {'led': led_v, 'panel': panel_v, 'leak': leak_v}


def _apply_custom_station(
    sid: str,
    *,
    leds: list[str] | None = None,
    panel: str | None = None,
    panel_mat: dict | None = None,
    led_mats: list[dict | None] | None = None,
) -> None:
    """Replace one station's LED/panel visuals (leak untouched)."""
    import rclpy
    from gazebo_msgs.srv import DeleteEntity, SpawnEntity
    from geometry_msgs.msg import Pose
    from rclpy.node import Node

    from apply_vision_scene_gt import LED_MAT, PANEL_MAT, _rpy_to_quat

    prefix = f'cabinet_{sid.lower()}'
    poses = CABINET_POSE[sid]
    rclpy.init()
    node = Node('formal_test_apply_station')
    delete_cli = node.create_client(DeleteEntity, '/delete_entity')
    spawn_cli = node.create_client(SpawnEntity, '/spawn_entity')
    if not delete_cli.wait_for_service(timeout_sec=30.0):
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError('/delete_entity not ready')
    if not spawn_cli.wait_for_service(timeout_sec=5.0):
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError('/spawn_entity not ready')

    def call_delete(name: str) -> None:
        req = DeleteEntity.Request()
        req.name = name
        fut = delete_cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)

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
            raise RuntimeError(f'spawn failed {name}: {msg}')

    try:
        led_states = leds or ['green', 'green', 'green']
        for i, state in enumerate(led_states, start=1):
            name = f'{prefix}_led_{i}'
            pose = poses['leds'][i - 1]
            call_delete(name)
            time.sleep(0.05)
            custom = (led_mats or [None, None, None])[i - 1]
            if custom is not None:
                # inline material override via temporary LED_MAT key
                from apply_vision_scene_gt import _material_xml

                xml = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <pose>0 0 0 0 0 0</pose>
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><cylinder><radius>0.04</radius><length>0.02</length></cylinder></geometry>
        <material>{_material_xml(custom)}</material>
      </visual>
    </link>
  </model>
</sdf>
"""
            else:
                xml = led_model_xml(name, (0, 0, 0, 0, 0, 0), state)
            call_spawn(name, xml, pose)
            time.sleep(0.05)
        pname = f'{prefix}_panel'
        ppose = poses['panel']
        call_delete(pname)
        time.sleep(0.05)
        if panel_mat is not None:
            from apply_vision_scene_gt import _material_xml

            xml = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{pname}">
    <pose>0 0 0 0 0 0</pose>
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>0.30 0.01 0.15</size></box></geometry>
        <material>{_material_xml(panel_mat)}</material>
      </visual>
    </link>
  </model>
</sdf>
"""
        else:
            xml = panel_model_xml(pname, (0, 0, 0, 0, 0, 0), panel or 'lit')
        call_spawn(pname, xml, ppose)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _spawn_occlusion(sid: str, entity: str) -> None:
    """Temporary partial occlusion box in front of panel (raw hard case)."""
    import rclpy
    from gazebo_msgs.srv import SpawnEntity
    from geometry_msgs.msg import Pose
    from rclpy.node import Node

    from apply_vision_scene_gt import _rpy_to_quat

    panel = CABINET_POSE[sid]['panel']
    # Place slightly toward robot (+Y for A/B, -Y for C)
    if sid in ('A', 'B'):
        pose = (panel[0], panel[1] + 0.25, panel[2], 0.0, 0.0, 0.0)
    else:
        pose = (panel[0], panel[1] - 0.25, panel[2], 0.0, 0.0, 0.0)
    xml = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{entity}">
    <pose>0 0 0 0 0 0</pose>
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>0.18 0.06 0.18</size></box></geometry>
        <material>
          <ambient>0.05 0.05 0.05 1</ambient>
          <diffuse>0.08 0.08 0.08 1</diffuse>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
    rclpy.init()
    node = Node('formal_test_occ')
    spawn_cli = node.create_client(SpawnEntity, '/spawn_entity')
    if not spawn_cli.wait_for_service(timeout_sec=20.0):
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError('spawn_entity missing')
    req = SpawnEntity.Request()
    req.name = entity
    req.xml = xml
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
    node.destroy_node()
    rclpy.shutdown()
    if not fut.done() or fut.result() is None or not fut.result().success:
        raise RuntimeError(f'occlusion spawn failed: {entity}')


def _delete_entity(name: str) -> None:
    import rclpy
    from gazebo_msgs.srv import DeleteEntity
    from rclpy.node import Node

    rclpy.init()
    node = Node('formal_test_del')
    cli = node.create_client(DeleteEntity, '/delete_entity')
    if cli.wait_for_service(timeout_sec=10.0):
        req = DeleteEntity.Request()
        req.name = name
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    node.destroy_node()
    rclpy.shutdown()


def _write_frame(
    batch_dir: Path,
    *,
    batch_id: str,
    image_id: str,
    bgr,
    meta: dict[str, Any],
) -> None:
    img_dir = batch_dir / 'images'
    st_dir = batch_dir / 'states'
    img_dir.mkdir(parents=True, exist_ok=True)
    st_dir.mkdir(parents=True, exist_ok=True)
    png_name = f'{image_id}.png'
    cv2.imwrite(str(img_dir / png_name), bgr)
    meta = dict(meta)
    meta['file_name'] = f'images/{png_name}'
    meta['image_id'] = image_id
    meta['batch_id'] = batch_id
    (st_dir / f'{image_id}.json').write_text(
        json.dumps(meta, indent=2, ensure_ascii=False) + '\n', encoding='utf-8'
    )
    # also place sidecar next to convenience path used by harness rglob
    (batch_dir / f'{image_id}.json').write_text(
        json.dumps(meta, indent=2, ensure_ascii=False) + '\n', encoding='utf-8'
    )


def capture_one(
    *,
    batch_dir: Path,
    batch_id: str,
    image_id: str,
    station_id: str,
    distance_m: float,
    seed: int | None,
    gt_scene: dict[str, Any],
    states: dict[str, Any],
    rules: dict[str, Any],
    checklist_ids: list[str],
    hard_case: list[str],
    unknown_case_id: str | None,
    notes: str,
    pose_override: dict[str, float] | None = None,
) -> dict[str, Any]:
    pose = pose_override or pose_at_distance(station_id, distance_m)
    print(f'  capture {image_id} @ {station_id} d={distance_m}', flush=True)
    _teleport(pose)
    grabbed = _grab_bgr(timeout_sec=15.0)
    dets = _detections_from_rules(rules, distance_m)
    trusted = abs(distance_m - 1.2) < 1e-6
    meta = {
        'schema': SCHEMA,
        'image_id': image_id,
        'batch_id': batch_id,
        'split': None,
        'usage': 'formal_test_candidate',
        'station_id': station_id,
        'distance_m': float(distance_m),
        'robot_pose': pose,
        'pose_frame_id': 'world',
        'frame_id': grabbed.get('frame_id') or 'camera_link',
        'image_stamp': grabbed['stamp'],
        'captured_at': _now_iso(),
        'seed': seed,
        'gt_scene': gt_scene,
        'states': states,
        'verdict_gt': _verdict_from_states(states),
        'detections': dets,
        'checklist_ids': checklist_ids,
        'hard_case': hard_case,
        'unknown_case_id': unknown_case_id,
        'notes': notes,
        'label_version': LABEL_VERSION,
        'synthetic': False,
        'parent_frame_id': None,
        'transform_type': None,
        'boxes_untrusted': not trusted,
        'boxes_candidate': trusted,
        'boxes_review_passed': False,
        'review': {'status': 'scene_gt_accepted', 'note': 'states from scene seed/recipe'},
    }
    _write_frame(batch_dir, batch_id=batch_id, image_id=image_id, bgr=grabbed['img'], meta=meta)
    return meta


def plan_sessions(date_tag: str) -> list[dict[str, Any]]:
    """3 independent sessions / batch_ids; seeds ≥400 (avoid C2 ≤204)."""
    s1_seeds = list(range(401, 409))  # 8 seeds × ABC = 24
    s3_seeds = list(range(420, 424))  # 4 × ABC = 12
    return [
        {
            'session': 1,
            'batch_id': f'test_{date_tag}_seed401_session1',
            'role': 'normal_rare',
            'seeds': s1_seeds,
        },
        {
            'session': 2,
            'batch_id': f'test_{date_tag}_seed410_session2',
            'role': 'unknown_hard',
        },
        {
            'session': 3,
            'batch_id': f'test_{date_tag}_seed420_session3',
            'role': 'fill_distance',
            'seeds': s3_seeds,
        },
    ]


def run_session_normal(
    root: Path, sess: dict[str, Any], rules: dict[str, Any]
) -> list[dict[str, Any]]:
    batch_id = sess['batch_id']
    batch_dir = root / 'batches' / batch_id
    batch_dir.mkdir(parents=True, exist_ok=True)
    metas: list[dict[str, Any]] = []
    for seed in sess['seeds']:
        gt = generate_gt(seed)
        gt_path = batch_dir / f'gt_seed{seed}.json'
        write_gt(gt_path, gt)
        print(f'apply seed={seed}', flush=True)
        apply_to_gazebo(gt)
        for sid in ('A', 'B', 'C'):
            st = gt['stations'][sid]
            states = {'leds': list(st['leds']), 'panel': st['panel'], 'leak': st['leak']}
            checklist = [f'N-{sid}1', 'T-A12' if sid == 'A' else ('T-B12' if sid == 'B' else 'T-C12')]
            if 'off' in states['leds']:
                checklist.append('T-OFF')
            if 'red' in states['leds']:
                checklist.append('T-RED')
            if 'yellow' in states['leds']:
                checklist.append('T-YEL')
            if 'green' in states['leds']:
                checklist.append('T-GREEN')
            checklist.append('T-LIT' if states['panel'] == 'lit' else 'T-DARK')
            checklist.append('T-LEAK' if states['leak'] == 'fixed_fail' else 'T-NOLK')
            image_id = f'{batch_id}_{sid}_d1p2_s{seed}'
            metas.append(
                capture_one(
                    batch_dir=batch_dir,
                    batch_id=batch_id,
                    image_id=image_id,
                    station_id=sid,
                    distance_m=1.2,
                    seed=seed,
                    gt_scene=states,
                    states=states,
                    rules=rules,
                    checklist_ids=sorted(set(checklist)),
                    hard_case=['none'],
                    unknown_case_id=None,
                    notes=f'formal raw; seed={seed}',
                )
            )
    _write_batch_manifest(batch_dir, batch_id, metas, sess)
    return metas


def run_session_unknown_hard(
    root: Path, sess: dict[str, Any], rules: dict[str, Any]
) -> list[dict[str, Any]]:
    """24 raw frames: UNKNOWN + hard via scene materials / temp occlusion / crop pose."""
    batch_id = sess['batch_id']
    batch_dir = root / 'batches' / batch_id
    batch_dir.mkdir(parents=True, exist_ok=True)
    metas: list[dict[str, Any]] = []
    recipes: list[dict[str, Any]] = []
    # 12 UNKNOWN-focused (raw scene materials; not image aug)
    for sid in ('A', 'B', 'C'):
        recipes.append({'kind': 'U_panel', 'station': sid, 'seed': 410 + ord(sid)})
        recipes.append({'kind': 'U_led', 'station': sid, 'seed': 413 + ord(sid)})
    recipes.append({'kind': 'U_panel', 'station': 'A', 'seed': 430})
    recipes.append({'kind': 'U_led', 'station': 'B', 'seed': 431})
    recipes.append({'kind': 'U_panel', 'station': 'C', 'seed': 432})
    recipes.append({'kind': 'U_led', 'station': 'A', 'seed': 433})
    recipes.append({'kind': 'U_panel', 'station': 'B', 'seed': 434})
    recipes.append({'kind': 'U_led', 'station': 'C', 'seed': 435})
    # 12 hard-focused (may also carry UNKNOWN labels)
    for sid in ('A', 'B', 'C'):
        recipes.append({'kind': 'H_OVR', 'station': sid, 'seed': 440 + ord(sid)})
        recipes.append({'kind': 'H_OCC', 'station': sid, 'seed': 443 + ord(sid)})
        recipes.append({'kind': 'H_CROP', 'station': sid, 'seed': 446 + ord(sid)})
        recipes.append({'kind': 'H_OVR', 'station': sid, 'seed': 450 + ord(sid)})

    if len(recipes) != 24:
        raise RuntimeError(f'unknown_hard recipes want 24 got {len(recipes)}')

    for idx, rec in enumerate(recipes, start=1):
        sid = rec['station']
        kind = rec['kind']
        seed = rec['seed']
        # baseline GT then override target station
        gt = generate_gt(seed)
        apply_to_gazebo(gt)
        st0 = gt['stations'][sid]
        states = {
            'leds': list(st0['leds']),
            'panel': st0['panel'],
            'leak': st0['leak'],
        }
        hard = ['none']
        unknown_id = None
        checklist = [f'T-{sid}12']
        notes = f'formal raw recipe {kind}'
        pose_override = None
        occ_name = None

        if kind == 'U_panel':
            _apply_custom_station(sid, leds=states['leds'], panel_mat=PANEL_MID)
            states['panel'] = 'UNKNOWN'
            unknown_id = 'U_panel_ambiguous'
            checklist += ['T-U-PNL', 'U-PNL']
        elif kind == 'U_led':
            mats: list[dict | None] = [None, LED_AMBIG, None]
            _apply_custom_station(
                sid, leds=states['leds'], panel=states['panel'], led_mats=mats
            )
            states['leds'][1] = 'UNKNOWN'
            unknown_id = 'U_led_ambiguous'
            checklist += ['T-U-LED', 'U-LED']
        elif kind == 'H_OVR':
            _apply_custom_station(
                sid,
                leds=['yellow', 'yellow', 'yellow'],
                panel_mat=PANEL_BLOOM,
                led_mats=[LED_BLOOM, LED_BLOOM, LED_BLOOM],
            )
            states['leds'] = ['yellow', 'yellow', 'yellow']
            # bloom may force UNKNOWN on panel; label what we intend for GT
            states['panel'] = 'UNKNOWN'
            unknown_id = 'U_panel_overexpose'
            hard = ['overexpose']
            checklist += ['T-H-OVR', 'H-OVR', 'T-U-PNL']
        elif kind == 'H_OCC':
            occ_name = f'formal_test_tmp_occ_{sid.lower()}_{idx}'
            _spawn_occlusion(sid, occ_name)
            states['panel'] = 'UNKNOWN'
            unknown_id = 'U_panel_occluded'
            hard = ['occlusion']
            checklist += ['T-H-OCC', 'H-OCC', 'T-U-PNL']
            notes += f'; occ={occ_name}'
        elif kind == 'H_CROP':
            pose_override = pose_at_distance(sid, 1.2)
            # lateral shift to clip targets
            pose_override = dict(pose_override)
            pose_override['x'] = float(pose_override['x']) + (0.35 if sid != 'C' else -0.35)
            hard = ['crop']
            checklist += ['T-H-MISS', 'H-MISS']
            notes += '; lateral crop pose'
        else:
            raise ValueError(kind)

        image_id = f'{batch_id}_{sid}_{kind}_{idx:02d}'
        try:
            metas.append(
                capture_one(
                    batch_dir=batch_dir,
                    batch_id=batch_id,
                    image_id=image_id,
                    station_id=sid,
                    distance_m=1.2,
                    seed=seed,
                    gt_scene={
                        'leds': list(gt['stations'][sid]['leds']),
                        'panel': gt['stations'][sid]['panel'],
                        'leak': gt['stations'][sid]['leak'],
                        'recipe': kind,
                    },
                    states=states,
                    rules=rules,
                    checklist_ids=sorted(set(checklist)),
                    hard_case=hard,
                    unknown_case_id=unknown_id,
                    notes=notes,
                    pose_override=pose_override,
                )
            )
        finally:
            if occ_name:
                try:
                    _delete_entity(occ_name)
                except Exception as exc:  # noqa: BLE001
                    print(f'warn: delete {occ_name}: {exc}', flush=True)

    _write_batch_manifest(batch_dir, batch_id, metas, sess)
    return metas


def run_session_fill_distance(
    root: Path, sess: dict[str, Any], rules: dict[str, Any]
) -> list[dict[str, Any]]:
    batch_id = sess['batch_id']
    batch_dir = root / 'batches' / batch_id
    batch_dir.mkdir(parents=True, exist_ok=True)
    metas: list[dict[str, Any]] = []
    # 12 @ 1.2
    for seed in sess['seeds']:
        gt = generate_gt(seed)
        write_gt(batch_dir / f'gt_seed{seed}.json', gt)
        apply_to_gazebo(gt)
        for sid in ('A', 'B', 'C'):
            st = gt['stations'][sid]
            states = {'leds': list(st['leds']), 'panel': st['panel'], 'leak': st['leak']}
            metas.append(
                capture_one(
                    batch_dir=batch_dir,
                    batch_id=batch_id,
                    image_id=f'{batch_id}_{sid}_d1p2_s{seed}',
                    station_id=sid,
                    distance_m=1.2,
                    seed=seed,
                    gt_scene=states,
                    states=states,
                    rules=rules,
                    checklist_ids=[f'T-{sid}12'],
                    hard_case=['none'],
                    unknown_case_id=None,
                    notes=f'fill 1.2 seed={seed}',
                )
            )
    # 5 @ 1.0 and 5 @ 1.5 — cover A/B/C
    dist_plan = [
        (1.0, 'A', 460),
        (1.0, 'B', 461),
        (1.0, 'C', 462),
        (1.0, 'A', 463),
        (1.0, 'B', 464),
        (1.5, 'A', 470),
        (1.5, 'B', 471),
        (1.5, 'C', 472),
        (1.5, 'A', 473),
        (1.5, 'C', 474),
    ]
    for dist, sid, seed in dist_plan:
        gt = generate_gt(seed)
        apply_to_gazebo(gt)
        st = gt['stations'][sid]
        states = {'leds': list(st['leds']), 'panel': st['panel'], 'leak': st['leak']}
        tag = 'd1p0' if abs(dist - 1.0) < 1e-6 else 'd1p5'
        metas.append(
            capture_one(
                batch_dir=batch_dir,
                batch_id=batch_id,
                image_id=f'{batch_id}_{sid}_{tag}_s{seed}',
                station_id=sid,
                distance_m=dist,
                seed=seed,
                gt_scene=states,
                states=states,
                rules=rules,
                checklist_ids=['T-D10' if dist < 1.2 else 'T-D15', f'T-{sid}12'],
                hard_case=['none'],
                unknown_case_id=None,
                notes=f'aux distance {dist}',
            )
        )
    _write_batch_manifest(batch_dir, batch_id, metas, sess)
    return metas


def _write_batch_manifest(
    batch_dir: Path, batch_id: str, metas: list[dict[str, Any]], sess: dict[str, Any]
) -> None:
    man = {
        'schema': 'vision_5b_batch_manifest_v1',
        'batch_id': batch_id,
        'session': sess.get('session'),
        'role': sess.get('role'),
        'n_frames': len(metas),
        'synthetic_forbidden': True,
        'image_ids': [m['image_id'] for m in metas],
        'created_at': _now_iso(),
    }
    (batch_dir / 'batch_manifest.json').write_text(
        json.dumps(man, indent=2, ensure_ascii=False) + '\n', encoding='utf-8'
    )
    (batch_dir / 'status.json').write_text(
        json.dumps(
            {
                'batch_id': batch_id,
                'status': 'captured',
                'n_frames': len(metas),
                'frozen': False,
                'updated_at': _now_iso(),
            },
            indent=2,
        )
        + '\n',
        encoding='utf-8',
    )


def coverage_summary(metas: list[dict[str, Any]]) -> dict[str, Any]:
    main = [m for m in metas if abs(float(m['distance_m']) - 1.2) < 1e-6]
    d10 = [m for m in metas if abs(float(m['distance_m']) - 1.0) < 1e-6]
    d15 = [m for m in metas if abs(float(m['distance_m']) - 1.5) < 1e-6]
    by_station = {s: sum(1 for m in main if m['station_id'] == s) for s in ('A', 'B', 'C')}
    unk = sum(1 for m in main if m.get('unknown_case_id'))
    hard = sum(1 for m in main if m.get('hard_case') and m['hard_case'] != ['none'])
    batches = sorted({m['batch_id'] for m in metas})
    return {
        'n_total': len(metas),
        'n_main_1_2': len(main),
        'n_d1_0': len(d10),
        'n_d1_5': len(d15),
        'by_station_main': by_station,
        'n_unknown_main': unk,
        'n_hard_main': hard,
        'batch_ids': batches,
        'gates': {
            'ge_60_main': len(main) >= 60,
            'ge_3_batches': len(batches) >= 3,
            'unknown_ge_10': unk >= 10,
            'hard_ge_10': hard >= 10,
            'd10_ge_5': len(d10) >= 5,
            'd15_ge_5': len(d15) >= 5,
            'stations_covered': all(by_station[s] >= 1 for s in ('A', 'B', 'C')),
        },
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        '--out',
        type=Path,
        default=None,
        help='output root (default bags/vision_5b_test_<YYYYMMDD>/)',
    )
    ap.add_argument('--date-tag', default=datetime.now().strftime('%Y%m%d'))
    ap.add_argument('--rules', type=Path, default=DEFAULT_RULES)
    ap.add_argument(
        '--sessions',
        default='1,2,3',
        help='which sessions to run (comma), default all',
    )
    args = ap.parse_args(argv)

    root = args.out or (REPO / 'bags' / f'vision_5b_test_{args.date_tag}')
    root.mkdir(parents=True, exist_ok=True)
    rules = _load_rules(args.rules)
    sessions = plan_sessions(args.date_tag)
    want = {int(x) for x in args.sessions.split(',') if x.strip()}
    all_metas: list[dict[str, Any]] = []

    for sess in sessions:
        if sess['session'] not in want:
            continue
        print(f"=== session {sess['session']} {sess['batch_id']} ({sess['role']}) ===", flush=True)
        if sess['role'] == 'normal_rare':
            all_metas.extend(run_session_normal(root, sess, rules))
        elif sess['role'] == 'unknown_hard':
            all_metas.extend(run_session_unknown_hard(root, sess, rules))
        elif sess['role'] == 'fill_distance':
            all_metas.extend(run_session_fill_distance(root, sess, rules))
        else:
            raise ValueError(sess['role'])

    cov = coverage_summary(all_metas)
    overview = {
        'schema': 'vision_5b_test_collect_overview_v1',
        'root': str(root),
        'created_at': _now_iso(),
        'n_frames': len(all_metas),
        'coverage': cov,
        'label_version': LABEL_VERSION,
        'note': 'captured; not frozen until formal_vision_test_freeze.py',
    }
    (root / 'manifests').mkdir(parents=True, exist_ok=True)
    (root / 'manifests' / 'collect_overview.json').write_text(
        json.dumps(overview, indent=2, ensure_ascii=False) + '\n', encoding='utf-8'
    )
    print(json.dumps(cov, indent=2))
    print(f'done -> {root}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
