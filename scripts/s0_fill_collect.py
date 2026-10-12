#!/usr/bin/env python3
"""S0-fill: collect 1.2m raw UNKNOWN + balanced A/B/C (no YOLO train / no test freeze).

Raw UNKNOWN = capture after Gazebo mid-band materials (not image aug).
New batch_ids only; refuse --force wipe of prior evidence.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from apply_vision_scene_gt import (  # noqa: E402
    STATION_POSE,
    _material_xml,
    _rpy_to_quat,
    generate_gt,
    led_model_xml,
    panel_model_xml,
)
from c1_collect_min import (  # noqa: E402
    DEFAULT_RULES,
    LABEL_VERSION,
    _base_meta,
    _detections_from_rules,
    _load_rules,
    _prepare_batch_dir,
    _write_frame,
)
from c2_collect_expand import (  # noqa: E402
    CameraSession,
    apply_box_tier_fields,
    collect_live_plan,
    coverage_gaps,
    excluded_ids,
    migrate_box_tier_sidecars,
    pose_at_distance,
    spot_check_c2,
    write_coverage_md,
    write_overview,
    _teleport_pose,
)

REPO = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = REPO / 'bags' / 'vision_5b_c2'

# Mid-band materials (scene-applied; capture is raw)
PANEL_MID_MAT = {
    'ambient': '0.18 0.18 0.22 1',
    'diffuse': '0.28 0.28 0.32 1',
    'emissive': '0.12 0.12 0.14 1',
}
LED_AMB_MAT = {
    'ambient': '0.18 0.14 0.10 1',
    'diffuse': '0.22 0.16 0.12 1',
    'emissive': '0.28 0.22 0.12 1',  # dull amber — not pure R/G/Y/off
}
LEAK_MID_POSE = (8.45, 0.72, 0.005, 0.0, 0.0, 0.0)


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


class GazeboEntities:
    """delete/spawn in an already-initialized rclpy context (shared with CameraSession)."""

    def __init__(self, rclpy_mod, node) -> None:
        from gazebo_msgs.srv import DeleteEntity, SpawnEntity

        self._rclpy = rclpy_mod
        self.node = node
        self.dcli = node.create_client(DeleteEntity, '/delete_entity')
        self.scli = node.create_client(SpawnEntity, '/spawn_entity')
        if not self.dcli.wait_for_service(timeout_sec=20.0):
            raise RuntimeError('/delete_entity unavailable')
        if not self.scli.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('/spawn_entity unavailable')

    def delete(self, name: str) -> None:
        from gazebo_msgs.srv import DeleteEntity

        req = DeleteEntity.Request()
        req.name = name
        fut = self.dcli.call_async(req)
        self._rclpy.spin_until_future_complete(self.node, fut, timeout_sec=15.0)

    def spawn(self, name: str, xml: str, pose: tuple[float, ...]) -> None:
        from gazebo_msgs.srv import SpawnEntity
        from geometry_msgs.msg import Pose

        req = SpawnEntity.Request()
        req.name = name
        req.xml = xml
        req.reference_frame = 'world'
        x, y, z, roll, pitch, yaw = pose
        req.initial_pose = Pose()
        req.initial_pose.position.x = float(x)
        req.initial_pose.position.y = float(y)
        req.initial_pose.position.z = float(z)
        qx, qy, qz, qw = _rpy_to_quat(roll, pitch, yaw)
        o = req.initial_pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        fut = self.scli.call_async(req)
        self._rclpy.spin_until_future_complete(self.node, fut, timeout_sec=20.0)
        if not fut.done() or fut.result() is None or not fut.result().success:
            raise RuntimeError(f'spawn failed: {name}')


def _panel_xml_mid(name: str) -> str:
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <pose>0 0 0 0 0 0</pose>
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>0.30 0.01 0.15</size></box></geometry>
        <material>{_material_xml(PANEL_MID_MAT)}</material>
      </visual>
    </link>
  </model>
</sdf>
"""


def _led_xml_amb(name: str) -> str:
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <pose>0 0 0 0 0 0</pose>
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><cylinder><radius>0.04</radius><length>0.02</length></cylinder></geometry>
        <material>{_material_xml(LED_AMB_MAT)}</material>
      </visual>
    </link>
  </model>
</sdf>
"""


def _leak_xml_mid(name: str, size_xy: tuple[float, float]) -> str:
    sx, sy = size_xy
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <pose>0 0 0 0 0 0</pose>
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>{sx} {sy} 0.01</size></box></geometry>
        <material>
          <ambient>0.0 0.55 0.55 1</ambient>
          <diffuse>0.0 0.75 0.7 1</diffuse>
          <emissive>0.0 0.45 0.4 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def _apply_seed_subprocess(seed: int, out: Path) -> dict[str, Any]:
    import subprocess

    cmd = [
        sys.executable,
        str(REPO / 'scripts' / 'apply_vision_scene_gt.py'),
        '--seed',
        str(seed),
        '--apply',
        '-o',
        str(out),
    ]
    subprocess.run(cmd, check=True)
    return json.loads(out.read_text(encoding='utf-8'))


def _restore_station_visuals(gz: GazeboEntities, sid: str, st: dict[str, Any]) -> None:
    prefix = f'cabinet_{sid.lower()}'
    for i, pose in enumerate(STATION_POSE[sid]['leds'], start=1):
        name = f'{prefix}_led_{i}'
        gz.delete(name)
        time.sleep(0.05)
        gz.spawn(name, led_model_xml(name, (0, 0, 0, 0, 0, 0), st['leds'][i - 1]), pose)
    pname = f'{prefix}_panel'
    gz.delete(pname)
    time.sleep(0.05)
    gz.spawn(
        pname,
        panel_model_xml(pname, (0, 0, 0, 0, 0, 0), st['panel']),
        STATION_POSE[sid]['panel'],
    )


def _restore_leak_full(gz: GazeboEntities) -> None:
    name = 'cabinet_b_inspection_region'
    gz.delete(name)
    time.sleep(0.08)
    xml = _leak_xml_mid(name, (0.22, 0.16))  # full FAIL size
    gz.spawn(name, xml, LEAK_MID_POSE)


def collect_unknown_raw(
    root: Path,
    batch_id: str,
    rules: dict[str, Any],
    cam: CameraSession,
    gz: GazeboEntities,
    *,
    settle_s: float,
) -> dict[str, Any]:
    batch_dir = root / 'batches' / batch_id
    _prepare_batch_dir(batch_dir, force=False)
    frames: list[str] = []
    dets = _detections_from_rules(rules)
    seq = 0

    # Base seed for non-target stations
    base = _apply_seed_subprocess(42, batch_dir / 'gt_seed42.json')

    plan: list[tuple[str, str, dict[str, Any]]] = []
    # (station, kind, state_overrides notes)
    for sid in ('A', 'B', 'C'):
        plan.append((sid, 'U_panel', {'tag': 'U-PNL'}))
        plan.append((sid, 'U_led', {'tag': 'U-LED'}))
    plan.append(('B', 'U_leak', {'tag': 'U-LEAK', 'leak_size': (0.08, 0.06)}))
    plan.append(('B', 'U_leak', {'tag': 'U-LEAK', 'leak_size': (0.10, 0.07)}))
    # extras for ≥12
    plan.append(('A', 'U_panel', {'tag': 'U-PNL'}))
    plan.append(('C', 'U_led', {'tag': 'U-LED'}))
    plan.append(('B', 'U_panel', {'tag': 'U-PNL'}))
    plan.append(('A', 'U_led', {'tag': 'U-LED'}))

    for sid, kind, meta_extra in plan:
        st = {
            'leds': list(base['stations'][sid]['leds']),
            'panel': base['stations'][sid]['panel'],
            'leak': base['stations'][sid]['leak'],
        }
        # start from restored base look for this station
        _restore_station_visuals(gz, sid, base['stations'][sid])
        if sid == 'B':
            _restore_leak_full(gz)
        time.sleep(0.15)

        unknown_id = None
        checklist = [f'N-{sid}1', 'N-MIX']
        if kind == 'U_panel':
            pname = f'cabinet_{sid.lower()}_panel'
            gz.delete(pname)
            time.sleep(0.05)
            gz.spawn(pname, _panel_xml_mid(pname), STATION_POSE[sid]['panel'])
            st['panel'] = 'UNKNOWN'
            unknown_id = 'U_panel_midband_raw'
            checklist.append('U-PNL')
        elif kind == 'U_led':
            prefix = f'cabinet_{sid.lower()}'
            for i, pose in enumerate(STATION_POSE[sid]['leds'], start=1):
                name = f'{prefix}_led_{i}'
                gz.delete(name)
                time.sleep(0.04)
                gz.spawn(name, _led_xml_amb(name), pose)
            st['leds'] = ['UNKNOWN', 'UNKNOWN', 'UNKNOWN']
            unknown_id = 'U_led_ambiguous_raw'
            checklist.append('U-LED')
        elif kind == 'U_leak':
            name = 'cabinet_b_inspection_region'
            gz.delete(name)
            time.sleep(0.05)
            gz.spawn(name, _leak_xml_mid(name, meta_extra['leak_size']), LEAK_MID_POSE)
            st['leak'] = 'UNKNOWN'
            unknown_id = 'U_leak_midband_raw'
            checklist.append('U-LEAK')
        else:
            raise ValueError(kind)

        pose = pose_at_distance(sid, 1.2)
        _teleport_pose(pose)
        grabbed = cam.grab(settle_s=settle_s)
        seq += 1
        image_id = f'{batch_id}_{sid}_{kind}_{seq:04d}'
        notes = (
            f's0-fill raw UNKNOWN kind={kind}; scene mid-band materials '
            f'(not image aug); unknown_case_id={unknown_id}'
        )
        meta = _base_meta(
            station_id=sid,
            seed=42,
            distance_m=1.2,
            pose=pose,
            stamp=grabbed.get('stamp'),
            gt_scene=dict(st),
            states=st,
            detections=list(dets),
            checklist_ids=sorted(set(checklist)),
            hard_case=['none'],
            unknown_case_id=unknown_id,
            notes=notes,
            split=None,
        )
        meta['synthetic'] = False
        meta['parent_frame_id'] = None
        meta['transform_type'] = None
        meta['annotator'] = 's0_fill_collect'
        meta['image_id'] = image_id
        apply_box_tier_fields(meta, distance_m=1.2)
        _write_frame(batch_dir, batch_id=batch_id, image_id=image_id, bgr=grabbed['img'], meta=meta)
        frames.append(image_id)
        print(f'  [{batch_id}] {image_id} {st}', flush=True)

        # restore after each shot
        _restore_station_visuals(gz, sid, base['stations'][sid])
        if sid == 'B' or kind == 'U_leak':
            _restore_leak_full(gz)

    man = {
        'schema': 'vision_5b_batch_manifest_v1',
        'batch_id': batch_id,
        'split': None,
        'suggested_split': 'train',
        'n_frames': len(frames),
        'image_ids': frames,
        'label_version': LABEL_VERSION,
        'note': 'S0-fill raw UNKNOWN at 1.2m (scene mid-band); not aug; not formal test',
    }
    (batch_dir / 'batch_manifest.json').write_text(
        json.dumps(man, indent=2, sort_keys=True) + '\n', encoding='utf-8'
    )
    return man


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('-o', '--root', type=Path, default=DEFAULT_ROOT)
    p.add_argument('--rules', type=Path, default=DEFAULT_RULES)
    p.add_argument('--settle', type=float, default=0.9)
    p.add_argument('--run-id', default='')
    p.add_argument('--force', action='store_true', help='rejected for S0-fill')
    args = p.parse_args(argv)
    if args.force:
        print('S0-fill refuses --force', file=sys.stderr)
        return 3

    root = args.root
    rules = _load_rules(args.rules)
    rid = args.run_id or datetime.now().strftime('%Y%m%d')
    b_unk = f'c2_{rid}_s0_unk'
    b_bal = f'c2_{rid}_s0_bal'
    for bid in (b_unk, b_bal):
        if (root / 'batches' / bid).exists():
            print(f'refusing: {bid} exists (no --force); pick new --run-id', file=sys.stderr)
            return 3

    batch_mans: list[dict[str, Any]] = []

    # 1) Balanced A/B/C first (CameraSession + apply_seed subprocess)
    print(f'S0-fill BALANCED {b_bal} ...', flush=True)
    cam1 = CameraSession()
    try:
        plan = [(s, seed, 1.2) for seed in (201, 202, 203, 204) for s in ('A', 'B', 'C')]
        batch_mans.append(
            collect_live_plan(
                root, b_bal, plan, rules, cam1, force=False, settle_s=args.settle
            )
        )
    finally:
        cam1.close()

    # 2) Raw UNKNOWN (shared rclpy: camera + delete/spawn)
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    print(f'S0-fill UNKNOWN {b_unk} ...', flush=True)
    rclpy.init()
    node = Node('s0_fill_io')
    bridge = CvBridge()
    box: dict[str, Any] = {'img': None, 'stamp': None}

    def cb(msg: Image) -> None:
        box['img'] = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        box['stamp'] = {
            'sec': int(msg.header.stamp.sec),
            'nanosec': int(msg.header.stamp.nanosec),
        }

    node.create_subscription(Image, '/camera/image_raw', cb, 10)

    class CamWrap:
        def grab(self, settle_s: float = 0.9, timeout_s: float = 8.0) -> dict[str, Any]:
            box['img'] = None
            t0 = time.time() + settle_s
            while time.time() < t0:
                rclpy.spin_once(node, timeout_sec=0.05)
                box['img'] = None
            deadline = time.time() + timeout_s
            while time.time() < deadline and box['img'] is None:
                rclpy.spin_once(node, timeout_sec=0.05)
            if box['img'] is None:
                raise RuntimeError('camera timeout')
            return {'img': box['img'].copy(), 'stamp': dict(box['stamp'] or {})}

    try:
        gz = GazeboEntities(rclpy, node)
        batch_mans.append(
            collect_unknown_raw(root, b_unk, rules, CamWrap(), gz, settle_s=args.settle)
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    migrate_box_tier_sidecars(root)
    all_mans = []
    for mp in sorted((root / 'batches').glob('*/batch_manifest.json')):
        all_mans.append(json.loads(mp.read_text(encoding='utf-8')))
    write_overview(root, all_mans, round_id=f'{rid}_s0fill')
    (root / 'manifests' / 's0_fill_overview.json').write_text(
        json.dumps(
            {
                'schema': 'vision_5b_s0_fill_overview_v1',
                'new_batches': batch_mans,
                'created_at': _now_iso(),
                'claims': {'yolo_trained': False, 'test_frozen': False, 'pct90': False},
            },
            indent=2,
            ensure_ascii=False,
        )
        + '\n',
        encoding='utf-8',
    )
    metas = []
    for jp in sorted(root.glob('batches/*/images/*.json')):
        metas.append(json.loads(jp.read_text(encoding='utf-8')))
    # attach image_id
    for m in metas:
        m.setdefault('image_id', m.get('image_id'))
    from c2_collect_expand import _iter_metas

    metas = _iter_metas(root)
    cov = coverage_gaps(metas, excluded=excluded_ids(root))
    spot = spot_check_c2(root, rules)
    write_coverage_md(root, cov, spot)
    print(
        json.dumps(
            {
                'new_batches': {m['batch_id']: m['n_frames'] for m in batch_mans},
                'spot_ok': spot['ok'],
                'on_disk': cov['spec6_rollup']['total_images_on_disk'],
                'credited': cov['spec6_rollup']['total_images_credited'],
            },
            indent=2,
            ensure_ascii=False,
        )
    )
    return 0 if spot['ok'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
