#!/usr/bin/env python3
"""E1 station-event MVP: ROI Decision → JSONL events + screenshots + dedup.

Proves judgment can become a traceable event. Does NOT prove detector accuracy,
Nav2 arrival, Web, or 5B/90%. No AnomalyEvent.msg change (JSON on disk first).
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import uuid
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

# Reuse R1 offline judge (same directory).
sys.path.insert(0, str(Path(__file__).resolve().parent))
from roi_judge_d1 import DEFAULT_RULES, judge_one, load_rules  # noqa: E402

REPO = Path(__file__).resolve().parents[1]
DEFAULT_D1 = REPO / 'bags' / 'vision_d1_seed42'
DEFAULT_OUT = REPO / 'bags' / 'vision_e1_demo'
DEFAULT_ROUTE = (
    REPO
    / 'ros2_ws'
    / 'src'
    / 'inspection_mission'
    / 'config'
    / 'patrol_route.yaml'
)

REQUIRED_FIELDS = (
    'event_id',
    'mission_id',
    'station_id',
    'object_type',
    'verdict',
    'status',
    'reason',
    'confidence',
    'timestamp',
    'image_stamp',
    'station_pose',
    'frame_id',
    'image_path',
    'model_version',
    'roi_rules_version',
    'source',
    'amcl_error_m',
)

VERDICTS = frozenset({'PASS', 'FAIL', 'UNKNOWN'})
LIFECYCLE = frozenset({'open', 'acknowledged', 'closed'})


@dataclass
class Decision:
    """Thin structured judgment consumed by the event layer (not a ROIProvider)."""

    object_type: str
    verdict: str
    reason: str
    roi_source: str
    roi_rules_version: str
    measurements: dict[str, Any] = field(default_factory=dict)
    confidence: float | None = None

    def __post_init__(self) -> None:
        if self.verdict not in VERDICTS:
            raise ValueError(f'bad verdict: {self.verdict}')


def load_patrol_poses(route_path: Path) -> dict[str, dict[str, float]]:
    import yaml

    data = yaml.safe_load(route_path.read_text(encoding='utf-8'))
    out: dict[str, dict[str, float]] = {}
    for sid, xyz_yaw in data['stations'].items():
        x, y, yaw = xyz_yaw
        out[str(sid)] = {'x': float(x), 'y': float(y), 'yaw': float(yaw), 'z': 0.05}
    return out


def _stamp_to_epoch(stamp: dict[str, Any] | None, fallback: float) -> float:
    if not stamp:
        return fallback
    return float(stamp.get('sec', 0)) + float(stamp.get('nanosec', 0)) / 1e9


def decisions_from_r1(
    judge: dict[str, Any],
    rules_version: str,
) -> list[Decision]:
    """Map R1 rollups → one Decision per object_type (fixed_window ROI)."""
    obj = judge['objects']
    verd = judge['verdict']
    dbg = judge.get('debug', {})
    decisions: list[Decision] = []

    leds = obj['leds']
    led_v = verd['leds']
    if led_v == 'PASS':
        reason = 'all_leds_green'
    elif led_v == 'FAIL':
        reason = f'led_non_green:{",".join(leds)}'
    else:
        reason = 'led_unknown_in_slots'
    decisions.append(
        Decision(
            object_type='indicator_light',
            verdict=led_v,
            reason=reason,
            roi_source='fixed_window',
            roi_rules_version=rules_version,
            measurements={
                'leds': leds,
                'led_slots': dbg.get('led_slots'),
            },
            confidence=None,
        )
    )

    panel = obj['panel']
    panel_v = verd['panel']
    if panel_v == 'PASS':
        reason = 'panel_lit'
    elif panel_v == 'FAIL':
        reason = 'panel_dark'
    else:
        reason = str(dbg.get('panel', {}).get('reason', 'panel_unknown'))
    decisions.append(
        Decision(
            object_type='panel',
            verdict=panel_v,
            reason=reason,
            roi_source='fixed_window',
            roi_rules_version=rules_version,
            measurements={'panel': panel, **(dbg.get('panel') or {})},
            confidence=None,
        )
    )

    leak = obj['leak']
    leak_v = verd['leak']
    if leak_v == 'PASS':
        reason = 'leak_none'
    elif leak_v == 'FAIL':
        reason = 'leak_fixed_fail'
    else:
        reason = str(dbg.get('leak', {}).get('reason', 'leak_unknown'))
    decisions.append(
        Decision(
            object_type='inspection_region',
            verdict=leak_v,
            reason=reason,
            roi_source='fixed_window',
            roi_rules_version=rules_version,
            measurements={'leak': leak, **(dbg.get('leak') or {})},
            confidence=None,
        )
    )
    return decisions


def decision_bbox_fixture(rules_version: str) -> Decision:
    """Protocol hook only: fake bbox Decision shape — NOT a real YOLO detector."""
    return Decision(
        object_type='inspection_region',
        verdict='FAIL',
        reason='leak_fixed_fail',
        roi_source='bbox_fixture',
        roi_rules_version=rules_version,
        measurements={
            'bbox_xyxy': [240, 300, 400, 400],
            'bbox_note': (
                'protocol hook only; synthetic fixture ≠ real YOLO / trained detector'
            ),
            'leak': 'fixed_fail',
            'cyan_frac': 0.2218,
        },
        confidence=0.42,
    )


def decision_to_event(
    decision: Decision,
    *,
    mission_id: str,
    station_id: str,
    station_pose: dict[str, float],
    frame_id: str,
    image_path: str,
    image_stamp: dict[str, Any] | None,
    source: str,
    amcl_error_m: float | None,
) -> dict[str, Any]:
    return {
        'event_id': str(uuid.uuid4()),
        'mission_id': mission_id,
        'station_id': station_id,
        'object_type': decision.object_type,
        'verdict': decision.verdict,
        'status': 'open',  # lifecycle only — never PASS/FAIL
        'reason': decision.reason,
        'confidence': decision.confidence,
        # timestamp (/ alias created_at): wall-clock when the event is written.
        # image_stamp: image ROS time from the frame — NOT the same clock;
        # do not subtract them to compute latency.
        'timestamp': datetime.now(timezone.utc).isoformat(),
        'image_stamp': image_stamp,
        'station_pose': {
            'x': station_pose['x'],
            'y': station_pose['y'],
            'z': station_pose.get('z', 0.05),
            'yaw': station_pose['yaw'],
        },
        'frame_id': frame_id,
        'image_path': image_path,
        'model_version': None,
        'roi_rules_version': decision.roi_rules_version,
        'roi_source': decision.roi_source,
        'source': source,
        'amcl_error_m': amcl_error_m,
        'measurements': decision.measurements,
    }


class EventStore:
    """Append-only JSONL with mission+station+object+time-window dedup."""

    def __init__(self, events_path: Path, dedup_window_sec: float) -> None:
        self.events_path = events_path
        self.dedup_window_sec = float(dedup_window_sec)
        self._events: list[dict[str, Any]] = []
        if events_path.is_file():
            for line in events_path.read_text(encoding='utf-8').splitlines():
                line = line.strip()
                if line:
                    self._events.append(json.loads(line))

    def _epoch(self, ev: dict[str, Any]) -> float:
        stamp = ev.get('image_stamp')
        if stamp:
            return _stamp_to_epoch(stamp, 0.0)
        # fallback parse ISO timestamp
        ts = ev.get('timestamp')
        if isinstance(ts, str):
            try:
                return datetime.fromisoformat(ts.replace('Z', '+00:00')).timestamp()
            except ValueError:
                pass
        return 0.0

    def _same_key(self, a: dict[str, Any], b: dict[str, Any]) -> bool:
        return (
            a['mission_id'] == b['mission_id']
            and a['station_id'] == b['station_id']
            and a['object_type'] == b['object_type']
        )

    def find_in_window(self, candidate: dict[str, Any]) -> dict[str, Any] | None:
        t = self._epoch(candidate)
        best: dict[str, Any] | None = None
        best_dt = None
        for ev in self._events:
            if not self._same_key(ev, candidate):
                continue
            dt = abs(self._epoch(ev) - t)
            if dt <= self.dedup_window_sec and (best_dt is None or dt < best_dt):
                best = ev
                best_dt = dt
        return best

    def try_add(self, candidate: dict[str, Any]) -> tuple[str, dict[str, Any]]:
        """Returns (action, event). action: created|deduped|blocked_unknown_to_pass."""
        existing = self.find_in_window(candidate)
        if existing is None:
            self._events.append(candidate)
            return 'created', candidate

        # UNKNOWN must never become PASS (same key + window).
        if existing['verdict'] == 'UNKNOWN' and candidate['verdict'] == 'PASS':
            return 'blocked_unknown_to_pass', existing

        return 'deduped', existing

    def flush(self) -> None:
        self.events_path.parent.mkdir(parents=True, exist_ok=True)
        with self.events_path.open('w', encoding='utf-8') as f:
            for ev in self._events:
                f.write(json.dumps(ev, ensure_ascii=False, sort_keys=True) + '\n')

    @property
    def events(self) -> list[dict[str, Any]]:
        return list(self._events)


def copy_station_image(src_png: Path, out_images: Path, name: str) -> Path:
    out_images.mkdir(parents=True, exist_ok=True)
    dest = out_images / name
    if not dest.is_file() or dest.stat().st_size == 0:
        shutil.copy2(src_png, dest)
    return dest


def emit_for_station(
    *,
    store: EventStore,
    meta_path: Path,
    rules: dict[str, Any],
    rules_version: str,
    mission_id: str,
    poses: dict[str, dict[str, float]],
    out_dir: Path,
    include_pass: bool,
    include_bbox_fixture: bool,
    source: str,
    amcl_error_m: float | None,
) -> dict[str, Any]:
    judge = judge_one(meta_path, rules)
    meta = json.loads(meta_path.read_text(encoding='utf-8'))
    station_id = str(meta['station_id'])
    src_img = meta_path.parent / meta['image']
    if not src_img.is_file() or src_img.stat().st_size == 0:
        raise FileNotFoundError(f'missing/empty image: {src_img}')

    rel_name = f'station{station_id}_seed{meta.get("seed", "x")}.png'
    dest = copy_station_image(src_img, out_dir / 'images', rel_name)
    image_path = f'images/{dest.name}'
    image_stamp = meta.get('image_stamp')

    pose = poses.get(station_id) or meta.get('robot_pose')
    if not pose:
        raise KeyError(f'no station_pose for {station_id}')
    frame_id = 'map'

    decisions = decisions_from_r1(judge, rules_version)
    if include_bbox_fixture and station_id == 'B':
        # Distinct object_type so bbox fixture is visible without colliding
        # with fixed_window inspection_region dedup key.
        bbox_d = decision_bbox_fixture(rules_version)
        decisions.append(
            Decision(
                object_type='inspection_region_bbox_demo',
                verdict=bbox_d.verdict,
                reason=bbox_d.reason,
                roi_source=bbox_d.roi_source,
                roi_rules_version=bbox_d.roi_rules_version,
                measurements=bbox_d.measurements,
                confidence=bbox_d.confidence,
            )
        )

    stats = {'created': 0, 'deduped': 0, 'skipped_pass': 0, 'blocked_unknown_to_pass': 0}
    written: list[dict[str, Any]] = []
    for d in decisions:
        if d.verdict == 'PASS' and not include_pass:
            stats['skipped_pass'] += 1
            continue
        ev = decision_to_event(
            d,
            mission_id=mission_id,
            station_id=station_id,
            station_pose=pose,
            frame_id=frame_id,
            image_path=image_path,
            image_stamp=image_stamp,
            source=source,
            amcl_error_m=amcl_error_m,
        )
        action, kept = store.try_add(ev)
        stats[action] = stats.get(action, 0) + 1
        if action == 'created':
            written.append(kept)

    return {
        'station_id': station_id,
        'judge_verdict': judge['verdict'],
        'image_path': image_path,
        'image_bytes': dest.stat().st_size,
        'stats': stats,
        'created_events': written,
    }


def validate_event(ev: dict[str, Any]) -> list[str]:
    errs: list[str] = []
    for k in REQUIRED_FIELDS:
        if k not in ev:
            errs.append(f'missing:{k}')
    if ev.get('verdict') not in VERDICTS:
        errs.append('bad_verdict')
    if ev.get('status') not in LIFECYCLE:
        errs.append('bad_status_lifecycle')
    if ev.get('status') in VERDICTS:
        errs.append('status_used_as_verdict')
    # Forbidden: stuffing ids into a description blob
    if 'description' in ev and ev['description']:
        errs.append('unexpected_description')
    pose = ev.get('station_pose') or {}
    for k in ('x', 'y', 'yaw'):
        if k not in pose:
            errs.append(f'missing_pose:{k}')
    return errs


def run_self_check(args: argparse.Namespace) -> int:
    """Minimal assertions — not a large test harness."""
    out = Path(args.output)
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)

    # 1) Fresh emit → ≥1 FAIL, fields, non-empty image
    args.mission_id = 'mission_e1_selfcheck'
    args.stations = ['B']
    args.include_pass = False
    args.include_bbox_fixture = True
    n1 = _run_emit(args)
    events = EventStore(out / 'events.jsonl', args.dedup_window_sec).events
    fails = [e for e in events if e['verdict'] == 'FAIL']
    assert fails, 'expected ≥1 FAIL'
    for ev in fails:
        errs = validate_event(ev)
        assert not errs, errs
        img = out / ev['image_path']
        assert img.is_file() and img.stat().st_size > 0, img
    assert n1['totals']['created'] >= 1

    # 2) Same mission + window again → no doubling
    n2 = _run_emit(args)
    events2 = EventStore(out / 'events.jsonl', args.dedup_window_sec).events
    assert len(events2) == len(events), (
        f'dedup failed: {len(events)} -> {len(events2)}; stats={n2["totals"]}'
    )
    assert n2['totals']['deduped'] >= 1

    # 3) Cross mission: must NOT merge / dedup against other missions
    args.mission_id = 'mission_e1_selfcheck_b'
    n3 = _run_emit(args)
    events3 = EventStore(out / 'events.jsonl', args.dedup_window_sec).events
    assert len(events3) > len(events2), 'new mission should create events'
    assert n3['totals']['created'] >= 1
    m_a = [e for e in events3 if e['mission_id'] == 'mission_e1_selfcheck']
    m_b = [e for e in events3 if e['mission_id'] == 'mission_e1_selfcheck_b']
    assert m_a and m_b, 'both missions must keep their own events'
    assert {e['event_id'] for e in m_a}.isdisjoint({e['event_id'] for e in m_b})
    # Same station+object_type still present under both missions (not collapsed)
    keys_a = {(e['station_id'], e['object_type']) for e in m_a}
    keys_b = {(e['station_id'], e['object_type']) for e in m_b}
    assert keys_a & keys_b, 'cross-mission should retain overlapping station/object keys'

    # 4) Cross station: same mission + window, A vs B must both persist
    args.mission_id = 'mission_e1_cross_station'
    args.stations = ['B']
    args.include_bbox_fixture = False
    before_cross = len(EventStore(out / 'events.jsonl', args.dedup_window_sec).events)
    n_b = _run_emit(args)
    after_b = EventStore(out / 'events.jsonl', args.dedup_window_sec).events
    assert n_b['totals']['created'] >= 1
    args.stations = ['A']
    n_a = _run_emit(args)
    after_a = EventStore(out / 'events.jsonl', args.dedup_window_sec).events
    assert n_a['totals']['created'] >= 1, 'station A must create its own events'
    assert len(after_a) > len(after_b) > before_cross
    cross = [e for e in after_a if e['mission_id'] == 'mission_e1_cross_station']
    assert {e['station_id'] for e in cross} >= {'A', 'B'}, cross

    # 5) Cross object_type: same mission+station+window → separate events
    store = EventStore(out / 'events.jsonl', args.dedup_window_sec)
    base = {
        'event_id': 'obj-a',
        'mission_id': 'mission_obj_types',
        'station_id': 'B',
        'object_type': 'indicator_light',
        'verdict': 'FAIL',
        'status': 'open',
        'reason': 'led_non_green:red,red,green',
        'confidence': None,
        'timestamp': datetime.now(timezone.utc).isoformat(),
        'image_stamp': {'sec': 2000, 'nanosec': 0},
        'station_pose': {'x': 8.45, 'y': 1.75, 'z': 0.05, 'yaw': -1.57},
        'frame_id': 'map',
        'image_path': 'images/stationB_seed42.png',
        'model_version': None,
        'roi_rules_version': 'roi_rules_v1',
        'roi_source': 'fixed_window',
        'source': 'roi_rule_replay',
        'amcl_error_m': None,
        'measurements': {},
    }
    a1, _ = store.try_add(base)
    other = dict(base)
    other['event_id'] = 'obj-b'
    other['object_type'] = 'inspection_region'
    other['reason'] = 'leak_fixed_fail'
    a2, _ = store.try_add(other)
    assert a1 == 'created' and a2 == 'created', (a1, a2)
    # Same object_type in-window still dedups
    dup = dict(base)
    dup['event_id'] = 'obj-a-dup'
    a3, _ = store.try_add(dup)
    assert a3 == 'deduped', a3

    # 6) UNKNOWN must not become PASS
    unknown = {
        'event_id': 'u-test',
        'mission_id': 'mission_unk',
        'station_id': 'B',
        'object_type': 'panel',
        'verdict': 'UNKNOWN',
        'status': 'open',
        'reason': 'panel_unknown',
        'confidence': None,
        'timestamp': datetime.now(timezone.utc).isoformat(),
        'image_stamp': {'sec': 1000, 'nanosec': 0},
        'station_pose': {'x': 0.0, 'y': 0.0, 'z': 0.05, 'yaw': 0.0},
        'frame_id': 'map',
        'image_path': 'images/stationB_seed42.png',
        'model_version': None,
        'roi_rules_version': 'roi_rules_v1',
        'roi_source': 'fixed_window',
        'source': 'roi_rule_replay',
        'amcl_error_m': None,
        'measurements': {},
    }
    store.try_add(unknown)
    pass_attempt = dict(unknown)
    pass_attempt['event_id'] = 'p-test'
    pass_attempt['verdict'] = 'PASS'
    pass_attempt['reason'] = 'panel_lit'
    action, kept = store.try_add(pass_attempt)
    assert action == 'blocked_unknown_to_pass', action
    assert kept['verdict'] == 'UNKNOWN'
    store.flush()

    print('SELF_CHECK_OK')
    print(
        f'  fail_events={len(fails)} after_same_mission_dedup={len(events2)} '
        f'after_new_mission={len(events3)} after_cross_station={len(after_a)} out={out}'
    )
    sample = fails[0]
    print('  sample_fail=', json.dumps(sample, ensure_ascii=False, sort_keys=True))
    return 0


def _run_emit(args: argparse.Namespace) -> dict[str, Any]:
    rules = load_rules(args.rules)
    rules_version = str(rules.get('schema', 'roi_rules_v1'))
    poses = load_patrol_poses(args.route)
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    store = EventStore(out / 'events.jsonl', args.dedup_window_sec)

    d1 = Path(args.d1_dir)
    station_reports = []
    totals = {'created': 0, 'deduped': 0, 'skipped_pass': 0, 'blocked_unknown_to_pass': 0}

    for sid in args.stations:
        metas = sorted(d1.glob(f'station{sid}_seed*.json'))
        if not metas:
            raise FileNotFoundError(f'no station{sid}_seed*.json under {d1}')
        report = emit_for_station(
            store=store,
            meta_path=metas[0],
            rules=rules,
            rules_version=rules_version,
            mission_id=args.mission_id,
            poses=poses,
            out_dir=out,
            include_pass=args.include_pass,
            include_bbox_fixture=args.include_bbox_fixture,
            source=args.source,
            amcl_error_m=args.amcl_error_m,
        )
        station_reports.append(report)
        for k, v in report['stats'].items():
            totals[k] = totals.get(k, 0) + v

    store.flush()

    # Manifest for humans
    manifest = {
        'schema': 'vision_e1_demo_v1',
        'output': str(out),
        'mission_id': args.mission_id,
        'stations': args.stations,
        'dedup_window_sec': args.dedup_window_sec,
        'totals': totals,
        'note': (
            'E1≠5B≠M2 exit. Fixed-window ROI replay ≠ live detector loop. '
            'status=lifecycle only; verdict=PASS|FAIL|UNKNOWN.'
        ),
        'stations_detail': [
            {
                'station_id': r['station_id'],
                'judge_verdict': r['judge_verdict'],
                'image_path': r['image_path'],
                'image_bytes': r['image_bytes'],
                'stats': r['stats'],
                'created_event_ids': [e['event_id'] for e in r['created_events']],
            }
            for r in station_reports
        ],
    }
    (out / 'manifest.json').write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + '\n', encoding='utf-8'
    )
    return manifest


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--rules', type=Path, default=DEFAULT_RULES)
    p.add_argument('--d1-dir', type=Path, default=DEFAULT_D1)
    p.add_argument('--route', type=Path, default=DEFAULT_ROUTE)
    p.add_argument(
        '-o',
        '--output',
        type=Path,
        default=DEFAULT_OUT,
        help='Artifact root: events.jsonl + images/ (default bags/vision_e1_demo)',
    )
    p.add_argument('--mission-id', default='mission_e1_seed42')
    p.add_argument(
        '--stations',
        nargs='+',
        default=['B'],
        help='Station ids to process (default: B for FAIL leak/leds)',
    )
    p.add_argument('--dedup-window-sec', type=float, default=60.0)
    p.add_argument(
        '--include-pass',
        action='store_true',
        help='Also persist PASS decisions (default: only FAIL/UNKNOWN)',
    )
    p.add_argument(
        '--include-bbox-fixture',
        action='store_true',
        default=True,
        help='Emit one bbox_fixture Decision on station B (default on)',
    )
    p.add_argument(
        '--no-bbox-fixture',
        action='store_false',
        dest='include_bbox_fixture',
    )
    p.add_argument('--source', default='roi_rule_replay')
    p.add_argument(
        '--amcl-error-m',
        type=float,
        default=None,
        help='Optional; null by default — never hard-blocks E1',
    )
    p.add_argument(
        '--self-check',
        action='store_true',
        help='Wipe output dir and run minimal acceptance assertions',
    )
    args = p.parse_args(argv)

    if args.self_check:
        return run_self_check(args)

    if not args.d1_dir.is_dir():
        print(f'missing D1 dir: {args.d1_dir}', file=sys.stderr)
        return 1

    manifest = _run_emit(args)
    store = EventStore(Path(args.output) / 'events.jsonl', args.dedup_window_sec)
    fails = [e for e in store.events if e['verdict'] == 'FAIL']
    print(
        f"wrote {args.output}/events.jsonl "
        f"events={len(store.events)} fails={len(fails)} totals={manifest['totals']}"
    )
    for ev in fails[:3]:
        print(
            f"  FAIL {ev['object_type']} reason={ev['reason']} "
            f"image={ev['image_path']} mission={ev['mission_id']}"
        )
    return 0


if __name__ == '__main__':
    sys.exit(main())
