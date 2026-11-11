#!/usr/bin/env python3
"""Freeze formal vision Test batches after capture + coverage check.

Writes splits/test_batches.txt + manifests/test_freeze_<date>.json (+ sha256),
updates candidate.formal_test. Refuses if gates fail or synthetic frames present.
After freeze: do not tune params / pick models / edit ROI against this set.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

REPO = Path(__file__).resolve().parents[1]
DEFAULT_CANDIDATE = (
    REPO / 'models' / 'candidates' / 'inspection_train1_20261012.candidate.json'
)


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def load_metas(root: Path) -> list[dict[str, Any]]:
    """Prefer states/*.json; fall back to unique image_id from batch JSON sidecars."""
    metas: list[dict[str, Any]] = []
    seen: set[str] = set()
    batches = root / 'batches'
    preferred = sorted(batches.glob('*/states/*.json'))
    search = preferred if preferred else sorted(batches.rglob('*.json'))
    for p in search:
        if p.name in ('batch_manifest.json', 'gt.json', 'status.json'):
            continue
        if p.name.startswith('gt_seed'):
            continue
        try:
            m = json.loads(p.read_text(encoding='utf-8'))
        except Exception:
            continue
        iid = m.get('image_id')
        if not iid or 'station_id' not in m or 'batch_id' not in m:
            continue
        if iid in seen:
            continue
        seen.add(str(iid))
        metas.append(m)
    return metas


def coverage(metas: list[dict[str, Any]]) -> dict[str, Any]:
    main = [m for m in metas if abs(float(m.get('distance_m', 0)) - 1.2) < 1e-6]
    d10 = [m for m in metas if abs(float(m.get('distance_m', 0)) - 1.0) < 1e-6]
    d15 = [m for m in metas if abs(float(m.get('distance_m', 0)) - 1.5) < 1e-6]
    by_station = {s: sum(1 for m in main if m['station_id'] == s) for s in ('A', 'B', 'C')}
    unk = sum(1 for m in main if m.get('unknown_case_id'))
    hard = sum(
        1
        for m in main
        if m.get('hard_case') and list(m.get('hard_case') or []) not in ([], ['none'])
    )
    batches = sorted({m['batch_id'] for m in metas})
    syn = [m['image_id'] for m in metas if m.get('synthetic') is True]
    # state coverage
    has = {
        'green': any('green' in (m.get('states') or {}).get('leds', []) for m in main),
        'red': any('red' in (m.get('states') or {}).get('leds', []) for m in main),
        'yellow': any('yellow' in (m.get('states') or {}).get('leds', []) for m in main),
        'off': any('off' in (m.get('states') or {}).get('leds', []) for m in main),
        'lit': any((m.get('states') or {}).get('panel') == 'lit' for m in main),
        'dark': any((m.get('states') or {}).get('panel') == 'dark' for m in main),
        'leak_fail': any((m.get('states') or {}).get('leak') == 'fixed_fail' for m in main),
        'leak_none': any((m.get('states') or {}).get('leak') == 'none' for m in main),
    }
    gates = {
        'ge_60_or_total60': len(main) >= 60 or len(metas) >= 60,
        'ge_3_batches': len(batches) >= 3,
        'unknown_ge_10': unk >= 10,
        'hard_ge_10': hard >= 10,
        'd10_ge_5': len(d10) >= 5,
        'd15_ge_5': len(d15) >= 5,
        'abc_covered': all(by_station[s] >= 1 for s in ('A', 'B', 'C')),
        'no_synthetic': len(syn) == 0,
        'light_panel_leak_flags': all(has.values()),
    }
    return {
        'n_total': len(metas),
        'n_main_1_2': len(main),
        'n_d1_0': len(d10),
        'n_d1_5': len(d15),
        'by_station_main': by_station,
        'n_unknown_main': unk,
        'n_hard_main': hard,
        'batch_ids': batches,
        'synthetic_image_ids': syn,
        'state_flags': has,
        'gates': gates,
        'all_gates_pass': all(gates.values()),
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root', type=Path, required=True)
    ap.add_argument('--candidate', type=Path, default=DEFAULT_CANDIDATE)
    ap.add_argument('--date-tag', default=datetime.now().strftime('%Y%m%d'))
    ap.add_argument('--force', action='store_true', help='freeze even if soft gates fail (still refuse synthetic)')
    args = ap.parse_args(argv)

    root = args.root
    metas = load_metas(root)
    cov = coverage(metas)
    if cov['synthetic_image_ids']:
        print('REFUSE: synthetic frames present:', cov['synthetic_image_ids'][:10], file=sys.stderr)
        return 2
    if not cov['all_gates_pass'] and not args.force:
        print('REFUSE: coverage gates failed:', json.dumps(cov['gates'], indent=2), file=sys.stderr)
        print(json.dumps(cov, indent=2))
        return 2

    splits = root / 'splits'
    manifests = root / 'manifests'
    splits.mkdir(parents=True, exist_ok=True)
    manifests.mkdir(parents=True, exist_ok=True)

    batches = cov['batch_ids']
    tb = splits / 'test_batches.txt'
    tb.write_text(
        '# frozen formal test batches — do not retune on this set\n'
        + '\n'.join(batches)
        + '\n',
        encoding='utf-8',
    )

    # per-frame sha256
    frame_hashes = []
    for m in metas:
        iid = m['image_id']
        # find png
        pngs = list((root / 'batches').rglob(f'{iid}.png'))
        if not pngs:
            print(f'REFUSE: missing png for {iid}', file=sys.stderr)
            return 2
        frame_hashes.append({'image_id': iid, 'png_sha256': sha256_file(pngs[0]), 'batch_id': m['batch_id']})

    head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=REPO, text=True).strip()
    freeze = {
        'schema': 'vision_5b_test_freeze_v1',
        'frozen': True,
        'frozen_at': datetime.now(timezone.utc).isoformat(),
        'root': str(root),
        'test_batches_file': str(tb.relative_to(REPO)) if tb.is_relative_to(REPO) else str(tb),
        'batch_ids': batches,
        'n_frames': len(metas),
        'coverage': cov,
        'frame_hashes': frame_hashes,
        'code_head': head,
        'label_version': metas[0].get('label_version') if metas else None,
        'policy': {
            'no_retune_on_test': True,
            'no_model_selection_on_test': True,
            'no_roi_edit_on_test': True,
            'no_aug': True,
        },
    }
    freeze_path = manifests / f'test_freeze_{args.date_tag}.json'
    freeze_path.write_text(json.dumps(freeze, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    freeze_sha = sha256_file(freeze_path)
    (manifests / f'test_freeze_{args.date_tag}.json.sha256').write_text(
        f'{freeze_sha}  {freeze_path.name}\n', encoding='utf-8'
    )
    tb_sha = sha256_file(tb)
    (splits / 'test_batches.txt.sha256').write_text(f'{tb_sha}  test_batches.txt\n', encoding='utf-8')

    # mark batch status frozen
    for bid in batches:
        status = root / 'batches' / bid / 'status.json'
        status.write_text(
            json.dumps(
                {
                    'batch_id': bid,
                    'status': 'frozen',
                    'frozen': True,
                    'frozen_at': freeze['frozen_at'],
                    'freeze_manifest': str(freeze_path),
                },
                indent=2,
            )
            + '\n',
            encoding='utf-8',
        )

    cand = json.loads(args.candidate.read_text(encoding='utf-8'))
    cand['formal_test'] = {
        'frozen': True,
        'frozen_at': freeze['frozen_at'],
        'test_root': str(root),
        'test_batches_file': str(tb.relative_to(REPO)) if tb.is_relative_to(REPO) else str(tb),
        'freeze_manifest': str(freeze_path.relative_to(REPO))
        if freeze_path.is_relative_to(REPO)
        else str(freeze_path),
        'freeze_sha256': freeze_sha,
        'n_frames': len(metas),
        'batch_ids': batches,
    }
    args.candidate.write_text(json.dumps(cand, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')

    print(json.dumps({
        'frozen': True,
        'freeze_manifest': str(freeze_path),
        'freeze_sha256': freeze_sha,
        'n_frames': len(metas),
        'batch_ids': batches,
        'gates_pass': cov['all_gates_pass'],
    }, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
