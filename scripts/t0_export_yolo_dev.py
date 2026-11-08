#!/usr/bin/env python3
"""T0: export review_passed soft train/val to YOLO detect layout (dev only).

Hard-stops on missing images, incomplete expected classes, illegal boxes,
duplicate IDs, train∩val, (station,seed) cross-split, or status words in names.
Does NOT write formal test_batches.txt / claim 5B freeze.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import cv2
import yaml

REPO = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = REPO / 'bags' / 'vision_5b_c2'
CLASS_NAMES = {0: 'indicator_light', 1: 'panel', 2: 'inspection_region'}
NAME_TO_ID = {v: k for k, v in CLASS_NAMES.items()}
EXPECTED = {'indicator_light': 3, 'panel': 1, 'inspection_region': 1}
STATUS_WORDS = frozenset(
    {'pass', 'fail', 'unknown', 'lit', 'dark', 'green', 'red', 'yellow', 'off'}
)


def _read_id_list(path: Path) -> list[str]:
    out: list[str] = []
    for line in path.read_text(encoding='utf-8').splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        out.append(line)
    return out


def _find_meta(root: Path, image_id: str) -> Path:
    matches = list((root / 'batches').rglob(f'{image_id}.json'))
    if not matches:
        raise FileNotFoundError(f'missing meta for {image_id}')
    if len(matches) > 1:
        raise RuntimeError(f'duplicate meta paths for {image_id}: {matches}')
    return matches[0]


def _xyxy_to_yolo(
    box: list[int], w: int, h: int
) -> tuple[float, float, float, float] | None:
    x1, y1, x2, y2 = [float(v) for v in box]
    if x2 <= x1 or y2 <= y1:
        return None
    if x1 < 0 or y1 < 0 or x2 > w or y2 > h:
        return None
    bw, bh = x2 - x1, y2 - y1
    if bw <= 0 or bh <= 0:
        return None
    cx = (x1 + x2) / 2.0 / w
    cy = (y1 + y2) / 2.0 / h
    return cx, cy, bw / w, bh / h


def _validate_isolation(
    records: list[dict[str, Any]], train_ids: set[str], val_ids: set[str]
) -> list[str]:
    errs: list[str] = []
    inter = train_ids & val_ids
    if inter:
        errs.append(f'train∩val non-empty: {sorted(inter)[:5]}...')
    key_split: dict[tuple[Any, Any], set[str]] = defaultdict(set)
    batch_split: dict[str, set[str]] = defaultdict(set)
    for r in records:
        sid = 'train' if r['image_id'] in train_ids else 'val'
        key_split[(r['station_id'], r['seed'])].add(sid)
        batch_split[r['batch_id']].add(sid)
    for key, sides in key_split.items():
        if len(sides) > 1:
            errs.append(f'(station,seed) cross-split: {key}')
    for bid, sides in batch_split.items():
        if len(sides) > 1:
            errs.append(f'batch cross-split: {bid}')
    return errs


def export(args: argparse.Namespace) -> int:
    root = Path(args.root)
    man = root / 'manifests'
    train_ids = _read_id_list(man / 'dev_train_image_ids.txt')
    val_ids = _read_id_list(man / 'dev_val_image_ids.txt')
    train_set, val_set = set(train_ids), set(val_ids)

    excluded = json.loads((man / 'excluded_frames.json').read_text(encoding='utf-8'))
    ex_ids = {
        str(f.get('image_id'))
        for f in excluded.get('frames') or []
        if f.get('image_id')
    }
    leaked = (train_set | val_set) & ex_ids
    if leaked:
        raise SystemExit(f'FATAL: excluded in split: {sorted(leaked)}')

    out = Path(args.out)
    if out.exists() and args.clean:
        shutil.rmtree(out)
    for split in ('train', 'val'):
        (out / 'images' / split).mkdir(parents=True, exist_ok=True)
        (out / 'labels' / split).mkdir(parents=True, exist_ok=True)

    records: list[dict[str, Any]] = []
    skip_reasons: Counter[str] = Counter()
    class_counts = {'train': Counter(), 'val': Counter()}
    fatal: list[str] = []
    frame_errors: list[dict[str, Any]] = []

    all_ids = [('train', i) for i in train_ids] + [('val', i) for i in val_ids]
    seen: set[str] = set()
    for split, image_id in all_ids:
        if image_id in seen:
            fatal.append(f'duplicate_id:{image_id}')
            continue
        seen.add(image_id)
        try:
            meta_path = _find_meta(root, image_id)
        except (FileNotFoundError, RuntimeError) as exc:
            fatal.append(str(exc))
            continue
        meta = json.loads(meta_path.read_text(encoding='utf-8'))
        png = meta_path.with_suffix('.png')
        if not png.is_file() or png.stat().st_size == 0:
            fatal.append(f'missing_image:{image_id}')
            continue
        img = cv2.imread(str(png))
        if img is None:
            fatal.append(f'undecodable:{image_id}')
            continue
        h, w = img.shape[:2]
        dets = meta.get('detections') or []
        cls_hist = Counter()
        yolo_lines: list[str] = []
        bad_boxes: list[str] = []
        for d in dets:
            name = str(d.get('class_name') or '')
            low = name.lower()
            if any(sw in low.split('_') or sw == low for sw in STATUS_WORDS):
                fatal.append(f'status_in_classname:{image_id}:{name}')
                continue
            if name not in NAME_TO_ID:
                fatal.append(f'unknown_class:{image_id}:{name}')
                continue
            box = d.get('bbox_xyxy')
            if not box or len(box) != 4:
                bad_boxes.append(f'{name}:no_box')
                continue
            yolo = _xyxy_to_yolo([int(x) for x in box], w, h)
            if yolo is None:
                bad_boxes.append(f'{name}:illegal_box:{box}')
                continue
            cid = NAME_TO_ID[name]
            cls_hist[name] += 1
            class_counts[split][name] += 1
            cx, cy, bw, bh = yolo
            yolo_lines.append(f'{cid} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}')

        # Expected targets: incomplete labels must not enter training.
        missing_expected = []
        for cname, n_exp in EXPECTED.items():
            n_got = cls_hist[cname]
            if n_got < n_exp:
                missing_expected.append(f'{cname}:{n_got}<{n_exp}')
        if bad_boxes or missing_expected:
            reason = {
                'image_id': image_id,
                'split': split,
                'bad_boxes': bad_boxes,
                'missing_expected': missing_expected,
                'cls_hist': dict(cls_hist),
            }
            frame_errors.append(reason)
            # Default: hard stop — do not write partial labels.
            fatal.append(
                f'incomplete_or_illegal:{image_id}:'
                f'{bad_boxes or missing_expected}'
            )
            skip_reasons['incomplete_or_illegal'] += 1
            continue

        records.append(
            {
                'image_id': image_id,
                'split': split,
                'station_id': meta.get('station_id'),
                'seed': meta.get('seed'),
                'batch_id': meta.get('batch_id'),
                'meta_path': str(meta_path),
                'png': str(png),
                'n_labels': len(yolo_lines),
            }
        )
        # write
        stem = image_id
        dst_img = out / 'images' / split / f'{stem}.png'
        dst_lbl = out / 'labels' / split / f'{stem}.txt'
        shutil.copy2(png, dst_img)
        dst_lbl.write_text('\n'.join(yolo_lines) + '\n', encoding='utf-8')

    iso_errs = _validate_isolation(records, train_set, val_set)
    fatal.extend(iso_errs)

    empty_lbl = [
        r for r in records if r['n_labels'] == 0
    ]
    if empty_lbl:
        fatal.append(f'empty_labels:{len(empty_lbl)}')

    data_yaml = {
        'path': str(out.resolve()),
        'train': 'images/train',
        'val': 'images/val',
        'names': CLASS_NAMES,
        'nc': len(CLASS_NAMES),
        'note': 'dev_only_not_formal_5b_test',
    }
    (out / 'data.yaml').write_text(
        yaml.safe_dump(data_yaml, sort_keys=False, allow_unicode=True),
        encoding='utf-8',
    )

    n_train = sum(1 for r in records if r['split'] == 'train')
    n_val = sum(1 for r in records if r['split'] == 'val')
    report_lines = [
        '# T0 YOLO 导出报告（开发用）',
        '',
        f'- out: `{out}`',
        f'- data.yaml: `{out / "data.yaml"}`',
        f'- n_train_exported: **{n_train}** / listed {len(train_ids)}',
        f'- n_val_exported: **{n_val}** / listed {len(val_ids)}',
        f'- note: `dev_only_not_formal_5b_test`；无正式 test 冻结',
        f'- aug: 未导出（Train-1 仅原始过审帧）',
        '',
        '## 类分布（框计数）',
        '',
        f'- train: `{dict(class_counts["train"])}`',
        f'- val: `{dict(class_counts["val"])}`',
        '',
        '## 跳过 / 错误',
        '',
        f'- skip_reasons: `{dict(skip_reasons)}`',
        f'- frame_errors: {len(frame_errors)}',
        f'- fatal: {len(fatal)}',
        '',
    ]
    if fatal:
        report_lines.append('### FATAL（导出应视为失败）')
        report_lines.append('')
        for f in fatal[:50]:
            report_lines.append(f'- `{f}`')
        report_lines.append('')
    if frame_errors:
        report_lines.append('### 帧级错误明细')
        report_lines.append('')
        report_lines.append('```json')
        report_lines.append(json.dumps(frame_errors[:20], indent=2, ensure_ascii=False))
        report_lines.append('```')
        report_lines.append('')

    report_path = man / 't0_export_report.md'
    report_path.write_text('\n'.join(report_lines) + '\n', encoding='utf-8')
    print(report_path.read_text(encoding='utf-8'))

    if fatal:
        print('T0 FAILED — refusing to treat dataset as train-ready', file=sys.stderr)
        return 1
    if n_train == 0 or n_val == 0:
        print('T0 FAILED — empty split', file=sys.stderr)
        return 1
    print(f'T0 OK: {out / "data.yaml"}')
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    ap.add_argument(
        '--out',
        type=Path,
        default=DEFAULT_ROOT / 'yolo_dev',
    )
    ap.add_argument('--clean', action='store_true', default=True)
    ap.add_argument('--no-clean', action='store_false', dest='clean')
    return export(ap.parse_args())


if __name__ == '__main__':
    raise SystemExit(main())
