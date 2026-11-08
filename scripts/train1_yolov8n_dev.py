#!/usr/bin/env python3
"""Train-1: yolov8n 3-class fine-tune on yolo_dev split (CPU/GPU).

Records command, weight hash, versions, device. Dev val ≠ formal 5B.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
DEFAULT_DATA = REPO / 'bags' / 'vision_5b_c2' / 'yolo_dev' / 'data.yaml'
DEFAULT_INIT = REPO / 'models' / 'weights' / 'yolov8n.pt'
DEFAULT_PROJECT = REPO / 'bags' / 'vision_5b_c2' / 'train1'


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--data', type=Path, default=DEFAULT_DATA)
    ap.add_argument('--weights', type=Path, default=DEFAULT_INIT)
    ap.add_argument('--project', type=Path, default=DEFAULT_PROJECT)
    ap.add_argument('--name', default='.')
    ap.add_argument('--epochs', type=int, default=50)
    ap.add_argument('--imgsz', type=int, default=640)
    ap.add_argument('--batch', type=int, default=8)
    ap.add_argument('--device', default='cpu')
    ap.add_argument('--workers', type=int, default=4)
    args = ap.parse_args()

    if not args.data.is_file():
        raise SystemExit(f'missing data.yaml: {args.data}')
    if not args.weights.is_file():
        raise SystemExit(f'missing init weights: {args.weights}')

    import torch
    from ultralytics import YOLO
    import ultralytics

    args.project.mkdir(parents=True, exist_ok=True)
    init_sha = sha256_file(args.weights)
    meta = {
        'schema': 'train1_dev_repro_v1',
        'note': 'dev_only_not_formal_5b_test',
        'started_at': datetime.now(timezone.utc).isoformat(),
        'command': ' '.join(sys.argv),
        'init_weights': str(args.weights),
        'init_weights_sha256': init_sha,
        'data_yaml': str(args.data),
        'dev_train_list': str(
            REPO / 'bags/vision_5b_c2/manifests/dev_train_image_ids.txt'
        ),
        'dev_val_list': str(REPO / 'bags/vision_5b_c2/manifests/dev_val_image_ids.txt'),
        'hyper': {
            'epochs': args.epochs,
            'imgsz': args.imgsz,
            'batch': args.batch,
            'device': args.device,
            'workers': args.workers,
        },
        'versions': {
            'python': sys.version,
            'platform': platform.platform(),
            'torch': torch.__version__,
            'cuda_available': torch.cuda.is_available(),
            'ultralytics': ultralytics.__version__,
        },
    }
    (args.project / 'train1_repro.json').write_text(
        json.dumps(meta, indent=2) + '\n', encoding='utf-8'
    )
    print(json.dumps(meta, indent=2))

    model = YOLO(str(args.weights))
    results = model.train(
        data=str(args.data),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=args.workers,
        project=str(args.project),
        name=args.name,
        exist_ok=True,
        pretrained=True,
        plots=True,
        save=True,
    )
    best = args.project / 'weights' / 'best.pt'
    # ultralytics with name='.' may put weights under project/weights
    if not best.is_file():
        # try project/name/weights
        alt = args.project / args.name / 'weights' / 'best.pt'
        if alt.is_file():
            best = alt
    meta['finished_at'] = datetime.now(timezone.utc).isoformat()
    meta['best_pt'] = str(best) if best.is_file() else None
    if best.is_file():
        meta['best_pt_sha256'] = sha256_file(best)
    # metrics if available
    try:
        m = getattr(results, 'results_dict', None) or {}
        meta['results_dict'] = dict(m) if m else None
    except Exception:
        meta['results_dict'] = None
    (args.project / 'train1_repro.json').write_text(
        json.dumps(meta, indent=2) + '\n', encoding='utf-8'
    )
    print('best:', best, 'exists', best.is_file())
    return 0 if best.is_file() else 1


if __name__ == '__main__':
    raise SystemExit(main())
