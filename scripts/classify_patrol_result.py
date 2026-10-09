#!/usr/bin/env python3
"""Classify one PATROL_RESULT line into INIT/ROUTE/ACCEPT layers.

Stdout (single line, space-separated):
  class action_success acceptance A_rec B_rec C_rec fail_stage summary

class ∈ {
  ACCEPT_OK,
  ROUTE_OK_ACCEPT_FAIL,
  ROUTE_FAIL,
  NO_RESULT,
  BAD_RESULT,
}
"""

from __future__ import annotations

import json
import sys


def classify(data: dict) -> tuple[str, str]:
    actions_ok = bool(data.get('action_success'))
    accepted = bool(data.get('acceptance'))
    error = data.get('error') or ''
    stations = {item.get('station'): item for item in data.get('stations', [])}

    if accepted and actions_ok:
        return 'ACCEPT_OK', '-'

    # First action failure, else first gate failure.
    for name in ('A', 'B', 'C'):
        st = stations.get(name)
        if st is None:
            continue
        if not st.get('action_success', False):
            return 'ROUTE_FAIL', name
    if actions_ok and not accepted:
        failed_gate = next(
            (
                name for name in ('A', 'B', 'C')
                if name in stations and not stations[name].get('gate', {}).get('pass', False)
            ),
            'gate',
        )
        return 'ROUTE_OK_ACCEPT_FAIL', failed_gate
    if error:
        return 'ROUTE_FAIL', 'error'
    return 'ROUTE_FAIL', '?'


def main() -> int:
    if len(sys.argv) != 2:
        print('usage: classify_patrol_result.py <run.log>', file=sys.stderr)
        return 2
    lines = open(sys.argv[1], encoding='utf-8', errors='replace').read().splitlines()
    line = next((row for row in reversed(lines) if 'PATROL_RESULT:' in row), '')
    if not line:
        print('NO_RESULT false false - - - no_result no_PATROL_RESULT')
        return 0
    try:
        data = json.loads(line.split('PATROL_RESULT:', 1)[1].strip())
    except json.JSONDecodeError:
        print('BAD_RESULT false false - - - bad_result invalid_PATROL_RESULT')
        return 0

    stations = {item.get('station'): item for item in data.get('stations', [])}
    recs = [str(stations.get(name, {}).get('recoveries', '-')) for name in ('A', 'B', 'C')]
    cls, stage = classify(data)
    summary = ';'.join(
        f"{name}:action={stations.get(name, {}).get('action_success', False)}"
        f",gate={stations.get(name, {}).get('gate', {}).get('pass', False)}"
        f",amcl={stations.get(name, {}).get('gate', {}).get('amcl_error_m', '-')}"
        for name in ('A', 'B', 'C') if name in stations
    ) or (data.get('error') or 'no_stations')
    print(
        f"{cls} {bool(data.get('action_success'))} {bool(data.get('acceptance'))} "
        f"{' '.join(recs)} {stage} {summary}"
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
