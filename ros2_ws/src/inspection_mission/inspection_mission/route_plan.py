"""Fixed route IDs and station sequences for MVP-R1/R7 (no coordinate search)."""

from __future__ import annotations

from typing import Any

ROUTE_A = 'test_room_A'
ROUTE_ABC = 'test_room_ABC'
ALLOWED_ROUTE_IDS = frozenset({ROUTE_A, ROUTE_ABC})

# Backward-compatible alias used across R1–R6 code.
ALLOWED_ROUTE_ID = ROUTE_A
STATION_A = 'A'


def stations_for_route(route_id: str, route_yaml: dict[str, Any] | None = None) -> list[str]:
    """Return ordered station ids for an accepted route_id."""
    rid = str(route_id or '').strip()
    if rid == ROUTE_A:
        return [STATION_A]
    if rid == ROUTE_ABC:
        order = None
        if isinstance(route_yaml, dict):
            order = route_yaml.get('inspection_order')
        if isinstance(order, list) and order:
            return [str(s) for s in order]
        return ['A', 'B', 'C']
    raise ValueError(f'unsupported route_id={route_id!r}')


def transfer_vias_before_station(
    route_yaml: dict[str, Any],
    *,
    from_station: str,
    to_station: str,
) -> list[list[float]]:
    """Return via world poses [x,y,yaw] to run before navigating to to_station.

    Uses patrol_route.yaml transfers (e.g. B_to_C). Empty if none.
    """
    transfers = route_yaml.get('transfers') or {}
    key = f'{from_station}_to_{to_station}'
    block = transfers.get(key) or {}
    vias = block.get('via') or []
    out: list[list[float]] = []
    for item in vias:
        if isinstance(item, (list, tuple)) and len(item) >= 3:
            out.append([float(item[0]), float(item[1]), float(item[2])])
    return out


def transfer_through_world_poses(
    route_yaml: dict[str, Any],
    *,
    from_station: str,
    to_station: str,
) -> list[list[float]]:
    """Ordered world poses for one NavigateThroughPoses: vias then goal station.

    Matches patrol_mission_node.navigate(): ``via + [stations[name]]``.
    Empty list means caller should use single NavigateToPose to to_station.
    """
    vias = transfer_vias_before_station(
        route_yaml, from_station=from_station, to_station=to_station,
    )
    if not vias:
        return []
    stations = route_yaml.get('stations') or {}
    goal = stations.get(to_station)
    if not isinstance(goal, (list, tuple)) or len(goal) < 3:
        raise ValueError(f'missing station pose for {to_station!r}')
    return vias + [[float(goal[0]), float(goal[1]), float(goal[2])]]


def aggregate_inspection_status(statuses: list[str]) -> str:
    """Mission-level inspection rollup.

    Rules (explicit):
    1) Any station FAIL → FAIL (UNKNOWN must not mask confirmed anomalies)
    2) Else any UNKNOWN → UNKNOWN
    3) Else all non-empty are PASS → PASS
    4) Else empty string (no inspection results yet)
    """
    cleaned = [str(s) for s in statuses if str(s or '').strip()]
    if any(s == 'FAIL' for s in cleaned):
        return 'FAIL'
    if any(s == 'UNKNOWN' for s in cleaned):
        return 'UNKNOWN'
    if cleaned and all(s == 'PASS' for s in cleaned):
        return 'PASS'
    return ''
