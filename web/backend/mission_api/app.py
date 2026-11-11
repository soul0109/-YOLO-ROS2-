"""Mission FastAPI: HTTP → ExecuteInspection → Runtime → SQLite → HTTP query.

Prefer entry scripts (shared defaults in ``scripts/mission_defaults.sh``)::

    bash scripts/run_mission_history.sh   # browse archive, no Gazebo
    bash scripts/run_mission_full.sh      # cold-start sim + Runtime + Web

Env: ``INSPECTION_ARCHIVE_DB`` (default ``bags/inspection_archive.sqlite3``).
"""

from __future__ import annotations

import os
import sys
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles

# Allow importing inspection_mission from workspace source when install not sourced.
_REPO = Path(__file__).resolve().parents[3]
_MISSION_SRC = _REPO / 'ros2_ws' / 'src' / 'inspection_mission'
if _MISSION_SRC.is_dir() and str(_MISSION_SRC) not in sys.path:
    sys.path.insert(0, str(_MISSION_SRC))

from inspection_mission.mission_archive import MissionArchiveReader  # noqa: E402
from inspection_mission.route_plan import aggregate_inspection_status  # noqa: E402

from .ros_bridge import (  # noqa: E402
    ALLOWED_ROUTE_ID,
    OBS_CLEAR,
    OBS_PENDING,
    OBS_UNKNOWN,
    RosMissionBridge,
)
from .schemas import (  # noqa: E402
    ErrorBody,
    MissionCreateRequest,
    MissionCreateResponse,
    MissionStatusResponse,
    StationResponse,
)

DEFAULT_DB = Path.home() / 'inspection-robot' / 'bags' / 'inspection_archive.sqlite3'


def _db_path() -> Path:
    raw = os.environ.get('INSPECTION_ARCHIVE_DB', str(DEFAULT_DB))
    return Path(raw).expanduser()


def _open_reader() -> MissionArchiveReader:
    path = _db_path()
    if not path.is_file():
        raise HTTPException(
            status_code=503,
            detail={
                'error_code': 'archive_unavailable',
                'message': f'archive db not found: {path}',
            },
        )
    try:
        return MissionArchiveReader(path)
    except Exception as exc:
        raise HTTPException(
            status_code=503,
            detail={
                'error_code': 'archive_unavailable',
                'message': str(exc),
            },
        ) from exc


def _archive_to_status(mission: dict, *, observation_status: str = OBS_CLEAR) -> MissionStatusResponse:
    return MissionStatusResponse(
        mission_id=str(mission.get('mission_id') or ''),
        route_id=str(mission.get('route_id') or ''),
        source='archive',
        status=str(mission.get('execution_status') or ''),
        phase='',
        error_code=str(mission.get('error_code') or 'NONE'),
        message=str(mission.get('message') or ''),
        started_at=str(mission.get('started_at') or ''),
        ended_at=str(mission.get('ended_at') or ''),
        observation_status=observation_status,
    )


def _live_to_status(live) -> MissionStatusResponse:
    return MissionStatusResponse(
        mission_id=live.mission_id,
        route_id=live.route_id,
        source='live',
        status=live.status,
        phase=live.phase,
        error_code=live.error_code,
        message=live.message,
        inspection_status=live.inspection_status,
        station_id=live.station_id,
        screenshot_path=live.screenshot_path,
        station_result_path=live.station_result_path,
        model_version=live.model_version,
        image_stamp=live.image_stamp,
        started_at=live.started_at,
        ended_at=live.ended_at,
        observation_status=getattr(live, 'observation_status', OBS_CLEAR) or OBS_CLEAR,
    )


def create_app(
    *,
    bridge: Optional[RosMissionBridge] = None,
    start_ros: bool = True,
) -> FastAPI:
    bridge = bridge or RosMissionBridge()

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        app.state.bridge = bridge
        if start_ros:
            bridge.start()
        yield
        if start_ros:
            bridge.stop()

    app = FastAPI(
        title='Inspection Mission API',
        version='0.1.0',
        lifespan=lifespan,
    )
    # Allow Windows-host browser → VM API when not using Vite /api proxy.
    app.add_middleware(
        CORSMiddleware,
        allow_origins=['*'],
        allow_credentials=False,
        allow_methods=['*'],
        allow_headers=['*'],
    )

    @app.get('/health')
    def health():
        return {
            'ok': True,
            'ros_bridge': bridge.started,
            'action_ready': bridge.is_action_server_ready() if bridge.started else False,
            'console_hub': bool(getattr(bridge, 'console', None) and bridge.console.started),
            'archive_db': str(_db_path()),
            'archive_exists': _db_path().is_file(),
        }

    @app.get('/console/telemetry')
    def console_telemetry():
        """Read-only map/pose/path/trajectory/stations/goal/camera freshness."""
        hub = getattr(bridge, 'console', None)
        if hub is None:
            return JSONResponse(
                status_code=503,
                content={
                    'error_code': 'console_unavailable',
                    'message': 'console hub not configured',
                },
            )
        payload = hub.get_telemetry()
        # Attach live mission summary without claiming evidence.
        live = bridge.list_live() if bridge.started else []
        active = next((s for s in live if not s.terminal), None)
        payload['live_mission'] = None if active is None else {
            'mission_id': active.mission_id,
            'route_id': active.route_id,
            'status': active.status,
            'phase': active.phase,
            'station_id': active.station_id,
            'observation_status': active.observation_status,
            'message': active.message,
        }
        return payload

    @app.get('/console/map')
    def console_map():
        hub = getattr(bridge, 'console', None)
        if hub is None:
            raise HTTPException(
                status_code=503,
                detail={
                    'error_code': 'console_unavailable',
                    'message': 'console hub not configured',
                },
            )
        return hub.get_map_payload()

    @app.get('/console/camera.jpg')
    def console_camera():
        hub = getattr(bridge, 'console', None)
        if hub is None:
            raise HTTPException(
                status_code=503,
                detail={
                    'error_code': 'console_unavailable',
                    'message': 'console hub not configured',
                },
            )
        jpeg, meta = hub.get_camera_jpeg()
        if not jpeg:
            raise HTTPException(
                status_code=404,
                detail={
                    'error_code': 'camera_unavailable',
                    'message': 'no live camera frame yet (preview only, not evidence)',
                    'freshness': meta.get('freshness'),
                },
            )
        headers = {
            'X-Camera-Stale': '1' if meta['freshness'].get('stale') else '0',
            'X-Camera-Unknown': '1' if meta['freshness'].get('unknown') else '0',
            'X-Camera-Stamp': str(meta.get('stamp') or ''),
            'Cache-Control': 'no-store',
        }
        return Response(content=jpeg, media_type='image/jpeg', headers=headers)

    @app.post(
        '/missions',
        response_model=MissionCreateResponse,
        status_code=202,
        responses={
            400: {'model': ErrorBody},
            409: {'model': ErrorBody},
            503: {'model': ErrorBody},
            504: {'model': ErrorBody},
        },
    )
    def create_mission(body: MissionCreateRequest):
        def archived_ids() -> set[str]:
            path = _db_path()
            if not path.is_file():
                return set()
            reader = MissionArchiveReader(path)
            try:
                return reader.list_mission_ids()
            finally:
                reader.close()

        code, err, msg, live = bridge.submit_goal(
            body.mission_id,
            body.route_id or ALLOWED_ROUTE_ID,
            archived_ids=archived_ids,
        )
        if code == 504 and live is not None:
            # Observation PENDING — do not forge FAILED; client must keep polling.
            return JSONResponse(
                status_code=504,
                content={
                    'error_code': err,
                    'message': msg,
                    'mission_id': live.mission_id,
                    'route_id': live.route_id,
                    'accepted': False,
                    'status': live.status,
                    'observation_status': getattr(
                        live, 'observation_status', OBS_PENDING,
                    ) or OBS_PENDING,
                },
            )
        if code != 202 or live is None:
            return JSONResponse(
                status_code=code,
                content={
                    'error_code': err,
                    'message': msg,
                    'mission_id': str(body.mission_id or '').strip(),
                    'observation_status': OBS_CLEAR,
                },
            )
        return MissionCreateResponse(
            mission_id=live.mission_id,
            route_id=live.route_id,
            accepted=True,
            status=live.status,
            message=msg,
            observation_status=getattr(live, 'observation_status', OBS_CLEAR) or OBS_CLEAR,
        )

    @app.get('/missions', response_model=list[MissionStatusResponse])
    def list_missions(limit: int = Query(20, ge=1, le=200)):
        reader = _open_reader()
        try:
            rows = reader.list_recent_missions(limit=limit)
        finally:
            reader.close()
        return [_archive_to_status(r) for r in rows]

    @app.get(
        '/missions/{mission_id}',
        response_model=MissionStatusResponse,
        responses={404: {'model': ErrorBody}},
    )
    def get_mission(mission_id: str):
        mission_id = mission_id.strip()
        live = bridge.get_live(mission_id)
        path = _db_path()
        archived = None
        stations = []
        if path.is_file():
            reader = MissionArchiveReader(path)
            try:
                archived = reader.get_mission(mission_id)
                if archived is not None:
                    stations = reader.get_stations(mission_id)
            finally:
                reader.close()

        # Terminal archive is authoritative for business status.
        if archived is not None:
            arch_status = str(archived.get('execution_status') or '')
            if arch_status and arch_status != 'RUNNING':
                # R4: reconcile matching live/busy so completed page cannot force 409 forever.
                bridge.reconcile_from_archive_terminal(
                    mission_id,
                    execution_status=arch_status,
                    error_code=str(archived.get('error_code') or 'NONE'),
                    message=str(archived.get('message') or ''),
                )
                live = bridge.get_live(mission_id)
                resp = _archive_to_status(archived, observation_status=OBS_CLEAR)
                if stations:
                    resp.inspection_status = aggregate_inspection_status(
                        [str(s.get('inspection_status') or '') for s in stations],
                    )
                    last = stations[-1]
                    resp.station_id = str(last.get('station_id') or '')
                    resp.screenshot_path = str(last.get('screenshot_path') or '')
                    resp.station_result_path = str(
                        last.get('station_result_path') or '',
                    )
                    resp.model_version = str(last.get('model_version') or '')
                    resp.image_stamp = str(last.get('image_stamp') or '')
                if live is not None and not live.terminal:
                    resp.source = 'live+archive'
                return resp

            # RUNNING shell in DB: live feedback wins when present; else UNKNOWN.
            if live is not None:
                resp = _live_to_status(live)
                resp.source = 'live+archive'
                if stations:
                    resp.inspection_status = aggregate_inspection_status(
                        [str(s.get('inspection_status') or '') for s in stations],
                    ) or resp.inspection_status
                return resp
            resp = _archive_to_status(archived, observation_status=OBS_UNKNOWN)
            resp.message = (
                (resp.message + '; ' if resp.message else '')
                + 'archive shows RUNNING but no live observation — execution unknown'
            )
            if stations:
                resp.inspection_status = aggregate_inspection_status(
                    [str(s.get('inspection_status') or '') for s in stations],
                )
            return resp

        if live is not None:
            return _live_to_status(live)

        raise HTTPException(
            status_code=404,
            detail={
                'error_code': 'mission_not_found',
                'message': f"mission_id '{mission_id}' not in live state or archive",
                'mission_id': mission_id,
            },
        )

    @app.get(
        '/missions/{mission_id}/stations',
        response_model=list[StationResponse],
        responses={404: {'model': ErrorBody}},
    )
    def get_stations(mission_id: str):
        mission_id = mission_id.strip()
        live = bridge.get_live(mission_id)
        path = _db_path()
        if path.is_file():
            reader = MissionArchiveReader(path)
            try:
                if reader.has_mission(mission_id):
                    stations = reader.get_stations(mission_id)
                    return [
                        StationResponse(
                            station_id=str(s.get('station_id') or ''),
                            inspection_status=str(s.get('inspection_status') or ''),
                            image_stamp=str(s.get('image_stamp') or ''),
                            screenshot_path=str(s.get('screenshot_path') or ''),
                            annotated_path=str(s.get('annotated_path') or ''),
                            station_result_path=str(s.get('station_result_path') or ''),
                            model_version=str(s.get('model_version') or ''),
                            roi_rules_version=str(s.get('roi_rules_version') or ''),
                            objects=s.get('objects'),
                        )
                        for s in stations
                    ]
            finally:
                reader.close()

        if live is not None:
            # In-flight / just-finished before archive visible: empty list, not 404.
            return []

        raise HTTPException(
            status_code=404,
            detail={
                'error_code': 'mission_not_found',
                'message': f"mission_id '{mission_id}' not found",
                'mission_id': mission_id,
            },
        )

    def _image_response(mission_id: str, station_id: str, kind: str) -> FileResponse:
        reader = _open_reader()
        try:
            if not reader.has_mission(mission_id):
                raise HTTPException(
                    status_code=404,
                    detail={
                        'error_code': 'mission_not_found',
                        'message': f"mission_id '{mission_id}' not archived",
                        'mission_id': mission_id,
                    },
                )
            path = reader.resolve_station_image_path(mission_id, station_id, kind)
        finally:
            reader.close()
        if path is None:
            raise HTTPException(
                status_code=404,
                detail={
                    'error_code': 'image_not_found',
                    'message': (
                        f'{kind} not available for '
                        f'mission={mission_id} station={station_id}'
                    ),
                    'mission_id': mission_id,
                },
            )
        media = 'image/png' if path.suffix.lower() == '.png' else 'application/octet-stream'
        return FileResponse(path, media_type=media, filename=path.name)

    @app.get('/missions/{mission_id}/stations/{station_id}/screenshot')
    def get_screenshot(mission_id: str, station_id: str):
        return _image_response(mission_id.strip(), station_id.strip(), 'screenshot')

    @app.get('/missions/{mission_id}/stations/{station_id}/annotated')
    def get_annotated(mission_id: str, station_id: str):
        return _image_response(mission_id.strip(), station_id.strip(), 'annotated')

    # Optional same-origin UI: built files under web/frontend/dist
    dist = _REPO / 'web' / 'frontend' / 'dist'
    assets = dist / 'assets'
    if assets.is_dir():
        app.mount('/assets', StaticFiles(directory=str(assets)), name='ui-assets')

    @app.get('/')
    def ui_index():
        index = dist / 'index.html'
        if index.is_file():
            return FileResponse(index)
        return JSONResponse(
            {
                'message': 'UI not built. Run: cd web/frontend && npm run build '
                '(or use Vite dev on :5173 with /api proxy).',
            },
        )

    return app


app = create_app()
