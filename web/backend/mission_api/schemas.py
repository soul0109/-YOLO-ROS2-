"""HTTP request/response models for MVP-R5."""

from __future__ import annotations

from typing import Any, Optional

from pydantic import BaseModel, Field


class MissionCreateRequest(BaseModel):
    mission_id: str = Field(..., min_length=1)
    route_id: str = Field(default='test_room_ABC')


class MissionCreateResponse(BaseModel):
    mission_id: str
    route_id: str
    accepted: bool
    status: str
    message: str = ''
    observation_status: str = 'CLEAR'  # CLEAR | PENDING | UNKNOWN


class MissionStatusResponse(BaseModel):
    mission_id: str
    route_id: str = ''
    source: str  # live | archive | live+archive
    status: str  # ACCEPTED | RUNNING | COMPLETED | FAILED | CANCELED | INTERRUPTED | REJECTED
    phase: str = ''
    error_code: str = 'NONE'
    message: str = ''
    inspection_status: str = ''
    station_id: str = ''
    screenshot_path: str = ''
    station_result_path: str = ''
    model_version: str = ''
    image_stamp: str = ''
    started_at: str = ''
    ended_at: str = ''
    # Independent of Action execution_status (V2-02).
    observation_status: str = 'CLEAR'  # CLEAR | PENDING | UNKNOWN


class StationResponse(BaseModel):
    station_id: str
    inspection_status: str = ''
    image_stamp: str = ''
    screenshot_path: str = ''
    annotated_path: str = ''
    station_result_path: str = ''
    model_version: str = ''
    roi_rules_version: str = ''
    objects: Optional[Any] = None


class ErrorBody(BaseModel):
    error_code: str
    message: str
    mission_id: str = ''
