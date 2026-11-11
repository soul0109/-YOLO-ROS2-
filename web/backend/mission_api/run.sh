#!/usr/bin/env bash
# Single-worker FastAPI (same VM as Runtime). Prefer scripts/run_mission_*.sh.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/mission_defaults.sh"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
export INSPECTION_ARCHIVE_DB
cd "$ROOT/web/backend"
exec python3 -m uvicorn mission_api.app:app --host 0.0.0.0 --port "${PORT}" --workers 1
