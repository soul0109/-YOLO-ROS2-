#!/usr/bin/env bash
# History browse only: FastAPI + built Vue on :8000. NO Gazebo / Runtime required.
#
# Usage:
#   bash scripts/run_mission_history.sh
#   INSPECTION_ARCHIVE_DB=~/inspection-robot/bags/mvp_r7_through_.../inspection_archive.sqlite3 \
#     bash scripts/run_mission_history.sh
# Stop:
#   bash scripts/run_mission_history.sh stop
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/mission_defaults.sh"

cmd="${1:-start}"

if [[ "$cmd" == "stop" ]]; then
  bash "$ROOT/scripts/run_mission_web.sh" stop
  exit 0
fi

mkdir -p "$(dirname "$INSPECTION_ARCHIVE_DB")" "$INSPECTION_CAPTURE_DIR"
if [[ ! -f "$INSPECTION_ARCHIVE_DB" ]]; then
  echo "WARN: archive DB missing: $INSPECTION_ARCHIVE_DB"
  echo "HINT: for R7 demo browse, point at an existing evidence DB, e.g."
  echo "  INSPECTION_ARCHIVE_DB=$ROOT/bags/mvp_r7_through_20261110_111120/inspection_archive.sqlite3 \\"
  echo "    bash scripts/run_mission_history.sh"
fi

echo "MODE=history (no Gazebo)"
echo "DB=$INSPECTION_ARCHIVE_DB"
echo "PORT=$PORT ROUTE_DEFAULT=$ROUTE_ID"
# Reuse web starter (builds Vue, starts uvicorn with same DB env).
INSPECTION_ARCHIVE_DB="$INSPECTION_ARCHIVE_DB" PORT="$PORT" \
  bash "$ROOT/scripts/run_mission_web.sh" start
mission_urls
echo "Open / with empty query (no mission_id bound). Pick a row under 最近归档."
