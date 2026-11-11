#!/usr/bin/env bash
# Same-origin Web+API on :PORT (built Vue under FastAPI).
# Shared defaults: scripts/mission_defaults.sh (DB / port / URLs).
#
# Usage:
#   bash scripts/run_mission_web.sh
#   INSPECTION_ARCHIVE_DB=... PORT=8000 bash scripts/run_mission_web.sh
#   SKIP_FRONTEND_BUILD=1 bash scripts/run_mission_web.sh   # reuse dist/
# Stop:
#   bash scripts/run_mission_web.sh stop
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/mission_defaults.sh"

cmd="${1:-start}"

stop_web() {
  if [[ -f "$INSPECTION_WEB_PID_FILE" ]]; then
    kill "$(cat "$INSPECTION_WEB_PID_FILE")" 2>/dev/null || true
    rm -f "$INSPECTION_WEB_PID_FILE"
  fi
  pkill -f 'uvicorn mission_api.app:app' 2>/dev/null || true
  echo "stopped"
}

if [[ "$cmd" == "stop" ]]; then
  stop_web
  exit 0
fi

export PATH="${HOME}/.local/node/bin:${PATH}"
if [[ "${SKIP_FRONTEND_BUILD:-0}" != "1" ]]; then
  cd "$ROOT/web/frontend"
  npm run build
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
set -u

export INSPECTION_ARCHIVE_DB
mkdir -p "$(dirname "$INSPECTION_WEB_LOG")" "$(dirname "$INSPECTION_ARCHIVE_DB")"
stop_web >/dev/null 2>&1 || true

cd "$ROOT/web/backend"
nohup python3 -m uvicorn mission_api.app:app --host 0.0.0.0 --port "$PORT" --workers 1 \
  >"$INSPECTION_WEB_LOG" 2>&1 &
echo $! >"$INSPECTION_WEB_PID_FILE"
sleep 2
echo "PID=$(cat "$INSPECTION_WEB_PID_FILE") DB=$INSPECTION_ARCHIVE_DB CAPTURE_DIR=$INSPECTION_CAPTURE_DIR"
mission_urls
curl -sS "http://127.0.0.1:${PORT}/health" || true
echo
