# Shared defaults for Mission Web / API / Runtime entry scripts.
# Source from other scripts:  source "$(dirname "$0")/mission_defaults.sh"
# Override any variable before sourcing, or export afterward.

: "${ROOT:=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"

# Unified archive + capture paths (NOT bound to a one-off evidence bag).
: "${INSPECTION_ARCHIVE_DB:=$ROOT/bags/inspection_archive.sqlite3}"
: "${INSPECTION_CAPTURE_DIR:=$ROOT/bags/mission_captures}"

: "${PORT:=8000}"
: "${HOST_IP:=192.168.203.129}"
: "${ROUTE_ID:=test_room_ABC}"

# Frozen birth / spawn (world). Used only on cold bring-up before first move.
: "${SPAWN_WORLD_X:=0.9}"
: "${SPAWN_WORLD_Y:=3.0}"
: "${SPAWN_YAW:=0.0}"

: "${INSPECTION_WEB_PID_FILE:=/tmp/inspection_mission_web.pid}"
: "${INSPECTION_WEB_LOG:=$ROOT/bags/mission_web_uvicorn.log}"
: "${INSPECTION_RUNTIME_PID_FILE:=/tmp/inspection_mission_runtime.pid}"
: "${INSPECTION_RUNTIME_LOG:=$ROOT/bags/mission_runtime.log}"
: "${INSPECTION_LAUNCH_PID_FILE:=/tmp/inspection_nav2_launch.pid}"
: "${INSPECTION_LAUNCH_LOG:=$ROOT/bags/mission_nav2_launch.log}"

export ROOT INSPECTION_ARCHIVE_DB INSPECTION_CAPTURE_DIR PORT HOST_IP ROUTE_ID
export SPAWN_WORLD_X SPAWN_WORLD_Y SPAWN_YAW
export INSPECTION_WEB_PID_FILE INSPECTION_WEB_LOG
export INSPECTION_RUNTIME_PID_FILE INSPECTION_RUNTIME_LOG
export INSPECTION_LAUNCH_PID_FILE INSPECTION_LAUNCH_LOG

mission_urls() {
  echo "URL_VM=http://127.0.0.1:${PORT}/"
  echo "URL_HOST=http://${HOST_IP}:${PORT}/"
}
