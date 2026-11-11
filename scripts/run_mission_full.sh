#!/usr/bin/env bash
# Full inspection cold start: frozen Nav2 launch → confirm birth pose → ONE initialpose
# → wait_nav_ready → Runtime + Web. Does NOT re-send charge-pose initialpose after move.
#
# Usage:
#   bash scripts/run_mission_full.sh          # start
#   bash scripts/run_mission_full.sh stop     # stop web + runtime + launch (precise)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/mission_defaults.sh"

cmd="${1:-start}"

WORLD_MARKER="${ROOT}/ros2_ws/install/simulation_worlds/share/simulation_worlds/worlds/test_room.world"
LAUNCH_MARKER='nav2_test_room.launch.py'
RUNTIME_MARKER='mission_runtime_node'
WEB_MARKER='uvicorn mission_api.app:app'

# PIDs whose cmdline matches pattern (full cmdline via /proc).
_pids_matching() {
  local pat="$1"
  local pid cmd
  for pid in $(pgrep -f -- "$pat" 2>/dev/null || true); do
    [[ -r "/proc/$pid/cmdline" ]] || continue
    cmd="$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)"
    case "$cmd" in
      *"$pat"*) printf '%s\n' "$pid" ;;
    esac
  done
}

# Only gzserver for THIS repo's test_room.world (never blanket killall gzserver).
_pids_repo_gzserver() {
  local pid cmd
  for pid in $(pgrep -x gzserver 2>/dev/null || true); do
    [[ -r "/proc/$pid/cmdline" ]] || continue
    cmd="$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)"
    case "$cmd" in
      *"${WORLD_MARKER}"*|*"${ROOT}/ros2_ws/src/simulation_worlds/worlds/test_room.world"*)
        printf '%s\n' "$pid"
        ;;
    esac
  done
}

# Descendants of a PID (BFS via pgrep -P).
_pids_descendants() {
  local root="$1"
  local -a queue=("$root")
  local -A seen=()
  local cur child
  while ((${#queue[@]})); do
    cur="${queue[0]}"
    queue=("${queue[@]:1}")
    [[ -n "${seen[$cur]:-}" ]] && continue
    seen[$cur]=1
    for child in $(pgrep -P "$cur" 2>/dev/null || true); do
      printf '%s\n' "$child"
      queue+=("$child")
    done
  done
}

_kill_pid_list() {
  local sig="$1"
  shift
  local pid
  for pid in "$@"; do
    [[ -n "$pid" ]] || continue
    kill "-$sig" "$pid" 2>/dev/null || true
  done
}

_unique_pids() {
  sort -u | grep -E '^[0-9]+$' || true
}

# Orphan nav helpers only if they still reference this repo path in cmdline
# (cmd_vel_timeout from simulation_worlds). Never blanket-kill all nav2_*.
_pids_repo_helpers_orphan() {
  local pid cmd
  for pid in $(pgrep -f -- 'cmd_vel_timeout.py' 2>/dev/null || true); do
    [[ -r "/proc/$pid/cmdline" ]] || continue
    cmd="$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)"
    case "$cmd" in
      *"${ROOT}"*|*/simulation_worlds/lib/simulation_worlds/cmd_vel_timeout*)
        printf '%s\n' "$pid"
        ;;
    esac
  done
}

stack_remaining() {
  {
    _pids_matching "$LAUNCH_MARKER"
    _pids_matching "$RUNTIME_MARKER"
    _pids_matching "$WEB_MARKER"
    _pids_repo_gzserver
    _pids_repo_helpers_orphan
  } | _unique_pids
}

print_stack_status() {
  local rem
  rem="$(stack_remaining | tr '\n' ' ')"
  if [[ -z "${rem// }" ]]; then
    echo "STACK_CLEAR"
    return 0
  fi
  echo "STACK_DIRTY"
  local pid
  for pid in $rem; do
    printf '  %s %s\n' "$pid" "$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null | cut -c1-160 || echo '?')"
  done
  return 1
}

stop_all() {
  local wait_s="${STACK_STOP_WAIT_S:-25}"
  local pid launch_pids runtime_pids web_pids tree_pids gz_pids orphan_pids all_pids

  bash "$ROOT/scripts/run_mission_web.sh" stop >/dev/null 2>&1 || true

  if [[ -f "$INSPECTION_RUNTIME_PID_FILE" ]]; then
    pid="$(cat "$INSPECTION_RUNTIME_PID_FILE" 2>/dev/null || true)"
    [[ -n "${pid:-}" ]] && kill "$pid" 2>/dev/null || true
    rm -f "$INSPECTION_RUNTIME_PID_FILE"
  fi

  mapfile -t launch_pids < <(
    {
      [[ -f "$INSPECTION_LAUNCH_PID_FILE" ]] && cat "$INSPECTION_LAUNCH_PID_FILE" 2>/dev/null || true
      _pids_matching "$LAUNCH_MARKER"
    } | _unique_pids
  )
  rm -f "$INSPECTION_LAUNCH_PID_FILE"

  # Snapshot launch descendants (nav2_*, gzserver, cmd_vel_timeout, …) before killing roots.
  tree_pids=()
  for pid in "${launch_pids[@]}"; do
    mapfile -t -O "${#tree_pids[@]}" tree_pids < <(_pids_descendants "$pid")
  done

  mapfile -t runtime_pids < <(_pids_matching "$RUNTIME_MARKER" | _unique_pids)
  mapfile -t web_pids < <(_pids_matching "$WEB_MARKER" | _unique_pids)
  mapfile -t gz_pids < <(_pids_repo_gzserver | _unique_pids)

  # Persist tree snapshot so STACK_CLEAR can see launch children still alive after root exit.
  local tree_file="/tmp/inspection_mission_launch_tree.pids"
  printf '%s\n' "${tree_pids[@]}" "${launch_pids[@]}" >"$tree_file"

  _tree_survivors() {
    local p
    [[ -f "$tree_file" ]] || return 0
    while read -r p; do
      [[ -n "$p" && -d "/proc/$p" ]] && printf '%s\n' "$p"
    done <"$tree_file"
  }

  # TERM roots + tree + repo-scoped gz/runtime/web (no killall / no foreign gzserver).
  _kill_pid_list 15 "${launch_pids[@]}" "${tree_pids[@]}" "${runtime_pids[@]}" "${web_pids[@]}" "${gz_pids[@]}"

  local deadline=$((SECONDS + wait_s))
  while (( SECONDS < deadline )); do
    mapfile -t all_pids < <({ stack_remaining; _tree_survivors; } | _unique_pids)
    if ((${#all_pids[@]} == 0)); then
      rm -f "$tree_file"
      echo "stopped (web+runtime+launch+repo-gzserver)"
      echo "STACK_CLEAR"
      return 0
    fi
    sleep 1
  done

  mapfile -t all_pids < <(
    {
      stack_remaining
      _tree_survivors
      _pids_repo_gzserver
      _pids_repo_helpers_orphan
      for pid in "${launch_pids[@]}"; do _pids_descendants "$pid"; done
    } | _unique_pids
  )

  if ((${#all_pids[@]})); then
    echo "stop: escalating TERM→KILL for remaining repo stack PIDs: ${all_pids[*]}" >&2
    _kill_pid_list 15 "${all_pids[@]}"
    sleep 1
    mapfile -t all_pids < <({ stack_remaining; _tree_survivors; _pids_repo_gzserver; } | _unique_pids)
    _kill_pid_list 9 "${all_pids[@]}"
    sleep 1
  fi

  echo "stopped (web+runtime+launch+repo-gzserver)"
  mapfile -t all_pids < <({ stack_remaining; _tree_survivors; } | _unique_pids)
  if ((${#all_pids[@]} == 0)); then
    rm -f "$tree_file"
    echo "STACK_CLEAR"
  else
    echo "STACK_DIRTY"
    local p
    for p in "${all_pids[@]}"; do
      printf '  %s %s\n' "$p" "$(tr '\0' ' ' <"/proc/$p/cmdline" 2>/dev/null | cut -c1-160 || echo '?')"
    done
  fi
}

if [[ "$cmd" == "stop" ]]; then
  stop_all
  exit 0
fi

export DISPLAY="${DISPLAY:-:0}"
if [[ -f /run/user/1000/mutter-Xauthority ]]; then
  export XAUTHORITY=/run/user/1000/mutter-Xauthority
fi

mkdir -p "$(dirname "$INSPECTION_ARCHIVE_DB")" "$INSPECTION_CAPTURE_DIR" \
  "$(dirname "$INSPECTION_LAUNCH_LOG")" "$(dirname "$INSPECTION_RUNTIME_LOG")"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
set -u

echo "MODE=full_inspection"
echo "DB=$INSPECTION_ARCHIVE_DB"
echo "CAPTURE_DIR=$INSPECTION_CAPTURE_DIR"
echo "SPAWN_WORLD=($SPAWN_WORLD_X, $SPAWN_WORLD_Y, yaw=$SPAWN_YAW)"
echo "ROUTE_DEFAULT=$ROUTE_ID PORT=$PORT"

stop_all >/dev/null 2>&1 || true

# 1) Frozen launch
nohup ros2 launch navigation_config nav2_test_room.launch.py gui:=false rviz:=false \
  >"$INSPECTION_LAUNCH_LOG" 2>&1 &
echo $! >"$INSPECTION_LAUNCH_PID_FILE"
echo "LAUNCH_PID=$(cat "$INSPECTION_LAUNCH_PID_FILE") log=$INSPECTION_LAUNCH_LOG"

# 2) Wait AMCL active, then confirm GT near birth before initialpose
deadline=$((SECONDS + 120))
while (( SECONDS < deadline )); do
  if timeout 5 ros2 lifecycle get /amcl 2>/dev/null | grep -q active; then
    break
  fi
  sleep 2
done
if ! timeout 5 ros2 lifecycle get /amcl 2>/dev/null | grep -q active; then
  echo "ERROR: /amcl not active; see $INSPECTION_LAUNCH_LOG" >&2
  exit 1
fi

python3 - <<'PY'
"""Confirm ground_truth near frozen birth before publishing initialpose."""
import math, os, sys, time
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

sx = float(os.environ['SPAWN_WORLD_X'])
sy = float(os.environ['SPAWN_WORLD_Y'])
tol = 0.35
rclpy.init()
node = rclpy.create_node('confirm_birth_pose')
box = {'msg': None}

def cb(msg):
    box['msg'] = msg

node.create_subscription(Odometry, '/ground_truth', cb, qos_profile_sensor_data)
deadline = time.time() + 60.0
while time.time() < deadline and rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.2)
    m = box['msg']
    if m is None:
        continue
    x = m.pose.pose.position.x
    y = m.pose.pose.position.y
    err = math.hypot(x - sx, y - sy)
    if err <= tol:
        print(f'BIRTH_OK: gt=({x:.3f},{y:.3f}) spawn=({sx},{sy}) err={err:.3f}m')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    print(f'BIRTH_WAIT: gt=({x:.3f},{y:.3f}) err={err:.3f}m (want <= {tol})')
    time.sleep(0.5)
print('ERROR: ground_truth not near birth; refusing default charge initialpose', file=sys.stderr)
node.destroy_node()
rclpy.shutdown()
sys.exit(1)
PY

# 3) ONE initialpose at confirmed birth — never repeat after robot moves
python3 "$ROOT/scripts/publish_amcl_initial_pose.py" \
  --world-x "$SPAWN_WORLD_X" --world-y "$SPAWN_WORLD_Y" --yaw "$SPAWN_YAW"

# Wait bt_navigator active (may need map TF from initialpose)
deadline=$((SECONDS + 90))
while (( SECONDS < deadline )); do
  if timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q active; then
    break
  fi
  sleep 2
done
python3 "$ROOT/scripts/wait_for_nav_ready.py"

# 4) Runtime (env-unified DB + capture dir)
nohup ros2 run inspection_mission mission_runtime_node --ros-args \
  -p "sqlite_db:=${INSPECTION_ARCHIVE_DB}" \
  -p "capture_output_dir:=${INSPECTION_CAPTURE_DIR}" \
  >"$INSPECTION_RUNTIME_LOG" 2>&1 &
echo $! >"$INSPECTION_RUNTIME_PID_FILE"
deadline=$((SECONDS + 40))
while (( SECONDS < deadline )); do
  if ros2 action list 2>/dev/null | grep -q '/inspection/execute'; then
    echo "RUNTIME_OK"
    break
  fi
  sleep 0.5
done
if ! ros2 action list 2>/dev/null | grep -q '/inspection/execute'; then
  echo "ERROR: /inspection/execute not up; see $INSPECTION_RUNTIME_LOG" >&2
  exit 1
fi

# 5) Web (same DB)
INSPECTION_ARCHIVE_DB="$INSPECTION_ARCHIVE_DB" PORT="$PORT" \
  bash "$ROOT/scripts/run_mission_web.sh" start

echo
echo "READY — do NOT re-publish default charge initialpose after the robot moves."
mission_urls
echo "STOP: bash scripts/run_mission_full.sh stop"
echo "LOGS: launch=$INSPECTION_LAUNCH_LOG runtime=$INSPECTION_RUNTIME_LOG web=$INSPECTION_WEB_LOG"
