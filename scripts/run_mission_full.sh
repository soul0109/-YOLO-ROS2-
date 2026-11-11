#!/usr/bin/env bash
# Full inspection cold start: frozen Nav2 launch → confirm birth pose → ONE initialpose
# → wait_nav_ready → Runtime + Web. Does NOT re-send charge-pose initialpose after move.
#
# Usage:
#   bash scripts/run_mission_full.sh          # start
#   bash scripts/run_mission_full.sh stop     # stop web + runtime + launch
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck disable=SC1091
source "$ROOT/scripts/mission_defaults.sh"

cmd="${1:-start}"

stop_all() {
  bash "$ROOT/scripts/run_mission_web.sh" stop >/dev/null 2>&1 || true
  if [[ -f "$INSPECTION_RUNTIME_PID_FILE" ]]; then
    kill "$(cat "$INSPECTION_RUNTIME_PID_FILE")" 2>/dev/null || true
    rm -f "$INSPECTION_RUNTIME_PID_FILE"
  fi
  pkill -f 'mission_runtime_node' 2>/dev/null || true
  if [[ -f "$INSPECTION_LAUNCH_PID_FILE" ]]; then
    kill "$(cat "$INSPECTION_LAUNCH_PID_FILE")" 2>/dev/null || true
    rm -f "$INSPECTION_LAUNCH_PID_FILE"
  fi
  pkill -f 'nav2_test_room.launch.py' 2>/dev/null || true
  # Leave gzserver cleanup to launch teardown; force if still up.
  sleep 1
  echo "stopped (web+runtime+launch signalled)"
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
