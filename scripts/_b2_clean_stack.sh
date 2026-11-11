#!/usr/bin/env bash
set +e
ROOT=/home/charles/inspection-robot
LOG="$ROOT/log/b2_vision_test_20261111"
mkdir -p "$LOG"
OUT="$LOG/02_manual_clean.txt"
{
  echo "=== CLEAN $(date -Iseconds) ==="
  if [[ -f /tmp/inspection_mission_launch_tree.pids ]]; then
    while read -r p; do
      if [[ -n "$p" && -d "/proc/$p" ]]; then
        echo "kill tree $p"
        kill -9 "$p" 2>/dev/null
      fi
    done < /tmp/inspection_mission_launch_tree.pids
    rm -f /tmp/inspection_mission_launch_tree.pids
  fi
  pkill -9 -f nav2_test_room.launch.py
  pkill -9 -f mission_runtime_node
  pkill -9 -f 'uvicorn mission_api.app:app'
  pkill -9 -f cmd_vel_timeout.py
  for pat in nav2_controller nav2_planner nav2_bt_navigator nav2_smoother nav2_behaviors nav2_waypoint nav2_velocity lifecycle_manager map_server '/opt/ros/humble/lib/nav2_amcl/amcl'; do
    pkill -9 -f "$pat"
  done
  for pid in $(pgrep -x gzserver); do
    cmd=$(tr '\0' ' ' <"/proc/$pid/cmdline")
    echo "gz pid=$pid"
    case "$cmd" in *test_room.world*) kill -9 "$pid"; echo "killed_gz_$pid";; esac
  done
  sleep 2
  rem=$(pgrep -af 'gzserver|nav2_test_room|mission_runtime|uvicorn mission_api|lifecycle_manager|/map_server|/amcl' | grep -v pgrep || true)
  if [[ -z "$rem" ]]; then
    echo STACK_CLEAR
  else
    echo STACK_DIRTY
    echo "$rem"
  fi
} | tee "$OUT"
