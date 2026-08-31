#!/usr/bin/env bash
# 4.2a 自动化冒烟：录 12s → bag info → play --check
# 相机插件需 Gazebo 渲染，使用 gui:=true（与人工录制一致）。
# VM：bash ~/inspection-robot/scripts/smoke_test_stage42a_bag.sh

set -eo pipefail

WS="${HOME}/inspection-robot/ros2_ws"
ROOT="${HOME}/inspection-robot"
LOG="/tmp/smoke_stage42a.log"
WORLD="$WS/src/simulation_worlds/worlds/test_room.world"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_DIR="${ROOT}/bags/inspection/smoke_${STAMP}"
RECORD_SEC=12

if [[ -z "${DISPLAY:-}" ]] && [[ -S /tmp/.X11-unix/X0 ]]; then
  export DISPLAY=:0
fi

source /opt/ros/humble/setup.bash
cd "$WS"

echo ">>> [1/5] 编译..."
colcon build --packages-select robot_description simulation_worlds 2>&1 | tee "$LOG"
source install/setup.bash

echo ">>> [2/5] 清理旧进程..."
killall -9 gzserver 2>/dev/null || true
killall -9 gzclient 2>/dev/null || true
pkill -f cmd_vel_timeout 2>/dev/null || true
pkill -f 'gazebo_robot_v0.launch' 2>/dev/null || true
pkill -f 'ros2 bag record' 2>/dev/null || true
sleep 2

echo ">>> [3/5] 启动 test_room（gui:=true, spawn 0.9,3.0）..."
ros2 launch simulation_worlds gazebo_robot_v0.launch.py \
  "world:=${WORLD}" gui:=true spawn_x:=0.9 spawn_y:=3.0 >"$LOG.launch" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill "$LAUNCH_PID" 2>/dev/null || true
  pkill -f 'ros2 bag record' 2>/dev/null || true
  killall -9 gzserver 2>/dev/null || true
  killall -9 gzclient 2>/dev/null || true
  rm -rf "$BAG_DIR"
}
trap cleanup EXIT

for topic in /cmd_vel /odom /scan /camera/image_raw; do
  ok=0
  for _ in $(seq 1 60); do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      ok=1
      break
    fi
    sleep 1
  done
  [[ "$ok" == "1" ]] || { echo "FAIL: $topic"; tail -20 "$LOG.launch"; exit 1; }
  echo "  OK: $topic"
done

mkdir -p "$(dirname "$BAG_DIR")"
echo ">>> [4/5] 录制 ${RECORD_SEC}s → ${BAG_DIR}"
ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
  >/tmp/smoke_stage42a.pub.log 2>&1 &
VEL_PID=$!
sleep 1
ros2 bag record -o "$BAG_DIR" \
  /cmd_vel /odom /scan /camera/image_raw /tf >/tmp/smoke_stage42a.record.log 2>&1 &
REC_PID=$!
sleep "$RECORD_SEC"
kill "$VEL_PID" 2>/dev/null || true
kill "$REC_PID" 2>/dev/null || true
wait "$REC_PID" 2>/dev/null || true
sleep 1

if [[ ! -f "${BAG_DIR}/metadata.yaml" ]]; then
  echo "FAIL: bag 未生成"
  cat /tmp/smoke_stage42a.record.log || true
  exit 1
fi

echo ">>> [5/5] bag info + 回放 --check"
ros2 bag info "$BAG_DIR"
kill "$LAUNCH_PID" 2>/dev/null || true
killall -9 gzserver 2>/dev/null || true
killall -9 gzclient 2>/dev/null || true
LAUNCH_PID=""
trap - EXIT

bash "${ROOT}/scripts/play_inspection_bag.sh" "$BAG_DIR" --check --rate 2.0

rm -rf "$BAG_DIR"
echo ""
echo "=========================================="
echo "  阶段 4.2a rosbag2 冒烟通过"
echo "  人工录环: bash scripts/record_inspection_bag.sh"
echo "=========================================="
