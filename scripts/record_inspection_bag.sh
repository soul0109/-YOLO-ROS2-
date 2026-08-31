#!/usr/bin/env bash
# 4.2a：test_room 内录制四话题 + TF（手动走巡检环）
#
# 用法：
#   bash ~/inspection-robot/scripts/record_inspection_bag.sh
#   bash ~/inspection-robot/scripts/record_inspection_bag.sh --build
#   bash ~/inspection-robot/scripts/record_inspection_bag.sh --no-rqt
#
# 流程：启动仿真 → 录 bag → 本终端 WASD 遥控走一圈 → Ctrl+C 结束
# 话题：/cmd_vel /odom /scan /camera/image_raw /tf
# 建议路线：充电点 → 房 A → 走廊 → 房 B → 房 C → 回充电点（~2 min）

set -eo pipefail

WS="${HOME}/inspection-robot/ros2_ws"
ROOT="${HOME}/inspection-robot"
LOG="/tmp/record_inspection_bag.launch.log"
DO_BUILD=0
NO_RQT=0

TOPICS=(
  /cmd_vel
  /odom
  /scan
  /camera/image_raw
  /tf
)

usage() {
  sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'
  exit "${1:-0}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) DO_BUILD=1; shift ;;
    --no-rqt) NO_RQT=1; shift ;;
    -h|--help) usage 0 ;;
    *) echo "未知参数: $1"; usage 1 ;;
  esac
done

if [[ ! -t 0 ]]; then
  echo "错误: 请在 VM 普通终端运行（键盘遥控需要 TTY）。"
  exit 1
fi

export DISPLAY="${DISPLAY:-:0}"

source /opt/ros/humble/setup.bash

echo ">>> [1/7] 关闭旧 Gazebo / 遥控 / bag..."
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -f cmd_vel_timeout 2>/dev/null || true
pkill -f keyboard_teleop 2>/dev/null || true
pkill -f 'gazebo_robot_v0.launch' 2>/dev/null || true
pkill -f 'ros2 bag record' 2>/dev/null || true
pkill -f 'rqt_image_view' 2>/dev/null || true
sleep 2

cd "$WS"
if [[ "$DO_BUILD" == "1" ]] || [[ ! -f install/setup.bash ]]; then
  echo ">>> [2/7] 编译 robot_description + simulation_worlds..."
  colcon build --packages-select robot_description simulation_worlds
else
  echo ">>> [2/7] 跳过编译（加 --build 强制）"
fi
source install/setup.bash

WORLD="$(ros2 pkg prefix simulation_worlds)/share/simulation_worlds/worlds/test_room.world"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_DIR="${ROOT}/bags/inspection/run_${STAMP}"

ROS_ENV="source /opt/ros/humble/setup.bash && source ${WS}/install/setup.bash"
LAUNCH_CMD="${ROS_ENV} && ros2 launch simulation_worlds gazebo_robot_v0.launch.py world:=${WORLD} spawn_x:=0.9 spawn_y:=3.0 spawn_yaw:=0.0"
TELEOP_CMD="${ROS_ENV} && ros2 run simulation_worlds keyboard_teleop.py"
RQT_CMD="${ROS_ENV} && ros2 run rqt_image_view rqt_image_view /camera/image_raw"
TOPIC_ARGS="${TOPICS[*]}"
BAG_CMD="${ROS_ENV} && ros2 bag record -o ${BAG_DIR} ${TOPIC_ARGS}"

LAUNCH_PID=""
BAG_PID=""
RQT_PID=""

cleanup() {
  echo ""
  echo ">>> 停止录制与仿真..."
  pkill -f keyboard_teleop 2>/dev/null || true
  [[ -n "$BAG_PID" ]] && kill "$BAG_PID" 2>/dev/null || true
  pkill -f 'ros2 bag record' 2>/dev/null || true
  [[ -n "$RQT_PID" ]] && kill "$RQT_PID" 2>/dev/null || true
  pkill -f 'rqt_image_view' 2>/dev/null || true
  [[ -n "$LAUNCH_PID" ]] && kill "$LAUNCH_PID" 2>/dev/null || true
  pkill -9 -f gzserver 2>/dev/null || true
  pkill -9 -f gzclient 2>/dev/null || true
  pkill -f cmd_vel_timeout 2>/dev/null || true
  if [[ -d "$BAG_DIR" ]]; then
    echo ""
    echo "=========================================="
    echo "  bag 已保存: ${BAG_DIR}"
    echo "  回放: bash ${ROOT}/scripts/play_inspection_bag.sh ${BAG_DIR}"
    echo "  信息: ros2 bag info ${BAG_DIR}"
    echo "=========================================="
  fi
}
trap cleanup EXIT INT TERM

wait_for_topics() {
  local topic
  for topic in "${TOPICS[@]}"; do
    [[ "$topic" == "/tf" ]] && continue
    local ok=0
    for _ in $(seq 1 45); do
      if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
        ok=1
        break
      fi
      sleep 1
    done
    if [[ "$ok" != "1" ]]; then
      echo "FAIL: $topic 未就绪"
      return 1
    fi
    echo "  OK: $topic"
  done
  return 0
}

echo ">>> [3/7] 启动 test_room（Gazebo + spawn 充电垫）..."
: >"$LOG"
bash -lc "$LAUNCH_CMD" >>"$LOG" 2>&1 &
LAUNCH_PID=$!

echo ">>> [4/7] 等待四话题（/tf 随仿真自动发布）..."
if ! wait_for_topics; then
  echo "查日志: $LOG"
  grep -iE 'error|died|camera|scan' "$LOG" | tail -15 || tail -15 "$LOG"
  exit 1
fi

if [[ "$NO_RQT" != "1" ]]; then
  echo ">>> [5/7] 启动 rqt_image_view（可选，便于边走边看）..."
  bash -lc "$RQT_CMD" >/tmp/record_inspection_bag.rqt.log 2>&1 &
  RQT_PID=$!
  sleep 2
else
  echo ">>> [5/7] 跳过 rqt（--no-rqt）"
fi

mkdir -p "$(dirname "$BAG_DIR")"
echo ">>> [6/7] 开始录 bag → ${BAG_DIR}"
bash -lc "$BAG_CMD" >/tmp/record_inspection_bag.bag.log 2>&1 &
BAG_PID=$!
sleep 2

echo ">>> [7/7] 键盘遥控（W/A/S/D，空格急停）"
echo ""
echo "  录制话题: ${TOPICS[*]}"
echo "  请走巡检环：充电点 → 房 A → 走廊 → 房 B → 房 C → 回充电点"
echo "  建议录 1~3 分钟；走完在本终端按 Ctrl+C 结束"
echo ""

if ! python3 -c "import evdev" 2>/dev/null; then
  exec bash -lc "$TELEOP_CMD"
fi
if groups | grep -qw input; then
  exec bash -lc "$TELEOP_CMD"
fi
exec sg input -c "$TELEOP_CMD"
