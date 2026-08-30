#!/usr/bin/env bash
# T3：test_room 内录制 /odom vs /ground_truth（手动走巡检环）
#
# 用法：
#   bash ~/inspection-robot/scripts/record_odom_drift.sh
#   bash ~/inspection-robot/scripts/record_odom_drift.sh --build
#
# 流程：启动仿真 → 录 bag → 本终端 WASD 遥控走一圈 → Ctrl+C 结束
# 建议路线：充电点 → 房 A → 走廊 → 房 B → 房 C → 回充电点（~20 m）

set -eo pipefail

WS="${HOME}/inspection-robot/ros2_ws"
ROOT="${HOME}/inspection-robot"
LOG="/tmp/record_odom_drift.launch.log"
DO_BUILD=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) DO_BUILD=1; shift ;;
    -h|--help)
      sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *) echo "未知参数: $1"; exit 1 ;;
  esac
done

if [[ ! -t 0 ]]; then
  echo "错误: 请在 VM 普通终端运行（键盘遥控需要 TTY）。"
  exit 1
fi

export DISPLAY="${DISPLAY:-:0}"

source /opt/ros/humble/setup.bash

echo ">>> [1/6] 关闭旧 Gazebo / 遥控 / bag..."
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -f cmd_vel_timeout 2>/dev/null || true
pkill -f keyboard_teleop 2>/dev/null || true
pkill -f 'gazebo_robot_v0.launch' 2>/dev/null || true
pkill -f 'ros2 bag record' 2>/dev/null || true
sleep 2

cd "$WS"
if [[ "$DO_BUILD" == "1" ]] || [[ ! -f install/setup.bash ]]; then
  echo ">>> [2/6] 编译 robot_description + simulation_worlds..."
  colcon build --packages-select robot_description simulation_worlds
else
  echo ">>> [2/6] 跳过编译（加 --build 强制）"
fi
source install/setup.bash

WORLD="$(ros2 pkg prefix simulation_worlds)/share/simulation_worlds/worlds/test_room.world"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_DIR="${ROOT}/bags/odom_drift/run_${STAMP}"

ROS_ENV="source /opt/ros/humble/setup.bash && source ${WS}/install/setup.bash"
LAUNCH_CMD="${ROS_ENV} && ros2 launch simulation_worlds gazebo_robot_v0.launch.py world:=${WORLD} spawn_x:=0.9 spawn_y:=3.0 spawn_yaw:=0.0"
TELEOP_CMD="${ROS_ENV} && ros2 run simulation_worlds keyboard_teleop.py"
BAG_CMD="${ROS_ENV} && ros2 bag record -o ${BAG_DIR} /odom /ground_truth /tf"

LAUNCH_PID=""
BAG_PID=""

cleanup() {
  echo ""
  echo ">>> 停止录制与仿真..."
  pkill -f keyboard_teleop 2>/dev/null || true
  [[ -n "$BAG_PID" ]] && kill "$BAG_PID" 2>/dev/null || true
  pkill -f 'ros2 bag record' 2>/dev/null || true
  [[ -n "$LAUNCH_PID" ]] && kill "$LAUNCH_PID" 2>/dev/null || true
  pkill -9 -f gzserver 2>/dev/null || true
  pkill -9 -f gzclient 2>/dev/null || true
  pkill -f cmd_vel_timeout 2>/dev/null || true
  if [[ -d "$BAG_DIR" ]]; then
    echo ""
    echo "=========================================="
    echo "  bag 已保存: ${BAG_DIR}"
    echo "  分析: python3 ${ROOT}/scripts/odom_drift_analysis.py ${BAG_DIR}"
    echo "=========================================="
  fi
}
trap cleanup EXIT INT TERM

echo ">>> [3/6] 启动 test_room（Gazebo + spawn 充电垫）..."
: >"$LOG"
bash -lc "$LAUNCH_CMD" >>"$LOG" 2>&1 &
LAUNCH_PID=$!

echo ">>> [4/6] 等待 /odom 与 /ground_truth..."
ready=0
for _ in $(seq 1 45); do
  if ros2 topic list 2>/dev/null | grep -qx '/odom' \
     && ros2 topic list 2>/dev/null | grep -qx '/ground_truth'; then
    ready=1
    break
  fi
  printf '.'
  sleep 1
done
echo ""
if [[ "$ready" != "1" ]]; then
  echo "FAIL: 话题未就绪。查日志: $LOG"
  grep -iE 'error|ground_truth|p3d|died' "$LOG" | tail -15 || tail -15 "$LOG"
  exit 1
fi
echo "OK: /odom + /ground_truth"

mkdir -p "$(dirname "$BAG_DIR")"
echo ">>> [5/6] 开始录 bag → ${BAG_DIR}"
bash -lc "$BAG_CMD" >/tmp/record_odom_drift.bag.log 2>&1 &
BAG_PID=$!
sleep 2

echo ">>> [6/6] 键盘遥控（W/A/S/D，空格急停）"
echo ""
echo "  请走巡检环：充电点 → 房 A → 走廊 → 房 B → 房 C → 回充电点"
echo "  走完一圈后在本终端按 Ctrl+C 结束录制"
echo ""

if ! python3 -c "import evdev" 2>/dev/null; then
  exec bash -lc "$TELEOP_CMD"
fi
if groups | grep -qw input; then
  exec bash -lc "$TELEOP_CMD"
fi
exec sg input -c "$TELEOP_CMD"
