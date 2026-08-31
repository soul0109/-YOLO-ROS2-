#!/usr/bin/env bash
# 4.3：test_room 键盘遥控 + slam_toolbox 建图（本终端 WASD）
#
# 用法：
#   bash ~/inspection-robot/scripts/run_slam_mapping.sh
#   bash ~/inspection-robot/scripts/run_slam_mapping.sh --build
#
# 另开终端保存地图（建图满意后）：
#   bash ~/inspection-robot/scripts/save_test_room_map.sh

set -eo pipefail

WS="${HOME}/inspection-robot/ros2_ws"
ROOT="${HOME}/inspection-robot"
LOG="/tmp/run_slam_mapping.launch.log"
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

if ! ros2 pkg prefix slam_toolbox >/dev/null 2>&1; then
  echo "错误: 未安装 slam_toolbox"
  echo "  运行: bash ${ROOT}/scripts/install_nav_slam_deps.sh"
  exit 1
fi

echo ">>> [1/5] 关闭旧 Gazebo / SLAM / 遥控..."
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -f cmd_vel_timeout 2>/dev/null || true
pkill -f keyboard_teleop 2>/dev/null || true
pkill -f 'gazebo_robot_v0.launch' 2>/dev/null || true
pkill -f 'slam_toolbox' 2>/dev/null || true
pkill -f 'async_slam_toolbox' 2>/dev/null || true
sleep 2

cd "$WS"
if [[ "$DO_BUILD" == "1" ]] || [[ ! -f install/setup.bash ]]; then
  echo ">>> [2/5] 编译 navigation_config + simulation_worlds + robot_description..."
  colcon build --packages-select robot_description simulation_worlds navigation_config
else
  echo ">>> [2/5] 跳过编译（加 --build 强制）"
fi
source install/setup.bash

ROS_ENV="source /opt/ros/humble/setup.bash && source ${WS}/install/setup.bash"
LAUNCH_CMD="${ROS_ENV} && ros2 launch navigation_config slam_mapping_test_room.launch.py"
TELEOP_CMD="${ROS_ENV} && ros2 run simulation_worlds keyboard_teleop.py"

LAUNCH_PID=""

cleanup() {
  echo ""
  echo ">>> 停止建图与仿真..."
  pkill -f keyboard_teleop 2>/dev/null || true
  [[ -n "$LAUNCH_PID" ]] && kill "$LAUNCH_PID" 2>/dev/null || true
  pkill -f 'slam_toolbox' 2>/dev/null || true
  pkill -9 -f gzserver 2>/dev/null || true
  pkill -9 -f gzclient 2>/dev/null || true
  pkill -f cmd_vel_timeout 2>/dev/null || true
  echo ""
  echo "  保存地图（另开终端，建图仍运行时执行）："
  echo "    bash ${ROOT}/scripts/save_test_room_map.sh"
}
trap cleanup EXIT INT TERM

echo ">>> [3/5] 启动 test_room + slam_toolbox + RViz..."
: >"$LOG"
bash -lc "$LAUNCH_CMD" >>"$LOG" 2>&1 &
LAUNCH_PID=$!

echo ">>> [4/5] 等待 /scan /map /odom..."
ready=0
for _ in $(seq 1 60); do
  if ros2 topic list 2>/dev/null | grep -qx '/scan' \
     && ros2 topic list 2>/dev/null | grep -qx '/map' \
     && ros2 topic list 2>/dev/null | grep -qx '/odom'; then
    ready=1
    break
  fi
  printf '.'
  sleep 1
done
echo ""
if [[ "$ready" != "1" ]]; then
  echo "FAIL: SLAM 话题未就绪。查日志: $LOG"
  grep -iE 'error|slam|died' "$LOG" | tail -20 || tail -20 "$LOG"
  exit 1
fi
echo "OK: /scan + /map + /odom"

echo ">>> [5/5] 键盘遥控建图（W/A/S/D）"
echo ""
echo "  RViz 应显示地图随移动生长；慢速走遍三机房+走廊"
echo "  满意后另开终端: bash ${ROOT}/scripts/save_test_room_map.sh"
echo "  本终端 Ctrl+C 结束"
echo ""

if ! python3 -c "import evdev" 2>/dev/null; then
  exec bash -lc "$TELEOP_CMD"
fi
if groups | grep -qw input; then
  exec bash -lc "$TELEOP_CMD"
fi
exec sg input -c "$TELEOP_CMD"
