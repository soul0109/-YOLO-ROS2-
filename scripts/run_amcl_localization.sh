#!/usr/bin/env bash
# 4.4：test_room + AMCL 定位 + 键盘遥控
#
# 用法：
#   bash ~/inspection-robot/scripts/run_amcl_localization.sh
#   bash ~/inspection-robot/scripts/run_amcl_localization.sh --build
#
# 另开终端验收（慢走 30s 后）：
#   python3 ~/inspection-robot/scripts/check_amcl_accuracy.py --duration 30

set -eo pipefail

WS="${HOME}/inspection-robot/ros2_ws"
ROOT="${HOME}/inspection-robot"
LOG="/tmp/run_amcl_localization.launch.log"
DO_BUILD=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) DO_BUILD=1; shift ;;
    -h|--help)
      sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'
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

if ! ros2 pkg prefix nav2_amcl >/dev/null 2>&1; then
  echo "错误: 未安装 nav2_amcl。运行: bash ${ROOT}/scripts/install_nav_slam_deps.sh"
  exit 1
fi

MAP_YAML="${WS}/src/navigation_config/maps/test_room.yaml"
if [[ ! -f "$MAP_YAML" ]]; then
  echo "错误: 找不到地图 ${MAP_YAML}，请先完成 4.3 SLAM 建图"
  exit 1
fi

echo ">>> [1/6] 关闭旧 Gazebo / SLAM / AMCL / 遥控..."
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -f cmd_vel_timeout 2>/dev/null || true
pkill -f keyboard_teleop 2>/dev/null || true
pkill -f 'gazebo_robot_v0.launch' 2>/dev/null || true
pkill -f 'slam_toolbox' 2>/dev/null || true
pkill -f 'nav2_amcl/amcl' 2>/dev/null || true
pkill -f 'lifecycle_manager_localization' 2>/dev/null || true
pkill -f 'map_server' 2>/dev/null || true
sleep 2

cd "$WS"
if [[ "$DO_BUILD" == "1" ]] || [[ ! -f install/setup.bash ]]; then
  echo ">>> [2/6] 编译 navigation_config + simulation_worlds + robot_description..."
  colcon build --packages-select robot_description simulation_worlds navigation_config
else
  echo ">>> [2/6] 跳过编译（加 --build 强制）"
fi
source install/setup.bash

ROS_ENV="source /opt/ros/humble/setup.bash && source ${WS}/install/setup.bash"
LAUNCH_CMD="${ROS_ENV} && ros2 launch navigation_config amcl_test_room.launch.py"
TELEOP_CMD="${ROS_ENV} && ros2 run simulation_worlds keyboard_teleop.py"
POSE_CMD="python3 ${ROOT}/scripts/publish_amcl_initial_pose.py --map-yaml ${MAP_YAML} --world-x 0.9 --world-y 3.0 --yaw 0"

LAUNCH_PID=""

cleanup() {
  echo ""
  echo ">>> 停止 AMCL 与仿真..."
  pkill -f keyboard_teleop 2>/dev/null || true
  [[ -n "$LAUNCH_PID" ]] && kill "$LAUNCH_PID" 2>/dev/null || true
  pkill -f 'nav2_amcl/amcl' 2>/dev/null || true
pkill -f 'lifecycle_manager_localization' 2>/dev/null || true
  pkill -f 'map_server' 2>/dev/null || true
  pkill -9 -f gzserver 2>/dev/null || true
  pkill -9 -f gzclient 2>/dev/null || true
  pkill -f cmd_vel_timeout 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo ">>> [3/6] 启动 test_room + map_server + AMCL + RViz..."
: >"$LOG"
bash -lc "$LAUNCH_CMD" >>"$LOG" 2>&1 &
LAUNCH_PID=$!

echo ">>> [4/6] 等待 /map /scan（AMCL 节点）..."
ready=0
for _ in $(seq 1 60); do
  if ros2 topic list 2>/dev/null | grep -qx '/map' \
     && ros2 topic list 2>/dev/null | grep -qx '/scan' \
     && ros2 node list 2>/dev/null | grep -q '/amcl'; then
    ready=1
    break
  fi
  printf '.'
  sleep 1
done
echo ""
if [[ "$ready" != "1" ]]; then
  echo "FAIL: AMCL 未就绪。查日志: $LOG"
  grep -iE 'error|amcl|map_server|died' "$LOG" | tail -20 || tail -20 "$LOG"
  exit 1
fi
echo "OK: /map + /scan + amcl 节点"

echo ">>> [5/6] 发布初始位姿（充电垫 world 0.9, 3.0）..."
sleep 2
bash -lc "${ROS_ENV} && ${POSE_CMD}"

echo ">>> [6/6] 键盘遥控（W/A/S/D）"
echo ""
echo "  RViz：Fixed Frame=map，看粒子云是否聚成一束"
echo "  慢走几步后另开终端验收:"
echo "    python3 ${ROOT}/scripts/check_amcl_accuracy.py --duration 30"
echo "  粒子散：RViz 用「2D Pose Estimate」点在黄充电垫上"
echo ""

if ! python3 -c "import evdev" 2>/dev/null; then
  exec bash -lc "$TELEOP_CMD"
fi
if groups | grep -qw input; then
  exec bash -lc "$TELEOP_CMD"
fi
exec sg input -c "$TELEOP_CMD"
