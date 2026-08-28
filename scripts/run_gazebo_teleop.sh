#!/usr/bin/env bash
# 一键：杀旧 Gazebo → 编译（如需）→ 启动仿真 + 方向键遥控
#
# VM 用法：
#   bash ~/inspection-robot/scripts/run_gazebo_teleop.sh --build

set -eo pipefail

WS="${HOME}/inspection-robot/ros2_ws"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "未找到 ROS2 Humble，请先安装。"
  exit 1
fi

source /opt/ros/humble/setup.bash

echo ">>> 关闭旧 Gazebo 进程..."
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
pkill -f cmd_vel_timeout 2>/dev/null || true
pkill -f 'gazebo_robot_v0' 2>/dev/null || true
pkill -f 'gazebo.launch' 2>/dev/null || true
sleep 1

cd "$WS"

if [[ "${1:-}" == "--build" ]] || [[ ! -f install/setup.bash ]]; then
  echo ">>> 编译 robot_description + simulation_worlds..."
  colcon build --packages-select robot_description simulation_worlds
fi

source install/setup.bash

echo ">>> 启动 Gazebo + 方向键遥控（约 4 秒后在本终端出现按键提示）"
echo ">>> 请保持本终端焦点，用方向键控制；q 退出"
echo ""

exec ros2 launch simulation_worlds gazebo_robot_v0_teleop.launch.py
