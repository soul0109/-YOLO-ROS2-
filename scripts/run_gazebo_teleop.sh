#!/usr/bin/env bash
# 一键：Gazebo 后台 + 本终端方向键遥控（必须交互式终端）
#
# VM 用法：
#   bash ~/inspection-robot/scripts/run_gazebo_teleop.sh --build
#
# 注意：不要在 launch 里嵌键盘节点（无 TTY 会 termios 崩溃）。

set -eo pipefail

WS="${HOME}/inspection-robot/ros2_ws"

if [[ ! -t 0 ]]; then
  echo "错误: 请在 VM 的普通终端里运行本脚本（需要真实 TTY 读方向键）。"
  echo "      IDE 后台任务 / 重定向 stdin 都不行。"
  exit 1
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "未找到 ROS2 Humble，请先安装。"
  exit 1
fi

source /opt/ros/humble/setup.bash

echo ">>> 关闭旧 Gazebo / 残留节点..."
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
pkill -f cmd_vel_timeout 2>/dev/null || true
pkill -f keyboard_teleop 2>/dev/null || true
pkill -f 'gazebo_robot_v0' 2>/dev/null || true
sleep 2

cd "$WS"

if [[ "${1:-}" == "--build" ]] || [[ ! -f install/setup.bash ]]; then
  echo ">>> 编译 robot_description + simulation_worlds..."
  colcon build --packages-select robot_description simulation_worlds
fi

source install/setup.bash

cleanup() {
  echo ""
  echo ">>> 关闭仿真..."
  pkill -f keyboard_teleop 2>/dev/null || true
  pkill -f gzserver 2>/dev/null || true
  pkill -f gzclient 2>/dev/null || true
  pkill -f cmd_vel_timeout 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo ">>> 后台启动 Gazebo（无键盘节点）..."
ros2 launch simulation_worlds gazebo_robot_v0.launch.py >/tmp/gazebo_teleop.launch.log 2>&1 &
LAUNCH_PID=$!

echo ">>> 等待机器人 spawn（约 15s）..."
for i in $(seq 1 30); do
  if grep -q 'Successfully spawned entity' /tmp/gazebo_teleop.launch.log 2>/dev/null; then
    if ros2 node list 2>/dev/null | grep -q cmd_vel_timeout; then
      break
    fi
  fi
  sleep 1
done

if ! grep -q 'cmd_vel_gazebo' /tmp/gazebo_teleop.launch.log 2>/dev/null; then
  echo "WARN: 检查 launch 日志: /tmp/gazebo_teleop.launch.log"
  grep -i 'diff_drive\|cmd_vel\|died\|Error' /tmp/gazebo_teleop.launch.log | tail -10 || true
fi

echo ">>> 在本终端用方向键控制（q 退出并关 Gazebo）"
echo ""

ros2 run simulation_worlds keyboard_teleop.py --ros-args -p use_sim_time:=true
