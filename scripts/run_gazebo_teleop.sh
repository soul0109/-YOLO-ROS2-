#!/usr/bin/env bash
# 一键：Gazebo 后台 + 本终端键盘遥控（W/A/S/D，必须交互式终端）
#
# VM 用法：
#   bash ~/inspection-robot/scripts/run_gazebo_teleop.sh --build

set -eo pipefail

WS="${HOME}/inspection-robot/ros2_ws"
LOG="/tmp/gazebo_teleop.launch.log"

if [[ ! -t 0 ]]; then
  echo "错误: 请在 VM 的普通终端里运行（需要 TTY）。"
  exit 1
fi

source /opt/ros/humble/setup.bash

echo ">>> [1/4] 彻底关闭旧 Gazebo / 残留 ROS 节点..."
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -f cmd_vel_timeout 2>/dev/null || true
pkill -f keyboard_teleop 2>/dev/null || true
pkill -f spawn_entity 2>/dev/null || true
pkill -f 'gazebo_robot_v0.launch' 2>/dev/null || true
sleep 2

# 确认清干净了
if pgrep -f gzserver >/dev/null 2>&1; then
  echo "WARN: 仍有 gzserver 进程，请手动: pkill -9 -f gzserver"
fi

cd "$WS"

if [[ "${1:-}" == "--build" ]] || [[ ! -f install/setup.bash ]]; then
  echo ">>> [2/4] 编译..."
  colcon build --packages-select robot_description simulation_worlds
else
  echo ">>> [2/4] 跳过编译（加 --build 强制重编）"
fi

source install/setup.bash

cleanup() {
  echo ""
  echo ">>> 关闭仿真..."
  pkill -f keyboard_teleop 2>/dev/null || true
  pkill -9 -f gzserver 2>/dev/null || true
  pkill -9 -f gzclient 2>/dev/null || true
  pkill -f cmd_vel_timeout 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo ">>> [3/4] 启动 Gazebo..."
: >"$LOG"
ros2 launch simulation_worlds gazebo_robot_v0.launch.py >>"$LOG" 2>&1 &
LAUNCH_PID=$!

READY=0
for i in $(seq 1 25); do
  if grep -q 'process has died.*gzserver' "$LOG" 2>/dev/null; then
    echo ""
    echo "FAIL: gzserver 启动失败（常因旧进程未杀干净或显卡问题）"
    tail -15 "$LOG"
    exit 1
  fi
  if grep -q 'Successfully spawned entity' "$LOG" 2>/dev/null \
     && grep -q 'Subscribed to \[/cmd_vel_gazebo\]' "$LOG" 2>/dev/null; then
    READY=1
    break
  fi
  printf '.'
  sleep 1
done
echo ""

if [[ "$READY" != "1" ]]; then
  echo "WARN: 25s 内未完全就绪，继续尝试键盘（可查日志: $LOG）"
  grep -iE 'died|Error|cmd_vel|spawn' "$LOG" | tail -8 || true
fi

echo ">>> [4/4] 启动键盘遥控（请保持本终端焦点，用 W/A/S/D，不要用鼠标点 Gazebo）"
echo ""

# 键盘节点不要用 use_sim_time（否则定时器跟仿真时钟走，会卡）
exec ros2 run simulation_worlds keyboard_teleop.py
