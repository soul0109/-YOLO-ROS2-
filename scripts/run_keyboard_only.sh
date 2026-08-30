#!/usr/bin/env bash
# 仅键盘遥控：不杀、不重开 Gazebo（仿真已在别的终端跑着时用）
#
# 三终端联调示例：
#   终端 A: ros2 launch simulation_worlds gazebo_robot_v0.launch.py world:=$WORLD
#   终端 B: ros2 run rqt_image_view rqt_image_view /camera/image_raw
#   终端 C: bash ~/inspection-robot/scripts/run_keyboard_only.sh

set -eo pipefail

WS="${HOME}/inspection-robot/ros2_ws"

if [[ ! -t 0 ]]; then
  echo "错误: 请在 VM 的普通终端里运行（需要 TTY）。"
  exit 1
fi

source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"

if ! ros2 topic list 2>/dev/null | grep -qx '/cmd_vel'; then
  echo "错误: 未发现 /cmd_vel，请先在另一终端启动 gazebo_robot_v0.launch.py"
  exit 1
fi

echo ">>> 键盘遥控（W/A/S/D，空格急停）"
echo "    保持本终端焦点，不要点 Gazebo / rqt 窗口"
echo ""

TELEOP_CMD="source /opt/ros/humble/setup.bash && source ${WS}/install/setup.bash && ros2 run simulation_worlds keyboard_teleop.py"

if ! python3 -c "import evdev" 2>/dev/null; then
  echo "提示: 未安装 evdev → TTY 点按模式。安装: bash scripts/setup_keyboard_evdev.sh"
  exec bash -lc "$TELEOP_CMD"
fi

if groups | grep -qw input; then
  exec bash -lc "$TELEOP_CMD"
fi

exec sg input -c "$TELEOP_CMD"
