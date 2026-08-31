#!/usr/bin/env bash
# 保存 test_room SLAM 地图到 navigation_config/maps/
#
# 用法：在建图 launch 仍运行时，另开终端执行
#   bash ~/inspection-robot/scripts/save_test_room_map.sh

set -eo pipefail

WS="${HOME}/inspection-robot/ros2_ws"
MAP_DIR="${WS}/src/navigation_config/maps"
MAP_BASE="${MAP_DIR}/test_room"

source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"

if ! ros2 topic list 2>/dev/null | grep -qx '/map'; then
  echo "错误: 无 /map 话题。请先运行 run_slam_mapping.sh"
  exit 1
fi

mkdir -p "$MAP_DIR"

echo ">>> 保存地图 → ${MAP_BASE}.yaml / .pgm"
ros2 run nav2_map_server map_saver_cli -f "$MAP_BASE"

if [[ -f "${MAP_BASE}.yaml" && -f "${MAP_BASE}.pgm" ]]; then
  echo ""
  echo "=========================================="
  echo "  地图已保存:"
  echo "    ${MAP_BASE}.yaml"
  echo "    ${MAP_BASE}.pgm"
  echo "  可用 git add 提交（体积小）"
  echo "=========================================="
else
  echo "FAIL: 地图文件未生成"
  exit 1
fi
