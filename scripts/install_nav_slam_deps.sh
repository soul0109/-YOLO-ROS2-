#!/usr/bin/env bash
# 阶段 4.3+：安装 SLAM / Nav2 系统依赖（Ubuntu 22.04 + Humble）
#
# 用法：bash ~/inspection-robot/scripts/install_nav_slam_deps.sh

set -eo pipefail

if [[ "$(. /etc/os-release && echo "$VERSION_ID")" != "22.04" ]]; then
  echo "WARN: 本项目主环境为 Ubuntu 22.04 + Humble"
fi

source /opt/ros/humble/setup.bash 2>/dev/null || {
  echo "错误: 未找到 ROS2 Humble，请先安装 ROS2"
  exit 1
}

PKGS=(
  ros-humble-slam-toolbox
  ros-humble-nav2-bringup
  ros-humble-nav2-map-server
  ros-humble-navigation2
)

echo ">>> 安装: ${PKGS[*]}"
sudo apt update
sudo apt install -y "${PKGS[@]}"

echo ""
echo "=========================================="
echo "  SLAM/Nav2 依赖已安装"
echo "  验证: ros2 pkg prefix slam_toolbox"
echo "=========================================="
