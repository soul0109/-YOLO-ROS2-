#!/usr/bin/env bash
# Environment bootstrap checklist for Ubuntu 22.04 + ROS2 Humble.
# Run inside the Ubuntu VM after the base system is ready.
set -euo pipefail

echo "[check] OS"
. /etc/os-release
echo "  $PRETTY_NAME"
if [[ "${VERSION_ID:-}" != "22.04" ]]; then
  echo "  WARN: this project targets Ubuntu 22.04 for ROS2 Humble."
fi

echo "[check] ROS2"
if command -v ros2 >/dev/null 2>&1; then
  ros2 --version || true
  echo "  DISTRO=${ROS_DISTRO:-unset}"
else
  echo "  ros2 not found — install Humble first."
fi

echo "[check] Gazebo"
if command -v gazebo >/dev/null 2>&1; then
  gazebo --version || true
else
  echo "  gazebo not found"
fi

echo "[check] Python"
python3 --version

echo "Done. Next: turtlesim + Gazebo official demo, then colcon build."
