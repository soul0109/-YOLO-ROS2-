#!/usr/bin/env bash
# Install ROS2 Humble + Gazebo Classic tooling on Ubuntu 22.04.
# Usage: bash scripts/install_ros2_humble.sh
set -euo pipefail

if [[ "$(. /etc/os-release && echo "$VERSION_ID")" != "22.04" ]]; then
  echo "ERROR: This script targets Ubuntu 22.04 only (found $(. /etc/os-release && echo "$PRETTY_NAME"))."
  exit 1
fi

export DEBIAN_FRONTEND=noninteractive

echo "==> locale"
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "==> prerequisites"
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe -y

echo "==> ROS2 apt repository (Tsinghua mirror)"
sudo mkdir -p /usr/share/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "==> install Humble desktop + Gazebo bridge + build tools"
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-argcomplete \
  python3-vcstool \
  ros-dev-tools

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

BASHRC="$HOME/.bashrc"
MARKER="# >>> ros2 humble >>>"
if ! grep -qF "$MARKER" "$BASHRC" 2>/dev/null; then
  cat >> "$BASHRC" <<'EOF'

# >>> ros2 humble >>>
source /opt/ros/humble/setup.bash
# <<< ros2 humble <<<
EOF
  echo "==> appended source /opt/ros/humble/setup.bash to ~/.bashrc"
fi

echo
echo "Done. Open a new terminal (or: source ~/.bashrc), then verify:"
echo "  source /opt/ros/humble/setup.bash"
echo "  ros2 run turtlesim turtlesim_node"
echo "  gazebo --verbose"
echo "  bash scripts/check_env.sh"
