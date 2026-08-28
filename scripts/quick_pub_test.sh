#!/usr/bin/env bash
# Minimal relay test - run while gazebo_robot_v0.launch.py is up
set -eo pipefail
source /opt/ros/humble/setup.bash
cd /home/charles/inspection-robot/ros2_ws
source install/setup.bash

echo "nodes:"; ros2 node list 2>/dev/null | grep -E 'cmd_vel|diff' || true

echo "pub 2s..."
timeout 2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 20
sleep 0.2
echo "echo:"
timeout 2 ros2 topic echo /cmd_vel_gazebo --once
