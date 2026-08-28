#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
cd /home/charles/inspection-robot/ros2_ws
source install/setup.bash

pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
sleep 2

ros2 launch simulation_worlds gazebo_robot_v0.launch.py >/tmp/lt2.log 2>&1 &
LP=$!
trap 'kill $LP 2>/dev/null; pkill -f gzserver 2>/dev/null || true' EXIT

echo "waiting for nodes..."
for i in $(seq 1 40); do
  if ros2 node list 2>/dev/null | grep -q cmd_vel_timeout; then
    break
  fi
  sleep 1
done

ros2 node list
echo "--- python relay test ---"
python3 /tmp/test_cmd_vel_chain.py
echo "python exit: $?"

echo "--- ros2 topic pub test ---"
timeout 2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 20 >/dev/null 2>&1 || true
sleep 0.3
timeout 3 ros2 topic echo /cmd_vel_gazebo --once 2>&1
