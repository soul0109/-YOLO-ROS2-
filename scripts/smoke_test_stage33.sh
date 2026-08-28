#!/usr/bin/env bash
# 阶段 3.3 自动化冒烟测试（无需人工按键）
# VM 用法：bash ~/inspection-robot/scripts/smoke_test_stage33.sh

set -eo pipefail

WS="${HOME}/inspection-robot/ros2_ws"
LOG="/tmp/smoke_stage33.log"

source /opt/ros/humble/setup.bash
cd "$WS"

echo ">>> [1/6] 编译..."
colcon build --packages-select robot_description simulation_worlds 2>&1 | tee -a "$LOG"

source install/setup.bash

echo ">>> [2/6] 检查 URDF 插件订阅话题..."
if ! xacro "$WS/src/robot_description/urdf/robot_v0.urdf.xacro" 2>/dev/null | grep -q 'cmd_vel:=cmd_vel_gazebo'; then
  echo "FAIL: URDF 未配置 cmd_vel_gazebo remapping"
  exit 1
fi
echo "OK: diff_drive remapping cmd_vel:=cmd_vel_gazebo"

echo ">>> [3/6] 关闭旧 Gazebo..."
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
sleep 2

echo ">>> [4/6] 启动仿真（无键盘节点）..."
ros2 launch simulation_worlds gazebo_robot_v0.launch.py >"$LOG.launch" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill "$LAUNCH_PID" 2>/dev/null || true
  pkill -f gzserver 2>/dev/null || true
  pkill -f gzclient 2>/dev/null || true
}
trap cleanup EXIT

for i in $(seq 1 40); do
  if ros2 topic list 2>/dev/null | grep -q cmd_vel_gazebo; then
    if ros2 topic info /cmd_vel_gazebo -v 2>/dev/null | grep -q 'Node name: cmd_vel_timeout'; then
      if grep -q 'cmd_vel_gazebo' "$LOG.launch" 2>/dev/null; then
        break
      fi
    fi
  fi
  sleep 1
done

if ! ros2 topic list 2>/dev/null | grep -q cmd_vel_gazebo; then
  echo "FAIL: /cmd_vel_gazebo 未出现"
  tail -30 "$LOG.launch"
  exit 1
fi
echo "OK: /cmd_vel_gazebo 已存在"

echo ">>> [5/6] 检查中继节点与插件..."
TOPIC_INFO=$(ros2 topic info /cmd_vel_gazebo -v 2>&1 || true)
echo "$TOPIC_INFO" | grep -q 'Node name: cmd_vel_timeout' || {
  echo "FAIL: cmd_vel_timeout 未发布 /cmd_vel_gazebo"
  echo "$TOPIC_INFO"
  exit 1
}
if ! grep -q 'cmd_vel_gazebo' "$LOG.launch"; then
  echo "WARN: launch 日志未出现 cmd_vel_gazebo"
  grep -i 'diff_drive\|cmd_vel' "$LOG.launch" | tail -5 || true
fi
echo "OK: cmd_vel_timeout 正在发布"

echo ">>> [6/6] 测试前进 + 超时刹车..."
# 等 /cmd_vel 订阅就绪（DDS 发现）
for i in $(seq 1 15); do
  if ros2 topic info /cmd_vel -v 2>/dev/null | grep -q 'Node name: cmd_vel_timeout'; then
    break
  fi
  sleep 1
done

timeout 2 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 20 >/dev/null 2>&1 || true
sleep 0.2
FORWARD=$(timeout 3 ros2 topic echo /cmd_vel_gazebo --once 2>/dev/null || true)
echo "$FORWARD" | grep -q "x: 0.15" || {
  echo "FAIL: 转发后 linear.x 不是 0.15"
  echo "$FORWARD"
  exit 1
}
echo "OK: 前进命令已转发"

sleep 0.8
STOP=$(timeout 3 ros2 topic echo /cmd_vel_gazebo --once 2>/dev/null || true)
echo "$STOP" | grep -q "x: 0.0" || {
  echo "FAIL: 超时后未刹车为零"
  echo "$STOP"
  exit 1
}
echo "OK: 超时刹车为零"

echo ""
echo "=========================================="
echo "  阶段 3.3 冒烟测试全部通过"
echo "  人工验收：bash ~/inspection-robot/scripts/run_gazebo_teleop.sh --build"
echo "=========================================="
