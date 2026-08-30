#!/usr/bin/env bash
# 阶段 3.4 自动化冒烟：/scan 有数据且 frame_id=laser_link（无需 RViz）
# VM 用法：bash ~/inspection-robot/scripts/smoke_test_stage34.sh

set -eo pipefail

WS="${HOME}/inspection-robot/ros2_ws"
LOG="/tmp/smoke_stage34.log"
WORLD="$WS/src/simulation_worlds/worlds/lidar_test.world"

source /opt/ros/humble/setup.bash
cd "$WS"

echo ">>> [1/5] 编译..."
colcon build --packages-select robot_description simulation_worlds 2>&1 | tee "$LOG"

source install/setup.bash

echo ">>> [2/5] 检查 URDF 含 laser_link + ray 插件..."
URDF_OUT=$(xacro "$WS/src/robot_description/urdf/robot_v0.urdf.xacro")
echo "$URDF_OUT" | grep -q 'name="laser_link"' || {
  echo "FAIL: URDF 无 laser_link"
  exit 1
}
echo "$URDF_OUT" | grep -q 'libgazebo_ros_ray_sensor.so' || {
  echo "FAIL: URDF 无 ray 插件"
  exit 1
}
echo "$URDF_OUT" | grep -q 'scan' || {
  echo "FAIL: URDF 未 remap 到 scan"
  exit 1
}
echo "OK: laser_link + ray → scan"

echo ">>> [3/5] 关闭旧 Gazebo / 残留节点..."
# 勿用 pkill -f gzserver：会误杀命令行里带该字串的当前脚本
killall -9 gzserver 2>/dev/null || true
killall -9 gzclient 2>/dev/null || true
pkill -f cmd_vel_timeout 2>/dev/null || true
pkill -f 'gazebo_robot_v0.launch' 2>/dev/null || true
pkill -f patrol_action 2>/dev/null || true
pkill -f log_publisher 2>/dev/null || true
sleep 3

echo ">>> [4/5] 启动仿真（lidar_test.world, gui:=false）..."
ros2 launch simulation_worlds gazebo_robot_v0.launch.py \
  "world:=${WORLD}" gui:=false >"$LOG.launch" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill "$LAUNCH_PID" 2>/dev/null || true
  killall -9 gzserver 2>/dev/null || true
  killall -9 gzclient 2>/dev/null || true
}
trap cleanup EXIT

SCAN_READY=0
for i in $(seq 1 50); do
  if ros2 topic list 2>/dev/null | grep -qx '/scan'; then
    SCAN_READY=1
    break
  fi
  sleep 1
done

if [[ "$SCAN_READY" -ne 1 ]]; then
  echo "FAIL: /scan 话题未出现"
  tail -40 "$LOG.launch"
  exit 1
fi
echo "OK: /scan 已出现"

echo ">>> [5/5] echo /scan --once，校验 frame_id 与 ranges..."
# 等待首帧（仿真时钟起来）
MSG=""
for i in $(seq 1 30); do
  MSG=$(timeout 3 ros2 topic echo /scan --once 2>/dev/null || true)
  if echo "$MSG" | grep -q 'frame_id:'; then
    break
  fi
  sleep 1
done

if ! echo "$MSG" | grep -q 'frame_id: laser_link'; then
  echo "FAIL: frame_id 不是 laser_link"
  echo "$MSG" | head -40
  exit 1
fi

if ! echo "$MSG" | grep -q 'ranges:'; then
  echo "FAIL: 无 ranges 字段"
  echo "$MSG" | head -40
  exit 1
fi

# 障碍物世界里应有部分距离 < max（12.0）；至少有一条非空数值
RANGE_SAMPLE=$(echo "$MSG" | grep -E '^- [0-9]' | head -5 || true)
if [[ -z "$RANGE_SAMPLE" ]]; then
  echo "FAIL: ranges 列表为空"
  echo "$MSG" | head -60
  exit 1
fi

echo "OK: frame_id=laser_link，ranges 有数据"
echo "$MSG" | grep -E 'frame_id:|angle_min:|angle_max:|range_min:|range_max:|ranges:' | head -10

echo ""
echo "=========================================="
echo "  阶段 3.4 冒烟测试通过"
echo "  RViz 人工：ros2 run rviz2 rviz2 -d \\"
echo "    \$(ros2 pkg prefix robot_description)/share/robot_description/rviz/robot_v0_scan.rviz"
echo "  Fixed Frame=odom，应看到红色激光点打在箱子/墙上"
echo "=========================================="
