#!/usr/bin/env bash
# 阶段 3 闸门：同一 launch 下四话题齐（/cmd_vel /odom /scan /camera/image_raw）
# VM 用法：bash ~/inspection-robot/scripts/smoke_test_stage3_gate.sh

set -eo pipefail

if [[ -z "${DISPLAY:-}" ]] && [[ -S /tmp/.X11-unix/X0 ]]; then
  export DISPLAY=:0
fi

WS="${HOME}/inspection-robot/ros2_ws"
LOG="/tmp/smoke_stage3_gate.log"
WORLD="$WS/src/simulation_worlds/worlds/lidar_test.world"

source /opt/ros/humble/setup.bash
cd "$WS"

echo ">>> [1/6] 编译..."
colcon build --packages-select robot_description simulation_worlds 2>&1 | tee "$LOG"
source install/setup.bash

echo ">>> [2/6] 清理旧进程..."
killall -9 gzserver 2>/dev/null || true
killall -9 gzclient 2>/dev/null || true
pkill -f cmd_vel_timeout 2>/dev/null || true
pkill -f 'gazebo_robot_v0.launch' 2>/dev/null || true
sleep 3

echo ">>> [3/6] 启动统一 launch（lidar_test.world, gui:=false）..."
ros2 launch simulation_worlds gazebo_robot_v0.launch.py \
  "world:=${WORLD}" gui:=false spawn_x:=0.0 spawn_y:=0.0 >"$LOG.launch" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill "$LAUNCH_PID" 2>/dev/null || true
  killall -9 gzserver 2>/dev/null || true
  killall -9 gzclient 2>/dev/null || true
}
trap cleanup EXIT

echo ">>> [4/6] 等待四话题..."
for topic in /cmd_vel /odom /scan /camera/image_raw; do
  ok=0
  for _ in $(seq 1 60); do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      ok=1
      break
    fi
    sleep 1
  done
  if [[ "$ok" -ne 1 ]]; then
    echo "FAIL: $topic 未出现"
    tail -30 "$LOG.launch"
    exit 1
  fi
  echo "  OK: $topic"
done

echo ">>> [5/6] 校验 frame_id..."
SCAN_MSG=""
ODOM_MSG=""
for _ in $(seq 1 30); do
  SCAN_MSG=$(timeout 3 ros2 topic echo /scan --once 2>/dev/null || true)
  ODOM_MSG=$(timeout 3 ros2 topic echo /odom --once 2>/dev/null || true)
  if echo "$SCAN_MSG" | grep -q 'frame_id: laser_link' && echo "$ODOM_MSG" | grep -q 'child_frame_id: base_footprint'; then
    break
  fi
  sleep 1
done
echo "$SCAN_MSG" | grep -q 'frame_id: laser_link' || { echo "FAIL: /scan frame_id"; exit 1; }
echo "$ODOM_MSG" | grep -q 'child_frame_id: base_footprint' || { echo "FAIL: /odom child_frame"; exit 1; }
echo "  OK: laser_link + base_footprint"

echo ">>> [6/6] 校验相机首帧..."
python3 << 'PY'
import sys
import rclpy
from sensor_msgs.msg import Image
import numpy as np

class Once:
    def __init__(self):
        self.node = rclpy.create_node('stage3_gate_cam')
        self.got = False
        self.node.create_subscription(Image, '/camera/image_raw', self.cb, 10)
    def cb(self, msg):
        if self.got:
            return
        self.got = True
        self.msg = msg

rclpy.init()
o = Once()
for _ in range(60):
    rclpy.spin_once(o.node, timeout_sec=0.5)
    if o.got:
        break
if not o.got:
    print('FAIL: 无相机图像')
    sys.exit(1)
m = o.msg
if m.header.frame_id != 'camera_link':
    print(f'FAIL: frame_id={m.header.frame_id}')
    sys.exit(1)
if m.width != 640 or m.height != 480:
    print(f'FAIL: size {m.width}x{m.height}')
    sys.exit(1)
arr = np.frombuffer(m.data, dtype=np.uint8)
if arr.std() < 5:
    print('FAIL: 图像近乎纯色（可能朝地/全黑）')
    sys.exit(1)
print(f'OK: camera_link 640x480 std={arr.std():.1f}')
o.node.destroy_node()
rclpy.shutdown()
PY

echo ""
echo "=========================================="
echo "  阶段 3 闸门冒烟通过（四话题 + frame_id）"
echo "  人工看图（作品集证据）："
echo "    终端A: ros2 launch simulation_worlds gazebo_robot_v0.launch.py world:=\$WORLD"
echo "    终端B: ros2 run rqt_image_view rqt_image_view /camera/image_raw"
echo "=========================================="
