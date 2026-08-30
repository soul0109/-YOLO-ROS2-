#!/usr/bin/env bash
# 阶段 3.5 自动化冒烟：/camera/image_raw 有数据且 frame_id=camera_link（无需 GUI）
# VM 用法：bash ~/inspection-robot/scripts/smoke_test_stage35.sh

set -eo pipefail

# 相机传感器需要 OGRE 渲染；无 DISPLAY 时插件不会创建 ROS 话题（激光 ray 不受影响）
if [[ -z "${DISPLAY:-}" ]] && [[ -S /tmp/.X11-unix/X0 ]]; then
  export DISPLAY=:0
fi

WS="${HOME}/inspection-robot/ros2_ws"
LOG="/tmp/smoke_stage35.log"
WORLD="$WS/src/simulation_worlds/worlds/lidar_test.world"

source /opt/ros/humble/setup.bash
cd "$WS"

echo ">>> [1/5] 编译..."
colcon build --packages-select robot_description simulation_worlds 2>&1 | tee "$LOG"

source install/setup.bash

echo ">>> [2/5] 检查 URDF 含 camera_link + camera 插件..."
URDF_OUT=$(xacro "$WS/src/robot_description/urdf/robot_v0.urdf.xacro")
echo "$URDF_OUT" | grep -q 'name="camera_link"' || {
  echo "FAIL: URDF 无 camera_link"
  exit 1
}
echo "$URDF_OUT" | grep -q 'libgazebo_ros_camera.so' || {
  echo "FAIL: URDF 无 camera 插件"
  exit 1
}
echo "$URDF_OUT" | grep -q 'camera/image_raw' || {
  echo "FAIL: URDF 未 remap 到 camera/image_raw"
  exit 1
}
echo "OK: camera_link + camera → /camera/image_raw"

echo ">>> [3/5] 关闭旧 Gazebo / 残留节点..."
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

CAM_READY=0
for i in $(seq 1 60); do
  if ros2 topic list 2>/dev/null | grep -qx '/camera/image_raw'; then
    CAM_READY=1
    break
  fi
  sleep 1
done

if [[ "$CAM_READY" -ne 1 ]]; then
  echo "FAIL: /camera/image_raw 话题未出现"
  tail -40 "$LOG.launch"
  exit 1
fi
echo "OK: /camera/image_raw 已出现"

echo ">>> [5/5] echo 首帧，校验 frame_id / 分辨率..."
FRAME_ID=""
WIDTH=""
HEIGHT=""
for i in $(seq 1 40); do
  RAW=$(timeout 8 ros2 topic echo /camera/image_raw --once 2>/dev/null || true)
  if echo "$RAW" | grep -q 'frame_id:'; then
    FRAME_ID=$(echo "$RAW" | awk '/frame_id:/{print $2}' | tr -d "'\"")
    WIDTH=$(echo "$RAW" | awk '/^width:/{print $2}')
    HEIGHT=$(echo "$RAW" | awk '/^height:/{print $2}')
    break
  fi
  sleep 1
done

if [[ "$FRAME_ID" != "camera_link" ]]; then
  echo "FAIL: frame_id 不是 camera_link（得到: '$FRAME_ID'）"
  exit 1
fi

if [[ "$WIDTH" != "640" ]] || [[ "$HEIGHT" != "480" ]]; then
  echo "FAIL: 分辨率不是 640x480（得到: ${WIDTH}x${HEIGHT}）"
  exit 1
fi

echo "OK: frame_id=camera_link, 640x480"
echo ""
echo "=========================================="
echo "  阶段 3.5 冒烟测试通过"
echo "  人工看图："
echo "    ros2 run rqt_image_view rqt_image_view /camera/image_raw"
echo "  或 RViz：Add → Image → Topic=/camera/image_raw"
echo "=========================================="
