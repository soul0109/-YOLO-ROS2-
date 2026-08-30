#!/usr/bin/env bash
# 一键阶段 3 联调：Gazebo + rqt 相机窗口 + 本终端键盘遥控
#
# 默认：Gazebo 后台（日志写 /tmp/gazebo_camera_teleop.launch.log），
#       rqt_image_view 弹出独立 Qt 窗口，本终端 WASD。
# 可选 --log-window：另开 gnome-terminal  tail 仿真日志（调试用）。
#
# 用法：
#   bash ~/inspection-robot/scripts/run_gazebo_camera_teleop.sh
#   bash ~/inspection-robot/scripts/run_gazebo_camera_teleop.sh --build
#   bash ~/inspection-robot/scripts/run_gazebo_camera_teleop.sh --world test_room.world
#   INSPECTION_GAZEBO_WORLD=test_room.world bash ~/inspection-robot/scripts/run_gazebo_camera_teleop.sh
#   bash ~/inspection-robot/scripts/run_gazebo_camera_teleop.sh --no-camera
#   bash ~/inspection-robot/scripts/run_gazebo_camera_teleop.sh --log-window

set -eo pipefail

WS="${HOME}/inspection-robot/ros2_ws"
LOG="/tmp/gazebo_camera_teleop.launch.log"
WORLD_FILE=""
DO_BUILD=0
NO_CAMERA=0
LOG_WINDOW=0

usage() {
  sed -n '2,13p' "$0" | sed 's/^# \{0,1\}//'
  exit "${1:-0}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) DO_BUILD=1; shift ;;
    --no-camera) NO_CAMERA=1; shift ;;
    --log-window) LOG_WINDOW=1; shift ;;
    --world)
      [[ $# -ge 2 ]] || { echo "错误: --world 需要参数"; exit 1; }
      WORLD_FILE="$2"
      shift 2
      ;;
    -h|--help) usage 0 ;;
    --inline|--tabs)
      echo "提示: --inline / --tabs 已废弃，当前默认即「Gazebo 后台 + rqt 弹窗 + 本终端遥控」"
      shift
      ;;
    *) echo "未知参数: $1"; usage 1 ;;
  esac
done

if [[ ! -t 0 ]]; then
  echo "错误: 请在 VM 的普通终端里运行（键盘遥控需要 TTY）。"
  exit 1
fi

export DISPLAY="${DISPLAY:-:0}"

source /opt/ros/humble/setup.bash

echo ">>> [1/5] 关闭旧 Gazebo / rqt / 遥控..."
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -f cmd_vel_timeout 2>/dev/null || true
pkill -f keyboard_teleop 2>/dev/null || true
pkill -f spawn_entity 2>/dev/null || true
pkill -f 'gazebo_robot_v0.launch' 2>/dev/null || true
pkill -f 'rqt_image_view' 2>/dev/null || true
sleep 2

cd "$WS"

if [[ "$DO_BUILD" == "1" ]] || [[ ! -f install/setup.bash ]]; then
  echo ">>> [2/5] 编译 robot_description + simulation_worlds..."
  colcon build --packages-select robot_description simulation_worlds
else
  echo ">>> [2/5] 跳过编译（加 --build 强制重编）"
fi

source install/setup.bash

PKG_WORLDS="$(ros2 pkg prefix simulation_worlds)/share/simulation_worlds/worlds"
if [[ -z "$WORLD_FILE" && -n "${INSPECTION_GAZEBO_WORLD:-}" ]]; then
  WORLD_FILE="${INSPECTION_GAZEBO_WORLD}"
fi
if [[ -z "$WORLD_FILE" ]]; then
  WORLD_FILE="${PKG_WORLDS}/lidar_test.world"
elif [[ "$WORLD_FILE" != /* ]]; then
  if [[ -f "${PKG_WORLDS}/${WORLD_FILE}" ]]; then
    WORLD_FILE="${PKG_WORLDS}/${WORLD_FILE}"
  elif [[ -f "$WORLD_FILE" ]]; then
    WORLD_FILE="$(realpath "$WORLD_FILE")"
  else
    echo "错误: 找不到 world: $WORLD_FILE"
    exit 1
  fi
fi

SPAWN_ARGS=""
if [[ "$(basename "$WORLD_FILE")" == "test_room.world" ]]; then
  SPAWN_X="${INSPECTION_SPAWN_X:-0.9}"
  SPAWN_Y="${INSPECTION_SPAWN_Y:-3.0}"
  SPAWN_YAW="${INSPECTION_SPAWN_YAW:-0.0}"
  SPAWN_ARGS=" spawn_x:=${SPAWN_X} spawn_y:=${SPAWN_Y} spawn_yaw:=${SPAWN_YAW}"
fi

ROS_ENV="source /opt/ros/humble/setup.bash && source ${WS}/install/setup.bash"
LAUNCH_CMD="${ROS_ENV} && ros2 launch simulation_worlds gazebo_robot_v0.launch.py world:=${WORLD_FILE}${SPAWN_ARGS}"
RQT_CMD="${ROS_ENV} && ros2 run rqt_image_view rqt_image_view /camera/image_raw"
TELEOP_CMD="${ROS_ENV} && ros2 run simulation_worlds keyboard_teleop.py"

RQT_PID=""
LAUNCH_PID=""
LOG_TAIL_PID=""

cleanup() {
  echo ""
  echo ">>> 关闭联调进程..."
  pkill -f keyboard_teleop 2>/dev/null || true
  [[ -n "$RQT_PID" ]] && kill "$RQT_PID" 2>/dev/null || true
  pkill -f 'rqt_image_view' 2>/dev/null || true
  [[ -n "$LOG_TAIL_PID" ]] && kill "$LOG_TAIL_PID" 2>/dev/null || true
  [[ -n "$LAUNCH_PID" ]] && kill "$LAUNCH_PID" 2>/dev/null || true
  pkill -9 -f gzserver 2>/dev/null || true
  pkill -9 -f gzclient 2>/dev/null || true
  pkill -f cmd_vel_timeout 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait_for_topics() {
  local ready=0
  for _ in $(seq 1 35); do
    if ros2 topic list 2>/dev/null | grep -qx '/cmd_vel' \
       && { [[ "$NO_CAMERA" == "1" ]] || ros2 topic list 2>/dev/null | grep -qx '/camera/image_raw'; }; then
      ready=1
      break
    fi
    printf '.'
    sleep 1
  done
  echo ""
  [[ "$ready" == "1" ]]
}

start_rqt_window() {
  echo ">>> 启动 rqt_image_view（独立相机窗口，DISPLAY=${DISPLAY}）..."
  # shellcheck disable=SC2091
  bash -lc "$RQT_CMD" >/tmp/rqt_camera.log 2>&1 &
  RQT_PID=$!
  sleep 2
  if ! kill -0 "$RQT_PID" 2>/dev/null; then
    echo "FAIL: rqt_image_view 启动失败，查 /tmp/rqt_camera.log"
    tail -15 /tmp/rqt_camera.log 2>/dev/null || true
    return 1
  fi
  echo "    相机窗口应已弹出（任务栏找 Image View / rqt）"
}

run_teleop() {
  echo ">>> 键盘遥控（W/A/S/D，空格急停）— 保持本终端焦点"
  echo "    仿真日志: $LOG"
  echo ""
  if ! python3 -c "import evdev" 2>/dev/null; then
    echo "提示: 未安装 evdev → TTY 点按模式。安装: bash scripts/setup_keyboard_evdev.sh"
    exec bash -lc "$TELEOP_CMD"
  fi
  if groups | grep -qw input; then
    exec bash -lc "$TELEOP_CMD"
  fi
  echo "提示: 用 sg input 读取组合键；永久: sudo usermod -aG input \$USER 后重登"
  exec sg input -c "$TELEOP_CMD"
}

echo ">>> [3/5] 启动 Gazebo 后台（world=$(basename "$WORLD_FILE")）..."
: >"$LOG"
bash -lc "$LAUNCH_CMD" >>"$LOG" 2>&1 &
LAUNCH_PID=$!

if [[ "$LOG_WINDOW" == "1" ]] && command -v gnome-terminal >/dev/null 2>&1; then
  gnome-terminal --title="Gazebo Log" -- bash -lc "tail -f ${LOG}" &
  LOG_TAIL_PID=$!
  echo "    已打开日志窗口: gnome-terminal tail -f ${LOG}"
fi

echo ">>> [4/5] 等待 /cmd_vel$([[ "$NO_CAMERA" != "1" ]] && echo ' + /camera/image_raw')..."
if ! wait_for_topics; then
  echo "WARN: 话题未在 35s 内就绪，查日志: $LOG"
  grep -iE 'died|Error|spawn|camera' "$LOG" | tail -12 || tail -12 "$LOG"
fi

if [[ "$NO_CAMERA" != "1" ]]; then
  start_rqt_window || echo "WARN: 相机窗口未起来，可手动: ros2 run rqt_image_view rqt_image_view /camera/image_raw"
else
  echo ">>> [4/5] 跳过相机（--no-camera）"
fi

echo ">>> [5/5] 就绪。应看到：Gazebo 3D 窗口$([[ "$NO_CAMERA" != "1" ]] && echo ' + rqt 相机窗口') + 本终端遥控"
run_teleop
