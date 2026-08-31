#!/usr/bin/env bash
# 4.2a：回放 inspection bag（四话题 + TF）
#
# 用法：
#   bash ~/inspection-robot/scripts/play_inspection_bag.sh bags/inspection/run_YYYYMMDD_HHMMSS
#   bash ~/inspection-robot/scripts/play_inspection_bag.sh <bag_dir> --rqt
#   bash ~/inspection-robot/scripts/play_inspection_bag.sh <bag_dir> --check
#   bash ~/inspection-robot/scripts/play_inspection_bag.sh <bag_dir> --rate 0.5 --loop
#
# --check：回放前校验 bag 含四话题；回放时抽样验证 /scan 与 /camera/image_raw 有数据
# --rqt ：弹出 rqt_image_view 看相机（需 DISPLAY）

set -eo pipefail

ROOT="${HOME}/inspection-robot"
WS="${HOME}/inspection-robot/ros2_ws"
BAG_DIR=""
DO_RQT=0
DO_CHECK=0
RATE="1.0"
LOOP=0

REQUIRED_TOPICS=(
  /cmd_vel
  /odom
  /scan
  /camera/image_raw
)

usage() {
  sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'
  exit "${1:-0}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --rqt) DO_RQT=1; shift ;;
    --check) DO_CHECK=1; shift ;;
    --loop) LOOP=1; shift ;;
    --rate)
      [[ $# -ge 2 ]] || { echo "错误: --rate 需要数值"; exit 1; }
      RATE="$2"
      shift 2
      ;;
    -h|--help) usage 0 ;;
    -*)
      echo "未知参数: $1"
      usage 1
      ;;
    *)
      if [[ -z "$BAG_DIR" ]]; then
        BAG_DIR="$1"
        shift
      else
        echo "多余参数: $1"
        usage 1
      fi
      ;;
  esac
done

if [[ -z "$BAG_DIR" ]]; then
  echo "错误: 请指定 bag 目录"
  usage 1
fi

if [[ "$BAG_DIR" != /* ]]; then
  BAG_DIR="${ROOT}/${BAG_DIR#${ROOT}/}"
fi
BAG_DIR="$(realpath "$BAG_DIR")"

if [[ ! -d "$BAG_DIR" ]]; then
  echo "错误: bag 目录不存在: $BAG_DIR"
  exit 1
fi

export DISPLAY="${DISPLAY:-:0}"
source /opt/ros/humble/setup.bash
[[ -f "${WS}/install/setup.bash" ]] && source "${WS}/install/setup.bash"

check_bag_metadata() {
  local info missing=0
  info="$(ros2 bag info "$BAG_DIR" 2>/dev/null)" || {
    echo "FAIL: ros2 bag info 失败"
    return 1
  }
  echo "$info"
  echo ""
  echo ">>> 校验必需话题..."
  local topic
  for topic in "${REQUIRED_TOPICS[@]}"; do
    if echo "$info" | grep -q "Topic: ${topic} "; then
      echo "  OK: ${topic}"
    else
      echo "  FAIL: 缺少 ${topic}"
      missing=1
    fi
  done
  if echo "$info" | grep -q 'Topic: /tf '; then
    echo "  OK: /tf"
  else
    echo "  WARN: 无 /tf（部分离线工具仍可工作）"
  fi
  [[ "$missing" == "0" ]]
}

live_check_topics() {
  echo ">>> 回放中抽样验证 /scan 与 /camera/image_raw..."
  python3 << 'PY'
import sys
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

class Checker:
    def __init__(self):
        self.node = rclpy.create_node('bag_play_checker')
        self.scan_ok = False
        self.image_ok = False
        self.node.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.node.create_subscription(Image, '/camera/image_raw', self.on_image, 10)

    def on_scan(self, msg):
        if len(msg.ranges) > 0:
            self.scan_ok = True

    def on_image(self, msg):
        if msg.width > 0 and len(msg.data) > 0:
            self.image_ok = True

rclpy.init()
c = Checker()
for _ in range(80):
    rclpy.spin_once(c.node, timeout_sec=0.25)
    if c.scan_ok and c.image_ok:
        break
ok = c.scan_ok and c.image_ok
if c.scan_ok:
    print('  OK: /scan 有数据')
else:
    print('  FAIL: /scan 无有效数据')
if c.image_ok:
    print('  OK: /camera/image_raw 有数据')
else:
    print('  FAIL: /camera/image_raw 无有效数据')
c.node.destroy_node()
rclpy.shutdown()
sys.exit(0 if ok else 1)
PY
}

RQT_PID=""
PLAY_PID=""

cleanup() {
  [[ -n "$PLAY_PID" ]] && kill "$PLAY_PID" 2>/dev/null || true
  pkill -f 'ros2 bag play' 2>/dev/null || true
  [[ -n "$RQT_PID" ]] && kill "$RQT_PID" 2>/dev/null || true
  pkill -f 'rqt_image_view' 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo ">>> bag: ${BAG_DIR}"
if ! check_bag_metadata; then
  exit 1
fi

if [[ "$DO_RQT" == "1" ]]; then
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "WARN: 无 DISPLAY，跳过 rqt"
  else
    echo ">>> 启动 rqt_image_view..."
  ROS_ENV="source /opt/ros/humble/setup.bash"
  bash -lc "${ROS_ENV} && ros2 run rqt_image_view rqt_image_view /camera/image_raw" \
    >/tmp/play_inspection_bag.rqt.log 2>&1 &
    RQT_PID=$!
    sleep 2
  fi
fi

PLAY_ARGS=(ros2 bag play "$BAG_DIR" --rate "$RATE")
if [[ "$LOOP" == "1" ]]; then
  PLAY_ARGS+=(--loop)
fi

echo ">>> 回放: ${PLAY_ARGS[*]}"
if [[ "$DO_CHECK" == "1" ]]; then
  "${PLAY_ARGS[@]}" &
  PLAY_PID=$!
  sleep 2
  if live_check_topics; then
    echo ""
    echo "=========================================="
    echo "  回放校验通过（四话题 bag 可离线复盘）"
    echo "=========================================="
  else
    echo "FAIL: 回放抽样未收到 scan/图像"
    exit 1
  fi
  wait "$PLAY_PID" 2>/dev/null || true
else
  echo "  （Ctrl+C 停止；加 --check 可自动抽样验证）"
  exec "${PLAY_ARGS[@]}"
fi
