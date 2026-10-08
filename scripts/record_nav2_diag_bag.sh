#!/usr/bin/env bash
# 可解释实验：录制 cmd_vel 三层 + 规划/定位/里程计（先探测实际话题）
#
#   bash scripts/record_nav2_diag_bag.sh [输出目录]
#
# 建议在「已新启动 Nav2 并 tee 了 launch 日志」之后、跑探针之前启动。
# Ctrl+C 结束录制。

set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${1:-$ROOT/bags/nav2_diag/bag_$STAMP}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
set -u

mkdir -p "$(dirname "$OUT")"

echo "=== 探测相关话题 ==="
ros2 topic list | grep -E 'cmd_vel|plan|costmap|amcl_pose|particle_cloud|odom|ground_truth|tf|scan' || true
echo ""

# 候选：存在才录（区分 DWB / smoother / Gazebo 插件）
CANDIDATES=(
  /cmd_vel_nav
  /cmd_vel
  /cmd_vel_gazebo
  /plan
  /local_plan
  /amcl_pose
  /particle_cloud
  /odom
  /ground_truth
  /scan
  /tf
  /tf_static
)

# 一次拉列表，避免逐次 ros2 topic list 竞态漏检
mapfile -t ALL_TOPICS < <(ros2 topic list 2>/dev/null || true)
EXISTING=()
for t in "${CANDIDATES[@]}"; do
  found=0
  for a in "${ALL_TOPICS[@]}"; do
    if [[ "$a" == "$t" ]]; then
      found=1
      break
    fi
  done
  if (( found )); then
    EXISTING+=("$t")
  else
    echo "[skip] missing: $t"
  fi
done

if ((${#EXISTING[@]} == 0)); then
  echo "错误: 无可用话题。请先启动 nav2_test_room.launch.py"
  exit 2
fi

echo ""
echo "录制到: $OUT"
echo "话题 (${#EXISTING[@]}): ${EXISTING[*]}"
echo "Ctrl+C 结束"
echo ""

# shellcheck disable=SC2086
exec ros2 bag record -o "$OUT" "${EXISTING[@]}"
