#!/usr/bin/env bash
# 交付1：初始化偏移取证 + 可确认清控。
#
# 前置：另开终端保持
#   ros2 launch navigation_config nav2_test_room.launch.py
#
# 本脚本：
#   bash scripts/run_init_forensics.sh
#   bash scripts/run_init_forensics.sh /tmp/my_forensics
#
# 协议：
#   - 先 clear（cancel 确认 + cmd_vel 静默），再 delete/spawn
#   - initialpose 只发规定出生位姿 (0.9, 3.0)，绝不用 GT 重设
#   - 位置不符 → 记录 INIT_FAIL 证据，不自动修正

set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${1:-$ROOT/bags/init_forensics_$STAMP}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
set -u
cd "$ROOT"
mkdir -p "$OUT"

echo "输出目录: $OUT"
echo "HEAD: $(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
if [[ -n "$(git status --porcelain 2>/dev/null || true)" ]]; then
  echo "工作区: dirty（取证记录会标注）"
fi

# 轻量预检：需要 spawn / ground_truth 链路
if ! timeout 5 ros2 service type /spawn_entity >/dev/null 2>&1; then
  echo "错误: /spawn_entity 不可用。请先启动："
  echo "  ros2 launch navigation_config nav2_test_room.launch.py"
  exit 2
fi

set +e
python3 "$ROOT/scripts/init_spawn_forensics.py" --output-dir "$OUT"
EC=$?
set -e

echo ""
echo "exit=$EC"
echo "请审查:"
echo "  $OUT/results.md"
echo "  $OUT/result.json"
echo "  $OUT/clear.json"
exit "$EC"
