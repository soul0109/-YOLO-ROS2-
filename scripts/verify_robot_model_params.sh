#!/usr/bin/env bash
# 确认 install / live 的 robot_description 与当前基线一致。
#
# 当前基线（33a47d7）：fixed caster_link 球体 + mu=0.1，不是真脚轮。
# 用法：
#   bash scripts/verify_robot_model_params.sh           # 期望 mu=0.1、fixed 球、有 diff_drive
#   bash scripts/verify_robot_model_params.sh 0.1
#   bash scripts/verify_robot_model_params.sh 0.1 false

set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EXPECT_MU="${1:-0.1}"
EXPECT_DIFF="${2:-true}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
set -u

XACRO="$(ros2 pkg prefix robot_description)/share/robot_description/urdf/robot_v0.urdf.xacro"
if [[ ! -f "$XACRO" ]]; then
  echo "错误: 找不到 install 侧 xacro: $XACRO"
  echo "先: cd ~/inspection-robot/ros2_ws && colcon build --packages-select robot_description --symlink-install"
  exit 2
fi

URDF="$(mktemp)"
trap 'rm -f "$URDF"' EXIT
# 基线 xacro 无 caster_mu / enable_diff_drive 参数；直接展开
xacro "$XACRO" >"$URDF"

echo "install xacro: $XACRO"

if grep -q 'caster_swivel_joint\|caster_wheel_joint\|caster_wheel_link' "$URDF"; then
  echo "FAIL: 发现真脚轮残留（swivel/wheel），与 33a47d7 fixed 球基线不符"
  exit 1
fi
if ! grep -q 'name="caster_joint"' "$URDF"; then
  echo "FAIL: 缺少 fixed caster_joint"
  exit 1
fi
if ! grep -q 'type="fixed"' "$URDF"; then
  echo "FAIL: URDF 中未见 fixed 关节标记（异常）"
  exit 1
fi
# caster_joint 应为 fixed
JOINT_LINE="$(grep -n 'name="caster_joint"' "$URDF" | head -1 | cut -d: -f1)"
JOINT_BLOCK="$(sed -n "${JOINT_LINE},$((JOINT_LINE + 6))p" "$URDF")"
echo "caster_joint block:"
echo "$JOINT_BLOCK"
if ! echo "$JOINT_BLOCK" | grep -q 'type="fixed"'; then
  echo "FAIL: caster_joint 不是 fixed"
  exit 1
fi

MU_LINE="$(awk '/reference="caster_link"/,/<\/gazebo>/' "$URDF" | grep -m1 '<mu1>' || true)"
echo "caster_link mu1 line: ${MU_LINE:-MISSING}"
if [[ "$MU_LINE" != *"<mu1>${EXPECT_MU}</mu1>"* ]]; then
  echo "FAIL: 期望 caster_link mu1=${EXPECT_MU}，实际: ${MU_LINE:-none}"
  exit 1
fi

# 接触不对称探针（诊断用，不判失败）
CASTER_HAS_MIN="$(awk '/reference="caster_link"/,/<\/gazebo>/' "$URDF" | grep -c '<minDepth>' || true)"
WHEEL_HAS_MIN="$(awk '/reference="left_wheel_link"/,/<\/gazebo>/' "$URDF" | grep -c '<minDepth>' || true)"
echo "contact asymmetry probe: caster_minDepth_tags=${CASTER_HAS_MIN} left_wheel_minDepth_tags=${WHEEL_HAS_MIN}"

if grep -q 'libgazebo_ros_diff_drive.so' "$URDF"; then
  HAS_DIFF=true
else
  HAS_DIFF=false
fi
echo "diff_drive plugin present: $HAS_DIFF (expect $EXPECT_DIFF)"
if [[ "$HAS_DIFF" != "$EXPECT_DIFF" ]]; then
  echo "FAIL: enable_diff_drive 与期望不一致"
  exit 1
fi

if timeout 3 ros2 param get /robot_state_publisher robot_description >/tmp/rsp_urdf.txt 2>/dev/null; then
  if grep -q 'caster_swivel_joint\|caster_wheel_joint' /tmp/rsp_urdf.txt; then
    echo "WARN: live 仍有真脚轮 → 必须重启 launch"
    exit 3
  fi
  if ! grep -q 'name="caster_joint"' /tmp/rsp_urdf.txt; then
    echo "WARN: live 无 caster_joint → 必须重启 launch"
    exit 3
  fi
  LIVE_MU="$(awk '/reference="caster_link"/,/<\/gazebo>/' /tmp/rsp_urdf.txt | grep -m1 '<mu1>' || true)"
  echo "live caster_link mu1: ${LIVE_MU:-MISSING}"
  if [[ -n "$LIVE_MU" && "$LIVE_MU" != *"<mu1>${EXPECT_MU}</mu1>"* ]]; then
    echo "WARN: live 摩擦与期望不符 → 重启 launch"
    exit 3
  fi
else
  echo "info: 仿真未运行（仅校验了 install 展开）"
fi

echo "OK: baseline fixed caster_link mu=${EXPECT_MU}, diff_drive=${EXPECT_DIFF}"
exit 0
