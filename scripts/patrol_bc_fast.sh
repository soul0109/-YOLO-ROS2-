#!/usr/bin/env bash
# B→C 快速诊断：不跑完整 A→B→C×10。
# 每轮：spawn 到 B → 硬 initialpose(B) → NavigateToPose(C) → 记表。
#
# 前置：终端 1 已 launch nav2_test_room
#   bash scripts/patrol_bc_fast.sh        # 默认 5 次
#   bash scripts/patrol_bc_fast.sh 5
#   AMCL_RANDOM_SEED=42 bash scripts/patrol_bc_fast.sh 5
#     （若已在 amcl_test_room.yaml 把 random_seed 改成 42 并重启 launch）
#
# 注意：改 amcl yaml 后必须重启 launch 才生效。

set -o pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
N="${1:-5}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${2:-$ROOT/bags/patrol_bc_fast_$STAMP}"
SPAWN_RETRIES=3

# B / C world（与 patrol_mission_node 一致）
BX=8.45; BY=1.75; BYAW=-1.57079632679
CX=5.25; CY=4.25; CYAW=1.57079632679

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
set -u
cd "$ROOT"
mkdir -p "$OUT"

RESULTS_MD="$OUT/results.md"
RESULTS_CSV="$OUT/results.csv"

service_ok() { timeout 5 ros2 service type "$1" >/dev/null 2>&1; }

reset_to_b() {
  local attempt=1
  while (( attempt <= SPAWN_RETRIES )); do
    echo "[reset] spawn at B attempt $attempt"
    timeout 20 ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity \
      "{name: 'robot_v0'}" >/dev/null 2>&1 || true
    sleep 2
    if timeout 45 ros2 run gazebo_ros spawn_entity.py \
      -entity robot_v0 -topic robot_description \
      -x "$BX" -y "$BY" -z 0.05 -Y "$BYAW"
    then
      sleep 2
      return 0
    fi
    ((attempt++)) || true
    sleep 2
  done
  return 1
}

check_amcl_finite() {
  python3 - <<'PY'
import math, sys
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
rclpy.init()
node = rclpy.create_node('bc_amcl_check')
box = {'msg': None}

def cb(m):
    box['msg'] = m

node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', cb, qos)
end = node.get_clock().now().nanoseconds + int(8e9)
# wall timeout
import time
t0 = time.time()
while time.time() - t0 < 8.0 and box['msg'] is None:
    rclpy.spin_once(node, timeout_sec=0.1)
msg = box['msg']
node.destroy_node()
rclpy.shutdown()
if msg is None:
    print('AMCL_CHECK: no /amcl_pose')
    sys.exit(2)
p, q = msg.pose.pose.position, msg.pose.pose.orientation
cov = msg.pose.covariance
vals = [p.x, p.y, q.x, q.y, q.z, q.w, cov[0], cov[7], cov[35]]
ok = all(math.isfinite(v) for v in vals)
qn = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
print(f'AMCL_CHECK: ok={ok} xy=({p.x:.3f},{p.y:.3f}) qn={qn:.4f} '
      f'var_xy=({cov[0]:.3f},{cov[7]:.3f}) var_yaw={cov[35]:.3f}')
sys.exit(0 if ok else 3)
PY
}

echo "输出: $OUT  (B→C ×$N)"
if ! ros2 action list 2>/dev/null | grep -q '/navigate_to_pose'; then
  echo "错误: 请先 ros2 launch navigation_config nav2_test_room.launch.py"
  exit 2
fi
if ! service_ok /spawn_entity; then
  echo "错误: /spawn_entity 不可用"
  exit 2
fi

{
  echo "# B→C 快速诊断 ×${N}"
  echo ""
  echo "- 开始: $(date -Iseconds)"
  echo "- HEAD: $(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
  echo "- 每轮: spawn@B + hard initialpose(B) + NavigateToPose(C)"
  echo "- soft reanchor: 不使用（本脚本）"
  echo ""
  echo "| # | 结果 | exit | 摘要 |"
  echo "|---|---|---|---|"
} > "$RESULTS_MD"
echo "run,result,exit,summary" > "$RESULTS_CSV"

pass_n=0
fail_n=0

for i in $(seq 1 "$N"); do
  printf -v tag "%02d" "$i"
  LOG="$OUT/run${tag}.log"
  : > "$LOG"
  echo ""
  echo "========== B→C RUN $i / $N =========="

  if ! reset_to_b >>"$LOG" 2>&1; then
    echo "| $i | INFRA | 3 | spawn@B failed |" >> "$RESULTS_MD"
    echo "$i,INFRA,3,spawn_failed" >> "$RESULTS_CSV"
    continue
  fi

  echo "[run$i] hard initialpose at B" | tee -a "$LOG"
  if ! python3 "$ROOT/scripts/publish_amcl_initial_pose.py" \
      --world-x "$BX" --world-y "$BY" --yaw "$BYAW" >>"$LOG" 2>&1; then
    echo "| $i | INFRA | 2 | initialpose B failed |" >> "$RESULTS_MD"
    echo "$i,INFRA,2,initialpose_failed" >> "$RESULTS_CSV"
    continue
  fi
  sleep 2

  echo "[run$i] check /amcl_pose finite" | tee -a "$LOG"
  if ! check_amcl_finite >>"$LOG" 2>&1; then
    echo "| $i | FAIL | 4 | amcl non-finite or missing |" >> "$RESULTS_MD"
    echo "$i,FAIL,4,amcl_bad" >> "$RESULTS_CSV"
    ((fail_n++)) || true
    echo "[run$i] STOP — AMCL 非法，优先查 TF/NaN，勿继续盲跑"
    break
  fi

  echo "[run$i] NavigateToPose C ..." | tee -a "$LOG"
  set +e
  python3 "$ROOT/scripts/navigate_to_pose_gate.py" \
    --world-x "$CX" --world-y "$CY" --yaw "$CYAW" --timeout 180 >>"$LOG" 2>&1
  EC=$?
  set -u
  SUMMARY=$(grep -E 'FINAL:|Position Error|Yaw Error|Stopped|FAIL|PASS' "$LOG" | tr '\n' '; ' | sed 's/; $//' | cut -c1-160)
  if grep -q 'FINAL: PASS' "$LOG"; then
    RES=PASS
    ((pass_n++)) || true
  else
    RES=FAIL
    ((fail_n++)) || true
  fi
  echo "| $i | $RES | $EC | ${SUMMARY} |" >> "$RESULTS_MD"
  echo "$i,$RES,$EC,$(echo "$SUMMARY" | tr ',' ';')" >> "$RESULTS_CSV"
  echo "[run$i] done → $RES"
  sleep 1
done

{
  echo ""
  echo "## 统计"
  echo "- 结束: $(date -Iseconds)"
  echo "- PASS: **${pass_n}** / ${N}"
  echo "- FAIL: **${fail_n}**"
  echo "- 明细: \`runNN.log\`"
} | tee -a "$RESULTS_MD"

echo "完成: $RESULTS_MD"
