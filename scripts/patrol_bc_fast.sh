#!/usr/bin/env bash
# B→C 快速诊断（Phase1 A/B）：
#   direct（默认）：spawn@B → hard pose(B) → NavigateToPose(C)
#   split：         spawn@B → hard pose(B) → B_egress → C_approach → C
#
# 前置：终端 1 已 launch nav2_test_room
#   bash scripts/patrol_bc_fast.sh 10
#   bash scripts/patrol_bc_fast.sh 10 split
#   bash scripts/patrol_bc_fast.sh 5 direct /tmp/bc_out
#
# 注意：改 amcl yaml 后必须重启 launch；改 patrol 后需 colcon build --packages-select inspection_mission

set -o pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

N=5
MODE=direct
OUT=""
for arg in "$@"; do
  case "$arg" in
    direct|split) MODE="$arg" ;;
    '' ) ;;
    *)
      if [[ "$arg" =~ ^[0-9]+$ ]]; then
        N="$arg"
      else
        OUT="$arg"
      fi
      ;;
  esac
done
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${OUT:-$ROOT/bags/patrol_bc_fast_${MODE}_$STAMP}"
SPAWN_RETRIES=3

# 与 patrol_mission_node Phase1 对齐（world）
BX=8.45; BY=1.75; BYAW=-1.57079632679
# B_egress：门外走廊（勿用柜前 1.90+朝北——会原地拧 180° 蹭墙）
BEX=8.45; BEY=2.55; BEYAW=3.14159265359
CAX=5.25; CAY=3.00; CAYAW=1.57079632679
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

run_gate() {
  # name wx wy yaw timeout — 独立 seg 日志再并入 run LOG
  local name="$1" wx="$2" wy="$3" yaw="$4" timeout="${5:-180}"
  local seg_log="$OUT/.seg_${name}.log"
  : > "$seg_log"
  echo "[seg] $name → ($wx,$wy,yaw=$yaw)" | tee -a "$LOG"
  set +e
  python3 "$ROOT/scripts/navigate_to_pose_gate.py" \
    --world-x "$wx" --world-y "$wy" --yaw "$yaw" --timeout "$timeout" \
    >"$seg_log" 2>&1
  local ec=$?
  set -u
  cat "$seg_log" >> "$LOG"
  local rec=0
  if grep -q 'recoveries during nav:' "$seg_log"; then
    rec=$(grep 'recoveries during nav:' "$seg_log" | tail -1 | grep -oE '[0-9]+$' || echo 0)
  fi
  local final=FAIL
  if grep -q 'FINAL: PASS' "$seg_log"; then
    final=PASS
  fi
  echo "SEG:$name:$final:rec=${rec}:ec=$ec"
  return "$ec"
}

echo "输出: $OUT  (mode=$MODE ×$N)"
if ! ros2 action list 2>/dev/null | grep -q '/navigate_to_pose'; then
  echo "错误: 请先 ros2 launch navigation_config nav2_test_room.launch.py"
  exit 2
fi
if ! service_ok /spawn_entity; then
  echo "错误: /spawn_entity 不可用"
  exit 2
fi

{
  echo "# B→C 快速诊断 ×${N} (mode=${MODE})"
  echo ""
  echo "- 开始: $(date -Iseconds)"
  echo "- HEAD: $(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
  if [[ "$MODE" == split ]]; then
    echo "- 每轮: spawn@B + hard pose(B) + B_egress → C_approach → C"
  else
    echo "- 每轮: spawn@B + hard pose(B) + NavigateToPose(C)"
  fi
  echo "- soft reanchor: 不使用"
  echo "- AMCL/Smac/Progress/Recovery: 未改（Phase1 只改任务路径）"
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
  echo "========== B→C RUN $i / $N (mode=$MODE) =========="

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

  SUMMARY=""
  EC=0
  if [[ "$MODE" == split ]]; then
    echo "[run$i] split: B_egress → C_approach → C ..." | tee -a "$LOG"
    # 任一段 FAIL 立即停本轮，避免带着脏定位硬跑后续段
    set +e
    out1=$(run_gate B_egress "$BEX" "$BEY" "$BEYAW" 180)
    e1=$?
    set -u
    SUMMARY="$out1"
    if (( e1 != 0 )); then
      echo "[run$i] STOP after B_egress FAIL (不继续 C_approach/C)" | tee -a "$LOG"
      RES=FAIL; EC=1; ((fail_n++)) || true
    else
      set +e
      out2=$(run_gate C_approach "$CAX" "$CAY" "$CAYAW" 180)
      e2=$?
      set -u
      SUMMARY="${SUMMARY}; ${out2}"
      if (( e2 != 0 )); then
        echo "[run$i] STOP after C_approach FAIL (不继续 C)" | tee -a "$LOG"
        RES=FAIL; EC=1; ((fail_n++)) || true
      else
        set +e
        out3=$(run_gate C "$CX" "$CY" "$CYAW" 180)
        e3=$?
        set -u
        SUMMARY="${SUMMARY}; ${out3}"
        if (( e3 == 0 )); then
          RES=PASS; EC=0; ((pass_n++)) || true
        else
          RES=FAIL; EC=1; ((fail_n++)) || true
        fi
      fi
    fi
    echo "$SUMMARY" | tee -a "$LOG"
  else
    echo "[run$i] NavigateToPose C ..." | tee -a "$LOG"
    set +e
    python3 "$ROOT/scripts/navigate_to_pose_gate.py" \
      --world-x "$CX" --world-y "$CY" --yaw "$CYAW" --timeout 180 >>"$LOG" 2>&1
    EC=$?
    set -u
    SUMMARY=$(grep -E 'FINAL:|recoveries during nav|Position Error|Yaw Error' "$LOG" \
      | tr '\n' '; ' | sed 's/; $//' | cut -c1-200)
    if grep -q 'FINAL: PASS' "$LOG"; then
      RES=PASS
      ((pass_n++)) || true
    else
      RES=FAIL
      ((fail_n++)) || true
    fi
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
  echo "- mode: **${MODE}**"
  echo "- PASS: **${pass_n}** / ${N}"
  echo "- FAIL: **${fail_n}**"
  echo "- 明细: \`runNN.log\`"
  echo ""
  echo "## 读表"
  echo "- direct = 基线（单 goal B→C）"
  echo "- split = Phase1（B_egress → C_approach → C）"
  echo "- 对比 recoveries：看各 SEG 行 / \`[diag] recoveries during nav\`"
} | tee -a "$RESULTS_MD"

echo "完成: $RESULTS_MD"
