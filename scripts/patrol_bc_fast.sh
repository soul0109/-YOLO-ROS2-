#!/usr/bin/env bash
# B→C Phase1 探针（只改任务路径 + 本脚本；不改 Nav2/AMCL yaml）
#
#   direct：spawn@B → initialpose → AMCL_LOCKED → NavigateToPose(C)
#   split ：同上 → B_egress → B_corridor_turn → C_approach → C  (BC_SPLIT_V3)
#
#   bash scripts/patrol_bc_fast.sh 10 direct
#   bash scripts/patrol_bc_fast.sh 10 split
#
# 改 patrol 后：
#   cd ~/inspection-robot/ros2_ws && colcon build --packages-select inspection_mission --symlink-install
#   source install/setup.bash

set -o pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WAYPOINT_VERSION=BC_SPLIT_V3

N=5
MODE=direct
OUT=""
for arg in "$@"; do
  case "$arg" in
    direct|split) MODE="$arg" ;;
    '') ;;
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

# world 坐标（与 patrol_mission_node BC_SPLIT_V3 一致；gate 脚本内部做一次 world→map）
BX=8.45; BY=1.75; BYAW=-1.57079632679
BEX=8.45; BEY=1.55; BEYAW=1.57079632679
BCTX=8.45; BCTY=3.00; BCTYAW=3.14159265359
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
MAP_YAML="$ROOT/ros2_ws/src/navigation_config/maps/test_room.yaml"

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

# AMCL 静止时因 update_min_d/a 往往不再发新 stamp → 不能「要求 10 个新 stamp」。
# 正确：等到位姿锁在 B，再连续 ~1s 轮询「最新消息仍合格」（同 stamp 也算）。
check_amcl_locked_at_b() {
  MAP_YAML="$MAP_YAML" python3 - <<'PY'
import math, sys, time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

BX_W, BY_W = 8.45, 1.75
MAX_DIST = 0.25
MAX_YAW_VAR = 0.2
HOLD_SEC = 1.0          # 锁住后保持多久
POLL_DT = 0.1
NEED_POLLS = int(HOLD_SEC / POLL_DT)  # 10
WAIT_SEC = 25.0

map_yaml = Path(__import__('os').environ['MAP_YAML'])
origin = yaml.safe_load(map_yaml.read_text(encoding='utf-8')).get('origin', [0, 0, 0])
ox, oy = float(origin[0]), float(origin[1])

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
rclpy.init()
node = rclpy.create_node(
    'bc_amcl_lock_check',
    parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
)
box = {'msg': None}

def cb(m):
    box['msg'] = m

node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', cb, qos)

t_clock = time.time() + 10.0
while time.time() < t_clock and node.get_clock().now().nanoseconds == 0:
    rclpy.spin_once(node, timeout_sec=0.05)

# 丢弃门禁启动瞬间的旧 latched 消息：等第一个「新 stamp」或超时后再评
t0 = time.time()
first = box['msg']
first_stamp = None
if first is not None:
    first_stamp = (first.header.stamp.sec, first.header.stamp.nanosec)
# 最多等 3s 出现比 latched 更新的 stamp（initialpose 后应有一次）
while time.time() - t0 < 3.0:
    rclpy.spin_once(node, timeout_sec=0.05)
    m = box['msg']
    if m is None:
        continue
    st = (m.header.stamp.sec, m.header.stamp.nanosec)
    if first_stamp is None or st != first_stamp:
        break

good_polls = 0
t0 = time.time()
last_print = None
unique_stamps = set()
while time.time() - t0 < WAIT_SEC:
    rclpy.spin_once(node, timeout_sec=POLL_DT)
    msg = box['msg']
    if msg is None:
        good_polls = 0
        continue
    st = (msg.header.stamp.sec, msg.header.stamp.nanosec)
    unique_stamps.add(st)
    p = msg.pose.pose.position
    cov = msg.pose.covariance
    wx, wy = p.x + ox, p.y + oy
    dist = math.hypot(wx - BX_W, wy - BY_W)
    yaw_var = cov[35]
    finite = all(math.isfinite(v) for v in (wx, wy, yaw_var, cov[0], cov[7]))
    ok = finite and dist < MAX_DIST and yaw_var < MAX_YAW_VAR
    good_polls = good_polls + 1 if ok else 0
    line = (
        f'[AMCL_GATE] target=B amcl_world=({wx:.3f},{wy:.3f}) '
        f'distance={dist:.3f} var_yaw={yaw_var:.3f} '
        f'hold={good_polls}/{NEED_POLLS} ok={ok} stamps={len(unique_stamps)}'
    )
    if line != last_print:
        print(line)
        last_print = line
    if good_polls >= NEED_POLLS:
        print('[AMCL_GATE] AMCL_LOCKED')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

node.destroy_node()
rclpy.shutdown()
print(
    f'[AMCL_GATE] INFRA_FAIL — AMCL not locked at B '
    f'(unique_stamps={len(unique_stamps)}; '
    f'stationary AMCL may only publish once after initialpose)'
)
sys.exit(3)
PY
}

run_gate() {
  local name="$1" wx="$2" wy="$3" yaw="$4" timeout="${5:-180}"
  local seg_log="$OUT/.seg_${name}.log"
  : > "$seg_log"
  echo "[WAYPOINT] name=$name world=($wx,$wy) yaw=$yaw (gate 内一次 world→map)" | tee -a "$LOG"
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

run_split_chain() {
  # 任一段非 SUCCEEDED（gate FAIL）立即停
  local names=(B_egress B_corridor_turn C_approach C)
  local xs=("$BEX" "$BCTX" "$CAX" "$CX")
  local ys=("$BEY" "$BCTY" "$CAY" "$CY")
  local yaws=("$BEYAW" "$BCTYAW" "$CAYAW" "$CYAW")
  local summary="" i out ec
  for i in 0 1 2 3; do
    set +e
    out=$(run_gate "${names[$i]}" "${xs[$i]}" "${ys[$i]}" "${yaws[$i]}" 180)
    ec=$?
    set -u
    if [[ -n "$summary" ]]; then
      summary="${summary}; ${out}"
    else
      summary="$out"
    fi
    if (( ec != 0 )); then
      echo "[run] STOP after ${names[$i]} FAIL" | tee -a "$LOG"
      echo "$summary"
      return 1
    fi
  done
  echo "$summary"
  return 0
}

echo "输出: $OUT  (mode=$MODE ×$N)  VERSION=$WAYPOINT_VERSION"
echo "pkg prefix: $(ros2 pkg prefix inspection_mission 2>/dev/null || echo n/a)"
if ! ros2 action list 2>/dev/null | grep -q '/navigate_to_pose'; then
  echo "错误: 请先 ros2 launch navigation_config nav2_test_room.launch.py"
  exit 2
fi
if ! service_ok /spawn_entity; then
  echo "错误: /spawn_entity 不可用"
  exit 2
fi

{
  echo "# B→C 快速诊断 ×${N} (mode=${MODE}, ${WAYPOINT_VERSION})"
  echo ""
  echo "- 开始: $(date -Iseconds)"
  echo "- HEAD: $(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
  echo "- VERSION: ${WAYPOINT_VERSION}"
  if [[ "$MODE" == split ]]; then
    echo "- 每轮: spawn@B + initialpose + AMCL hold~1s@B → B_egress→B_corridor_turn→C_approach→C"
    echo "- B_egress=(8.45,1.55,+π/2) B_corridor_turn=(8.45,3.00,π) C_approach=(5.25,3.00,+π/2)"
  else
    echo "- 每轮: spawn@B + initialpose + AMCL hold~1s@B → NavigateToPose(C)"
  fi
  echo "- Nav2/AMCL/Progress/Recovery: 未改"
  echo ""
  echo "| # | 结果 | exit | 摘要 |"
  echo "|---|---|---|---|"
} > "$RESULTS_MD"
echo "run,result,exit,summary" > "$RESULTS_CSV"

pass_n=0
fail_n=0
infra_n=0

for i in $(seq 1 "$N"); do
  printf -v tag "%02d" "$i"
  LOG="$OUT/run${tag}.log"
  : > "$LOG"
  echo ""
  echo "========== B→C RUN $i / $N (mode=$MODE $WAYPOINT_VERSION) =========="

  if ! reset_to_b >>"$LOG" 2>&1; then
    echo "| $i | INFRA | 3 | spawn@B failed |" >> "$RESULTS_MD"
    echo "$i,INFRA,3,spawn_failed" >> "$RESULTS_CSV"
    ((infra_n++)) || true
    continue
  fi

  echo "[run$i] hard initialpose at B" | tee -a "$LOG"
  if ! python3 "$ROOT/scripts/publish_amcl_initial_pose.py" \
      --world-x "$BX" --world-y "$BY" --yaw "$BYAW" >>"$LOG" 2>&1; then
    echo "| $i | INFRA | 2 | initialpose B failed |" >> "$RESULTS_MD"
    echo "$i,INFRA,2,initialpose_failed" >> "$RESULTS_CSV"
    ((infra_n++)) || true
    continue
  fi

  echo "[run$i] AMCL_GATE (hold ~1s locked at B; stamp 可不更新)" | tee -a "$LOG"
  if ! check_amcl_locked_at_b >>"$LOG" 2>&1; then
    echo "| $i | INFRA | 4 | amcl not locked at B |" >> "$RESULTS_MD"
    echo "$i,INFRA,4,amcl_not_at_B" >> "$RESULTS_CSV"
    ((infra_n++)) || true
    echo "[run$i] INFRA — 不发 NavigateToPose（勿归因航点）"
    continue
  fi

  SUMMARY=""
  EC=0
  if [[ "$MODE" == split ]]; then
    echo "[run$i] split V3 chain ..." | tee -a "$LOG"
    set +e
    SUMMARY=$(run_split_chain)
    EC=$?
    set -u
    echo "$SUMMARY" | tee -a "$LOG"
    if (( EC == 0 )); then
      RES=PASS
      ((pass_n++)) || true
    else
      RES=FAIL
      ((fail_n++)) || true
    fi
  else
    echo "[run$i] NavigateToPose C ..." | tee -a "$LOG"
    set +e
    out=$(run_gate C "$CX" "$CY" "$CYAW" 180)
    EC=$?
    set -u
    SUMMARY="$out"
    echo "$SUMMARY" | tee -a "$LOG"
    if (( EC == 0 )); then
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
  echo "- VERSION: **${WAYPOINT_VERSION}**"
  echo "- mode: **${MODE}**"
  echo "- PASS: **${pass_n}**"
  echo "- FAIL: **${fail_n}**"
  echo "- INFRA: **${infra_n}**"
  echo "- 有效导航轮: PASS+FAIL = $((pass_n + fail_n)) / ${N}"
  echo "- 明细: \`runNN.log\`"
  echo ""
  echo "## 读表"
  echo "- INFRA = spawn/initialpose/AMCL 未锁，**不算航点失败**"
  echo "- direct = 实验 A（只验证门禁 + 单点 C）"
  echo "- split = 实验 B（V3 四段机动）"
} | tee -a "$RESULTS_MD"

echo "完成: $RESULTS_MD"
