#!/usr/bin/env bash
# B→C Phase1 探针（只改任务路径 + 本脚本；不改 Nav2/AMCL yaml）
#
#   direct：spawn@B → clear → initialpose → AMCL_LOCKED → clear → NavigateToPose(C)
#   split ：同上 → B_clear → B_arc_east → B_arc_north → B_corridor_in
#            → B_corridor_turn_spin(+π/2) → C_approach → C  (BC_SPLIT_V3_4)
#
#   分段闸门（split 第三参，默认 arc1 —— 先验东向 90° 弧）：
#     arc1 ：只到 B_arc_east
#     arc2 ：到 B_arc_north
#     full ：完整 B→C 链
#
#   bash scripts/patrol_bc_fast.sh 1 split arc1
#   bash scripts/patrol_bc_fast.sh 1 split arc2
#   bash scripts/patrol_bc_fast.sh 1 split full
#   bash scripts/patrol_bc_fast.sh 10 direct
#
# 改 patrol 后：
#   cd ~/inspection-robot/ros2_ws && colcon build --packages-select inspection_mission --symlink-install
#   source install/setup.bash
#
# 诊断（另开终端，见 docs/当前进度.md）：
#   1) launch | tee bags/nav2_diag/nav2_launch.log
#   2) bash scripts/record_nav2_diag_bag.sh bags/nav2_diag/run_bag
#   3) 本脚本 1 split arc1；SEG/SPIN 失败即停，勿跑第二轮

set -o pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WAYPOINT_VERSION=BC_SPLIT_V3_4

N=5
MODE=direct
STAGE=arc1
OUT=""
for arg in "$@"; do
  case "$arg" in
    direct|split) MODE="$arg" ;;
    arc1|arc2|full) STAGE="$arg" ;;
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
if [[ "$MODE" == split ]]; then
  OUT="${OUT:-$ROOT/bags/patrol_bc_fast_${MODE}_${STAGE}_$STAMP}"
else
  OUT="${OUT:-$ROOT/bags/patrol_bc_fast_${MODE}_$STAMP}"
fi
SPAWN_RETRIES=3
# 段动作 SUCCEEDED（含 gate 停稳窗≈1s）后再等，再采 LOC（排除短暂重收敛误报）
POST_SEG_LOC_DWELL_SEC=2.0

# world / spin（与 patrol_mission_node BC_SPLIT_V3_4 一致）
BX=8.45; BY=1.75; BYAW=-1.57079632679
BCLX=8.45; BCLY=1.35; BCLYAW=-1.57079632679
# 路径切向短弧（探针正式候选；mission_node 待 arc2+穿门通过后再同步）
# B_arc_east=9.05：弃用 8.75（LOC 不合格）；9.05 在门洞右缘外侧，须经 B_arc_north 回收
# B_arc_north：东→北 + 收回门洞可用范围，再交给 B_corridor_in 穿门
BAEX=9.05; BAEY=1.35; BAEYAW=0.0
BANX=8.75; BANY=1.75; BANYAW=1.57079632679
BCIX=8.45; BCIY=3.00; BCIYAW=1.57079632679
SPIN_TURN=1.57079632679; EXPECT_TURN=3.14159265359
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

# 与 patrol_mission_node 对齐：每个 NavigateToPose 前清 global/local
clear_costmaps() {
  local g=/global_costmap/clear_entirely_global_costmap
  local l=/local_costmap/clear_entirely_local_costmap
  local label="${1:-}"
  echo "[clear] costmaps${label:+ ($label)}"
  if service_ok "$g"; then
    timeout 8 ros2 service call "$g" nav2_msgs/srv/ClearEntireCostmap "{}" >/dev/null 2>&1 \
      || echo "[clear] WARN: global clear failed/timeout"
  else
    echo "[clear] WARN: $g not ready"
  fi
  if service_ok "$l"; then
    timeout 8 ros2 service call "$l" nav2_msgs/srv/ClearEntireCostmap "{}" >/dev/null 2>&1 \
      || echo "[clear] WARN: local clear failed/timeout"
  else
    echo "[clear] WARN: $l not ready"
  fi
  sleep 0.3
}

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
# 门禁：均值距 B <0.25 且 cov[0]<0.25 cov[7]<0.25 cov[35]<0.20，保持 ~1s。
# MAX_XY_VAR=0.25 较宽松（σ≈0.5m），仅排除明显发散；非最终定位质量标准。
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
MAX_XY_VAR = 0.25
MAX_YAW_VAR = 0.20
HOLD_SEC = 1.0
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

t0 = time.time()
first = box['msg']
first_stamp = None
if first is not None:
    first_stamp = (first.header.stamp.sec, first.header.stamp.nanosec)
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
    var_x, var_y, var_yaw = cov[0], cov[7], cov[35]
    finite = all(math.isfinite(v) for v in (wx, wy, var_x, var_y, var_yaw))
    ok = (
        finite
        and dist < MAX_DIST
        and var_x < MAX_XY_VAR
        and var_y < MAX_XY_VAR
        and var_yaw < MAX_YAW_VAR
    )
    good_polls = good_polls + 1 if ok else 0
    line = (
        f'[AMCL_GATE] target=B amcl_world=({wx:.3f},{wy:.3f}) '
        f'distance={dist:.3f} cov[0]={var_x:.3f} cov[7]={var_y:.3f} '
        f'cov[35]={var_yaw:.3f} hold={good_polls}/{NEED_POLLS} ok={ok} '
        f'stamps={len(unique_stamps)}'
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
    f'need dist<{MAX_DIST} cov_xy<{MAX_XY_VAR} cov_yaw<{MAX_YAW_VAR})'
)
sys.exit(3)
PY
}

run_gate() {
  local name="$1" wx="$2" wy="$3" yaw="$4" timeout="${5:-180}"
  local seg_log="$OUT/.seg_${name}.log"
  : > "$seg_log"
  # 进度写 LOG + stderr（stdout 只留 SEG: 行，供 command substitution）
  {
    clear_costmaps "before SEG $name"
    echo "[WAYPOINT] name=$name action=navigate world=($wx,$wy) yaw=$yaw"
  } | tee -a "$LOG" >&2
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

# 分段定位健康快照（诊断门禁；恶化 exit 4 → 截断，不发下一 goal）
run_loc_snap() {
  local label="$1"
  local snap_log="$OUT/.loc_${label}.log"
  : > "$snap_log"
  echo "[LOC] snapshot after ${label}" | tee -a "$LOG" >&2
  set +e
  python3 "$ROOT/scripts/amcl_health_snapshot.py" --label "$label" --wait 1.0 \
    >"$snap_log" 2>&1
  local ec=$?
  set -u
  cat "$snap_log" >> "$LOG"
  local line
  line=$(grep -E "^LOC:" "$snap_log" | tail -1 || echo "LOC:${label}:UNKNOWN")
  echo "$line" >&2
  # stdout 一行给 summary
  echo "$line"
  return "$ec"
}

# 显式 /spin；不走 navigate_to_pose_gate，也不直接发 /cmd_vel
# spin_gate 内部：accept≤10s，超时销毁 client 后只重试 1 次；已 accepted 只等 result
run_spin_gate() {
  local name="$1" target_yaw="$2" expect_yaw="$3" timeout="${4:-60}"
  local seg_log="$OUT/.spin_${name}.log"
  : > "$seg_log"
  {
    clear_costmaps "before SPIN $name"
    echo "[WAYPOINT] name=$name action=spin target_yaw=$target_yaw expect_yaw=$expect_yaw"
    echo "[dwell] 1.5s after clear before /spin (握手时序)"
    sleep 1.5
  } | tee -a "$LOG" >&2
  set +e
  python3 "$ROOT/scripts/spin_gate.py" \
    --name "$name" --target-yaw "$target_yaw" --expect-yaw "$expect_yaw" \
    --timeout "$timeout" --accept-timeout 10 \
    >"$seg_log" 2>&1
  local ec=$?
  set -u
  cat "$seg_log" >> "$LOG"
  # 优先透传 gate 的 SPIN:name:PASS/FAIL:accepted=:retry= 行
  local line
  line=$(grep -E "^SPIN:${name}:" "$seg_log" | tail -1 || true)
  if [[ -z "$line" ]]; then
    local final=FAIL
    if grep -q 'FINAL: PASS' "$seg_log"; then
      final=PASS
    fi
    line="SPIN:$name:$final:accepted=?:retry=?:ec=$ec"
  else
    line="${line}:ec=$ec"
  fi
  if grep -q 'transport_retry=1' "$seg_log"; then
    echo "[note] transport_retry=1 on $name — 本轮不算最终稳定通过" | tee -a "$LOG" >&2
  fi
  echo "$line"
  return "$ec"
}

run_split_chain() {
  # nav / spin 交错；段成功后做 LOC 快照，诊断门禁 FAIL(ec=4) 立即停
  # STAGE=arc1|arc2|full：先验东向 90° 弧，再北向，再接走廊
  local summary="" out ec loc_out
  local steps=()
  case "$STAGE" in
    arc1)
      steps=(
        "nav|B_clear|$BCLX|$BCLY|$BCLYAW"
        "nav|B_arc_east|$BAEX|$BAEY|$BAEYAW"
      )
      ;;
    arc2)
      steps=(
        "nav|B_clear|$BCLX|$BCLY|$BCLYAW"
        "nav|B_arc_east|$BAEX|$BAEY|$BAEYAW"
        "nav|B_arc_north|$BANX|$BANY|$BANYAW"
      )
      ;;
    full)
      steps=(
        "nav|B_clear|$BCLX|$BCLY|$BCLYAW"
        "nav|B_arc_east|$BAEX|$BAEY|$BAEYAW"
        "nav|B_arc_north|$BANX|$BANY|$BANYAW"
        "nav|B_corridor_in|$BCIX|$BCIY|$BCIYAW"
        "dwell|pre_corridor_turn|1.5"
        "spin|B_corridor_turn|$SPIN_TURN|$EXPECT_TURN"
        "nav|C_approach|$CAX|$CAY|$CAYAW"
        "nav|C|$CX|$CY|$CYAW"
      )
      ;;
    *)
      echo "[run] unknown STAGE=$STAGE (want arc1|arc2|full)" | tee -a "$LOG" >&2
      return 2
      ;;
  esac
  local step kind name a b c
  for step in "${steps[@]}"; do
    IFS='|' read -r kind name a b c <<< "$step"
    if [[ "$kind" == dwell ]]; then
      echo "[dwell] ${a}s before next step ($name)" | tee -a "$LOG" >&2
      sleep "$a"
      continue
    fi
    set +e
    if [[ "$kind" == spin ]]; then
      out=$(run_spin_gate "$name" "$a" "$b" 60)
    else
      out=$(run_gate "$name" "$a" "$b" "$c" 180)
    fi
    ec=$?
    set -u
    if [[ -n "$summary" ]]; then
      summary="${summary}; ${out}"
    else
      summary="$out"
    fi
    if (( ec != 0 )); then
      echo "[run] STOP after ${name} FAIL" | tee -a "$LOG"
      echo "$summary"
      return 1
    fi
    # 段成功 → 固定等待后再 LOC（区分「旋转后短暂重收敛」vs 真发散）
    # gate 内 STOPPED_WINDOW≈1s；此处再 dwell，合计约 3s 静止后采 cov
    echo "[dwell] ${POST_SEG_LOC_DWELL_SEC}s after ${name} before LOC snap" | tee -a "$LOG" >&2
    sleep "$POST_SEG_LOC_DWELL_SEC"
    set +e
    loc_out=$(run_loc_snap "after_${name}")
    ec=$?
    set -u
    summary="${summary}; ${loc_out}"
    if (( ec != 0 )); then
      echo "[run] STOP after ${name}: LOC health FAIL (首发散点候选)" | tee -a "$LOG"
      echo "$summary"
      return 4
    fi
  done
  echo "$summary"
  return 0
}

echo "输出: $OUT  (mode=$MODE stage=$STAGE ×$N)  VERSION=$WAYPOINT_VERSION"
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
  echo "# B→C 快速诊断 ×${N} (mode=${MODE}, stage=${STAGE}, ${WAYPOINT_VERSION})"
  echo ""
  echo "- 开始: $(date -Iseconds)"
  echo "- HEAD: $(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
  echo "- VERSION: ${WAYPOINT_VERSION}"
  if [[ "$MODE" == split ]]; then
    echo "- 每轮: spawn@B + clear + initialpose + AMCL(cov)@B → nav/spin 交错（每段前 clear）"
    echo "- STAGE=${STAGE}"
    case "$STAGE" in
      arc1) echo "- 链: B_clear → B_arc_east(${BAEX},${BAEY},${BAEYAW})  [probe 候选；mission 未同步]" ;;
      arc2) echo "- 链: B_clear → B_arc_east(${BAEX},${BAEY},0) → B_arc_north(${BANX},${BANY},+π/2)  [穿门前回收；mission 未同步]" ;;
      full) echo "- 链: B_clear → B_arc_east → B_arc_north → B_corridor_in → B_corridor_turn_spin → C_approach → C" ;;
    esac
    echo "- 房内不用纯 Spin；走廊 turn 仍 /spin（仅 full）；段后 dwell ${POST_SEG_LOC_DWELL_SEC}s + LOC 截断；门禁不放宽"
  else
    echo "- 每轮: spawn@B + clear + initialpose + AMCL(cov)@B → clear → NavigateToPose(C)"
  fi
  echo "- AMCL 门禁: dist<0.25 且 cov[0]<0.25 cov[7]<0.25 cov[35]<0.20 保持~1s"
  echo "- 段后 LOC: action PASS → dwell ${POST_SEG_LOC_DWELL_SEC}s → snapshot（cov_xy>0.25 / cov_yaw>0.20 / spread>0.5 → 截断）"
  echo "- Nav2/AMCL/Progress/Recovery: 未改"
  echo "- 本探针 ≠ patrol_mission_node；PASS 不能宣布 4.5b"
  echo ""
  echo "| # | 结果 | exit | 摘要 |"
  echo "|---|---|---|---|"
} > "$RESULTS_MD"
echo "run,result,exit,summary" > "$RESULTS_CSV"

pass_n=0
fail_n=0
infra_n=0
loc_n=0

for i in $(seq 1 "$N"); do
  printf -v tag "%02d" "$i"
  LOG="$OUT/run${tag}.log"
  : > "$LOG"
  echo ""
  echo "========== B→C RUN $i / $N (mode=$MODE stage=$STAGE $WAYPOINT_VERSION) =========="

  if ! reset_to_b >>"$LOG" 2>&1; then
    echo "| $i | INFRA | 3 | spawn@B failed |" >> "$RESULTS_MD"
    echo "$i,INFRA,3,spawn_failed" >> "$RESULTS_CSV"
    ((infra_n++)) || true
    continue
  fi

  clear_costmaps "after reset@B" | tee -a "$LOG"

  echo "[run$i] hard initialpose at B" | tee -a "$LOG"
  if ! python3 "$ROOT/scripts/publish_amcl_initial_pose.py" \
      --world-x "$BX" --world-y "$BY" --yaw "$BYAW" >>"$LOG" 2>&1; then
    echo "| $i | INFRA | 2 | initialpose B failed |" >> "$RESULTS_MD"
    echo "$i,INFRA,2,initialpose_failed" >> "$RESULTS_CSV"
    ((infra_n++)) || true
    continue
  fi

  echo "[run$i] AMCL_GATE (dist+cov xy/yaw hold~1s)" | tee -a "$LOG"
  if ! check_amcl_locked_at_b >>"$LOG" 2>&1; then
    echo "| $i | INFRA | 4 | amcl not locked at B |" >> "$RESULTS_MD"
    echo "$i,INFRA,4,amcl_not_at_B" >> "$RESULTS_CSV"
    ((infra_n++)) || true
    echo "[run$i] INFRA — 不发 NavigateToPose（勿归因航点）"
    continue
  fi

  # B 初始锁定后基线快照
  set +e
  loc_b=$(run_loc_snap "B_locked")
  loc_ec=$?
  set -u
  echo "$loc_b" | tee -a "$LOG"
  if (( loc_ec != 0 )); then
    echo "| $i | LOC | 4 | ${loc_b} |" >> "$RESULTS_MD"
    echo "$i,LOC,4,$(echo "$loc_b" | tr ',' ';')" >> "$RESULTS_CSV"
    ((loc_n++)) || true
    echo "[run$i] LOC — B_locked 已发散，不发 goal"
    continue
  fi

  SUMMARY="$loc_b"
  EC=0
  if [[ "$MODE" == split ]]; then
    echo "[run$i] split V3_4 chain (stage=$STAGE +LOC) ..." | tee -a "$LOG"
    set +e
    chain=$(run_split_chain)
    EC=$?
    set -u
    SUMMARY="${SUMMARY}; ${chain}"
    echo "$SUMMARY" | tee -a "$LOG"
    if (( EC == 0 )); then
      RES=PASS
      ((pass_n++)) || true
    elif (( EC == 4 )); then
      RES=LOC
      ((loc_n++)) || true
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
    SUMMARY="${SUMMARY}; ${out}"
    echo "$SUMMARY" | tee -a "$LOG"
    if (( EC == 0 )); then
      set +e
      loc_out=$(run_loc_snap "after_C")
      loc_ec=$?
      set -u
      SUMMARY="${SUMMARY}; ${loc_out}"
      if (( loc_ec != 0 )); then
        RES=LOC
        EC=4
        ((loc_n++)) || true
      else
        RES=PASS
        ((pass_n++)) || true
      fi
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
  echo "- LOC: **${loc_n}**（段后诊断门禁截断；找首发散点）"
  echo "- INFRA: **${infra_n}**"
  echo "- 有效导航轮: PASS+FAIL+LOC = $((pass_n + fail_n + loc_n)) / ${N}"
  echo "- 明细: \`runNN.log\` + \`.loc_*.log\`"
  echo ""
  echo "## 读表"
  echo "- INFRA = spawn/initialpose/AMCL 未锁，**不算航点失败**"
  echo "- LOC = 段动作成功但 AMCL 诊断门禁 FAIL（cov/spread）；**首发散点**"
  echo "- direct = 实验 A；split = V3_2；**≠ 4.5b 验收**"
  echo "- 摘要: SEG:… / SPIN:… / LOC:after_*:PASS|FAIL"
} | tee -a "$RESULTS_MD"

echo "完成: $RESULTS_MD"
