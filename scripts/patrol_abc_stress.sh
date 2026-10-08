#!/usr/bin/env bash
# 4.5b 压力复验：A→B→C 连跑 N 次，自动记表（不改导航参数）。
#
# 前置（另开终端保持运行）：
#   cd ~/inspection-robot
#   source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
#   ros2 launch navigation_config nav2_test_room.launch.py
#   # 无人值守可加：gui:=false rviz:=false
#
# 本脚本（再开终端）：
#   bash scripts/patrol_abc_stress.sh          # 默认 10 次
#   bash scripts/patrol_abc_stress.sh 10
#   bash scripts/patrol_abc_stress.sh 5 /tmp/my_out
#
# 输出：
#   bags/patrol_stress_<时间戳>/results.md
#   bags/patrol_stress_<时间戳>/results.csv
#   bags/patrol_stress_<时间戳>/runNN.log

set -o pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
N="${1:-10}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${2:-$ROOT/bags/patrol_stress_$STAMP}"
SPAWN_RETRIES=3

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
SUMMARY_TXT="$OUT/summary.txt"

extract_rec() {
  # $1=log $2=waypoint letter → recoveries 数字或 -
  local log="$1" wp="$2" line
  line=$(grep -E "PATROL: ${wp} SUCCEEDED" "$log" | tail -1 || true)
  if [[ -z "$line" ]]; then
    echo "-"
    return
  fi
  if [[ "$line" =~ recoveries=([0-9]+) ]]; then
    echo "${BASH_REMATCH[1]}"
  else
    echo "?"
  fi
}

fail_stage() {
  # 从摘要推断卡在哪
  local log="$1"
  if grep -q 'PATROL: FINAL PASS' "$log"; then
    echo "-"
    return
  fi
  if grep -q 'initialpose' "$log" && ! grep -q 'PATROL: sending goal' "$log"; then
    echo "initialpose"
    return
  fi
  if grep -q 'spawn' "$log" && ! grep -q 'Successfully spawned' "$log"; then
    echo "spawn"
    return
  fi
  local last
  last=$(grep -E 'PATROL: (FAIL|A |B |C )' "$log" | tail -1 || true)
  if [[ "$last" =~ FAIL.*\ —\ ([ABC]) ]]; then
    echo "${BASH_REMATCH[1]}"
  elif [[ "$last" =~ PATROL:\ ([ABC])\ ABORT ]]; then
    echo "${BASH_REMATCH[1]}"
  elif grep -q 'PATROL: C ' "$log"; then
    echo "C"
  elif grep -q 'PATROL: B ' "$log"; then
    echo "B"
  elif grep -q 'PATROL: A ' "$log"; then
    echo "A"
  else
    echo "?"
  fi
}

wait_nav_ready() {
  # 在 initialpose 之后调用。用 service type，避免 service list 漏项误判。
  local deadline=$((SECONDS + 90))
  local g=/global_costmap/clear_entirely_global_costmap
  local l=/local_costmap/clear_entirely_local_costmap
  echo "[wait] costmap clear services + /amcl_pose ..."
  while (( SECONDS < deadline )); do
    if service_ok "$g" && service_ok "$l"; then
      if timeout 8 ros2 topic echo /amcl_pose --once >/dev/null 2>&1; then
        echo "[wait] nav ready"
        return 0
      fi
    fi
    sleep 1
  done
  echo "[wait] timeout — Nav2/AMCL 未就绪（建议清场后重开 launch）"
  return 1
}

reset_robot() {
  local attempt=1
  while (( attempt <= SPAWN_RETRIES )); do
    echo "[reset] attempt $attempt/$SPAWN_RETRIES delete+spawn"
    timeout 20 ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity \
      "{name: 'robot_v0'}" >/dev/null 2>&1 || true
    sleep 2
    if timeout 45 ros2 run gazebo_ros spawn_entity.py \
      -entity robot_v0 \
      -topic robot_description \
      -x 0.9 -y 3.0 -z 0.05 -Y 0.0
    then
      sleep 3
      return 0
    fi
    echo "[reset] spawn failed, retry..."
    sleep 2
    ((attempt++)) || true
  done
  return 1
}

service_ok() {
  # 比 ros2 service list | grep 稳：list 在 DDS 下会偶发漏项（你刚踩到的坑）
  local name="$1"
  timeout 5 ros2 service type "$name" >/dev/null 2>&1
}

wait_service() {
  local name="$1" secs="${2:-60}"
  local deadline=$((SECONDS + secs))
  echo "[wait] service $name (最多 ${secs}s，用 service type) ..."
  while (( SECONDS < deadline )); do
    if service_ok "$name"; then
      echo "[wait] $name ok"
      return 0
    fi
    sleep 1
  done
  echo "[wait] 超时: $name"
  return 1
}

preflight() {
  # 只做轻量检查；不因 delete_entity / amcl 误杀整次测试
  local load
  load=$(awk '{print $1}' /proc/loadavg)
  echo "当前 load(1m)=$load"
  if awk -v l="$load" 'BEGIN{exit !(l+0 > 6.0)}'; then
    echo "警告: load 偏高。可先清场再测。"
  fi

  echo "[wait] action /navigate_to_pose ..."
  local deadline=$((SECONDS + 60))
  while (( SECONDS < deadline )); do
    if ros2 action list 2>/dev/null | grep -q '/navigate_to_pose'; then
      echo "[wait] /navigate_to_pose ok"
      break
    fi
    sleep 1
  done
  if ! ros2 action list 2>/dev/null | grep -q '/navigate_to_pose'; then
    echo "错误: 找不到 /navigate_to_pose。请保持终端 1："
    echo "  ros2 launch navigation_config nav2_test_room.launch.py"
    exit 2
  fi

  if ! wait_service /spawn_entity 90; then
    echo "错误: /spawn_entity 不可用。请在同一终端执行："
    echo "  ros2 service type /spawn_entity"
    exit 2
  fi
  # delete 不做硬门槛（你手动 list 看得到，脚本 list 却漏报）
  if service_ok /delete_entity; then
    echo "[wait] /delete_entity ok"
  else
    echo "警告: 此刻探测不到 /delete_entity，reset 时仍会尝试调用"
  fi
  echo "预检通过，开始连跑。"
}

echo "输出目录: $OUT"
echo "计划次数: $N"
preflight

{
  echo "# 4.5b 压力复验 A→B→C ×${N}"
  echo ""
  echo "- 开始: $(date -Iseconds)"
  echo "- HEAD: $(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
  echo "- 重置: delete_entity + spawn（充电位 0.9, 3.0）+ initialpose"
  echo "- 节点: \`patrol_mission_node\`（不改参）"
  echo ""
  echo "| # | 结果 | exit | A_rec | B_rec | C_rec | 卡住 | 摘要 |"
  echo "|---|---|---|---|---|---|---|---|"
} > "$RESULTS_MD"

echo "run,result,exit,A_rec,B_rec,C_rec,fail_stage,summary" > "$RESULTS_CSV"

pass_n=0
fail_n=0
infra_n=0

for i in $(seq 1 "$N"); do
  printf -v tag "%02d" "$i"
  LOG="$OUT/run${tag}.log"
  : > "$LOG"
  echo ""
  echo "========== RUN $i / $N =========="

  echo "[run$i] reset robot" | tee -a "$LOG"
  if ! reset_robot >>"$LOG" 2>&1; then
    echo "| $i | INFRA | 3 | - | - | - | spawn | spawn failed after ${SPAWN_RETRIES} retries |" >> "$RESULTS_MD"
    echo "$i,INFRA,3,-,-,-,spawn,spawn_failed" >> "$RESULTS_CSV"
    ((infra_n++)) || true
    echo "[run$i] INFRA spawn failed"
    continue
  fi

  echo "[run$i] initialpose" | tee -a "$LOG"
  if ! python3 "$ROOT/scripts/publish_amcl_initial_pose.py" >>"$LOG" 2>&1; then
    echo "| $i | INFRA | 2 | - | - | - | initialpose | initialpose failed |" >> "$RESULTS_MD"
    echo "$i,INFRA,2,-,-,-,initialpose,initialpose_failed" >> "$RESULTS_CSV"
    ((infra_n++)) || true
    echo "[run$i] INFRA initialpose failed"
    continue
  fi
  sleep 2
  if ! wait_nav_ready >>"$LOG" 2>&1; then
    echo "| $i | INFRA | 4 | - | - | - | nav_not_ready | costmap/amcl not ready |" >> "$RESULTS_MD"
    echo "$i,INFRA,4,-,-,-,nav_not_ready,nav_not_ready" >> "$RESULTS_CSV"
    ((infra_n++)) || true
    echo "[run$i] INFRA nav not ready — 建议停测、清场、重开 launch"
    break
  fi

  echo "[run$i] patrol ..." | tee -a "$LOG"
  set +e
  ros2 run inspection_mission patrol_mission_node >>"$LOG" 2>&1
  EC=$?
  set -u

  A_REC=$(extract_rec "$LOG" A)
  B_REC=$(extract_rec "$LOG" B)
  C_REC=$(extract_rec "$LOG" C)
  STAGE=$(fail_stage "$LOG")
  SUMMARY=$(grep -E 'PATROL: (A |B |C |FINAL|FAIL)' "$LOG" | tr '\n' '; ' | sed 's/; $//' | cut -c1-160)

  if grep -q 'PATROL: FINAL PASS' "$LOG"; then
    RES=PASS
    ((pass_n++)) || true
  else
    RES=FAIL
    ((fail_n++)) || true
  fi

  echo "| $i | $RES | $EC | $A_REC | $B_REC | $C_REC | $STAGE | ${SUMMARY} |" >> "$RESULTS_MD"
  # CSV 摘要去逗号
  CSV_SUM=$(echo "$SUMMARY" | tr ',' ';')
  echo "$i,$RES,$EC,$A_REC,$B_REC,$C_REC,$STAGE,$CSV_SUM" >> "$RESULTS_CSV"

  echo "[run$i] done → $RES  A=$A_REC B=$B_REC C=$C_REC  stage=$STAGE"
  sleep 2
done

END_TS=$(date -Iseconds)
VALID=$((pass_n + fail_n))
{
  echo ""
  echo "## 统计"
  echo ""
  echo "- 结束: $END_TS"
  echo "- PASS: **${pass_n}** / ${N}"
  echo "- FAIL: **${fail_n}** / ${N}"
  echo "- INFRA（spawn/initialpose）: **${infra_n}** / ${N}"
  if (( VALID > 0 )); then
    echo "- 有效巡环通过率: **$(( pass_n * 100 / VALID ))%** （PASS/(PASS+FAIL)，不含 INFRA）"
  fi
  echo ""
  echo "## 读表提示"
  echo ""
  echo "- \`A_rec/B_rec/C_rec\`：该航点 Nav2 recoveries（越小越好；目标建议 ≤2）"
  echo "- \`卡住\`：FAIL 时大致卡在哪；PASS 为 \`-\`"
  echo "- 明细: \`run01.log\` … \`run${N}.log\`"
} | tee -a "$RESULTS_MD" | tee "$SUMMARY_TXT"

echo ""
echo "完成。把下面两个文件内容发给我即可一起归因："
echo "  $RESULTS_MD"
echo "  $RESULTS_CSV"
