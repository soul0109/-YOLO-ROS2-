#!/usr/bin/env bash
# BC_ROUTE_V1 重复验收：A→B→C 连跑 N 次，自动记表（不改导航参数）。
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

fail_stage() {
  # 从摘要推断卡在哪
  local log="$1"
  if grep -q 'PATROL_RESULT:' "$log"; then
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
  if grep -q 'PATROL: sending C' "$log"; then
    echo "C"
  elif grep -q 'PATROL: sending B' "$log"; then
    echo "B"
  elif grep -q 'PATROL: sending A' "$log"; then
    echo "A"
  else
    echo "?"
  fi
}

parse_result() {
  # Emit: class action_success acceptance A_rec B_rec C_rec fail_stage summary
  python3 "$ROOT/scripts/classify_patrol_result.py" "$1"
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

clear_nav_before_reset() {
  # 取消任务必须确认结束；仅发零速不能证明控制器已停。
  # 正式协议不用 GT 重设 initialpose；清控失败记 INFRA。
  local out_json="${1:-}"
  local args=(python3 "$ROOT/scripts/clear_nav_control.py")
  if [[ -n "$out_json" ]]; then
    args+=(--output "$out_json")
  fi
  "${args[@]}"
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
  echo "# BC_ROUTE_V1 重复验收 A→B→C ×${N}"
  echo ""
  echo "- 开始: $(date -Iseconds)"
  echo "- HEAD: $(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
  echo "- 重置: clear(cancel确认+cmd静默) → delete_entity + spawn（充电位 0.9, 3.0）+ 规定 initialpose"
  echo "- 节点: \`patrol_mission_node\`（不改参，诊断默认开启）"
  echo "- 比较口径: 每轮固定出生位姿和初始位姿；**不用 GT 重设种子**；先通过 clock/TF/AMCL 前置条件"
  echo "- 统计双分母: 流程率=ACCEPT_OK/started；条件率排除明确外部故障；另报 INIT/ROUTE 原始计数"
  echo ""
  echo "| # | 分类 | exit | init | route | accept | A_rec | B_rec | C_rec | 卡住 | 摘要 |"
  echo "|---|---|---|---|---|---|---|---|---|---|---|"
} > "$RESULTS_MD"

echo "run,class,exit,init,route,accept,A_rec,B_rec,C_rec,fail_stage,summary" > "$RESULTS_CSV"

# 双分母：
#   started  = 进入本轮 clear 的次数（含 setup/init 失败；提前中止未跑轮次不计）
#   external = 明确外部故障（清控 status 未见/cancel 服务不可用、spawn 彻底失败）
#   conditioned = started - external
#   ready_attempted = 进入 wait_for_nav_ready 的次数（定位门槛专用）
started_n=0
external_n=0
setup_flow_fail_n=0
ready_attempted_n=0
init_ok_n=0
init_fail_n=0
route_ok_n=0
route_fail_n=0
accept_ok_n=0
route_ok_accept_fail_n=0
not_run_after_stop_n=0

is_external_clear_fail() {
  local json="$1"
  [[ -f "$json" ]] || return 1
  python3 - "$json" <<'PY'
import json, sys
d=json.loads(open(sys.argv[1], encoding='utf-8').read())
reason=str(d.get('reason') or '')
inactive=(d.get('inactive') or {}).get('reason') or ''
text=reason + ' ' + inactive
# 仅把明确“服务不存在/调用失败”当外部故障；
# idle_status_unobserved 已允许 proceed，不应再标 EXTERNAL。
keys=('cancel_service_unavailable', 'cancel_unavailable_or_failed', 'cancel_call_failed')
sys.exit(0 if any(k in text for k in keys) else 1)
PY
}

for i in $(seq 1 "$N"); do
  printf -v tag "%02d" "$i"
  LOG="$OUT/run${tag}.log"
  : > "$LOG"
  echo ""
  echo "========== RUN $i / $N =========="

  ((started_n++)) || true

  echo "[run$i] clear nav control (cancel confirm + quiet cmd_vel)" | tee -a "$LOG"
  if ! clear_nav_before_reset "$OUT/run${tag}.clear.json" >>"$LOG" 2>&1; then
    CLS="SETUP_FAIL"
    if is_external_clear_fail "$OUT/run${tag}.clear.json"; then
      CLS="EXTERNAL"
      ((external_n++)) || true
    else
      ((setup_flow_fail_n++)) || true
    fi
    echo "| $i | $CLS | 6 | FAIL | NOT_RUN | FAIL | - | - | - | clear_nav | cancel/quiet failed |" >> "$RESULTS_MD"
    echo "$i,$CLS,6,FAIL,NOT_RUN,FAIL,-,-,-,clear_nav,clear_failed" >> "$RESULTS_CSV"
    echo "[run$i] $CLS clear_nav failed — 不进入 spawn"
    continue
  fi

  echo "[run$i] reset robot" | tee -a "$LOG"
  if ! reset_robot >>"$LOG" 2>&1; then
    ((external_n++)) || true
    echo "| $i | EXTERNAL | 3 | FAIL | NOT_RUN | FAIL | - | - | - | spawn | spawn failed after ${SPAWN_RETRIES} retries |" >> "$RESULTS_MD"
    echo "$i,EXTERNAL,3,FAIL,NOT_RUN,FAIL,-,-,-,spawn,spawn_failed" >> "$RESULTS_CSV"
    echo "[run$i] EXTERNAL spawn failed"
    continue
  fi

  echo "[run$i] initialpose" | tee -a "$LOG"
  if ! python3 "$ROOT/scripts/publish_amcl_initial_pose.py" >>"$LOG" 2>&1; then
    # 发布失败计入流程失败（可能是 AMCL 未订阅），不是直接剔除分母
    ((setup_flow_fail_n++)) || true
    echo "| $i | SETUP_FAIL | 2 | FAIL | NOT_RUN | FAIL | - | - | - | initialpose | initialpose publish failed |" >> "$RESULTS_MD"
    echo "$i,SETUP_FAIL,2,FAIL,NOT_RUN,FAIL,-,-,-,initialpose,initialpose_failed" >> "$RESULTS_CSV"
    echo "[run$i] SETUP_FAIL initialpose publish failed"
    continue
  fi
  sleep 2
  if ! wait_nav_ready >>"$LOG" 2>&1; then
    # costmap/amcl 话题未就绪：记流程失败；是否停测由负载决定，但仍算 started
    ((setup_flow_fail_n++)) || true
    echo "| $i | SETUP_FAIL | 4 | FAIL | NOT_RUN | FAIL | - | - | - | nav_not_ready | costmap/amcl not ready |" >> "$RESULTS_MD"
    echo "$i,SETUP_FAIL,4,FAIL,NOT_RUN,FAIL,-,-,-,nav_not_ready,nav_not_ready" >> "$RESULTS_CSV"
    echo "[run$i] SETUP_FAIL nav not ready — 建议停测、清场、重开 launch"
    remaining=$((N - i))
    if (( remaining > 0 )); then
      not_run_after_stop_n=$remaining
    fi
    break
  fi

  # /initialpose 可能先于 AMCL 激活被丢弃；在 ready 后再发布一次，
  # 让每轮从同一个已确认的初始位姿开始。
  if ! python3 "$ROOT/scripts/publish_amcl_initial_pose.py" >>"$LOG" 2>&1; then
    ((setup_flow_fail_n++)) || true
    echo "| $i | SETUP_FAIL | 2 | FAIL | NOT_RUN | FAIL | - | - | - | initialpose_ready | initialpose retry failed |" >> "$RESULTS_MD"
    echo "$i,SETUP_FAIL,2,FAIL,NOT_RUN,FAIL,-,-,-,initialpose_ready,initialpose_retry_failed" >> "$RESULTS_CSV"
    continue
  fi
  sleep 2

  ((ready_attempted_n++)) || true

  READY_JSON="$OUT/run${tag}.ready.json"
  if ! python3 "$ROOT/scripts/wait_for_nav_ready.py" \
      --output "$READY_JSON" >>"$LOG" 2>&1; then
    echo "| $i | INIT_FAIL | 5 | FAIL | NOT_RUN | FAIL | - | - | - | initial_localization | see run${tag}.ready.json |" >> "$RESULTS_MD"
    echo "$i,INIT_FAIL,5,FAIL,NOT_RUN,FAIL,-,-,-,initial_localization,see_run${tag}.ready.json" >> "$RESULTS_CSV"
    ((init_fail_n++)) || true
    echo "[run$i] INIT_FAIL initial localization did not reach stable gate → ROUTE=NOT_RUN"
    continue
  fi
  ((init_ok_n++)) || true

  echo "[run$i] patrol ..." | tee -a "$LOG"
  set +e
  RUN_DIR="$OUT/run${tag}"
  mkdir -p "$RUN_DIR"
  ros2 run inspection_mission patrol_mission_node --ros-args \
    -p output_dir:="$RUN_DIR" >>"$LOG" 2>&1
  EC=$?
  set -u

  PARSED=$(parse_result "$LOG")
  read -r CLS ACTION_OK ACCEPT_OK A_REC B_REC C_REC STAGE SUMMARY <<< "$PARSED"
  if [[ "$CLS" == "NO_RESULT" || "$CLS" == "BAD_RESULT" ]]; then
    STAGE="no_result"
    CLS="ROUTE_FAIL"
    ACTION_OK="false"
    ACCEPT_OK="false"
  fi

  ROUTE_CELL="FAIL"
  ACCEPT_CELL="FAIL"
  if [[ "$ACTION_OK" == "True" || "$ACTION_OK" == "true" ]]; then
    ROUTE_CELL="OK"
    ((route_ok_n++)) || true
  else
    ((route_fail_n++)) || true
  fi
  if [[ "$ACCEPT_OK" == "True" || "$ACCEPT_OK" == "true" ]]; then
    ACCEPT_CELL="OK"
    ((accept_ok_n++)) || true
  elif [[ "$CLS" == "ROUTE_OK_ACCEPT_FAIL" ]]; then
    ((route_ok_accept_fail_n++)) || true
  fi

  echo "| $i | $CLS | $EC | OK | $ROUTE_CELL | $ACCEPT_CELL | $A_REC | $B_REC | $C_REC | $STAGE | ${SUMMARY} |" >> "$RESULTS_MD"
  CSV_SUM=$(echo "$SUMMARY" | tr ',' ';')
  echo "$i,$CLS,$EC,OK,$ROUTE_CELL,$ACCEPT_CELL,$A_REC,$B_REC,$C_REC,$STAGE,$CSV_SUM" >> "$RESULTS_CSV"

  echo "[run$i] done → $CLS  route=$ROUTE_CELL accept=$ACCEPT_CELL  A=$A_REC B=$B_REC C=$C_REC  stage=$STAGE"
  sleep 2
done

END_TS=$(date -Iseconds)
conditioned_n=$((started_n - external_n))
flow_accept_rate="n/a"
cond_accept_rate="n/a"
init_rate="n/a"
route_rate="n/a"
if (( started_n > 0 )); then
  flow_accept_rate="$(( accept_ok_n * 100 / started_n ))%"
fi
if (( conditioned_n > 0 )); then
  cond_accept_rate="$(( accept_ok_n * 100 / conditioned_n ))%"
fi
if (( ready_attempted_n > 0 )); then
  init_rate="$(( init_ok_n * 100 / ready_attempted_n ))%"
fi
if (( init_ok_n > 0 )); then
  route_rate="$(( route_ok_n * 100 / init_ok_n ))%"
fi
{
  echo ""
  echo "## 统计（双分母 + 原始计数）"
  echo ""
  echo "- 结束: $END_TS"
  echo "- 计划轮次: ${N}"
  echo "- started（进入 clear）: **${started_n}**"
  echo "- external（明确外部故障）: **${external_n}**"
  echo "- conditioned = started - external: **${conditioned_n}**"
  echo "- ready_attempted（进入 wait_for_nav_ready）: **${ready_attempted_n}**"
  echo "- setup_flow_fail（initialpose/costmap 等流程失败）: **${setup_flow_fail_n}**"
  echo "- 提前停止后未执行: **${not_run_after_stop_n}**"
  echo ""
  echo "| 指标 | 计数 | 分母 | 比率 |"
  echo "|---|---|---|---|"
  echo "| 流程验收通过率 ACCEPT_OK | ${accept_ok_n} | started=${started_n} | ${flow_accept_rate} |"
  echo "| 条件验收通过率 ACCEPT_OK | ${accept_ok_n} | conditioned=${conditioned_n} | ${cond_accept_rate} |"
  echo "| 初始化成功率 INIT_OK | ${init_ok_n} | ready_attempted=${ready_attempted_n} | ${init_rate} |"
  echo "| 路线 Action 成功率 ROUTE_OK | ${route_ok_n} | INIT_OK=${init_ok_n} | ${route_rate} |"
  echo ""
  echo "- INIT_FAIL: **${init_fail_n}**（ROUTE=NOT_RUN）"
  echo "- ROUTE_FAIL: **${route_fail_n}**"
  echo "- ROUTE_OK_ACCEPT_FAIL: **${route_ok_accept_fail_n}**"
  echo ""
  echo "## 读表提示"
  echo ""
  echo "- \`EXTERNAL\`/\`SETUP_FAIL\` 计入 started；仅 EXTERNAL 从条件分母剔除"
  echo "- initialpose 未生效导致的不就绪应落在 SETUP_FAIL/INIT_FAIL，不再默认为可忽略 INFRA"
  echo "- 明细: \`run01.log\` … \`run${N}.log\`"
} | tee -a "$RESULTS_MD" | tee "$SUMMARY_TXT"

echo ""
echo "完成。把下面两个文件内容发给我即可一起归因："
echo "  $RESULTS_MD"
echo "  $RESULTS_CSV"
