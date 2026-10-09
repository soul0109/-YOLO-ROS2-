#!/usr/bin/env bash
# 最小场景前爬诊断（empty.world，无 Nav2）
#
# 假说标签（写入 debug NDJSON）：
#   H_A  前球缺少 minDepth/maxVel（与后轮不对称）→ 恒定 body+x 蠕变
#   H_B  驱动轮 implicit_spring_damper 维持微滚
#   H_C  仍有非零 /cmd_vel*（应排除）
#   H_D  持续俯仰/高度偏差与蠕变共存（接触未理想平衡）
#   H_E  仅落地瞬态（settle 后应消失；若 60s 仍在则排除）
#
# 前置：另开终端只起 Gazebo+机器人（不要 Nav2）：
#   ros2 launch simulation_worlds gazebo_robot_v0.launch.py gui:=true
#
# 用法：
#   bash scripts/run_creep_min_scene.sh baseline 60
#   bash scripts/run_creep_min_scene.sh contact_unified 60   # 改 URDF 并 rebuild/重启后再跑
#   bash scripts/run_creep_min_scene.sh no_spring_damper 60

set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LABEL="${1:-baseline}"
DWELL_SEC="${2:-60}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$ROOT/bags/creep_min_${LABEL}_${STAMP}"
DEBUG_LOG="$ROOT/.cursor/debug-7f3450.log"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
set -u
cd "$ROOT"
mkdir -p "$OUT" "$(dirname "$DEBUG_LOG")"

echo "=== creep min-scene: label=$LABEL dwell=${DWELL_SEC}s ==="
bash "$ROOT/scripts/verify_robot_model_params.sh" 0.1

if ! timeout 8 ros2 service type /spawn_entity >/dev/null 2>&1; then
  echo "错误: 本终端看不到 /spawn_entity（仿真 ROS 图为空或不在同一 domain）。"
  echo "诊断（请把下面输出贴回）："
  echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
  echo "  ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset>}"
  echo "--- ros2 node list ---"
  timeout 5 ros2 node list 2>&1 || true
  echo "--- ros2 service list | grep -E 'spawn|gazebo' ---"
  timeout 5 ros2 service list 2>&1 | grep -E 'spawn|gazebo' || echo '(no spawn/gazebo services)'
  echo "--- pgrep gz ---"
  pgrep -a gzserver || echo '(no gzserver)'
  pgrep -a gzclient || echo '(no gzclient)'
  echo ""
  echo "处理：确认终端 A 的 launch 仍在跑且无报错退出；两终端都 source 同一环境后重试。"
  echo "  终端 A: ros2 launch simulation_worlds gazebo_robot_v0.launch.py"
  echo "  若 A 已死：先 pkill -f gzserver; pkill -f gzclient; 再重新 launch"
  exit 2
fi

# 记录模型接触片段到 debug log（假说 A/B 配置快照）
python3 - "$DEBUG_LOG" "$LABEL" <<'PY'
import json, time, subprocess, sys
from pathlib import Path
log_path, label = Path(sys.argv[1]), sys.argv[2]
urdf = subprocess.run(
    ['bash','-lc','source /opt/ros/humble/setup.bash && source ~/inspection-robot/ros2_ws/install/setup.bash && xacro $(ros2 pkg prefix robot_description)/share/robot_description/urdf/robot_v0.urdf.xacro'],
    capture_output=True, text=True, check=False,
).stdout
def block(ref):
    import re
    m = re.search(rf'<gazebo reference="{ref}">(.*?)</gazebo>', urdf, re.S)
    return m.group(1).strip() if m else ''
caster = block('caster_link')
left = block('left_wheel_link')
spring = 'implicit_spring_damper>true' in urdf
row = {
  'sessionId': '7f3450',
  'runId': label,
  'hypothesisId': 'H_A',
  'location': 'run_creep_min_scene.sh:model_snapshot',
  'message': 'contact_param_snapshot',
  'data': {
    'label': label,
    'caster_has_minDepth': '<minDepth>' in caster,
    'caster_has_maxVel': '<maxVel>' in caster,
    'wheel_has_minDepth': '<minDepth>' in left,
    'wheel_has_maxVel': '<maxVel>' in left,
    'caster_snippet': caster.replace('\n',' ')[:400],
    'left_wheel_snippet': left.replace('\n',' ')[:400],
    'implicit_spring_damper': spring,
  },
  'timestamp': int(time.time()*1000),
}
log_path.parent.mkdir(parents=True, exist_ok=True)
with log_path.open('a') as f:
    f.write(json.dumps(row, ensure_ascii=False) + '\n')
print('debug: wrote model snapshot', log_path)
PY

PI2="$(python3 - <<'PY'
import math
print(math.pi / 2)
PY
)"

# empty 场景出生在原点附近；settle 由 forensics 内 spawn 后 2s + fit skip 2s；此处再要求 skip_clear
run_one() {
  local name="$1" yaw="$2"
  python3 "$ROOT/scripts/static_dwell_forensics.py" \
    --output-dir "$OUT/$name" \
    --dwell-sec "$DWELL_SEC" \
    --spawn-x 0.0 \
    --spawn-y 0.0 \
    --spawn-z 0.05 \
    --spawn-yaw "$yaw" \
    --skip-clear \
    --entity-sample-period 0
}

set +e
run_one yaw0 0.0
EC0=$?
run_one yaw_pi2 "$PI2"
EC1=$?
set -e

python3 - "$OUT" "$DEBUG_LOG" "$LABEL" "$DWELL_SEC" <<'PY'
import json, math, time, sys
from pathlib import Path

out = Path(sys.argv[1])
log_path = Path(sys.argv[2])
label = sys.argv[3]
dwell = float(sys.argv[4])

def load(name):
    p = out / name / 'result.json'
    return json.loads(p.read_text()) if p.is_file() else None

def pitch_from_quat(q):
    # msg may store only yaw in trace; use first/last if orientation present — traces lack full quat.
    return None

rows = []
for name, yaw in (('yaw0', 0.0), ('yaw_pi2', math.pi/2)):
    d = load(name)
    if not d:
        rows.append({'name': name, 'missing': True})
        continue
    fit = d.get('gt_fit') or {}
    body = d.get('body_frame') or {}
    cmd = d.get('cmd_stats') or {}
    js = d.get('joint_stats') or {}
    first = fit.get('first') or {}
    last = fit.get('last') or {}
    # height proxy: not in pose row; use unavailable
    rows.append({
        'name': name,
        'yaw': yaw,
        'body_vx_mm_s': body.get('body_vx_mm_s'),
        'body_vy_mm_s': body.get('body_vy_mm_s'),
        'speed_m_s': fit.get('speed_m_s'),
        'dx_m': fit.get('dx_m'),
        'dy_m': fit.get('dy_m'),
        'dt_sec': fit.get('dt_sec'),
        'cmd_nonzero_n': cmd.get('nonzero_n'),
        'cmd_received': cmd.get('received_by_topic'),
        'left_pos_delta': js.get('left_pos_delta'),
        'right_pos_delta': js.get('right_pos_delta'),
        'left_roll_arc_m': js.get('left_roll_arc_m'),
        'right_roll_arc_m': js.get('right_roll_arc_m'),
        'first_yaw': first.get('yaw'),
        'last_yaw': last.get('yaw'),
        'timing': d.get('timing'),
    })

# Hypothesis evaluations (baseline evidence; not a fix)
cmd_clean = all((r.get('cmd_nonzero_n') or 0) == 0 for r in rows if not r.get('missing'))
body_vals = [abs(r['body_vx_mm_s']) for r in rows if r.get('body_vx_mm_s') is not None]
mean_body = sum(body_vals)/len(body_vals) if body_vals else None
sustained = mean_body is not None and mean_body > 0.05  # mm/s after fit skip
heading_follow = False
if len(body_vals) >= 2:
    heading_follow = all(
        abs(r.get('body_vx_mm_s') or 0) > 2 * abs(r.get('body_vy_mm_s') or 0)
        and abs(r.get('body_vx_mm_s') or 0) > 0.05
        for r in rows if not r.get('missing')
    )

wheel_match = []
for r in rows:
    if r.get('missing'):
        continue
    dx = abs(r.get('dx_m') or 0)
    # for yaw_pi2, world dx may be tiny; use speed*dt or body displacement proxy
    arc = abs(r.get('left_roll_arc_m') or 0)
    wheel_match.append({'name': r['name'], 'dx_or_body': dx, 'arc': arc, 'ratio': (arc/dx if dx > 1e-5 else None)})

def emit(hid, message, data):
    row = {
        'sessionId': '7f3450',
        'runId': label,
        'hypothesisId': hid,
        'location': 'run_creep_min_scene.sh:analyze',
        'message': message,
        'data': data,
        'timestamp': int(time.time()*1000),
    }
    with log_path.open('a') as f:
        f.write(json.dumps(row, ensure_ascii=False) + '\n')

emit('H_C', 'cmd_path_clean_check', {
    'cmd_clean': cmd_clean,
    'rows': [{k: r.get(k) for k in ('name','cmd_nonzero_n','cmd_received')} for r in rows],
})
emit('H_E', 'sustained_vs_transient', {
    'dwell_sec': dwell,
    'mean_abs_body_vx_mm_s': mean_body,
    'sustained_creep': sustained,
    'note': 'fit skips first 2s sim; sustained if mean |body_vx|>0.05mm/s over dwell',
})
emit('H_A', 'creep_metrics_for_contact_asymmetry', {
    'label': label,
    'heading_follow': heading_follow,
    'mean_abs_body_vx_mm_s': mean_body,
    'per_yaw': rows,
    'compare_note': 'Run contact_unified after adding caster minDepth/maxVel; compare mean_abs_body_vx',
})
emit('H_B', 'spring_damper_context', {
    'label': label,
    'note': 'Compare this mean_body_vx with no_spring_damper run; do not change friction/inertia same time',
    'mean_abs_body_vx_mm_s': mean_body,
})
emit('H_D', 'wheel_roll_matches_translation', {
    'wheel_match': wheel_match,
    'note': 'arc≈body displacement supports rolling-compatible residual, not pure slide',
})

# human summary
lines = [
    f'# creep min-scene COMPARE ({label})',
    '',
    f'- dwell_sec: {dwell}',
    f'- mean |body_vx|: {mean_body} mm/s',
    f'- cmd_clean: {cmd_clean}',
    f'- heading_follow: {heading_follow}',
    f'- sustained_creep: {sustained}',
    '',
    '| yaw | body_vx mm/s | body_vy | dx m | cmd_nz | wheel arc L/R |',
    '|---|---|---|---|---|---|',
]
for r in rows:
    if r.get('missing'):
        lines.append(f"| {r['name']} | missing | | | | |")
        continue
    lines.append(
        f"| {r['name']} | {r.get('body_vx_mm_s')} | {r.get('body_vy_mm_s')} | "
        f"{r.get('dx_m')} | {r.get('cmd_nonzero_n')} | "
        f"{r.get('left_roll_arc_m')}/{r.get('right_roll_arc_m')} |"
    )
summary = '\n'.join(lines) + '\n'
(out / 'COMPARE.md').write_text(summary)
print(summary)
print(f'debug log: {log_path}')
PY

echo "exit yaw0=$EC0 yaw_pi2=$EC1"
echo "输出: $OUT"
echo "debug: $DEBUG_LOG"
