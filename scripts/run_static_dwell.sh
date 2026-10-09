#!/usr/bin/env bash
# 零指令静置漂移诊断（双朝向对照）
#
# 前置：另开终端保持
#   ros2 launch navigation_config nav2_test_room.launch.py
#
# 用法：
#   bash scripts/run_static_dwell.sh           # 默认 90s × yaw=0 与 yaw=π/2
#   bash scripts/run_static_dwell.sh 120
#   bash scripts/run_static_dwell.sh 90 /tmp/my_dwell
#
# 输出：
#   bags/static_dwell_<stamp>/yaw0/
#   bags/static_dwell_<stamp>/yaw_pi2/
#   bags/static_dwell_<stamp>/COMPARE.md

set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DWELL_SEC="${1:-90}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${2:-$ROOT/bags/static_dwell_$STAMP}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
set -u
cd "$ROOT"
mkdir -p "$OUT"

echo "输出目录: $OUT"
echo "静置时长: ${DWELL_SEC}s × 2 朝向"
echo "HEAD: $(git rev-parse --short HEAD 2>/dev/null || echo unknown)"

if ! timeout 5 ros2 service type /spawn_entity >/dev/null 2>&1; then
  echo "错误: /spawn_entity 不可用。请先启动 nav2_test_room.launch.py"
  exit 2
fi

PI2="$(python3 - <<'PY'
import math
print(math.pi / 2)
PY
)"

set +e
python3 "$ROOT/scripts/static_dwell_forensics.py" \
  --output-dir "$OUT/yaw0" \
  --dwell-sec "$DWELL_SEC" \
  --spawn-yaw 0.0
EC0=$?
python3 "$ROOT/scripts/static_dwell_forensics.py" \
  --output-dir "$OUT/yaw_pi2" \
  --dwell-sec "$DWELL_SEC" \
  --spawn-yaw "$PI2"
EC1=$?
set -e

python3 - "$OUT" <<'PY'
import json
import math
from pathlib import Path
import sys

out = Path(sys.argv[1])
rows = []
for name, yaw in (('yaw0', 0.0), ('yaw_pi2', math.pi / 2)):
    path = out / name / 'result.json'
    if not path.is_file():
        rows.append((name, yaw, None))
        continue
    rows.append((name, yaw, json.loads(path.read_text())))

lines = [
    '# 双朝向静置对照',
    '',
    '| 朝向 | GT vx mm/s | GT vy mm/s | body_vx mm/s | body_vy mm/s | cmd_nz(全程) | wheel ΔL/R rad | label |',
    '|---|---|---|---|---|---|---|---|',
]
for name, yaw, data in rows:
    if data is None:
        lines.append(f'| {name} (yaw={yaw:.3f}) | - | - | - | - | - | - | missing |')
        continue
    fit = data.get('gt_fit') or {}
    body = data.get('body_frame') or {}
    cause = data.get('cause_judgment') or {}
    cmd = data.get('cmd_stats') or {}
    js = data.get('joint_stats') or {}
    wheels = f"{js.get('left_pos_delta')}/{js.get('right_pos_delta')}"
    lines.append(
        f"| {name} (yaw={yaw:.3f}) | {fit.get('vx_mm_s')} | {fit.get('vy_mm_s')} | "
        f"{body.get('body_vx_mm_s')} | {body.get('body_vy_mm_s')} | "
        f"{cmd.get('nonzero_n')} | {wheels} | `{cause.get('label')}` |"
    )

# Simple cross-yaw judgment
a = rows[0][2]
b = rows[1][2]
verdict = 'insufficient_data'
notes = []
if a and b and a.get('gt_fit', {}).get('ok') and b.get('gt_fit', {}).get('ok'):
    a_body = abs((a.get('body_frame') or {}).get('body_vx_mm_s') or 0)
    b_body = abs((b.get('body_frame') or {}).get('body_vx_mm_s') or 0)
    a_world_x = abs((a.get('gt_fit') or {}).get('vx_mm_s') or 0)
    b_world_x = abs((b.get('gt_fit') or {}).get('vx_mm_s') or 0)
    a_nz = (a.get('cmd_stats') or {}).get('nonzero_n', 0)
    b_nz = (b.get('cmd_stats') or {}).get('nonzero_n', 0)
    if a_nz or b_nz:
        verdict = 'command_path_not_clean'
        notes.append('nonzero cmd observed; fix clear/zero path before physics conclusion')
    elif a_body > 0.05 and b_body > 0.05 and a_body > 2 * abs((a.get('body_frame') or {}).get('body_vy_mm_s') or 0) and b_body > 2 * abs((b.get('body_frame') or {}).get('body_vy_mm_s') or 0):
        verdict = 'supports_body_x_contact_creep'
        notes.append('both yaws show dominant body +x creep under zero command')
    elif a_world_x > 0.05 and b_world_x > 0.05 and abs(a_world_x - b_world_x) < 0.15 and abs((b.get('gt_fit') or {}).get('vy_mm_s') or 0) < 0.1:
        verdict = 'supports_world_fixed_bias'
        notes.append('world +x persists across yaw change')
    else:
        verdict = 'mixed_or_weak_signal'
        notes.append('inspect raw traces before changing friction/contact')

lines += [
    '',
    '## 对照判断',
    '',
    f"- verdict: `{verdict}`",
    f"- notes: {'; '.join(notes) if notes else '(none)'}",
    '',
    '本对照不修改导航参数；确认物理蠕变后再做最小模型修复并复测静置。',
    '',
]
(out / 'COMPARE.md').write_text('\n'.join(lines) + '\n')
print('\n'.join(lines))
PY

echo ""
echo "exit yaw0=$EC0 yaw_pi2=$EC1"
echo "请审查:"
echo "  $OUT/COMPARE.md"
echo "  $OUT/yaw0/results.md"
echo "  $OUT/yaw_pi2/results.md"
# Prefer non-zero if either run failed clear/spawn hard.
if (( EC0 == 2 || EC0 == 3 || EC1 == 2 || EC1 == 3 )); then
  exit 2
fi
exit 0
