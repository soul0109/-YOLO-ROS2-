#!/usr/bin/env bash
set -eo pipefail
ROOT=/home/charles/inspection-robot
LOG=$ROOT/log/b2_vision_test_20261111
cd "$ROOT"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export DISPLAY="${DISPLAY:-:0}"
if [[ -f /run/user/1000/mutter-Xauthority ]]; then
  export XAUTHORITY=/run/user/1000/mutter-Xauthority
fi
echo "COLLECT_START $(date -Iseconds)" | tee "$LOG/04_collect_start.txt"
python3 "$ROOT/scripts/formal_vision_test_collect.py" \
  --out "$ROOT/bags/vision_5b_test_20261111" \
  --date-tag 20261111 \
  --sessions 1,2,3 \
  2>&1 | tee "$LOG/04_collect_console.txt"
echo "COLLECT_END $(date -Iseconds)" | tee -a "$LOG/04_collect_start.txt"
echo EXIT:$?
