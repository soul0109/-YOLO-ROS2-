#!/usr/bin/env bash
# M1 developer smoke: bag image → yolo_detector → detections/annotated.
# Proves inference plumbing only — NOT scene detection quality.
set -eo pipefail

ROOT="${HOME}/inspection-robot"
BAG="${1:-$ROOT/bags/inspection/smoke_20260831_093222}"
MODEL="${MODEL_PATH:-$ROOT/models/weights/yolov8n.pt}"
EVIDENCE_DIR="${EVIDENCE_DIR:-$ROOT/bags/vision_m1_smoke_$(date +%Y%m%d_%H%M%S)}"
DURATION="${DURATION:-20}"

# ROS setup scripts reference unset vars; keep nounset off around them.
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export PATH="${HOME}/.local/bin:${PATH}"

if [[ ! -f "$MODEL" ]]; then
  echo "missing model: $MODEL" >&2
  exit 1
fi
if [[ ! -d "$BAG" ]]; then
  echo "missing bag: $BAG" >&2
  exit 1
fi

mkdir -p "$EVIDENCE_DIR"
echo "evidence -> $EVIDENCE_DIR"
echo "model=$MODEL" | tee "$EVIDENCE_DIR/meta.txt"
sha256sum "$MODEL" | tee -a "$EVIDENCE_DIR/meta.txt"
python3 -c "import ultralytics,torch; print('ultralytics',ultralytics.__version__,'torch',torch.__version__)" | tee -a "$EVIDENCE_DIR/meta.txt"

# Launch detector (YAML model_path; empty launch override must not clobber)
ros2 launch vision_perception yolo_detector.launch.py \
  >"$EVIDENCE_DIR/detector.log" 2>&1 &
DET_PID=$!
cleanup() {
  kill "$DET_PID" 2>/dev/null || true
  wait "$DET_PID" 2>/dev/null || true
}
trap cleanup EXIT

# Wait until node is up
for _ in $(seq 1 30); do
  if ros2 node list 2>/dev/null | grep -q '/yolo_detector'; then
    break
  fi
  sleep 0.5
done

ros2 param get /yolo_detector model_path | tee "$EVIDENCE_DIR/param_model_path.txt"
ros2 param get /yolo_detector image_topic | tee "$EVIDENCE_DIR/param_image_topic.txt"

ros2 bag play "$BAG" --topics /camera/image_raw --rate 1.0 \
  >"$EVIDENCE_DIR/bag_play.log" 2>&1 &
BAG_PID=$!

# Wait for publishers, then capture one detections message.
for _ in $(seq 1 40); do
  if ros2 topic list 2>/dev/null | grep -q '/vision/detections'; then
    break
  fi
  sleep 0.5
done

timeout 30 ros2 topic echo /vision/detections --once \
  >"$EVIDENCE_DIR/detections_once.txt" 2>"$EVIDENCE_DIR/detections_once.err" || true

# Brief annotated topic presence check
timeout 10 ros2 topic echo /vision/annotated --once \
  >"$EVIDENCE_DIR/annotated_once.txt" 2>"$EVIDENCE_DIR/annotated_once.err" || true

sleep "$DURATION"
kill "$BAG_PID" 2>/dev/null || true
wait "$BAG_PID" 2>/dev/null || true

# Backend line from detector log
grep -E 'backend=|loading YOLO|passthrough|error|Error' "$EVIDENCE_DIR/detector.log" \
  | tee "$EVIDENCE_DIR/detector_key_lines.txt" || true

# Topic stats
ros2 topic info /vision/detections >"$EVIDENCE_DIR/topic_detections_info.txt" || true
ros2 topic info /vision/annotated >"$EVIDENCE_DIR/topic_annotated_info.txt" || true

echo "---- summary ----"
fail=0
if grep -q 'backend=ultralytics' "$EVIDENCE_DIR/detector.log"; then
  echo "PASS: backend=ultralytics"
else
  echo "FAIL: backend not ultralytics" >&2
  fail=1
fi
if grep -qE 'AttributeError|_ARRAY_API|process has died|Traceback' "$EVIDENCE_DIR/detector.log"; then
  echo "FAIL: detector log shows runtime error" >&2
  fail=1
fi
if [[ -s "$EVIDENCE_DIR/detections_once.txt" ]] && grep -q 'frame_id:' "$EVIDENCE_DIR/detections_once.txt"; then
  echo "PASS: got /vision/detections sample (boxes may be empty on COCO+sim)"
else
  echo "FAIL: empty /vision/detections sample" >&2
  fail=1
fi
if [[ -s "$EVIDENCE_DIR/annotated_once.txt" ]]; then
  echo "PASS: got /vision/annotated sample"
else
  echo "FAIL: empty /vision/annotated sample" >&2
  fail=1
fi
echo "NOTE: M1 proves inference plumbing only — not inspection detection quality."
echo "done: $EVIDENCE_DIR"
exit "$fail"
