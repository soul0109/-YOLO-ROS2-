#!/usr/bin/env bash
# Pull host Cursor edits from VMware shared folder into the VM ROS workspace.
# Run inside the Ubuntu VM:
#   bash ~/inspection-robot/scripts/sync_from_share.sh

set -euo pipefail

SHARE="${SHARE_DIR:-/mnt/a_xm/基于YOLO+ROS2的智能巡检机器人仿真系统}"
DEST="${DEST_DIR:-$HOME/inspection-robot}"

if [[ ! -d "$SHARE" ]]; then
  echo "共享目录不存在: $SHARE"
  exit 1
fi

mkdir -p "$DEST"

rsync -a --delete \
  --exclude '.git/' \
  --exclude 'ros2_ws/build/' \
  --exclude 'ros2_ws/install/' \
  --exclude 'ros2_ws/log/' \
  --exclude '__pycache__/' \
  --exclude '.pytest_cache/' \
  --exclude '*.pyc' \
  --exclude '.workbuddy/' \
  "$SHARE"/ "$DEST"/

echo "同步完成: $SHARE -> $DEST"
echo "提示: VM 侧 .git 保留不动；正式提交仍在宿主机或 VM 各自仓库处理。"
