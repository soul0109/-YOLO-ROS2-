#!/usr/bin/env bash
# Pull host Cursor edits from VMware shared folder into the VM ROS workspace.
# Run inside the Ubuntu VM:
#   bash ~/inspection-robot/scripts/sync_from_share.sh

set -euo pipefail

PROJECT_NAME="基于YOLO+ROS2的智能巡检机器人仿真系统"
DEST="${DEST_DIR:-$HOME/inspection-robot}"

if [[ -n "${SHARE_DIR:-}" ]]; then
  SHARE="$SHARE_DIR"
else
  for candidate in \
    "/mnt/a_xm/${PROJECT_NAME}" \
    "/mnt/hgfs/a_xm/${PROJECT_NAME}"; do
    if [[ -d "$candidate" ]]; then
      SHARE="$candidate"
      break
    fi
  done
  SHARE="${SHARE:-/mnt/a_xm/${PROJECT_NAME}}"
fi

if [[ ! -d "$SHARE" ]]; then
  echo "共享目录不存在: $SHARE"
  echo "提示: 先挂载 VMware 共享，例如："
  echo "  sudo mkdir -p /mnt/hgfs"
  echo "  sudo vmhgfs-fuse .host:/ /mnt/hgfs -o allow_other"
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
