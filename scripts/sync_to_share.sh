#!/usr/bin/env bash
# Push VM workspace edits to the VMware shared folder (host Cursor can see them).
# Run inside the Ubuntu VM after a dev session or before host commit/push:
#   bash ~/inspection-robot/scripts/sync_to_share.sh
#
# Pair with sync_from_share.sh (host -> VM) when you edit on the host instead.

set -euo pipefail

PROJECT_NAME="基于YOLO+ROS2的智能巡检机器人仿真系统"
SRC="${SRC_DIR:-$HOME/inspection-robot}"

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

mkdir -p "$SHARE"

rsync -a --delete \
  --exclude '.git/' \
  --exclude 'ros2_ws/build/' \
  --exclude 'ros2_ws/install/' \
  --exclude 'ros2_ws/log/' \
  --exclude '__pycache__/' \
  --exclude '.pytest_cache/' \
  --exclude '*.pyc' \
  --exclude '.workbuddy/' \
  "$SRC"/ "$SHARE"/

echo "同步完成: $SRC -> $SHARE"
echo "提示: 宿主机打开共享文件夹即可看到最新代码；正式 push 仍在宿主机执行 sync_push.ps1。"
