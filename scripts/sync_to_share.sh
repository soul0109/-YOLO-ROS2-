#!/usr/bin/env bash
# VM → 宿主机 VMware 共享文件夹（单向覆盖，不拉取）
#
# 用法：
#   bash ~/inspection-robot/scripts/sync_to_share.sh
#
# 典型流程（Git push 失败时）：
#   bash ~/inspection-robot/scripts/git_sync_push.sh "feat: xxx"
#   # push 失败会自动调用本脚本

set -eo pipefail

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
  SHARE="${SHARE:-/mnt/hgfs/a_xm/${PROJECT_NAME}}"
fi

if [[ ! -d "$SHARE" ]]; then
  echo "共享目录不存在: $SHARE"
  echo "提示: 先挂载 VMware 共享，例如："
  echo "  sudo mkdir -p /mnt/hgfs"
  echo "  sudo vmhgfs-fuse .host:/ /mnt/hgfs -o allow_other"
  exit 1
fi

mkdir -p "$SHARE"

echo ">>> VM → 共享文件夹（镜像覆盖，不碰 VM 本地 .git）"
echo "    源: $SRC"
echo "    目标: $SHARE"
echo ""

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

echo ""
echo "同步完成。宿主机打开共享文件夹即可看到最新代码。"
