#!/usr/bin/env bash
# VM：先 git commit → 尝试 push → 失败则同步到宿主机共享文件夹
#
# 用法：
#   bash ~/inspection-robot/scripts/git_sync_push.sh "feat: 描述改动"
#
# 宿主机拉取优先级：
#   1. git pull origin main          （push 成功时）
#   2. 直接看共享文件夹              （push 失败时，本脚本自动 sync_to_share）
#   3. git pull ~/inspection-robot-vm.bundle main  （bundle 离线包）

set -eo pipefail

ROOT="${HOME}/inspection-robot"
MSG="${1:-}"

cd "$ROOT"

if [[ -z "$MSG" ]]; then
  echo "用法: bash scripts/git_sync_push.sh \"commit message\""
  echo ""
  git status -sb
  exit 1
fi

echo ">>> [1/3] git commit"
git status -sb
echo ""

if git diff --quiet && git diff --cached --quiet && [[ -z "$(git ls-files --others --exclude-standard)" ]]; then
  echo "（工作区干净，跳过 commit）"
else
  git add -A
  git commit -m "$MSG"
fi

echo ""
echo ">>> [2/3] git push origin/main ..."
if git push -u origin HEAD; then
  echo ""
  echo "=========================================="
  echo "  推送成功。宿主机: git pull origin main"
  echo "=========================================="
  if bash "${ROOT}/scripts/sync_to_share.sh" 2>/dev/null; then
    echo "  共享文件夹已同步（VMware 共享已挂载）"
  else
    echo "  （共享文件夹未挂载，跳过 sync_to_share）"
  fi
  exit 0
fi

echo ""
echo ">>> [3/3] push 失败，改同步到宿主机共享文件夹..."
BUNDLE="${HOME}/inspection-robot-vm.bundle"
git bundle create "$BUNDLE" origin/main..HEAD 2>/dev/null || git bundle create "$BUNDLE" HEAD

bash "${ROOT}/scripts/sync_to_share.sh"

echo ""
echo "=========================================="
echo "  GitHub push 失败，已用两种方式备份到宿主机："
echo "  方式 1: 共享文件夹已更新（Cursor 直接打开即可）"
echo "  方式 2: 离线 bundle: $BUNDLE"
echo "          宿主机仓库: git pull $BUNDLE main"
echo "=========================================="
exit 0
