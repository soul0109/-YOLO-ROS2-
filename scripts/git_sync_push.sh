#!/usr/bin/env bash
# VM：先 git commit → 尝试 push → 失败则生成 bundle 离线包
#
# 用法：
#   bash ~/inspection-robot/scripts/git_sync_push.sh "feat: 描述改动"
#
# 宿主机拉取：
#   git pull origin main
#   或 git pull ~/inspection-robot-vm.bundle main（push 失败时生成的 bundle）

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

echo ">>> [1/2] git commit"
git status -sb
echo ""

if git diff --quiet && git diff --cached --quiet && [[ -z "$(git ls-files --others --exclude-standard)" ]]; then
  echo "（工作区干净，跳过 commit）"
else
  git add -A
  git commit -m "$MSG"
fi

echo ""
echo ">>> [2/2] git push origin/main ..."
if git push -u origin HEAD; then
  echo ""
  echo "=========================================="
  echo "  推送成功。宿主机: git pull origin main"
  echo "=========================================="
  exit 0
fi

echo ""
echo ">>> push 失败，生成离线 bundle..."
BUNDLE="${HOME}/inspection-robot-vm.bundle"
git bundle create "$BUNDLE" origin/main..HEAD 2>/dev/null || git bundle create "$BUNDLE" HEAD

echo ""
echo "=========================================="
echo "  GitHub push 失败。宿主机可用："
echo "  git pull $BUNDLE main"
echo "=========================================="
exit 0
