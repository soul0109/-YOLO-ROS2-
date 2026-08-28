#!/usr/bin/env bash
# VM → GitHub → 宿主机 git pull
#
# VM 用法（提交并推送）：
#   bash ~/inspection-robot/scripts/git_sync_push.sh "feat: stage 3.3 teleop"
#
# 宿主机用法（拉取）：
#   cd <你的仓库目录>
#   git pull origin main

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

echo ">>> 当前分支与状态"
git status -sb
echo ""

if git diff --quiet && git diff --cached --quiet && [[ -z "$(git ls-files --others --exclude-standard)" ]]; then
  echo "没有需要提交的改动。"
else
  git add -A
  git commit -m "$MSG"
fi

echo ">>> 推送到 origin/main ..."
git push -u origin HEAD

echo ""
echo "=========================================="
echo "  推送完成。宿主机执行："
echo "    cd <仓库目录>"
echo "    git pull origin main"
echo "=========================================="
