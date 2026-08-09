#!/usr/bin/env bash
# Run inside the Ubuntu VM to pull the latest code from GitHub.
# Usage: bash scripts/sync_pull.sh

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [[ -n "$(git status --porcelain)" ]]; then
  echo "虚拟机仓库有未提交改动，先处理后再 pull："
  git status -sb
  exit 1
fi

git pull --ff-only
echo "同步完成: $(git log -1 --oneline)"
