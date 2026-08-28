# Push local commits to GitHub so the Ubuntu VM can git pull.
# Usage:
#   .\scripts\sync_push.ps1
#   .\scripts\sync_push.ps1 -Message "wip: pubsub demo"
# If there are uncommitted changes and -Message is set, they will be committed first.

param(
    [string]$Message = ""
)

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
Set-Location $Root

$status = git status --porcelain
if ($status) {
    if (-not $Message) {
        Write-Host "有未提交改动。请先提交，或带上 -Message 让脚本帮你提交：" -ForegroundColor Yellow
        git status -sb
        Write-Host ""
        Write-Host '示例: .\scripts\sync_push.ps1 -Message "wip: sync to vm"'
        exit 1
    }
    git add -A
    git commit -m $Message
}

git push -u origin HEAD
Write-Host ""
Write-Host "已推到 GitHub。在虚拟机里执行：" -ForegroundColor Green
Write-Host "  cd ~/inspection-robot && git pull"
