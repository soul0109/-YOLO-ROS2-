# Pull the latest working tree from the Ubuntu VM over SSH (VM -> host).
# Use when VMware shared folder is unavailable or you want a one-shot sync.
# Needs: OpenSSH client + sync.env (see sync.env.example)
# Usage: .\scripts\sync_from_vm.ps1

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$EnvFile = Join-Path $PSScriptRoot "sync.env"

if (-not (Test-Path $EnvFile)) {
    Write-Host "缺少 scripts\sync.env。先复制 sync.env.example 并填 VM_SSH / VM_REPO。" -ForegroundColor Yellow
    exit 1
}

Get-Content $EnvFile | ForEach-Object {
    if ($_ -match '^\s*#' -or $_ -match '^\s*$') { return }
    $k, $v = $_ -split '=', 2
    Set-Item -Path "Env:$($k.Trim())" -Value $v.Trim()
}

if (-not $env:VM_SSH -or $env:VM_SSH -match '192\.168\.x\.x') {
    Write-Host "请在 scripts\sync.env 里把 VM_SSH 改成真实地址，例如 charles@192.168.221.128" -ForegroundColor Yellow
    exit 1
}

$VmRepo = if ($env:VM_REPO) { $env:VM_REPO } else { "~/inspection-robot" }
$Stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$TarName = "inspection-sync-$Stamp.tar.gz"
$TarPath = Join-Path $env:TEMP $TarName

Write-Host "在虚拟机打包 $VmRepo ..."
$RemotePack = @"
set -e
cd $VmRepo
tar -czf /tmp/$TarName \
  --exclude='./ros2_ws/build' \
  --exclude='./ros2_ws/install' \
  --exclude='./ros2_ws/log' \
  --exclude='./.git' \
  --exclude='./.cursor' \
  --exclude='**/__pycache__' \
  --exclude='**/.pytest_cache' \
  .
echo "VM pack OK"
"@
& ssh $env:VM_SSH $RemotePack
if ($LASTEXITCODE -ne 0) { throw "ssh pack failed: $LASTEXITCODE" }

Write-Host "下载到宿主机 $Root ..."
& scp "$($env:VM_SSH):/tmp/$TarName" $TarPath
if ($LASTEXITCODE -ne 0) { throw "scp failed: $LASTEXITCODE" }

Write-Host "解压到宿主机工作区 ..."
Push-Location $Root
try {
    & tar -xzf $TarPath
    if ($LASTEXITCODE -ne 0) { throw "tar extract failed: $LASTEXITCODE" }
} finally {
    Pop-Location
}

& ssh $env:VM_SSH "rm -f /tmp/$TarName"
Remove-Item $TarPath -ErrorAction SilentlyContinue
Write-Host "同步完成（VM -> 宿主机）。可检查 diff 后执行 .\scripts\sync_push.ps1 推到 GitHub。" -ForegroundColor Green
