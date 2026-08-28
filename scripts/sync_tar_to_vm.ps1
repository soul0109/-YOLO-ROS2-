# Sync working tree to the VM over SSH without requiring a git commit.
# Needs: OpenSSH client + sync.env (see sync.env.example)
# Usage: .\scripts\sync_tar_to_vm.ps1

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

Write-Host "打包源码（排除 build/install/log/.git）..."
Push-Location $Root
try {
    & tar -czf $TarPath `
        --exclude='./ros2_ws/build' `
        --exclude='./ros2_ws/install' `
        --exclude='./ros2_ws/log' `
        --exclude='./.git' `
        --exclude='./.cursor' `
        --exclude='**/__pycache__' `
        --exclude='**/.pytest_cache' `
        .
    if ($LASTEXITCODE -ne 0) { throw "tar failed: $LASTEXITCODE" }
} finally {
    Pop-Location
}

Write-Host "上传到 $($env:VM_SSH):/tmp/$TarName ..."
& scp $TarPath "$($env:VM_SSH):/tmp/$TarName"
if ($LASTEXITCODE -ne 0) { throw "scp failed: $LASTEXITCODE" }

Write-Host "在虚拟机解压到 $VmRepo ..."
$Remote = @"
set -e
mkdir -p $VmRepo
cd $VmRepo
tar -xzf /tmp/$TarName
rm -f /tmp/$TarName
echo "VM sync OK: $VmRepo"
"@
& ssh $env:VM_SSH $Remote
if ($LASTEXITCODE -ne 0) { throw "ssh remote extract failed: $LASTEXITCODE" }

Remove-Item $TarPath -ErrorAction SilentlyContinue
Write-Host "同步完成（未走 git，虚拟机 .git 历史可能与工作区不一致；正式节点仍建议用 sync_push）。" -ForegroundColor Green
