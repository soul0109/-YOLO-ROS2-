#!/usr/bin/env bash
# 一次性配置：evdev 键盘 + VM DNS（apt 能下载）
#
# 用法：bash ~/inspection-robot/scripts/setup_keyboard_evdev.sh

set -eo pipefail

echo ">>> [1/3] 修复 DNS（VM 常见：ens33 无 DNS → apt 失败）..."
IFACE=$(ip route show default | awk '{print $5; exit}')
if [[ -n "$IFACE" ]]; then
  sudo resolvectl dns "$IFACE" 8.8.8.8 114.114.114.114
  sudo resolvectl domain "$IFACE" '~.'
  echo "    已设置 $IFACE DNS → 8.8.8.8"
fi
sudo mkdir -p /etc/systemd/resolved.conf.d
sudo tee /etc/systemd/resolved.conf.d/inspection-robot-dns.conf >/dev/null <<'EOF'
[Resolve]
DNS=8.8.8.8 114.114.114.114
FallbackDNS=1.1.1.1
EOF
sudo systemctl restart systemd-resolved

echo ">>> [2/3] 安装 python3-evdev ..."
sudo apt-get update -qq
sudo apt-get install -y python3-evdev

echo ">>> [3/3] 将用户 ${USER} 加入 input 组 ..."
sudo usermod -aG input "${USER}"

echo ""
echo "=========================================="
echo "  完成。请 注销并重新登录 VM（或 reboot）。"
echo "  重登后验证："
echo "    groups | grep input"
echo "    python3 -c \"import evdev; print('evdev ok')\""
echo "  然后："
echo "    bash ~/inspection-robot/scripts/run_gazebo_teleop.sh --build"
echo "  启动时应看到 [evdev 模式]"
echo "=========================================="
echo ""
echo "若暂不想重登，新开终端执行："
echo "  newgrp input"
echo "  bash ~/inspection-robot/scripts/run_gazebo_teleop.sh --build"
