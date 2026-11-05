#!/usr/bin/env bash
# MID-360 主机侧网络配置 + 雷达探测
# 用法: ./net_mid360.sh [网卡名]
set -uo pipefail

IFACE="${1:-enp8s0}"
HOST_IP="192.168.1.50"
CON_NAME="mid360"
SUDO() { printf '%s\n' "${SUDO_PASS:-}" | sudo -S "$@" 2>/dev/null; }

if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "[错误] 网卡 $IFACE 不存在，当前可用网卡："
  ip -br link | awk '{print "  " $1}'
  exit 1
fi

# 用 NetworkManager 建持久静态连接，避免 DHCP 反复重置接口把地址冲掉
if nmcli -t -f NAME connection show | grep -qx "$CON_NAME"; then
  echo "[跳过] 连接 $CON_NAME 已存在"
else
  echo "[创建] $CON_NAME -> $IFACE $HOST_IP/24"
  SUDO nmcli con add type ethernet con-name "$CON_NAME" ifname "$IFACE" \
    ipv4.method manual ipv4.addresses "$HOST_IP/24" ipv6.method ignore
  SUDO nmcli con mod "$CON_NAME" ipv4.never-default yes connection.autoconnect yes
fi

nmcli -t -f NAME,DEVICE connection show --active | grep -q "^$CON_NAME:" || SUDO nmcli con up "$CON_NAME"
sleep 1

echo "--- $IFACE 当前地址 ---"
ip -4 -br addr show "$IFACE"

echo "--- 抓取 8 秒雷达 UDP 包，识别雷达 IP ---"
SUDO timeout 8 tcpdump -n -i "$IFACE" 'udp and portrange 56100-56500' -c 4

echo "--- 说明 ---"
echo "上面出现 192.168.1.1xx > ... 的行，1xx 即雷达 IP，"
echo "填进 src/livox_ros_driver2/config/MID360_config.json 的 lidar_configs[0].ip"
echo "注意：雷达可能会 ARP who-has 192.168.1.5，那是雷达侧记录的旧目标主机，驱动连上后会自动纠正"
