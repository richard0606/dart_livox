#!/usr/bin/env bash
# MID-360 数据采集：录包 + 自动生成元数据
#
# 用法:
#   ./record.sh <场景名>        例: ./record.sh room1
#
# 走法要求:
#   起点静止 3 秒(IMU 初始化) -> 慢走一圈(转两个弯) -> 回起点静止 3 秒 -> Ctrl-C
set -uo pipefail

SCENE="${1:-}"
ROOT="${BAGS_ROOT:-$HOME/bags}"

if [ -z "$SCENE" ]; then
  echo "用法: $0 <场景名>"
  echo "  例: $0 room1"
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
[ -f "$HOME/ws_livox/install/setup.bash" ] && source "$HOME/ws_livox/install/setup.bash"
set -u

# --- 前置检查 ---
TMPLOG=$(mktemp)
timeout 5 ros2 topic hz /livox/lidar > "$TMPLOG" 2>&1 || true
if ! grep -q "average rate" "$TMPLOG"; then
  rm -f "$TMPLOG"
  echo "[错误] /livox/lidar 没有数据。先启动驱动："
  echo "  ros2 launch livox_ros_driver2 msg_MID360_launch.py"
  exit 1
fi
rm -f "$TMPLOG"

mkdir -p "$ROOT"
AVAIL=$(df -BG --output=avail "$ROOT" | tail -1 | tr -dc '0-9')
if [ "${AVAIL:-0}" -lt 5 ]; then
  echo "[错误] $ROOT 剩余空间 ${AVAIL}G，少于 5G"
  exit 1
fi

TS=$(date +%m%d_%H%M)
NAME="${TS}_${SCENE}"
DS="$ROOT/$NAME"
if [ -e "$DS" ]; then
  echo "[错误] 目录已存在: $DS"
  exit 1
fi
mkdir -p "$DS"

LIDAR_IP=$(python3 -c "
import json,os
p=os.path.expanduser('~/ws_livox/src/livox_ros_driver2/config/MID360_config.json')
print(json.load(open(p))['lidar_configs'][0]['ip'])
" 2>/dev/null || echo unknown)
LINK=$(cat /sys/class/net/enp8s0/speed 2>/dev/null || echo -1)
[ "${LINK:--1}" -lt 0 ] 2>/dev/null && LINK=unknown

cat <<EOF
==========================================
数据集: $NAME
目录:   $DS
雷达 IP: $LIDAR_IP    链路: ${LINK}Mb/s
剩余空间: ${AVAIL}G
------------------------------------------
现在开始录制。请按以下步骤走:
  1. 原地静止 3 秒
  2. 慢走一圈，转两个弯，尽量回到起点
  3. 回到起点后再静止 3 秒
  4. 按 Ctrl-C 结束
==========================================
EOF
sleep 2

# --- 录制 ---
trap ':' INT
ros2 bag record -s mcap -o "$DS/bag" /livox/lidar /livox/imu &
REC_PID=$!
wait $REC_PID 2>/dev/null
trap - INT

if [ ! -d "$DS/bag" ]; then
  echo "[错误] 录制失败，没有生成 bag 目录"
  rm -rf "$DS"
  exit 1
fi

# --- 统计与元数据 ---
sleep 1
ros2 bag info "$DS/bag" > "$DS/bag_info.txt" 2>&1
DUR=$(awk '/Duration:/{gsub(/s/,"",$2); printf "%.2f", $2}' "$DS/bag_info.txt" | head -1)
MSGS=$(awk '/Messages:/{print $2}' "$DS/bag_info.txt" | head -1)
SIZE=$(du -sh "$DS/bag" | cut -f1)

cat > "$DS/meta.yaml" <<EOF
# 采集元数据，回放/对比实验前先补全下面几项
dataset: $NAME
scene: $SCENE
recorded_at: $(date -Iseconds)
operator: ${USER}
location: ""            # 例: 实验室 3 楼走廊
duration_sec: ${DUR:-0}
messages: ${MSGS:-0}
bag_size: $SIZE
topics:
  - /livox/lidar
  - /livox/imu
lidar_ip: $LIDAR_IP
host_ip: 192.168.1.50
link_speed_mbps: $LINK
motion: "hand-held slow walk"     # hand-held / cart / backpack
path: closed-loop                  # closed-loop / open
ground_truth: none                 # none / total-station / RTK
notes: ""                          # 动态物体、电梯、楼梯、光照等
EOF

echo
echo "=========================================="
echo "完成: $DS"
echo "  时长: ${DUR:-?}s   消息: ${MSGS:-?}   大小: $SIZE"
echo "  元数据: $DS/meta.yaml  (请补全 location / path / notes)"
echo "  回放:   ~/ws_livox/tools/replay.sh $DS"
echo "=========================================="
