#!/usr/bin/env bash
# 离线回放数据集并跑 FAST-LIO
#
# 用法:
#   ./replay.sh <数据集目录> [倍速]
#   例: ./replay.sh ~/bags/1029_2345_room1
#       ./replay.sh ~/bags/1029_2345_room1 0.5    # 半速，慢走数据更稳
#
# 说明:
#   - 用 --clock 让 FAST-LIO 走仿真时间，保证可复现
#   - cd 到数据集目录，PCD 会保存在 <数据集>/PCD/ 下
set -uo pipefail

DS="${1:-}"
RATE="${2:-1.0}"

if [ -z "$DS" ] || [ ! -d "$DS" ]; then
  echo "用法: $0 <数据集目录> [倍速]"
  echo "可用数据集:"
  ls -1 "${BAGS_ROOT:-$HOME/bags}" 2>/dev/null | sed 's/^/  /'
  exit 1
fi

BAG=$(find "$DS" -maxdepth 2 -name metadata.yaml -printf '%h\n' 2>/dev/null | head -1)
if [ -z "$BAG" ]; then
  echo "[错误] $DS 下找不到 bag（metadata.yaml）"
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
[ -f "$HOME/ws_livox/install/setup.bash" ] && source "$HOME/ws_livox/install/setup.bash"
set -u

cd "$DS" || exit 1
rm -f "$DS/test.pcd"

echo "数据集: $DS"
echo "bag:    $BAG"
echo "倍速:   $RATE"
echo "日志:   $DS/fastlio.log"
echo "------------------------------------------"

nohup "$HOME/ws_livox/run_fastlio.sh" use_sim_time:=true > "$DS/fastlio.log" 2>&1 &
LIO_PID=$!
echo "FAST-LIO pid=$LIO_PID，等待初始化..."
sleep 8

if ! kill -0 "$LIO_PID" 2>/dev/null; then
  echo "[错误] FAST-LIO 启动失败，看日志:"
  tail -15 "$DS/fastlio.log"
  exit 1
fi

trap ':' INT
ros2 bag play "$BAG" --clock --rate "$RATE"
trap - INT

echo
echo "------------------------------------------"
echo "回放结束，保存地图（调 /map_save 服务）..."
ros2 service call /map_save std_srvs/srv/Trigger "{}" 2>&1 | tail -2
sleep 2
if [ -f "$DS/test.pcd" ]; then
  echo "  地图已保存: $DS/test.pcd  ($(du -h "$DS/test.pcd" | cut -f1))"
else
  echo "  [警告] 没有生成 test.pcd"
fi
echo "  FAST-LIO 仍在跑，看 RViz；结束: kill -INT $LIO_PID"
echo "------------------------------------------"
wait "$LIO_PID" 2>/dev/null
