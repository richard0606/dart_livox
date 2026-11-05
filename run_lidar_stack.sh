#!/usr/bin/env bash
# DART 雷达整套启动：Livox MID-360 驱动 + FAST-LIO + 目标检测
#
# 由 dart_lidar.service 调用；也可以手动运行。
# 日志写到 ~/ws_livox/lidar_logs/。

set -u

# 剔除 MVS 自带的旧版 libusb（与视觉工作区同样的坑）
export LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' \
  | grep -v '^/opt/MVS/lib' | grep -v '^$' | paste -sd:)"
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}"

set +u
source /opt/ros/humble/setup.bash
source "$HOME/ws_livox/install/setup.bash"
set -u

LOG_DIR="$HOME/ws_livox/lidar_logs"
mkdir -p "$LOG_DIR"

cleanup() {
  trap - SIGTERM SIGINT
  kill $(jobs -p) 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup SIGTERM SIGINT

echo "[lidar] starting livox driver"
ros2 launch livox_ros_driver2 msg_MID360_launch.py > "$LOG_DIR/livox.log" 2>&1 &
sleep 4

echo "[lidar] starting FAST-LIO"
ros2 launch fast_lio mapping.launch.py rviz:=false > "$LOG_DIR/fastlio.log" 2>&1 &
sleep 8

echo "[lidar] starting dart_lidar_targets"
ros2 launch lidar_sector_tools dart_lidar_targets.launch.py > "$LOG_DIR/targets.log" 2>&1 &

wait
