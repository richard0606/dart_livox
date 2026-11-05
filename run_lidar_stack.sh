#!/usr/bin/env bash
# DART 雷达整套启动：Livox MID-360 驱动 + FAST-LIO + 目标检测
#
# 由 dart_lidar.service 调用；也可以手动运行。
# 内置看门狗：每 5 秒检查三个节点，连续两次缺失就重启整套。
# 日志写到 ~/ws_livox/lidar_logs/，每次重启把上一份归档为 *.log.1。

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

rotate_logs() {
  for f in livox fastlio targets; do
    if [ -f "$LOG_DIR/$f.log" ]; then
      mv -f "$LOG_DIR/$f.log" "$LOG_DIR/$f.log.1" 2>/dev/null || true
    fi
  done
}

CHILD_PIDS=()

start_stack() {
  rotate_logs
  echo "[lidar] starting livox driver"
  ros2 launch livox_ros_driver2 msg_MID360_launch.py > "$LOG_DIR/livox.log" 2>&1 &
  CHILD_PIDS+=($!)
  sleep 4
  echo "[lidar] starting FAST-LIO"
  ros2 launch fast_lio mapping.launch.py rviz:=false > "$LOG_DIR/fastlio.log" 2>&1 &
  CHILD_PIDS+=($!)
  sleep 8
  echo "[lidar] starting dart_lidar_targets"
  ros2 launch lidar_sector_tools dart_lidar_targets.launch.py > "$LOG_DIR/targets.log" 2>&1 &
  CHILD_PIDS+=($!)
}

stop_stack() {
  for p in "${CHILD_PIDS[@]:-}"; do
    kill -INT "$p" 2>/dev/null || true
  done
  sleep 3
  for p in "${CHILD_PIDS[@]:-}"; do
    kill -TERM "$p" 2>/dev/null || true
  done
  wait 2>/dev/null || true
  CHILD_PIDS=()
}

cleanup() {
  trap - SIGTERM SIGINT
  stop_stack
  exit 0
}
trap cleanup SIGTERM SIGINT

node_present() {
  ros2 node list 2>/dev/null | grep -qx "$1"
}

MISS=0
while true; do
  start_stack
  MISS=0
  while true; do
    sleep 5
    missing=0
    for n in /livox_lidar_publisher /laser_mapping /dart_lidar_targets; do
      if ! node_present "$n"; then
        missing=1
        break
      fi
    done
    if [ "$missing" -eq 1 ]; then
      MISS=$((MISS + 1))
      echo "[lidar] watchdog: node missing (${MISS}/2)"
      if [ "$MISS" -ge 2 ]; then
        echo "[lidar] watchdog: restarting stack"
        stop_stack
        sleep 2
        break
      fi
    else
      MISS=0
    fi
  done
done
