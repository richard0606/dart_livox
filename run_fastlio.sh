#!/usr/bin/env bash
# 启动 FAST-LIO
# 原因：~/.bashrc 把 /opt/MVS/lib 放在 LD_LIBRARY_PATH 最前面，MVS 自带的旧版
# libusb-1.0.so.0 会被优先加载，导致 PCL 报 undefined symbol: libusb_set_option。
# 这里只对本进程剔除 MVS 路径，不影响其它项目用 MVS。
set -o pipefail

export LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' \
  | grep -v '^/opt/MVS/lib' | grep -v '^$' | paste -sd:)"
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH}"

set +u
source /opt/ros/humble/setup.bash
[ -f "$HOME/ws_livox/install/setup.bash" ] && source "$HOME/ws_livox/install/setup.bash"

exec ros2 launch fast_lio mapping.launch.py "$@"
