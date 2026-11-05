#!/usr/bin/env bash
# 数据集体检：话题、时长、消息数、帧率是否正常
#
# 用法:
#   ./check_bag.sh              # 列出所有数据集并体检
#   ./check_bag.sh <目录>       # 只体检一个
set -uo pipefail

ROOT="${BAGS_ROOT:-$HOME/bags}"

set +u
source /opt/ros/humble/setup.bash
[ -f "$HOME/ws_livox/install/setup.bash" ] && source "$HOME/ws_livox/install/setup.bash"
set -u

check_one() {
  local ds="$1"
  local bag
  bag=$(find "$ds" -maxdepth 2 -name metadata.yaml -printf '%h\n' 2>/dev/null | head -1)
  echo "=== $(basename "$ds") ==="
  if [ -z "$bag" ]; then
    echo "  [缺失] 没有 bag"
    return
  fi
  ros2 bag info "$bag" 2>&1 | grep -E "Bag size|Duration|Messages|Topic:|Count" \
    | sed 's/^/  /'
  if [ -f "$ds/meta.yaml" ]; then
    echo "  meta:"
    grep -E "^(scene|location|path|motion|notes):" "$ds/meta.yaml" | sed 's/^/    /'
  else
    echo "  [警告] 缺 meta.yaml"
  fi
  if [ -d "$ds/PCD" ]; then
    echo "  PCD: $(ls -1 "$ds/PCD" | wc -l) 个文件"
  fi
  echo
}

if [ $# -gt 0 ]; then
  check_one "$1"
else
  if [ ! -d "$ROOT" ]; then
    echo "没有 $ROOT"
    exit 0
  fi
  n=0
  for d in "$ROOT"/*/; do
    [ -d "$d" ] || continue
    check_one "${d%/}"
    n=$((n+1))
  done
  [ "$n" -eq 0 ] && echo "$ROOT 下还没有数据集"
fi
