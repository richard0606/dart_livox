#!/usr/bin/env bash
# 精度评测：回放数据集 -> 记录里程计 -> 计算轨迹长度与闭合误差
#
# 用法:
#   ./eval_closure.sh <数据集目录> [标签]
#   例: ./eval_closure.sh ~/bags/1031_0153_room1 tune_v2
#
# 输出: ~/bags/<数据集>/eval_<标签>.txt 与 trajectory_<标签>.csv
set -uo pipefail

DS="${1:-}"
TAG="${2:-$(date +%H%M)}"

if [ -z "$DS" ] || [ ! -d "$DS" ]; then
  echo "用法: $0 <数据集目录> [标签]"
  ls -1 "${BAGS_ROOT:-$HOME/bags}" 2>/dev/null | sed 's/^/  /'
  exit 1
fi

CSV="$DS/trajectory_${TAG}.csv"
OUT="$DS/eval_${TAG}.txt"

set +u
source /opt/ros/humble/setup.bash
[ -f "$HOME/ws_livox/install/setup.bash" ] && source "$HOME/ws_livox/install/setup.bash"
set -u

# 清理可能残留的实例，避免话题混数据
for p in $(ps -eo pid,args | grep -E "[f]astlio_mapping" | awk '{print $1}'); do
  kill -9 "$p" 2>/dev/null
done
sleep 1

rm -f "$CSV"
nohup python3 "$HOME/ws_livox/tools/traj_recorder.py" "$CSV" > /tmp/traj_rec.log 2>&1 &
REC_PID=$!
sleep 2

echo "回放中（约 60s），标签=$TAG"
"$HOME/ws_livox/tools/replay.sh" "$DS" > /tmp/eval_replay.log 2>&1 &
REPLAY_PID=$!

# 等回放结束（replay.sh 会一直 wait fast_lio，用日志判断）
for _ in $(seq 1 120); do
  grep -q "回放结束" /tmp/eval_replay.log 2>/dev/null && break
  sleep 2
done
sleep 3

kill -INT "$REC_PID" 2>/dev/null
sleep 1
kill -9 "$REC_PID" 2>/dev/null
for p in $(ps -eo pid,args | grep -E "[f]astlio_mapping|[r]viz2" | awk '{print $1}'); do
  kill -9 "$p" 2>/dev/null
done

if [ ! -s "$CSV" ]; then
  echo "[错误] 没有采到轨迹，看 /tmp/eval_replay.log"
  exit 1
fi

python3 - "$CSV" "$OUT" "$TAG" <<'PY'
import sys, numpy as np
csv, out, tag = sys.argv[1], sys.argv[2], sys.argv[3]
d = np.loadtxt(csv, delimiter=",", skiprows=1)
if d.ndim == 1: d = d[None, :]
x, y, z = d[:,1], d[:,2], d[:,3]
L = float(np.sum(np.linalg.norm(np.diff(np.stack([x,y,z],1), axis=0), axis=1)))
closed = float(np.linalg.norm([x[-1]-x[0], y[-1]-y[0], z[-1]-z[0]]))
ratio = closed / L * 100 if L > 0 else 0
lines = [
    f"标签            {tag}",
    f"采样点          {len(d)}",
    f"轨迹长度        {L:.2f} m",
    f"起点            ({x[0]:.2f}, {y[0]:.2f}, {z[0]:.2f})",
    f"终点            ({x[-1]:.2f}, {y[-1]:.2f}, {z[-1]:.2f})",
    f"闭合误差        {closed:.2f} m",
    f"闭合误差率      {ratio:.2f} %",
    f"z 终点偏差      {z[-1]-z[0]:.2f} m",
    f"z 起伏幅度      {z.max()-z.min():.2f} m",
]
open(out,"w").write("\n".join(lines)+"\n")
print("\n".join(lines))
PY

echo "结果已存: $OUT"
