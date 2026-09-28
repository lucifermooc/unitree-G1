# 两个流程脚本共用的函数
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAPS_DIR="$SIM_DIR/maps"
LOG_DIR="${LOG_DIR:-$SIM_DIR/logs}"
mkdir -p "$MAPS_DIR" "$LOG_DIR"
source "$SIM_DIR/env.sh"

BG_PIDS=()
start_bg() {  # start_bg <日志名> <命令...>   后台启动，日志写到 logs/<日志名>.log
  local name=$1; shift
  setsid "$@" > "$LOG_DIR/$name.log" 2>&1 &
  BG_PIDS+=($!)
  echo "  启动 $name（日志：logs/$name.log）"
}
stop_all() {
  for pid in "${BG_PIDS[@]}"; do kill -INT -- "-$pid" 2>/dev/null; done
  sleep 3
  for pid in "${BG_PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null; done
}
trap stop_all EXIT

wait_for() {  # wait_for <说明> <超时秒> <检查命令...>
  local what=$1 timeout=$2; shift 2
  for _ in $(seq 1 "$timeout"); do
    if "$@" >/dev/null 2>&1; then echo "  ✅ $what"; return 0; fi
    sleep 1
  done
  echo "  ❌ 等待超时：$what（看 $LOG_DIR 里的日志）"; exit 1
}
has_scan() { timeout 10 ros2 topic echo /scan --once --field range_max; }
has_tf() {  # tf2_echo 会一直运行，被 timeout 结束时返回非 0，所以先存输出再检查
  local out; out=$(timeout 10 ros2 run tf2_ros tf2_echo "$1" base_footprint 2>&1 || true)
  [[ $out == *Translation* ]]
}
