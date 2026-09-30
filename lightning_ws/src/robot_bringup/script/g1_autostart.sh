#!/usr/bin/env bash
# Thor 开机自启入口（由 g1_service.sh 安装的 systemd 服务调用，也可以手动执行）：
#   g1_autostart.sh robot      等同于在配好 DDS 的终端里 ros2 launch robot_bringup robot.launch.py
#   g1_autostart.sh semantic   语义地图 semantic_map.launch.py（离线本地模型 + GPU）
# 启动参数在同目录 g1_autostart.env；日志在 /opt/G1/logs/<robot|semantic>_*.log（*_latest.log 指向最新一份）。

WHAT="${1:-}"
case "$WHAT" in
  robot|semantic) ;;
  *) echo "usage: $0 robot|semantic" >&2; exit 2 ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
G1_WS="${G1_WS:-/opt/G1/lighting_ws}"
G1_ROBOT_ARGS=""
G1_SEMANTIC_ARGS="use_fp16:=true model_name:=${HOME}/models/bge-m3"
# shellcheck source=g1_autostart.env
[ -f "$SCRIPT_DIR/g1_autostart.env" ] && source "$SCRIPT_DIR/g1_autostart.env"
# 本机专属覆盖（相机序列号等），不在仓库里，rsync 不会覆盖
G1_LOCAL_ENV="${G1_LOCAL_ENV:-/opt/G1/g1_autostart.local.env}"
[ -f "$G1_LOCAL_ENV" ] && source "$G1_LOCAL_ENV"

# ---------- 日志：每次启动一个文件，保留最近 10 份 ----------
LOG_DIR="${G1_LOG_DIR:-/opt/G1/logs}"
mkdir -p "$LOG_DIR" 2>/dev/null || LOG_DIR="$HOME/g1_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/${WHAT}_$(date +%Y%m%d_%H%M%S).log"
: > "$LOG"
ln -sfn "$LOG" "$LOG_DIR/${WHAT}_latest.log"
ls -1t "$LOG_DIR/${WHAT}"_2*.log 2>/dev/null | tail -n +11 | xargs -r rm -f
echo "g1_autostart $WHAT: log -> $LOG"   # 这一行进 journal，其余都写日志文件
exec >>"$LOG" 2>&1
log() { echo "[g1_autostart $(date +%T)] $*"; }
log "start $WHAT, user=$(id -un), ws=$G1_WS, local env: $([ -f "$G1_LOCAL_ENV" ] && echo "$G1_LOCAL_ENV" || echo none)"

# ---------- ROS + 工作区 + DDS（顺序与 2026-09 实机验证过的手动启动一致） ----------
# 不能只 source unitree_ros2/setup.sh：它把 CYCLONEDDS_URI 设成组播，同机点云会走网线（见 CLAUDE.md）。
# dds_env.sh 先 source 它拿到 RMW，再把 CYCLONEDDS_URI 改成 system/cyclonedds_g1.xml（spdp + 参与者上限 100）。
source /opt/ros/jazzy/setup.bash
source "$G1_WS/install/setup.bash"
source "$G1_WS/src/robot_bringup/system/dds_env.sh"
export PYTHONUNBUFFERED=1   # 日志按行落盘
export LANG="${LANG:-C.UTF-8}"
log "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-} ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-}"

# CycloneDDS 绑定的网卡（连机器人本体/雷达的网段）没有 IP 时所有节点都会起不来：一直等到它就绪。
DDS_XML="$G1_WS/src/robot_bringup/system/cyclonedds_g1.xml"
IFACE="$(grep -o 'NetworkInterface name="[^"]*"' "$DDS_XML" | head -1 | cut -d'"' -f2)"
if [ -n "$IFACE" ]; then
  waited=0
  until ip -o -4 addr show dev "$IFACE" 2>/dev/null | grep -q 'inet '; do
    [ $((waited % 30)) -eq 0 ] && log "waiting for DDS interface $IFACE to get an IPv4 address (${waited}s)"
    sleep 2; waited=$((waited + 2))
  done
  log "DDS interface $IFACE: $(ip -o -4 addr show dev "$IFACE" | awk '{print $4}')"
fi

# 60-dds-buffers.conf 没装时，建图/定位丢 IMU（get abnormal dt）；只告警不阻止启动。
rmem="$(cat /proc/sys/net/core/rmem_max 2>/dev/null || echo 0)"
[ "$rmem" -lt 33554432 ] && log "WARN net.core.rmem_max=$rmem < 33554432: run 'g1_service.sh install' to install system/60-dds-buffers.conf"

cd "$HOME"
if [ "$WHAT" = robot ]; then
  # lightning（定位/建图）的 glog 默认写 /tmp，重启即丢（2026-09-30 因此丢了当天复现漂移的逐帧日志）。
  # 改到持久目录，由 launch_manager 拉起的子进程继承；保留 7 天。
  export GLOG_log_dir="$LOG_DIR/glog"
  mkdir -p "$GLOG_log_dir"
  find "$GLOG_log_dir" -type f -mtime +7 -delete 2>/dev/null
  log "lightning glog -> $GLOG_log_dir"
  # 已有一套栈（手动 nohup 启动的）时再起一套会同名冲突：拒绝，先 stop_all 或 g1_service.sh restart。
  if pgrep -u "$(id -u)" -af "ros2 launch [r]obot_bringup robot.launch.py|lib/robot_bringup/[r]obot_status_manager_node"; then
    log "ERROR another robot stack is already running (above); stop it first: g1_service.sh restart"
    exit 1
  fi
  read -r -a args <<< "$G1_ROBOT_ARGS"
  log "exec ros2 launch robot_bringup robot.launch.py ${args[*]}"
  exec ros2 launch robot_bringup robot.launch.py "${args[@]}"
fi

# ---------- semantic ----------
if pgrep -u "$(id -u)" -af "ros2 launch [s]emantic_map_ros semantic_map.launch.py|lib/semantic_map_ros/[s]emantic_map_server"; then
  log "ERROR semantic_map is already running (above); stop it first: g1_service.sh restart"
  exit 1
fi
# Qdrant 容器（docker restart 策略自启）通常比本服务晚就绪；节点连接是惰性的，这里只是等它起来免得前几次搜索报错。
if [[ "$G1_SEMANTIC_ARGS" != *qdrant_path:=* ]]; then
  for i in $(seq 60); do
    (exec 3<>/dev/tcp/127.0.0.1/6333) 2>/dev/null && { log "Qdrant :6333 ready"; break; }
    [ "$i" -eq 60 ] && log "WARN Qdrant :6333 not reachable after 120 s, starting anyway (docker ps | grep qdrant)"
    sleep 2
  done
fi
export HF_HUB_OFFLINE=1   # 模型用本地目录 ~/models/bge-m3，不联网
read -r -a args <<< "$G1_SEMANTIC_ARGS"
log "exec ros2 launch semantic_map_ros semantic_map.launch.py ${args[*]}"
exec ros2 launch semantic_map_ros semantic_map.launch.py "${args[@]}"
