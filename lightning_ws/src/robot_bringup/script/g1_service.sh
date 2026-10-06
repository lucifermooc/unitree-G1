#!/usr/bin/env bash
# 管理 Thor 开机自启（systemd 系统服务，以普通用户身份运行）。unit 文件在 system/g1-robot.service、
# system/g1-semantic-map.service，install 时拷到 /etc/systemd/system/；它们的 ExecStart 调 g1_autostart.sh
# （负责 source ROS/install/dds_env.sh、等 DDS 网卡、写日志，systemd 自己做不了这些）。
#
# 用法（以 unitree 用户执行，需要时自动 sudo）：
#   g1_service.sh install     安装并设为开机自启，同时装 DDS 内核缓冲 system/60-dds-buffers.conf，
#                             以及 /etc/sudoers.d/g1-autostart（只放行启停这两个服务免密）；不会立即启动
#   g1_service.sh restart     停掉服务和所有残留 ROS 进程（含手动 nohup 起的栈），再启动服务。部署后重启用这个
#   g1_service.sh stop        停服务 + 清理残留（等同 stop_all），下次开机仍会自启
#   g1_service.sh start       启动服务
#   g1_service.sh status      服务状态和日志位置
#   g1_service.sh log [robot|semantic]   跟踪最新日志（默认 robot）
#   g1_service.sh uninstall   取消开机自启并删除服务
#   g1_service.sh units       只打印将要安装的 unit 内容
#
# 部署后 src 会删除，所以执行 install 里的这份：
#   bash /opt/G1/lighting_ws/install/robot_bringup/share/robot_bringup/script/g1_service.sh <命令>

set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
G1_WS="${G1_WS:-/opt/G1/lighting_ws}"
RUN_USER="${G1_USER:-${SUDO_USER:-$(id -un)}}"
UNITS=(g1-robot.service g1-semantic-map.service)
UNIT_DIR=/etc/systemd/system
# 服务固定执行 install 里的入口，与本脚本从哪里执行无关：unit 里写 src 路径的话，删 src 后开机自启
# 报 "g1_autostart.sh: No such file or directory"（2026-10-03 128.146）。
G1_SHARE="$G1_WS/install/robot_bringup/share/robot_bringup"
AUTOSTART="$G1_SHARE/script/g1_autostart.sh"
SYSCTL_SRC="$G1_SHARE/system/60-dds-buffers.conf"
LOG_DIR="${G1_LOG_DIR:-/opt/G1/logs}"

as_root() { if [ "$(id -u)" -eq 0 ]; then "$@"; else sudo "$@"; fi; }
as_user() { if [ "$(id -u)" -eq 0 ]; then sudo -u "$RUN_USER" "$@"; else "$@"; fi; }

# unit 文件就是 system/g1-*.service（随包装进 install），这里只按实际用户/工作区替换后输出。
unit_file() {
  sed -e "s|^User=.*|User=$RUN_USER|" -e "s|/opt/G1/lighting_ws|$G1_WS|g" "$G1_SHARE/system/$1"
}

# 只放行启停这两个服务的 systemctl 命令免密（g1_service.sh start/stop/restart 与单独重启某一个），不开放整个 sudo。
# 参数逐条写死，不用通配符：sudoers 里 g1-* 也能匹配 "g1-x ssh.service"，等于放开任意服务启停。
sudoers_rule() {
  local sc; sc="$(command -v systemctl)"
  local cmds=() verb target
  for verb in start stop restart; do
    for target in "${UNITS[*]}" "${UNITS[0]}" "${UNITS[1]}"; do cmds+=("$sc $verb $target"); done
  done
  echo "# 由 g1_service.sh install 生成：$RUN_USER 免密启停 G1 自启服务（仅限以下命令）"
  local IFS=,
  echo "$RUN_USER ALL=(root) NOPASSWD: ${cmds[*]}"
}

cleanup_ros() {
  as_user python3 "$SCRIPT_DIR/stop_ros_processes.py" --workspace "$G1_WS"
}

# 旧版 install 写的 unit 指向 src；只告警不阻止（unit 要 sudo 重装）。
check_units() {
  local u unit
  for u in "${UNITS[@]}"; do
    unit="$(systemctl cat "$u" 2>/dev/null || true)"   # 不用管道接 grep -q：pipefail 下 SIGPIPE 会误报
    [[ "$unit" == *"ExecStart=/bin/bash $AUTOSTART "* ]] \
      || echo "WARN $u does not run $AUTOSTART (old unit pointing at src?); re-run: bash $G1_SHARE/script/g1_service.sh install" >&2
  done
}

case "${1:-}" in
  install)
    [ "$RUN_USER" = root ] && { echo "run as the robot user (e.g. unitree), not root" >&2; exit 1; }
    for f in "$AUTOSTART" "$G1_SHARE/system/dds_env.sh" "$SYSCTL_SRC" "${UNITS[@]/#/$G1_SHARE/system/}"; do
      [ -f "$f" ] || { echo "missing $f: colcon build --packages-select robot_bringup first" >&2; exit 1; }
    done
    for u in "${UNITS[@]}"; do unit_file "$u" | as_root tee "$UNIT_DIR/$u" >/dev/null; done
    tmp="$(mktemp)"; sudoers_rule > "$tmp"
    # 写坏的 sudoers 会让 sudo 整个不可用：先 visudo 校验再装
    if as_root visudo -cf "$tmp" >/dev/null; then
      as_root install -m 0440 -o root -g root "$tmp" /etc/sudoers.d/g1-autostart
    else
      echo "WARN sudoers rule failed visudo check, not installed:" >&2; cat "$tmp" >&2
    fi
    rm -f "$tmp"
    as_root install -m 644 "$SYSCTL_SRC" /etc/sysctl.d/60-dds-buffers.conf
    as_root sysctl -p /etc/sysctl.d/60-dds-buffers.conf
    as_root mkdir -p "$LOG_DIR"
    as_root chown "$RUN_USER:" "$LOG_DIR"
    as_root systemctl daemon-reload
    as_root systemctl enable "${UNITS[@]}"
    echo "installed: ${UNITS[*]} (user $RUN_USER), enabled at boot."
    echo "not started now; start with: $0 restart (also stops any stack started by hand)"
    ;;
  uninstall)
    as_root systemctl disable --now "${UNITS[@]}" || true
    for u in "${UNITS[@]}"; do as_root rm -f "$UNIT_DIR/$u"; done
    as_root rm -f /etc/sudoers.d/g1-autostart
    as_root systemctl daemon-reload
    echo "removed ${UNITS[*]} (/etc/sysctl.d/60-dds-buffers.conf kept)"
    ;;
  start)
    check_units
    as_root systemctl start "${UNITS[@]}"
    ;;
  stop)
    as_root systemctl stop "${UNITS[@]}"
    cleanup_ros
    ;;
  restart)
    check_units
    as_root systemctl stop "${UNITS[@]}"
    cleanup_ros
    sleep 2
    as_root systemctl start "${UNITS[@]}"
    systemctl --no-pager --lines=0 status "${UNITS[@]}" || true
    ;;
  status)
    check_units
    systemctl --no-pager --lines=3 status "${UNITS[@]}" || true
    echo; ls -l "$LOG_DIR"/*_latest.log 2>/dev/null || echo "no logs in $LOG_DIR yet"
    ;;
  log)
    tail -n 100 -F "$LOG_DIR/${2:-robot}_latest.log"
    ;;
  units)
    for u in "${UNITS[@]}"; do echo "# $UNIT_DIR/$u"; unit_file "$u"; echo; done
    echo; echo "# /etc/sudoers.d/g1-autostart"; sudoers_rule
    ;;
  *)
    sed -n '2,15p' "$0"; exit 2
    ;;
esac
