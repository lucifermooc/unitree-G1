#!/usr/bin/env bash
# 管理 Thor 开机自启（systemd 系统服务，以普通用户身份运行）：
#   g1-robot.service         -> g1_autostart.sh robot     (robot.launch.py)
#   g1-semantic-map.service  -> g1_autostart.sh semantic  (semantic_map.launch.py)
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

set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
G1_WS="${G1_WS:-/opt/G1/lighting_ws}"
RUN_USER="${G1_USER:-${SUDO_USER:-$(id -un)}}"
UNITS=(g1-robot.service g1-semantic-map.service)
UNIT_DIR=/etc/systemd/system
SYSCTL_SRC="$G1_WS/src/robot_bringup/system/60-dds-buffers.conf"
LOG_DIR="${G1_LOG_DIR:-/opt/G1/logs}"

as_root() { if [ "$(id -u)" -eq 0 ]; then "$@"; else sudo "$@"; fi; }
as_user() { if [ "$(id -u)" -eq 0 ]; then sudo -u "$RUN_USER" "$@"; else "$@"; fi; }

unit_robot() {
  cat <<EOF
[Unit]
Description=G1 robot stack (ros2 launch robot_bringup robot.launch.py, DDS cyclonedds_g1.xml)
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=$RUN_USER
ExecStart=/bin/bash $SCRIPT_DIR/g1_autostart.sh robot
# 与终端里 Ctrl-C 相同：SIGINT 发给本服务的全部进程（含 launch_manager 拉起的定位/导航/建图），
# 30 s 未退出的再 SIGKILL，不会留下孤儿进程。
KillSignal=SIGINT
TimeoutStopSec=30
# 不自动重启：stop_all 手动停栈后服务不会自己再起一套，避免两套同名进程。
Restart=no

[Install]
WantedBy=multi-user.target
EOF
}

unit_semantic() {
  cat <<EOF
[Unit]
Description=G1 semantic map (ros2 launch semantic_map_ros semantic_map.launch.py)
Wants=network-online.target
After=network-online.target docker.service g1-robot.service

[Service]
Type=simple
User=$RUN_USER
ExecStart=/bin/bash $SCRIPT_DIR/g1_autostart.sh semantic
KillSignal=SIGINT
TimeoutStopSec=20
Restart=no

[Install]
WantedBy=multi-user.target
EOF
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

case "${1:-}" in
  install)
    [ "$RUN_USER" = root ] && { echo "run as the robot user (e.g. unitree), not root" >&2; exit 1; }
    unit_robot | as_root tee "$UNIT_DIR/g1-robot.service" >/dev/null
    unit_semantic | as_root tee "$UNIT_DIR/g1-semantic-map.service" >/dev/null
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
    as_root systemctl start "${UNITS[@]}"
    ;;
  stop)
    as_root systemctl stop "${UNITS[@]}"
    cleanup_ros
    ;;
  restart)
    as_root systemctl stop "${UNITS[@]}"
    cleanup_ros
    sleep 2
    as_root systemctl start "${UNITS[@]}"
    systemctl --no-pager --lines=0 status "${UNITS[@]}" || true
    ;;
  status)
    systemctl --no-pager --lines=3 status "${UNITS[@]}" || true
    echo; ls -l "$LOG_DIR"/*_latest.log 2>/dev/null || echo "no logs in $LOG_DIR yet"
    ;;
  log)
    tail -n 100 -F "$LOG_DIR/${2:-robot}_latest.log"
    ;;
  units)
    echo "# $UNIT_DIR/g1-robot.service"; unit_robot
    echo; echo "# $UNIT_DIR/g1-semantic-map.service"; unit_semantic
    echo; echo "# /etc/sudoers.d/g1-autostart"; sudoers_rule
    ;;
  *)
    sed -n '2,15p' "$0"; exit 2
    ;;
esac
