#!/bin/bash
# 无 Thor 背包的 G1（Orin 30524）在两份 Nav2 本机覆盖之间切换控制器（装到 /opt/G1/nav_controller_switch.sh）：
#   rpp  -> /opt/G1/nav2_params.local.yaml       （现状：RPP）
#   shim -> /opt/G1/nav2_params.shim.local.yaml  （远处 MPPI、近目标 RPP，aid_nav_shim_controller）
# 只改 /opt/G1/g1_autostart.local.env 里 nav_params_overlay:= 的路径，改前把 local.env 备份到 /opt/G1/backup_switch/。
# 切换要重启 g1-robot 才生效（--restart 时顺带重启）；重启后定位在原点初始化，先把机器人放回原点或之后在网页重定位。
#
#   bash /opt/G1/nav_controller_switch.sh status
#   bash /opt/G1/nav_controller_switch.sh shim [--restart]
#   bash /opt/G1/nav_controller_switch.sh rpp  [--restart]
set -e
ENV_FILE=/opt/G1/g1_autostart.local.env
RPP=/opt/G1/nav2_params.local.yaml
SHIM=/opt/G1/nav2_params.shim.local.yaml
SERVICE=/opt/G1/lighting_ws/install/robot_bringup/share/robot_bringup/script/g1_service.sh

current() {
  grep -o 'nav_params_overlay:=[^ "]*' "$ENV_FILE" | head -1 | cut -d= -f2
}

case "${1:-status}" in
  status)
    cur=$(current)
    case "$cur" in
      "$RPP") echo "当前：rpp（$cur）" ;;
      "$SHIM") echo "当前：shim（$cur）" ;;
      *) echo "当前：其他（${cur:-未设置}）" ;;
    esac
    # 正在跑的 controller_server 实际加载的插件（改了 local.env 还没重启时与上一行不一致）
    # </dev/null：终端里跑时 g1_docker.sh 会给 docker exec 加 -it，而 timeout 把它放进后台进程组，一碰终端就被
    # SIGTTOU 停住，timeout 和 Ctrl-C 都收不掉（2026-10-26 实测卡死）；不给 stdin 就不带 -it。
    echo -n "运行中：controller_server FollowPath.plugin = "
    timeout -k 5 20 bash /opt/G1/docker/g1_docker.sh run bash -c \
      'ros2 param get /controller_server FollowPath.plugin 2>/dev/null | sed "s/^String value is: //"' </dev/null \
      || echo "（读不到，栈没在跑？）"
    ;;
  rpp|shim)
    target=$RPP
    [ "$1" = shim ] && target=$SHIM
    [ -f "$target" ] || { echo "缺少 $target" >&2; exit 1; }
    grep -q 'nav_params_overlay:=' "$ENV_FILE" || { echo "$ENV_FILE 里没有 nav_params_overlay:=，不改" >&2; exit 1; }
    cur=$(current)
    if [ "$cur" = "$target" ]; then
      echo "已经是 $1（$target）"
    else
      mkdir -p /opt/G1/backup_switch
      cp -p "$ENV_FILE" "/opt/G1/backup_switch/g1_autostart.local.env.$(date +%Y%m%d_%H%M%S)"
      sed -i "s#nav_params_overlay:=[^ \"]*#nav_params_overlay:=$target#" "$ENV_FILE"
      echo "已切到 $1：nav_params_overlay:=$(current)"
    fi
    if [ "${2:-}" = --restart ]; then
      bash "$SERVICE" restart
    else
      echo "重启 g1-robot 后生效：bash $SERVICE restart（重启后定位在原点初始化）"
    fi
    ;;
  *)
    echo "用法: $0 status | rpp [--restart] | shim [--restart]" >&2
    exit 1
    ;;
esac
