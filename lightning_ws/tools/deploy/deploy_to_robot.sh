#!/bin/bash
# 把本机源码部署到一台 G1 Thor：同步（只传内容不同的文件）→ 停栈 → 编译 → 启动 → 检查。
#
#   tools/deploy/deploy_to_robot.sh <机器人IP> [包名 ...]
#   默认包：lightning semantic_map_ros robot_bringup
#
# 前提：机器人上已执行过 install/robot_bringup/share/robot_bringup/script/g1_service.sh install（启停服务免密）；
#       ssh 免密登录 unitree@<IP>。
# 部署后 src 会删除：停/启服务用 install 里的 g1_service.sh；编译时 source install/setup.bash，
# 让不在 src 里的依赖包（lightning、消息包等）从 install 找到。
# 注意：
# - rsync 用 -c 按内容比较、不带 -t：只有内容变了的文件被传过去（修改时间 = 传输时刻），其余文件原样不动，只重编改动部分。
# - 必须排除 bin/：本机 x86 二进制绝不能覆盖到机器上（ARM）。
# - 排除 lightning-lm/thirdparty：机器上那份是编译用的原件，本机那份是从上游补的。
# - 排除 robot_bringup/script/g1_humble_env.sh：机器上有自己的版本（本机开发环境脚本，机器上不用）。
# - 编译期间定位栈必须停掉（install 会原地覆盖正在使用的 .so），编 lightning 要 PYTHONNOUSERSITE=1。
# - 重启后定位在原点初始化：执行前把机器人放在原点、朝向与建图时一致。
set -e
IP=${1:?用法: $0 <机器人IP> [包名 ...]}
shift
PKGS=${*:-lightning semantic_map_ros robot_bringup}
WS=/opt/G1/lighting_ws
SRC=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../src" && pwd)
SSH="ssh -o BatchMode=yes -o ConnectTimeout=8 unitree@$IP"

dir_of() { case $1 in lightning) echo lightning-lm ;; *) echo "$1" ;; esac; }
# 优先用 install 里的；旧 install 还没装 g1_service.sh 时，退回刚同步过去的 src 那份
SERVICE="S=$WS/install/robot_bringup/share/robot_bringup/script/g1_service.sh; \
[ -f \$S ] || S=$WS/src/robot_bringup/script/g1_service.sh; bash \$S"

echo "== 同步 $PKGS -> $IP"
$SSH "mkdir -p $WS/src"
for p in $PKGS; do
  d=$(dir_of "$p")
  [ -d "$SRC/$d" ] || { echo "本机没有 $SRC/$d" >&2; exit 1; }
  # -rlpc 而不是 -a：-a 含 -t，会把内容相同文件的修改时间也改成本机的，触发不必要的重编
  rsync -rlpc --itemize-changes \
    --exclude='bin/' --exclude='build/' --exclude='install/' --exclude='log/' \
    --exclude='__pycache__/' --exclude='.pytest_cache/' --exclude='*.pyc' \
    --exclude='/thirdparty/' --exclude='/script/g1_humble_env.sh' \
    "$SRC/$d/" "unitree@$IP:$WS/src/$d/" | grep -v '^\.d' || true
done

echo "== 停栈"
$SSH "$SERVICE stop" | tail -1

echo "== 编译 $PKGS"
$SSH "cd $WS && source /opt/ros/jazzy/setup.bash && { [ ! -f install/setup.bash ] || source install/setup.bash; } && \
      export PYTHONNOUSERSITE=1 && \
      colcon build --packages-select $PKGS 2>&1 | grep -E 'Finished|Failed|error:|Summary' | tail -20; exit \${PIPESTATUS[0]}"

echo "== 启动"
$SSH "$SERVICE start"
sleep 45
$SSH "systemctl is-active g1-robot g1-semantic-map; \
      G=\$(ls -t /opt/G1/logs/glog/run_loc_online*INFO* 2>/dev/null | head -1); \
      grep -h -o 'loc guard: [a-z +]*' \$G | head -1; grep -h -o 'ndt jump gate [^;]*' \$G | head -1; \
      grep -h -o 'text_in=[^,]*' /opt/G1/logs/semantic_latest.log | tail -1"
echo "== 部署完成。建议在原点跑一次 loc_check 确认初始化：python3 /opt/G1/bags/replay/loc_check.py <地图目录>"
