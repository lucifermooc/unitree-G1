#!/usr/bin/env bash
# 导航仿真平台一键安装（Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic）
# 三个机器人：Go2 机器狗、差速车、阿克曼车；两层楼场景 + 平面场景
#
# 用法：
#   ./install.sh                 # 安装依赖、下载 Go2 外观模型、编译工作区
#   ./install.sh --install-ros   # 本机还没装 ROS 2 Humble 时，顺带装 ros-humble-desktop
#   ./install.sh --no-build      # 只装依赖，不编译
#   ./install.sh --skip-apt      # 跳过 apt 安装（依赖已装好，只想重新下载模型/编译）
#
# 注意：ROS 2 Humble 官方搭配的是 Gazebo Fortress。这里装的是 OSRF 提供的
# Harmonic 版 ros_gz（ros-humble-ros-gzharmonic），它和 Fortress 版
# ros-humble-ros-gz* 包互相冲突，脚本会先卸载后者。
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$WS_DIR/src/robot_nav_sim"
SUDO=""
[ "$(id -u)" -ne 0 ] && SUDO="sudo"
INSTALL_ROS=0
DO_BUILD=1
DO_APT=1
ROS_PKG="${ROS_PKG:-desktop}"

for arg in "$@"; do
  case "$arg" in
    --install-ros) INSTALL_ROS=1 ;;
    --no-build) DO_BUILD=0 ;;
    --skip-apt) DO_APT=0 ;;
    -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
    *) echo "未知参数：$arg（用 --help 查看用法）"; exit 1 ;;
  esac
done

step() { echo -e "\n\033[1;36m==> $*\033[0m"; }
warn() { echo -e "\033[1;33m[提示] $*\033[0m"; }
die()  { echo -e "\033[1;31m[错误] $*\033[0m" >&2; exit 1; }

export DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------- 1. 系统检查
step "检查系统"
. /etc/os-release
[ "${VERSION_CODENAME:-}" = "jammy" ] || die "需要 Ubuntu 22.04 (jammy)，当前是 ${PRETTY_NAME:-未知}"
echo "系统：$PRETTY_NAME，架构：$(dpkg --print-architecture)"

if [ "$DO_APT" -eq 1 ]; then
  $SUDO apt-get update -qq || true
  $SUDO apt-get install -y -qq curl gnupg lsb-release ca-certificates git software-properties-common >/dev/null

  # -------------------------------------------------------------- 2. ROS apt 源
  # 2025 年 ROS 更换了 apt 签名密钥。还在用旧 ros-archive-keyring 的机器
  # apt update 会报 NO_PUBKEY / EXPKEYSIG。官方的新做法是安装 ros2-apt-source 包。
  step "配置 ROS 2 apt 源（ros2-apt-source）"
  if dpkg -s ros2-apt-source >/dev/null 2>&1; then
    echo "ros2-apt-source 已安装"
  else
    for f in /etc/apt/sources.list.d/*.list; do
      [ -f "$f" ] || continue
      if grep -q "packages.ros.org/ros2" "$f"; then
        warn "备份旧的 ROS 源 $f -> $f.bak（避免和新源冲突）"
        $SUDO mv "$f" "$f.bak"
      fi
    done
    VER="$(curl -fsSI https://github.com/ros-infrastructure/ros-apt-source/releases/latest \
           | grep -i '^location:' | sed -E 's#.*/tag/([^[:space:]]+).*#\1#' | tr -d '\r' || true)"
    VER="${VER:-1.3.0}"
    echo "ros2-apt-source 版本：$VER"
    curl -fsSL -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VER}/ros2-apt-source_${VER}.jammy_all.deb" \
      || die "下载 ros2-apt-source 失败，请检查网络（需要能访问 github.com）"
    $SUDO apt-get install -y -qq /tmp/ros2-apt-source.deb >/dev/null
  fi

  # -------------------------------------------------------------- 3. ROS 2 Humble
  if [ ! -f /opt/ros/humble/setup.bash ]; then
    if [ "$INSTALL_ROS" -eq 1 ]; then
      step "安装 ROS 2 Humble（ros-humble-$ROS_PKG），时间较长"
      $SUDO apt-get update -qq
      $SUDO apt-get install -y -qq "ros-humble-$ROS_PKG" >/dev/null
    else
      die "没找到 /opt/ros/humble。先装好 ROS 2 Humble，或者加 --install-ros 让脚本来装"
    fi
  fi

  # -------------------------------------------------------------- 4. Gazebo Harmonic 源
  step "配置 Gazebo (OSRF) apt 源"
  if [ ! -f /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg ]; then
    $SUDO curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  fi
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
    | $SUDO tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
  $SUDO apt-get update -qq

  # -------------------------------------------------------------- 5. 卸载冲突的 Fortress 版 ros_gz
  CONFLICT="$(dpkg-query -W -f='${Package} ${Status}\n' 'ros-humble-ros-gz*' 'ros-humble-ros-ign*' 2>/dev/null \
              | awk '$NF=="installed" && $1 !~ /gzharmonic/ {print $1}' || true)"
  if [ -n "$CONFLICT" ]; then
    warn "以下 Fortress 版 ros_gz 包与 Harmonic 版冲突，将被卸载："
    echo "$CONFLICT"
    # shellcheck disable=SC2086
    $SUDO apt-get remove -y -qq $CONFLICT >/dev/null
  fi

  # -------------------------------------------------------------- 6. 安装依赖
  step "安装 Gazebo Harmonic、ros_gz、Nav2、slam_toolbox 等（时间较长）"
  $SUDO apt-get install -y -qq \
    gz-harmonic \
    ros-humble-ros-gzharmonic \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-simple-commander \
    ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rviz2 \
    python3-colcon-common-extensions python3-yaml >/dev/null
  echo "依赖安装完成"
fi

# ---------------------------------------------------------------- 7. Go2 外观模型
# 来自 unitreerobotics/unitree_ros（BSD-3-Clause），只取 go2_description 的网格文件。
step "下载 Go2 外观模型（约 25 MB）"
MESH_DIR="$PKG_DIR/meshes/go2"
if [ -f "$MESH_DIR/base.dae" ]; then
  echo "已存在：$MESH_DIR"
else
  TMP="$(mktemp -d)"
  if git clone -q --depth 1 --filter=blob:none --sparse https://github.com/unitreerobotics/unitree_ros.git "$TMP/unitree_ros" \
     && git -C "$TMP/unitree_ros" sparse-checkout set robots/go2_description/dae >/dev/null 2>&1 \
     && [ -f "$TMP/unitree_ros/robots/go2_description/dae/base.dae" ]; then
    mkdir -p "$MESH_DIR"
    cp "$TMP/unitree_ros/robots/go2_description/dae/"*.dae "$MESH_DIR/"
    cp "$TMP/unitree_ros/LICENSE" "$MESH_DIR/LICENSE.unitree_ros" 2>/dev/null || true
    echo "已下载到 $MESH_DIR"
  else
    warn "下载失败，机器狗会用简化的几何外观（不影响导航和建图）。可以稍后重跑本脚本加 --skip-apt 重试"
  fi
  rm -rf "$TMP"
fi

# ---------------------------------------------------------------- 8. 编译
if [ "$DO_BUILD" -eq 1 ]; then
  step "编译工作区 $WS_DIR"
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
  cd "$WS_DIR"
  colcon build --symlink-install
fi

cat <<EOF

$(echo -e "\033[1;32m安装完成\033[0m")

每个新终端先执行：
  source /opt/ros/humble/setup.bash && source $WS_DIR/install/setup.bash
（可以把这一行加到 ~/.bashrc）

常用命令（robot 可选 go2 / diff / ackermann，world 可选 two_floor / flat）：
  # 只启动仿真，另开终端用键盘遥控
  ros2 launch robot_nav_sim sim.launch.py robot:=go2 world:=two_floor
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

  # 2D 建图（slam_toolbox），保存：ros2 run nav2_map_server map_saver_cli -f ~/my_map
  ros2 launch robot_nav_sim mapping.launch.py robot:=diff world:=flat

  # Nav2 导航（默认用生成好的一楼真值地图），在 RViz 里点 2D Goal Pose
  ros2 launch robot_nav_sim navigation.launch.py robot:=ackermann

  # 跨楼层导航（仅 Go2）：导航到楼梯口 -> 爬楼梯 -> 切换二楼地图 -> 导航到目标
  ros2 launch robot_nav_sim navigation.launch.py robot:=go2
  ros2 run robot_nav_sim go_to --floor 2 --x 3.0 --y 2.0

详细说明见 $WS_DIR/README.md
EOF
