#!/usr/bin/env bash
# 云端/没有 ROS apt 源的机器：用 micromamba + RoboStack 安装 ROS2 Jazzy + Nav2 + Cartographer + TurtleBot3 仿真。
# 本地 Ubuntu 24.04 请直接用 apt 安装（见 sim/README.md），不需要这个脚本。
set -euo pipefail
PREFIX=/opt/mamba
export MAMBA_ROOT_PREFIX=$PREFIX/root

if [ ! -x $PREFIX/bin/micromamba ]; then
  mkdir -p $PREFIX && cd $PREFIX
  curl -sSL -o mm.tar.bz2 https://conda.anaconda.org/conda-forge/linux-64/micromamba-2.9.0-0.tar.bz2
  tar xjf mm.tar.bz2 bin/micromamba
fi

$PREFIX/bin/micromamba create -y -n ros -c conda-forge -c robostack-jazzy python=3.12 \
  ros-jazzy-ros-base ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander \
  ros-jazzy-cartographer-ros ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-cartographer \
  ros-jazzy-turtlebot3-navigation2 ros-jazzy-turtlebot3-description ros-jazzy-ros-gz ros-jazzy-nav2-map-server

# 没有显卡时 Gazebo 需要 Mesa 软件渲染
apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq libgl1-mesa-dri libegl-mesa0 libegl1 libglx-mesa0

# 修补 RoboStack 打包问题：libcartographer_ros.so 里混进了离线建图工具（offline_node）的命令行参数定义，
# 和 cartographer_node 自己的同名参数冲突，启动即报 "flag 'xxx' was defined more than once"。
# 做法：把库里这几个参数的注册名改掉最后一个字母（同长度），变量本身不变，cartographer_node 功能不受影响。
LIB=$MAMBA_ROOT_PREFIX/envs/ros/lib
cp -n $LIB/libcartographer_ros.so $LIB/libcartographer_ros.so.orig || true
python3 - "$LIB" <<'EOF'
import re, sys
lib_dir = sys.argv[1]
lib = open(f"{lib_dir}/libcartographer_ros.so.orig", "rb").read()
node = open(f"{lib_dir}/cartographer_ros/cartographer_node", "rb").read()
pat = re.compile(rb"_ZN3fL.\d+FLAGS_([a-z0-9_]+?)(?:B5cxx11)?E")
common = {m.group(1) for m in pat.finditer(lib)} & {m.group(1) for m in pat.finditer(node)}
for name in common:
    lib = lib.replace(b"\0" + name + b"\0", b"\0" + name[:-1] + b"_\0")
    print("patched flag:", name.decode())
open(f"{lib_dir}/libcartographer_ros.so", "wb").write(lib)
EOF

# 语义地图依赖装进同一个 Python，导航程序才能同时 import rclpy 和语义地图
$MAMBA_ROOT_PREFIX/envs/ros/bin/pip install -q torch --index-url https://download.pytorch.org/whl/cpu
$MAMBA_ROOT_PREFIX/envs/ros/bin/pip install -q -r "$(dirname "$0")/../semantic_map/requirements.txt"
echo "✅ 安装完成。使用：source sim/env.sh"
