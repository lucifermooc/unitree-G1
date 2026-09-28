# 仿真用的 ROS2 环境变量。用法：source sim/env.sh
# 云端用 micromamba 装的 ROS2（见 sim/install_ros_cloud.sh）；本地 Ubuntu 24.04 用 apt 装的话会自动走 /opt/ros/jazzy。
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
else
  export MAMBA_ROOT_PREFIX=${MAMBA_ROOT_PREFIX:-/opt/mamba/root}
  eval "$(/opt/mamba/bin/micromamba shell hook -s bash)"
  micromamba activate ros
fi
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# 没有显卡的机器（云主机）用 Mesa 软件渲染，否则 Gazebo 的激光雷达仿真会崩溃
if [ -f /usr/share/glvnd/egl_vendor.d/50_mesa.json ] && ! command -v nvidia-smi >/dev/null; then
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json
fi
