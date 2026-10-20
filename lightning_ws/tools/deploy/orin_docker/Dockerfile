# G1 Orin（JetPack 5 / Ubuntu 20.04，装不了 jazzy）用的运行/编译环境：Ubuntu 24.04 + ROS 2 jazzy。
# 与 Thor 主机环境对齐：ros-jazzy-* 取 Thor 上手动安装的那批（去掉 rviz/rqt 等界面包），开发库同 Thor。
# 只装环境，不含工作区：/opt/G1、/home/unitree 运行时由 g1_docker.sh 挂进来，路径与 Thor 一致。
#
#   docker build -t g1-jazzy:latest /opt/G1/docker      （g1_docker.sh build-image）
FROM ubuntu:24.04

# http：基础镜像里还没有 ca-certificates，https 源会被 apt 忽略
ARG UBUNTU_MIRROR=http://mirrors.aliyun.com/ubuntu-ports
ARG ROS_MIRROR=https://mirrors.ustc.edu.cn/ros2/ubuntu
ARG PIP_INDEX=https://pypi.tuna.tsinghua.edu.cn/simple
ENV DEBIAN_FRONTEND=noninteractive LANG=C.UTF-8 LC_ALL=C.UTF-8 TZ=Asia/Shanghai

RUN sed -i "s|http://ports.ubuntu.com/ubuntu-ports|${UBUNTU_MIRROR}|g" /etc/apt/sources.list.d/ubuntu.sources \
 && apt-get update && apt-get install -y --no-install-recommends ca-certificates curl gnupg tzdata \
 && ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# ROS 2 源（国内镜像）。核对官方签名密钥指纹，不对就构建失败。
RUN curl -fsSL https://mirrors.ustc.edu.cn/rosdistro/ros.key -o /tmp/ros.key \
 && if grep -q 'BEGIN PGP' /tmp/ros.key; then gpg --batch --dearmor < /tmp/ros.key; else cat /tmp/ros.key; fi \
    > /usr/share/keyrings/ros-archive-keyring.gpg \
 && gpg --batch --show-keys --with-colons /usr/share/keyrings/ros-archive-keyring.gpg \
    | grep -q '^fpr:::::::::C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654:' \
 && echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${ROS_MIRROR} noble main" \
    > /etc/apt/sources.list.d/ros2.list && rm /tmp/ros.key

# Thor 上 apt-mark showmanual 的 ros-jazzy-*，去掉界面包（desktop、rviz*、rqt*、*-gui、qt-*）
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rosidl-generator-dds-idl \
    ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-mppi-controller ros-jazzy-nav2-smac-planner \
    ros-jazzy-nav2-regulated-pure-pursuit-controller ros-jazzy-nav2-rotation-shim-controller \
    ros-jazzy-nav2-graceful-controller ros-jazzy-nav2-velocity-smoother ros-jazzy-nav2-collision-monitor \
    ros-jazzy-nav2-smoother ros-jazzy-nav2-constrained-smoother ros-jazzy-nav2-route \
    ros-jazzy-nav2-theta-star-planner ros-jazzy-nav2-navfn-planner ros-jazzy-nav2-dwb-controller \
    ros-jazzy-nav2-waypoint-follower ros-jazzy-nav2-simple-commander ros-jazzy-nav2-amcl \
    ros-jazzy-opennav-docking ros-jazzy-spatio-temporal-voxel-layer ros-jazzy-openvdb-vendor \
    ros-jazzy-behaviortree-cpp ros-jazzy-robot-localization ros-jazzy-slam-toolbox \
    ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-pcl-msgs ros-jazzy-point-cloud-transport \
    ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-image-transport-plugins \
    ros-jazzy-compressed-image-transport ros-jazzy-compressed-depth-image-transport \
    ros-jazzy-theora-image-transport ros-jazzy-zstd-image-transport ros-jazzy-image-geometry \
    ros-jazzy-camera-info-manager ros-jazzy-depthimage-to-laserscan ros-jazzy-laser-geometry \
    ros-jazzy-diagnostic-updater ros-jazzy-usb-cam ros-jazzy-vision-msgs ros-jazzy-geographic-msgs \
    ros-jazzy-map-msgs ros-jazzy-interactive-markers ros-jazzy-xacro ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-state-publisher ros-jazzy-resource-retriever ros-jazzy-filters ros-jazzy-angles \
    ros-jazzy-rosbridge-server ros-jazzy-rosapi ros-jazzy-tf2-tools ros-jazzy-tf-transformations \
    ros-jazzy-ros2-control ros-jazzy-controller-manager ros-jazzy-joy ros-jazzy-teleop-twist-joy \
    ros-jazzy-teleop-twist-keyboard ros-jazzy-launch-pytest ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-libcurl-vendor \
 && rm -rf /var/lib/apt/lists/*

# 编译工具与开发库（同 Thor：lightning install_dep.sh + librealsense + Livox SDK2）
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config python3-colcon-common-extensions python3-rosdep python3-pip \
    python3-pytest python3-yaml python3-scipy python3-numpy python3-pandas python3-matplotlib \
    python3-pexpect python3-transforms3d \
    libopencv-dev libpcl-dev pcl-tools libyaml-cpp-dev libgoogle-glog-dev libgflags-dev libceres-dev \
    libsuitesparse-dev libtbb-dev libomp-dev libeigen3-dev libboost-all-dev libfmt-dev libspdlog-dev \
    libusb-1.0-0-dev libudev-dev libssl-dev libgl1-mesa-dev libglfw3-dev libglu1-mesa-dev \
    libxml2-dev uthash-dev \
    iproute2 iputils-ping net-tools procps psmisc sudo less nano htop tmux rsync usbutils \
 && rm -rf /var/lib/apt/lists/*

# 语义地图（CPU）：pip-constraints-thor.txt 把全部依赖钉到 Thor 的版本（不钉时 pyarrow 26 要 NumPy 2，而 ROS 用系统 NumPy 1.26，
# FlagEmbedding 导入失败）；其中 setuptools<80 是 colcon 的要求（torch 会顺带升到 84，必须挡住）
# torch 用 CPU 版（上交镜像）：PyPI 的 aarch64 torch 会带约 3 GB 的 CUDA 13 库，JetPack 5 容器里用不上
COPY pip-constraints-thor.txt /etc/pip-constraints.txt
RUN pip3 install --break-system-packages --no-cache-dir -c /etc/pip-constraints.txt \
    --index-url https://mirror.sjtu.edu.cn/pytorch-wheels/cpu --extra-index-url ${PIP_INDEX} \
    torch==2.14.0+cpu \
 && pip3 install --break-system-packages --no-cache-dir -i ${PIP_INDEX} -c /etc/pip-constraints.txt \
    FlagEmbedding==1.4.2 qdrant-client==1.19.1 transformers==5.17.0 sentence-transformers==6.1.0 \
    peft==0.21.0 accelerate==1.15.0 \
 && pip3 uninstall -y --break-system-packages setuptools \
 && apt-get update && apt-get install -y --no-install-recommends --reinstall python3-setuptools python3-pkg-resources \
 && rm -rf /var/lib/apt/lists/*
# ↑ 依赖链会把 setuptools 79 装进 /usr/local，colcon 编 lightning thirdparty 的 livox egg 报 InvalidVersion ''（CLAUDE.md 3.1.5）。
#   Thor 靠 PYTHONNOUSERSITE=1 挡住 ~/.local 那份；容器里装在系统目录挡不住，所以卸掉，回到 apt 的 68.1.2（与 Thor 编译时一致）。

# 与主机 unitree 同 uid/gid，挂进来的 /home/unitree、/opt/G1 读写权限一致；镜像自带的 ubuntu 用户占着 1000，先删
RUN userdel -r ubuntu 2>/dev/null; groupadd -g 1000 unitree \
 && useradd -m -u 1000 -g 1000 -G video,plugdev,dialout,sudo -s /bin/bash unitree \
 && echo 'unitree ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/unitree && chmod 440 /etc/sudoers.d/unitree

# 容器里交互 shell 的环境（g1_docker.sh shell 用 --rcfile 读它，不读主机 ~/.bashrc：那里会 source foxy）
COPY g1_env.sh /etc/g1_env.sh
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
USER unitree
WORKDIR /home/unitree
