#!/usr/bin/env bash
set -eo pipefail

source /home/unitree/unitree_ros2/setup.sh
source /opt/G1/lighting_ws/install/setup.bash
set -u

bag_root=/opt/G1/bags
bag_name=${1:-g1_slam_$(date +%Y%m%d_%H%M%S)}
duration=${2:-0}
bag_path=${bag_root}/${bag_name}

mkdir -p "${bag_root}"

# 必录：Lightning 离线回放的原始输入（CustomMsg 带逐点 offset_time，去畸变要用）+ 外参。
# 参考：宇树自身腿式/融合里程计，用来判断 LIO 在哪一帧跳变。
# 结果：定位在线输出，和离线回放结果对比。
# 默认不录 D435 图像/点云、/livox/points（体积大，与建图/定位问题无关，可再生成）。
# 导航/避障问题必须录 D435 点云：它是 STVL 层唯一的障碍来源，雷达近处有盲区、
# 且 obstacle_layer 的 obstacle_min_range=0.25，正前方近距离障碍只有相机能看到。
# 用 WITH_NAV=1 打开（约 +8 MB/s，压缩后约 200~300 MB/分钟，录完记得清理）。
topics=(
  /livox/lidar
  /livox/imu
  /tf_static
  /tf
  /joint_states
  /state_estimator/odom_torso
  /state_estimator/odom_pelvis
  /state_estimator/fusion_odom_torso
  /state_estimator/fusion_odom
  /odommodestate
  /base_link_pose
  /rosout
)

# 导航/避障调试用：相机点云 + Nav2 输入输出，可离线复现代价地图与控制决策
nav_topics=(
  /camera/camera/depth/color/points      # STVL 障碍来源（正前方近处靠它）
  /camera/camera/depth/camera_info
  /camera/camera/color/camera_info
  /lightning/registered_scan             # obstacle_layer 障碍来源
  /local_costmap/costmap_raw             # 5 Hz、120x120，约 14 KB/帧
  /local_costmap/stvl_voxel_layer_raw
  /local_costmap/obstacle_layer_raw
  /plan
  /cmd_vel
  /cmd_vel_nav
  /goal_pose
  /navigate_to_pose/_action/status
  /navigate_to_pose/_action/feedback
)
if [ "${WITH_NAV:-0}" = "1" ]; then
  topics+=("${nav_topics[@]}")
  echo "WITH_NAV=1：额外录制相机点云与 Nav2 话题（体积大，约 200~300 MB/分钟）"
fi

echo "Recording G1 SLAM bag to: ${bag_path}"
echo "Duration: ${duration}s (0 means stop with Ctrl-C)"

record_cmd=(
  ros2 bag record
  --storage mcap
  --storage-preset-profile zstd_fast
  --max-cache-size 536870912
  --output "${bag_path}"
  --topics "${topics[@]}"
)

if [[ "${duration}" =~ ^[1-9][0-9]*$ ]]; then
  timeout --signal=INT --kill-after=20s "${duration}" "${record_cmd[@]}" || status=$?
  if [[ ${status:-0} -ne 0 && ${status:-0} -ne 124 && ${status:-0} -ne 130 ]]; then
    exit "${status}"
  fi
else
  exec "${record_cmd[@]}"
fi

echo "Bag finalized: ${bag_path}"
ros2 bag info "${bag_path}"
