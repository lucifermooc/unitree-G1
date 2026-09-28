#!/bin/bash
# G1 导航/定位问题录包。Thor 上执行，只读，不发布任何指令。
#
#   ./record_nav_bag.sh [名字] [-t 秒数] [--light|--full] [--zstd]
#
# 话题清单按"这一轮每个分析实际用到什么"来组，改之前先看注释里标的用途。
# 默认不压缩：zstd 会吃 CPU，而 CPU 竞争本身就是我们在查的嫌疑之一（见 CLAUDE.md）。
# 不要加 set -u：ROS 的 setup.bash 引用 AMENT_TRACE_SETUP_FILES 等未定义变量会直接退出
set -o pipefail

NAME="${1:-nav_$(date +%m%d_%H%M)}"; [[ "$NAME" == -* ]] && NAME="nav_$(date +%m%d_%H%M)"
OUT_DIR="${BAG_DIR:-/opt/G1/bags}"
DUR=""; TIER="normal"; COMPRESS=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -t) DUR="$2"; shift 2;;
    --light) TIER="light"; shift;;
    --full)  TIER="full";  shift;;
    --zstd)  COMPRESS=(--compression-mode file --compression-format zstd); shift;;
    *) shift;;
  esac
done

source /opt/ros/jazzy/setup.bash
# 整套栈跑在 rmw_cyclonedds 上，不 source 这个会订阅不到（见 CLAUDE.md）
source /home/unitree/unitree_ros2/setup.sh >/dev/null 2>&1

# ---------------------------------------------------------------- 话题分组
# A. 位姿与 TF —— 支撑：定位跳变检测、点云按扫描时刻变换到 map、步态摆动分析
A=(/tf /tf_static /base_link_pose /odom /odommodestate
   /unitree_slam/high_rate_odometry
   /state_estimator/fusion_odom /state_estimator/odom_pelvis /state_estimator/odom_torso)

# B. 定位输入 —— 支撑：点云-地图匹配率/残差(p25)、近处盲区分环统计、自身点过滤
#    /lightning/registered_scan 是 LIO 去畸变后的单帧云，Nav2 障碍层也吃它，必录
B=(/lightning/registered_scan /map /map_updates)

# C. 代价地图 —— 支撑：中心格代价、最近 lethal 距离、footprint 自清验证、STVL 前后对比
#    *_raw 是 nav2_msgs/Costmap（带完整 0~255 代价），普通 costmap 是 OccupancyGrid(0~100)，两者都要
C=(/local_costmap/costmap /local_costmap/costmap_raw /local_costmap/costmap_updates
   /local_costmap/published_footprint
   /local_costmap/stvl_voxel_layer /local_costmap/stvl_voxel_layer_raw
   /global_costmap/costmap /global_costmap/costmap_raw /global_costmap/costmap_updates
   /global_costmap/published_footprint /global_costmap/stvl_voxel_layer_raw
   /local_costmap/mid360_voxel_layer_raw /global_costmap/mid360_voxel_layer_raw
   /lightning/registered_scan_nav
   /costmap_filter_info /keepout_filter_map)

# D. 规划与控制 —— 支撑：绕行系数、路径是否被挡、MPPI 局部轨迹、指令链路各级对比
#    /optimal_trajectory 和 /trajectories 只在 FollowPath.visualize:=true 时才发
D=(/plan /plan_smoothed /unsmoothed_plan /transformed_global_plan
   /optimal_trajectory /trajectories
   /cmd_vel /cmd_vel_smoothed /cmd_vel_safe
   /goal_pose /initialpose /nav_to_pose /patrol_path /robot_path)

# E. 状态与日志 —— 支撑：目标状态迁移、Optimizer fail / 清障 / patience exceeded 的时刻
#    /rosout 是抓 nav2 全部 WARN/ERROR 的唯一来源，必录
E=(/rosout /robot_status /task_status
   /navigate_to_pose/_action/status /navigate_to_pose/_action/feedback
   /follow_path/_action/status /compute_path_to_pose/_action/status)

# F. 原始传感器 —— 支撑：LIO 输入侧排查（IMU 断档、时间戳、去畸变前后对比）、D435 盲区几何
#    体积大：livox 原始云 ~10 MB/s，D435 点云 ~30 MB/s
F=(/livox/points /livox/imu /livox/lidar /camera/camera/depth/color/points)

# G. 相机图像 —— 只有 --full 才录，用于回看现场画面，几十 MB/s
G=(/camera/camera/color/image_raw /camera/camera/depth/image_rect_raw
   /camera/camera/depth/camera_info /camera/camera/color/camera_info)

case "$TIER" in
  light)  WANT=("${A[@]}" "${B[@]}" "${C[@]}" "${D[@]}" "${E[@]}") ;;
  full)   WANT=("${A[@]}" "${B[@]}" "${C[@]}" "${D[@]}" "${E[@]}" "${F[@]}" "${G[@]}") ;;
  *)      WANT=("${A[@]}" "${B[@]}" "${C[@]}" "${D[@]}" "${E[@]}" "${F[@]}") ;;
esac

# --------------------------------------------- 只录当前真实存在的话题（含隐藏的 action 话题）
mapfile -t LIVE < <(ros2 topic list --include-hidden-topics 2>/dev/null)
REC=(); MISS=()
for t in "${WANT[@]}"; do
  if printf '%s\n' "${LIVE[@]}" | grep -qx -- "$t"; then REC+=("$t"); else MISS+=("$t"); fi
done

if [[ ${#REC[@]} -eq 0 ]]; then
  echo "没有任何目标话题在线——整套栈起来了吗？"; exit 1
fi

echo "输出: $OUT_DIR/$NAME   档位: $TIER   压缩: ${COMPRESS[*]:-无}"
echo "将录制 ${#REC[@]} 个话题:"; printf '  %s\n' "${REC[@]}"
[[ ${#MISS[@]} -gt 0 ]] && { echo "未在线(跳过) ${#MISS[@]} 个:"; printf '  - %s\n' "${MISS[@]}"; }
echo
df -h "$OUT_DIR" | tail -1
echo "提示: 录制本身会占 CPU/IO。若正在排查定位抖动，先确认 rviz2 没在 Thor 本机跑。"
echo

mkdir -p "$OUT_DIR"
CMD=(ros2 bag record -o "$OUT_DIR/$NAME" --storage mcap --max-cache-size 268435456
     "${COMPRESS[@]}" "${REC[@]}")
# 注意：ros2 bag record 的 -d 是**分片时长**(--max-bag-duration)，不是"录多久就停"。
# 以前这里写成 -d "$DUR"，结果 -t 90 只是每 90 s 换一个文件，录制永不停止
# （一次踩坑：ssh 断开后孤儿进程继续录，几分钟就吃掉 5 GB）。要限时就用 timeout 包住。
echo "${CMD[*]}"
if [[ -n "$DUR" ]]; then
  echo "(限时 ${DUR}s，到点发 SIGINT 让 rosbag2 正常收尾写 metadata.yaml)"
  exec timeout -s INT "$DUR" "${CMD[@]}"
else
  exec "${CMD[@]}"
fi
