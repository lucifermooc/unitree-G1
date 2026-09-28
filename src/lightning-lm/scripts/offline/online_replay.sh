#!/usr/bin/env bash
# 在线回放：run_loc_online（MODE=slam 时为 run_slam_online）+ robot_pose_pub + 播放原始录包，在独立 ROS_DOMAIN_ID 上运行（不影响机器人上的栈），
# 由 tfrec.py 订阅实际发布的 /tf 与 /base_link_pose，最后用 lmlog.py online 逐条统计漂移与两者一致性。
# 用法: online_replay.sh <实验名> [段.键=值 ...]
#   BAG 原始录包目录（含 /tf_static）  MAP 地图目录（定位）  INST 待测安装目录  ROS_DOMAIN_ID 默认 85
#   LIDAR_DELAY 点云延迟秒数（复现实车传输滞后，见 lidar_delay.py），LIDAR_JITTER 附加抖动秒数
#   POSE_PUB 待测 robot_pose_pub_node 可执行文件（默认线上安装的）
# 输出: $OFF/online_<实验名>/{run_loc_online.INFO 或 run_slam_online.INFO, map_base.txt, base_body.txt, blp.txt, static.txt, tf.txt}
HERE=$(cd "$(dirname "$0")" && pwd)
name=$1; shift
OFF=${OFF:-/opt/G1/bags/offline}
INST=${INST:-/opt/G1/lighting_ws/install/lightning}
CFG=${CFG:-$INST/share/lightning/config/default_livox.yaml}
MAP=${MAP:-/home/unitree/maps/1789800448348}
MODE=${MODE:-loc}
BAG=${BAG:-/opt/G1/bags/map_0919_1438}
POSE_PUB=${POSE_PUB:-/opt/G1/lighting_ws/install/robot_bringup/lib/robot_bringup/robot_pose_pub_node}
source /home/unitree/unitree_ros2/setup.sh >/dev/null
source /opt/G1/lighting_ws/install/setup.bash
export LD_LIBRARY_PATH=$INST/lib:$LD_LIBRARY_PATH ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-85}
V=$OFF/online_$name; rm -rf "$V"; mkdir -p "$V"
if [ "$MODE" = slam ]; then
  python3 "$HERE/_cfg.py" "$CFG" "$V/cfg.yaml" "system.map_save_root=$V" "$@" || exit 1
else
  cp -a "$MAP" "$V/map"
  python3 "$HERE/_cfg.py" "$CFG" "$V/cfg.yaml" "system.map_path=$V/map" "system.pub_tf=true" "$@" || exit 1
fi
printf '/tf_static:\n  history: keep_last\n  depth: 100\n  reliability: reliable\n  durability: transient_local\n' > "$V/qos.yaml"
python3 "$HERE/tfrec.py" "$V" > "$V/tfrec.out" 2>&1 & REC=$!
GLOG_log_dir=$V "$INST/lib/lightning/run_${MODE}_online" --config="$V/cfg.yaml" > "$V/loc.out" 2>&1 & LOC=$!
"$POSE_PUB" > "$V/pose_pub.out" 2>&1 & POSE=$!
REMAP=(); DLY=
if [ -n "$LIDAR_DELAY" ]; then
  python3 "$HERE/lidar_delay.py" "$LIDAR_DELAY" "${LIDAR_JITTER:-0}" > "$V/delay.out" 2>&1 & DLY=$!
  REMAP=(--remap /livox/lidar:=/livox/lidar_raw)
fi
sleep 5
S=$(date +%s)
ros2 bag play "$BAG" --topics /livox/lidar /livox/imu /tf_static --qos-profile-overrides-path "$V/qos.yaml" "${REMAP[@]}" > "$V/play.out" 2>&1
echo "play done $(( $(date +%s) - S ))s" > "$V/timeline.txt"
sleep $(( 5 + ${LIDAR_DELAY%.*} + 1 ))   # 等延迟中的点云送完
[ -n "$DLY" ] && kill -INT $DLY
kill -INT $LOC; sleep 5
echo "loc stopped $(date +%s.%N)" >> "$V/timeline.txt"
sleep 5   # 定位停发 TF 后 /base_link_pose 的表现
kill -INT $POSE $REC; sleep 2; kill -9 $LOC $POSE $REC 2>/dev/null
python3 "$HERE/lmlog.py" online "$V"
