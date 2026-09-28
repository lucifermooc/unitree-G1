#!/usr/bin/env bash
# 离线定位回放。用法: loc_offline.sh <实验名> [段.键=值 ...]
#   MAP  地图目录（复制一份再用，不改原图）   BAG  sqlite 录包（见 slam_offline.sh）
#   INST 待测安装目录   CFG 基础配置（默认 $INST 里的 default_livox.yaml）   INIT 初始位姿 "x y yaw_deg"（缺省从地图原点初始化）
# 输出: $OFF/loc_<实验名>/{run.log, cfg.yaml, map/,，tf.txt}，最后打印 lmlog.py tfdrift
name=$1; shift
HERE=$(cd "$(dirname "$0")" && pwd)
OFF=${OFF:-/opt/G1/bags/offline}
INST=${INST:-/opt/G1/lighting_ws/install/lightning}
CFG=${CFG:-$INST/share/lightning/config/default_livox.yaml}
MAP=${MAP:-/home/unitree/maps/1789788726611}
BAG=${BAG:-$OFF/map_0919_1438_sqlite}
source /home/unitree/unitree_ros2/setup.sh >/dev/null
source /opt/G1/lighting_ws/install/setup.bash
export LD_LIBRARY_PATH=$INST/lib:$LD_LIBRARY_PATH ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-79}
V=$OFF/loc_$name; rm -rf "$V"; mkdir -p "$V"
cp -a "$MAP" "$V/map"
python3 "$HERE/_cfg.py" "$CFG" "$V/cfg.yaml" "system.map_path=$V/map" "$@" || exit 1
cd "$V"; S=$(date +%s)
nice -n 10 "$INST/lib/lightning/run_loc_offline" --input_bag="$BAG" --config="$V/cfg.yaml" --map_path="$V/map" --tf_out="$V/tf.txt" ${INIT:+--init_pose="$INIT"} > "$V/run.log" 2>&1
echo "loc_$name rc=$? $(( $(date +%s) - S ))s"
python3 "$HERE/lmlog.py" tfdrift "$V/tf.txt" "$V/run.log"
