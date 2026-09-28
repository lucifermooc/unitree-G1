#!/usr/bin/env bash
# 离线建图回放。用法: slam_offline.sh <实验名> [段.键=值 ...]
#   BAG  录包目录（mcap 会先转成只含 /livox/lidar /livox/imu 的 sqlite，缓存在 $OFF 下）
#   INST 待测安装目录（默认正式 install；新代码先编到 install_exp 再测，不动正式 install）
#   CFG  基础配置（默认 $INST/share/lightning/config/default_livox.yaml，即将要部署的那份）
# 输出: $OFF/<实验名>/{new_map/, run.log, cfg.yaml}
name=$1; shift
HERE=$(cd "$(dirname "$0")" && pwd)
OFF=${OFF:-/opt/G1/bags/offline}
INST=${INST:-/opt/G1/lighting_ws/install/lightning}
CFG=${CFG:-$INST/share/lightning/config/default_livox.yaml}
BAG=${BAG:-/opt/G1/bags/map_0919_1438}
source /home/unitree/unitree_ros2/setup.sh >/dev/null
source /opt/G1/lighting_ws/install/setup.bash
export LD_LIBRARY_PATH=$INST/lib:$LD_LIBRARY_PATH
DB=$BAG
if [ ! -f "$BAG/metadata.yaml" ] || grep -q mcap "$BAG/metadata.yaml"; then
  DB=$OFF/$(basename "$BAG")_sqlite
  if [ ! -d "$DB" ]; then
    [ -f "$BAG/metadata.yaml" ] || ros2 bag reindex "$BAG" -s mcap >/dev/null 2>&1
    printf "output_bags:\n- uri: $DB\n  storage_id: sqlite3\n  topics: [/livox/lidar, /livox/imu]\n" > $OFF/convert.yaml
    ros2 bag convert -i "$BAG" -o $OFF/convert.yaml > $OFF/convert.log 2>&1 || { echo "convert failed"; tail -5 $OFF/convert.log; exit 1; }
  fi
fi
V=$OFF/$name; rm -rf "$V"; mkdir -p "$V"
python3 "$HERE/_cfg.py" "$CFG" "$V/cfg.yaml" "system.map_save_root=$V" "$@" || exit 1
cd "$V"; S=$(date +%s)
nice -n 10 "$INST/lib/lightning/run_slam_offline" --input_bag="$DB" --config="$V/cfg.yaml" > "$V/run.log" 2>&1
echo "$name rc=$? $(( $(date +%s) - S ))s abnormal_dt=$(grep -c 'abnormal dt' "$V/run.log")"
grep -E "success:|loop outliers" "$V/run.log" | tail -2 | sed -E 's/.*\] //'
grep "create kf" "$V/run.log" | tail -1 | sed -E 's/.*create kf ([0-9]+), state: +([-0-9.e]+) +([-0-9.e]+).*kf opt pose: +([-0-9.e]+) +([-0-9.e]+).*/last kf \1 lio(\2,\3) opt(\4,\5)/'
