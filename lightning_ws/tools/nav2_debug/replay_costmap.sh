#!/bin/bash
# 离线回放代价地图：把 bag 里的 TF + 两个传感器点云喂给一个独立的 nav2 costmap 节点，
# 把它算出来的 costmap 录成新 bag。同一份输入 + 不同参数 = 可复现的 A/B。
#
#   ./replay_costmap.sh <输入bag> <输出名> [--start S] [--dur S] [--rate R] [--set k=v]...
#
# 例：
#   ./replay_costmap.sh /opt/G1/bags/demo2_0922_1350 base --start 700 --dur 100
#   ./replay_costmap.sh /opt/G1/bags/demo2_0922_1350 decay1 --start 700 --dur 100 \
#        --set stvl_voxel_layer.voxel_decay=1.0
#   # D435 标记过滤 A/B：--mark 四选一（默认 filt15 = YAML 原值 = 线上现状，2026-09-24 起两个标记源都吃过滤输出）
#   #   raw15 / raw20   原始点云 + obstacle_range 1.5 / 2.0（= launch use_d435_mark_filter:=false）
#   #   filt15 / filt20 d435_mark_filter 输出 + 1.5 / 2.0
#   # 过滤器效果只比同距离的 raw vs filt（单变量）。过滤节点读深度图，bag 里没有 image_rect_raw 时只能用 raw*。
#   # --filter-set k=v 覆盖 g1_nav_bridge/config/d435_mark_filter.yaml 里的某一级（可重复），逐级 A/B：
#   ./replay_costmap.sh /opt/G1/bags/xxx r15   --mark raw15
#   ./replay_costmap.sh /opt/G1/bags/xxx nb    --mark filt15 --filter-set temporal_filter:=false
#   ./replay_costmap.sh /opt/G1/bags/xxx nb_tp --mark filt15
#
# 跑在独立的 ROS_DOMAIN_ID（默认 77），不会和机器人上正在跑的栈互相看见。
set -o pipefail
BAG="$1"; NAME="$2"; shift 2
START=0; DUR=120; RATE=1.0; SETS=(); SECTION=local_costmap; MARK=filt15; FSETS=(); DOMAIN="${REPLAY_DOMAIN:-77}"
WS=/opt/G1/lighting_ws
PARAMS_SRC="${PARAMS_SRC:-$WS/install/aid_navigation2/share/aid_navigation2/param/nav2_params.yaml}"
OUT_DIR="${REPLAY_OUT:-/opt/G1/bags/replay}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --start) START="$2"; shift 2;;
    --dur) DUR="$2"; shift 2;;
    --rate) RATE="$2"; shift 2;;
    --set) SETS+=(--set "$2"); shift 2;;
    --params) PARAMS_SRC="$2"; shift 2;;
    --mark) MARK="$2"; shift 2;;
    --section) SECTION="$2"; shift 2;;
    --filter-set) FSETS+=(-p "$2"); shift 2;;
    *) echo "未知参数 $1"; exit 1;;
  esac
done
[[ -d "$BAG" ]] || { echo "找不到 bag: $BAG"; exit 1; }
# /map 和 /tf_static 一样只在 bag 开头发一次，--start-offset 会把它跳过；global 没图 → static_layer 全 255，
# 各层取 Max 时 255 盖住 254，传感器标记全被吞掉（召回 0、残影 0，看起来"很干净"）。
# global 只能从 0 播，分析时用 --skip 跳到想要的时间段。
if [[ "$SECTION" == global_costmap && "$START" != 0 ]]; then
  echo "--section global_costmap 必须 --start 0（/map 只在开头发一次），分析时用 --skip 跳过前段"; exit 1
fi
# 放在用户 --set 之前，用户的 --set 仍可覆盖
RAW_SETS=(--set stvl_voxel_layer.realsense_mark.topic=/camera/camera/depth/color/points
          --set stvl_voxel_layer.realsense_mark_tall.topic=/camera/camera/depth/color/points)
case "$MARK" in
  raw15)  SETS=("${RAW_SETS[@]}" "${SETS[@]}");;
  raw20)  SETS=("${RAW_SETS[@]}" --set stvl_voxel_layer.realsense_mark.obstacle_range=2.0 "${SETS[@]}");;
  filt15) ;;
  filt20) SETS=(--set stvl_voxel_layer.realsense_mark.obstacle_range=2.0 "${SETS[@]}");;
  *) echo "--mark 只能是 raw15 / raw20 / filt15 / filt20"; exit 1;;
esac
if [[ "$MARK" == filt* ]] && ! grep -q "image_rect_raw" "$BAG/metadata.yaml"; then
  echo "bag 里没有 /camera/camera/depth/image_rect_raw，d435_mark_filter 没有输入：用 --mark raw15"; exit 1
fi

source /opt/ros/jazzy/setup.bash
source /home/unitree/unitree_ros2/setup.sh >/dev/null 2>&1
# scan_range_filter / d435_mark_filter 在工作空间里：不 source 就全看调用方的环境（踩过：MID360 层零输入、静默跑完）
source "$WS/install/setup.bash"
# 原配置下 ≥2 个订阅进程的话题走组播上网线，与在线栈共用被交换机 PAUSE 压到 ~10 MB/s 的 enP2p1s0：
# 回放的 D435 一旦有 2 个读者（录制 + costmap 节点）就只剩 ~11 Hz，播放线程被拖慢，
# MID360 帧晚于 /clock 超过 observation_persistence 被整批丢掉（obstacle_layer 一格不标），还会反过来挤占在线栈。
# spdp = 组播只做发现、数据单播经 lo（见 /opt/G1/exp/dds/README_fix.md）。
SPDP_XML=/opt/G1/exp/dds/cyclonedds_thor_spdp.xml
[[ -f "$SPDP_XML" ]] && export CYCLONEDDS_URI="${REPLAY_DDS_URI:-file://$SPDP_XML}"
export ROS_DOMAIN_ID="$DOMAIN"
mkdir -p "$OUT_DIR"
PF="$OUT_DIR/params_$NAME.yaml"
python3 "$WS/tools/nav2_debug/make_replay_params.py" "$PARAMS_SRC" "$PF" --section "$SECTION" "${SETS[@]}" || exit 1

# 回放只喂传感器与 TF —— 绝不回放 /local_costmap/*，否则和被测节点自己发的混在一起
PLAY_TOPICS=(/tf /map /lightning/registered_scan /camera/camera/depth/color/points
             /base_link_pose /cmd_vel)
if [[ "$MARK" == filt* ]]; then
  # 过滤节点读深度图；bag 里没有就直接报错，不要静默跑出一张"D435 什么都没标"的图
  for t in /camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info; do
    grep -q -- "name: $t\$" "$BAG/metadata.yaml" || { echo "bag 里没有 $t（需 record_nav_bag.sh --full）"; exit 1; }
  done
  PLAY_TOPICS+=(/camera/camera/depth/image_rect_raw /camera/camera/depth/camera_info)
fi
rm -rf "$OUT_DIR/$NAME"
# 注意：独立的 nav2_costmap_2d 节点把代价地图发在**根命名空间**
# (/costmap_raw、/stvl_voxel_layer_raw…)，不是 /costmap/xxx。
# 分析脚本对应传 --prefix ""。
REC_TOPICS=(/costmap_raw /stvl_voxel_layer_raw /mid360_voxel_layer_raw /obstacle_layer_raw
            /published_footprint /base_link_pose /tf /tf_static /cmd_vel
            /camera/camera/depth/color/points /camera/camera/depth/mark_points /map)

# 清掉上一轮残留：ros2 run 只是个 python 壳，kill 它不会杀掉真正的
# /opt/ros/jazzy/lib/nav2_costmap_2d 子进程，残留节点会和新节点重名，
# 导致 `ros2 lifecycle set /costmap configure` 打到错的节点上直接失败。
# 只杀本轮（同名 / 同 domain）的节点：不同 REPLAY_DOMAIN 的回放可以并行跑（d435_mark_filter 除外）。
SCAN_NODE=replay_scan_range_filter_d$DOMAIN
kill_nodes() {
  pkill -f "nav2_costmap_2d --ros-args --params-file $OUT_DIR/params_$NAME.yaml" 2>/dev/null
  pkill -f "[d]435_mark_filter --ros-args --params-file" 2>/dev/null
  pkill -f "[s]can_range_filter --ros-args -r __node:=$SCAN_NODE " 2>/dev/null
}
kill_nodes; sleep 1

echo "== 启动 costmap 节点 (domain $DOMAIN)"
ros2 run nav2_costmap_2d nav2_costmap_2d --ros-args --params-file "$PF" \
     -p use_sim_time:=true > "$OUT_DIR/$NAME.node.log" 2>&1 &
NODE_PID=$!
FILT_PID=
if [[ "$MARK" == filt* ]]; then
  # 参数文件的顶层键是节点名 d435_mark_filter，这里不能改节点名，否则整份参数不生效；
  # 回放跑在独立 domain，不会和机器人上的同名节点冲突。
  ros2 run g1_nav_bridge d435_mark_filter --ros-args \
       --params-file "$WS/install/g1_nav_bridge/share/g1_nav_bridge/config/d435_mark_filter.yaml" \
       -p use_sim_time:=true "${FSETS[@]}" > "$OUT_DIR/$NAME.filter.log" 2>&1 &
  FILT_PID=$!
fi
# MID360 在 STVL 里时吃 scan_range_filter 的输出（STVL 没有 obstacle_min_range），回放也要起一个
SCAN_PID=
if grep -q "registered_scan_nav" "$PF"; then
  ros2 pkg prefix g1_nav_bridge >/dev/null 2>&1 || { echo "找不到 g1_nav_bridge（scan_range_filter），MID360 层会零输入"; kill $NODE_PID; kill_nodes; exit 1; }
  ros2 run g1_nav_bridge scan_range_filter --ros-args -r __node:=$SCAN_NODE \
       -p use_sim_time:=true -p min_range:=0.25 -p crop_radius:=0.50 > "$OUT_DIR/$NAME.scanfilter.log" 2>&1 &
  SCAN_PID=$!
fi
sleep 5
ros2 lifecycle set /costmap configure >/dev/null || { echo "configure 失败，看 $OUT_DIR/$NAME.node.log"; kill $NODE_PID; kill_nodes; exit 1; }

# 顺序很重要：Costmap2DROS 在 activate() 里阻塞等 map->base_link 的 TF，
# 而 TF 只有回放开始后才有 —— 先拉回放(--delay 给 activate 留时间)，再 activate，否则死等。
# /tf_static 由单独的 latched 发布器补，不能靠回放：--start-offset 会跳过 bag 开头那几条
python3 "$WS/tools/nav2_debug/replay_tf_static.py" "$BAG" > "$OUT_DIR/$NAME.tfs.log" 2>&1 &
TFS_PID=$!
sleep 2

# 录制必须先于回放启动：/tf_static 和 /map 是 latched(transient local) 话题，
# 回放时只发一次，录制晚起来就永远录不到，后续所有要查 TF 的分析都会归零。
REC_SECS=$(python3 -c "print(int($DUR/$RATE)+14)")
timeout -s INT "$REC_SECS" ros2 bag record -o "$OUT_DIR/$NAME" --storage mcap \
    --max-cache-size 268435456 "${REC_TOPICS[@]}" > "$OUT_DIR/$NAME.rec.log" 2>&1 &
REC_PID=$!
sleep 3

echo "== 回放 $BAG  start=$START dur=$DUR rate=$RATE"
ros2 bag play "$BAG" --clock 100 --start-offset "$START" --playback-duration "$DUR" \
    --rate "$RATE" --delay 4 --topics "${PLAY_TOPICS[@]}" > "$OUT_DIR/$NAME.play.log" 2>&1 &
PLAY_PID=$!
sleep 5
ros2 lifecycle set /costmap activate >/dev/null || { echo "activate 失败"; kill $NODE_PID $PLAY_PID $REC_PID; exit 1; }
echo "== costmap 已 active，等录制结束"
wait $REC_PID

ros2 lifecycle set /costmap deactivate >/dev/null 2>&1
kill -INT $PLAY_PID $TFS_PID $FILT_PID $SCAN_PID 2>/dev/null; kill -INT $NODE_PID 2>/dev/null
sleep 1; kill -9 $PLAY_PID $NODE_PID $TFS_PID $FILT_PID $SCAN_PID 2>/dev/null
kill_nodes
echo "== 输出: $OUT_DIR/$NAME"
[[ -n "$FILT_PID" ]] && tail -n 2 "$OUT_DIR/$NAME.filter.log"
grep -ciE "warn|error" "$OUT_DIR/$NAME.node.log" | xargs -I{} echo "   节点日志里 warn/error 行数: {}"
