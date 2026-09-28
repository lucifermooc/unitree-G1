#!/usr/bin/env bash
# 第 1 步：仿真里用 Cartographer 建图。
# 启动 TurtleBot3 World 仿真 → Cartographer 建图 → 机器人自动探索 → 保存地图。
# 输出：maps/tb3_world.pbstream（Cartographer 定位用）、maps/tb3_world.pgm + .yaml（地图图片，看/标注用）
# 用法：bash sim/1_build_map.sh   （可设 EXPLORE_SECONDS=300 延长探索时间）
set -eo pipefail
source "$(dirname "$0")/common.sh"
cd "$SIM_DIR"

echo "== 1. 启动仿真 =="
start_bg sim ros2 launch launch/tb3_world_headless.launch.py
wait_for "激光雷达有数据" 12 has_scan

echo "== 2. 启动 Cartographer（建图模式）=="
start_bg cartographer ros2 launch turtlebot3_cartographer cartographer.launch.py \
  use_sim_time:=true use_rviz:=false \
  cartographer_config_dir:="$SIM_DIR/config" configuration_basename:=tb3_mapping.lua
wait_for "Cartographer 发布 map 坐标系" 12 has_tf map

echo "== 3. 自动探索 ${EXPLORE_SECONDS:-240} 秒（仿真时间）=="
python3 scripts/auto_explore.py --duration "${EXPLORE_SECONDS:-240}"

echo "== 4. 保存地图 =="
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}" >/dev/null
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$MAPS_DIR/tb3_world.pbstream', include_unfinished_submaps: true}" >/dev/null
sleep 3  # 等最后一次 /map 发布
ros2 run nav2_map_server map_saver_cli -f "$MAPS_DIR/tb3_world" --ros-args -p use_sim_time:=true \
  -p map_subscribe_transient_local:=false -p save_map_timeout:=10.0 \
  > "$LOG_DIR/map_saver.log" 2>&1
ls -la "$MAPS_DIR"
echo "✅ 建图完成"
