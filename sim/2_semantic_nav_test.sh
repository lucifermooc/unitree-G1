#!/usr/bin/env bash
# 第 2 步：全流程仿真测试（需要先跑过 1_build_map.sh，并且 Qdrant 已启动）。
# 启动仿真 → Cartographer 纯定位 + Nav2 → 语义地图建库 → 一句话 → 导航 → 检查是否到达对应点位。
# 结果：屏幕输出 + logs/semantic_nav_test_result.json
# 用法：bash sim/2_semantic_nav_test.sh
set -eo pipefail
source "$(dirname "$0")/common.sh"
cd "$SIM_DIR"
[ -f maps/tb3_world.pbstream ] || { echo "❌ 没有地图，先运行 bash sim/1_build_map.sh"; exit 1; }
curl -fsS http://localhost:6333/ >/dev/null || { echo "❌ Qdrant 没运行：cd semantic_map && docker compose up -d"; exit 1; }

# 仿真用自己的地点数据（坐标来自这张仿真地图）和单独的 collection，不影响正式数据
export SEMANTIC_MAP_DATA="$SIM_DIR/semantic_map_tb3.json"
export SEMANTIC_MAP_COLLECTION=semantic_map_tb3

echo "== 1. 语义地图建库（仿真点位）=="
(cd ../semantic_map && python3 build_index.py 2>&1 | grep -E "读取|已写入")

echo "== 2. 启动仿真 =="
start_bg sim ros2 launch launch/tb3_world_headless.launch.py
wait_for "激光雷达有数据" 12 has_scan

echo "== 3. 启动 Cartographer 纯定位 + Nav2 =="
start_bg nav ros2 launch launch/localization_nav.launch.py
nav2_active() {
  if grep -q "Aborting bringup" "$LOG_DIR/nav.log"; then
    echo "  ❌ Nav2 启动失败（看 logs/nav.log；常见原因：上次的 Nav2 进程没退干净）"; exit 1
  fi
  grep -q "Managed nodes are active" "$LOG_DIR/nav.log"
}
wait_for "Nav2 全部节点已激活" 120 nav2_active
wait_for "Cartographer 定位出 map 坐标系" 12 has_tf map

echo "== 4. 语义导航测试 =="
python3 scripts/semantic_nav_test.py
