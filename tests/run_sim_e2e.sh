#!/usr/bin/env bash
# 一键仿真测试：编译 → 启动 Qdrant → 启动仿真（真实后端 + 模拟机器人 + 新前端）→ 单元测试 + 浏览器端到端测试 → 收尾。
#
#   bash tests/run_sim_e2e.sh
#
# 需要：ROS2 Jazzy（含 rosbridge_suite、nav2_msgs）、colcon、Docker、Node.js + playwright（npm i -g playwright）、
#       semantic_map_ros 的 Python 依赖（pip install -r lightning_ws/src/semantic_map_ros/requirements.txt）。
# 可选环境变量：
#   ROS_SETUP   ROS 环境脚本，默认 /opt/ros/jazzy/setup.bash
#   WORK_DIR    编译产物和仿真数据目录，默认 /tmp/g1_sim_test（不会碰 ~/maps）
#   CHROME      浏览器可执行文件（不填用 playwright 自带的）
set -eo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
WORK_DIR="${WORK_DIR:-/tmp/g1_sim_test}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
mkdir -p "$WORK_DIR"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
[ -f "$ROS_SETUP" ] && source "$ROS_SETUP"

echo "== 1. 编译（只编译需要的 5 个包）=="
(cd "$REPO/lightning_ws" && colcon --log-base "$WORK_DIR/log" build --build-base "$WORK_DIR/build" \
  --install-base "$WORK_DIR/install" --packages-select aid_robot_msgs aid_robot_py semantic_map_ros g1_web g1_sim) | tail -3
source "$WORK_DIR/install/setup.bash"

echo "== 2. 单元测试（semantic_map_ros、g1_web）=="
(cd "$REPO/lightning_ws/src" && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=semantic_map_ros:g1_web \
  python3 -B -m pytest semantic_map_ros/test g1_web/test -q -p no:cacheprovider)

echo "== 3. 启动 Qdrant =="
if ! curl -fsS http://localhost:6333/ >/dev/null 2>&1; then
  docker compose -f "$REPO/lightning_ws/src/semantic_map_ros/docker/docker-compose.yml" up -d
  for _ in $(seq 30); do curl -fsS http://localhost:6333/ >/dev/null 2>&1 && break; sleep 1; done
fi

echo "== 4. 启动仿真 =="
rm -rf "$WORK_DIR/home"
setsid ros2 launch g1_sim sim.launch.py sim_home:="$WORK_DIR/home" \
  > "$WORK_DIR/sim.log" 2>&1 &
SIM_PID=$!
cleanup() { kill -INT -- "-$SIM_PID" 2>/dev/null; sleep 3; kill -KILL -- "-$SIM_PID" 2>/dev/null || true; }
trap cleanup EXIT
for _ in $(seq 60); do
  grep -q "embedding model loaded" "$WORK_DIR/sim.log" && curl -fsS http://localhost:8080/ >/dev/null 2>&1 && break
  sleep 2
done
grep -q "embedding model loaded" "$WORK_DIR/sim.log" || { echo "❌ 语义地图模型没加载起来，看 $WORK_DIR/sim.log"; exit 1; }

echo "== 5. 浏览器端到端测试 =="
NODE_PATH="${NODE_PATH:-$(npm root -g)}" SHOT_DIR="$WORK_DIR" node "$REPO/tests/e2e/console.e2e.js"
echo "截图在 $WORK_DIR/e2e_*.png，仿真日志 $WORK_DIR/sim.log"
