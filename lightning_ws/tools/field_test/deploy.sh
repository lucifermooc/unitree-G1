#!/bin/bash
# 本地执行：同步指定文件到 Thor 并只编译受影响的包。
# 用法: ./deploy.sh "pkg1 pkg2" file1 [file2 ...]   （文件路径相对工作区根，如 src/robot_bringup/...）
set -e
THOR=unitree@192.168.128.146; WS=/opt/G1/lighting_ws
PKGS="$1"; shift
cd "$(dirname "$0")/../.."
rsync -a --relative "$@" $THOR:$WS/
ssh -f $THOR "cd $WS && rm -f /tmp/deploy_build.log && nohup bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --packages-select $PKGS --cmake-args -DCMAKE_BUILD_TYPE=Release > /tmp/deploy_build.log 2>&1; echo EXIT=\$? >> /tmp/deploy_build.log' >/dev/null 2>&1 </dev/null &"
echo "building: $PKGS"
for i in $(seq 1 180); do
  R=$(ssh $THOR 'grep -E "^EXIT=" /tmp/deploy_build.log 2>/dev/null' || true); [ -n "$R" ] && break; sleep 5
done
ssh $THOR 'grep -E "error:|Finished|Failed|^EXIT=" /tmp/deploy_build.log | head -20'
