#!/bin/bash
# Thor 上执行：模拟前端切换 mapping -> localization，检查进程是否按预期切换。不发速度。
source /opt/ros/jazzy/setup.bash; source /opt/G1/lighting_ws/install/setup.bash
procs() { echo "  loc=$(pgrep -fc '[r]un_loc_online') slam=$(pgrep -fc '[r]un_slam_online') controller=$(pgrep -fc '[c]ontroller_server') bridge=$(pgrep -fc '[c]mdvel_to_sport')"; }
call() {
  echo "== mode_set $1 ($(date +%T))"
  timeout 120 ros2 service call /mode_set aid_robot_msgs/srv/StatusChange "{action: $1}" 2>&1 | tail -2
  sleep "$2"; procs
  timeout 5 ros2 topic echo /robot_status --once 2>/dev/null | grep data
}
echo "== 初始"; procs
call mapping 10
[ "$(pgrep -fc '[r]un_slam_online')" = 1 ] && [ "$(pgrep -fc '[r]un_loc_online')" = 0 ] && echo "PASS mapping" || echo "FAIL mapping"
call localization 20
[ "$(pgrep -fc '[r]un_loc_online')" = 1 ] && [ "$(pgrep -fc '[r]un_slam_online')" = 0 ] && [ "$(pgrep -fc '[c]ontroller_server')" = 1 ] && echo "PASS localization" || echo "FAIL localization"
grep -E "robot_status_manager.*(ERROR|Failed|Cannot)|stop_launch_client" /tmp/robot_launch.log 2>/dev/null | tail -5
