# 手动开 rviz2 / ros2 CLI / 录包的终端里 source 本文件，让它们与整套栈用同一份 DDS 配置。
# robot.launch.py 本身不设置 DDS，全看启动它的 shell；开机自启入口 script/g1_autostart.sh 已 source 本文件。
# 用法: source /opt/G1/lighting_ws/install/robot_bringup/share/robot_bringup/system/dds_env.sh
_dds_self=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source /home/unitree/unitree_ros2/setup.sh >/dev/null   # 官方环境：RMW + 本体网段
export CYCLONEDDS_URI="file://${_dds_self}/cyclonedds_g1.xml"   # 覆盖其默认 URI，见该文件注释
unset _dds_self
echo "CYCLONEDDS_URI=$CYCLONEDDS_URI"
