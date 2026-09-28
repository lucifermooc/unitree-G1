# 手动开 rviz2 / ros2 CLI / 录包的终端里 source 本文件，让它们与整套栈用同一份 DDS 配置。
# robot.launch.py 启动的节点已自动使用（dds_config 参数），不需要再 source。
# 用法: source /opt/G1/lighting_ws/install/robot_bringup/share/robot_bringup/system/dds_env.sh
_dds_self=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source /home/unitree/unitree_ros2/setup.sh >/dev/null   # 官方环境：RMW + 本体网段
export CYCLONEDDS_URI="file://${_dds_self}/cyclonedds_g1.xml"   # 覆盖其默认 URI，见该文件注释
unset _dds_self
echo "CYCLONEDDS_URI=$CYCLONEDDS_URI"
