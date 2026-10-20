# 手动开 rviz2 / ros2 CLI / 录包的终端里 source 本文件，让它们与整套栈用同一份 DDS 配置。
# robot.launch.py 本身不设置 DDS，全看启动它的 shell；开机自启入口 script/g1_autostart.sh 已 source 本文件。
# 用法: source /opt/G1/lighting_ws/install/robot_bringup/share/robot_bringup/system/dds_env.sh
_dds_self=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source /home/unitree/unitree_ros2/setup.sh >/dev/null   # 官方环境：RMW + 本体网段
# 本机网卡名与 Thor（enP2p1s0）不同时（如 Orin 是 eth0），放一份本机覆盖 /opt/G1/cyclonedds_g1.local.xml（不在仓库里）
_dds_xml="${G1_DDS_XML:-/opt/G1/cyclonedds_g1.local.xml}"
[ -f "$_dds_xml" ] || _dds_xml="${_dds_self}/cyclonedds_g1.xml"
export CYCLONEDDS_URI="file://${_dds_xml}"   # 覆盖其默认 URI，见 cyclonedds_g1.xml 注释
unset _dds_self _dds_xml
echo "CYCLONEDDS_URI=$CYCLONEDDS_URI"
