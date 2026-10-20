# 容器里交互 shell 的环境（g1_docker.sh shell 以 --rcfile 读取）。不读主机 ~/.bashrc：Orin 的 bashrc 会 source foxy。
# 与 Thor 上手动跑 ros2 前的步骤相同：jazzy → 工作区 install → dds_env.sh（spdp + 参与者上限 100，见 CLAUDE.md 2.4）。
source /opt/ros/jazzy/setup.bash
G1_WS="${G1_WS:-/opt/G1/lighting_ws}"
[ -f "$G1_WS/install/setup.bash" ] && source "$G1_WS/install/setup.bash"
_dds="$G1_WS/install/robot_bringup/share/robot_bringup/system/dds_env.sh"
[ -f "$_dds" ] && [ -f /home/unitree/unitree_ros2/setup.sh ] && source "$_dds"
unset _dds
export PS1='(g1-jazzy) \u@\h:\w\$ '
alias ll='ls -alF'
