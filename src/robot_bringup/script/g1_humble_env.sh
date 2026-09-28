#!/usr/bin/env bash

# Local development environment for the G1 stack (Ubuntu 22.04 / ROS 2 Humble).
source /opt/ros/humble/setup.bash
source /home/lmw/project/unitree/G1/unitree_ros2/cyclonedds_ws/install/setup.bash
source /home/lmw/project/unitree/G1/lightning_ws/install/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

G1_INTERFACE="$(ip -o -4 addr show | awk '$4 ~ /^192\.168\.123\./ {print $2; exit}')"
if [[ -n "${G1_INTERFACE}" ]]; then
  export CYCLONEDDS_URI="<CycloneDDS><Domain Id=\"any\"><General><Interfaces><NetworkInterface name=\"${G1_INTERFACE}\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
  echo "G1 Humble environment ready: interface=${G1_INTERFACE}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
else
  unset CYCLONEDDS_URI
  echo "G1 Humble environment ready without robot NIC (no 192.168.123.x address found)"
fi
