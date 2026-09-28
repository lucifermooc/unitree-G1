#!/usr/bin/env bash
set -e
source /home/unitree/unitree_ros2/setup.sh
source /opt/G1/lighting_ws/install/setup.bash
exec ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:=1 -p multi_topic:=0 -p data_src:=0 \
  -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=mid360_link \
  -p user_config_path:=/opt/G1/lighting_ws/src/livox_ros_driver2/config/G1_MID360s_config.json
