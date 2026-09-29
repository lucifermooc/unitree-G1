"""启动语义地图节点（和 robot.launch.py 分开启动，不影响原有后端）。

    ros2 launch semantic_map_ros semantic_map.launch.py
    ros2 launch semantic_map_ros semantic_map.launch.py use_fp16:=true   # Thor 上用 GPU
    ros2 launch semantic_map_ros semantic_map.launch.py text_in_action:=go   # ASR 话题输入找到就导航
    HF_HUB_OFFLINE=1 ros2 launch semantic_map_ros semantic_map.launch.py use_fp16:=true \
        model_name:=$HOME/models/bge-m3                                  # 用本地模型，不联网

前提：Qdrant 已启动（docker/docker-compose.yml），map_manager_server / waypoint_manage 在运行。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory("semantic_map_ros"), "config", "semantic_map.yaml")
    args = {
        "qdrant_host": "localhost",
        "qdrant_port": "6333",
        "use_fp16": "false",
        "score_threshold": "0.52",
        "text_in_action": "search",   # ASR 话题输入：search 只搜索；go 找到就导航
        # 本地模型目录可跳过联网检查（配合 HF_HUB_OFFLINE=1），如 ~/.cache/huggingface/hub/models--BAAI--bge-m3/snapshots/<commit>
        "model_name": "BAAI/bge-m3",
    }
    types = {"qdrant_port": int, "use_fp16": bool, "score_threshold": float, "qdrant_host": str,
             "text_in_action": str, "model_name": str}
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()] + [
            Node(
                package="semantic_map_ros",
                executable="semantic_map_server",
                name="semantic_map_server",
                output="screen",
                parameters=[config, {k: ParameterValue(LaunchConfiguration(k), value_type=types[k])
                                     for k in args}],
            ),
        ])
