"""启动语义地图节点（和 robot.launch.py 分开启动，不影响原有后端）。

    ros2 launch semantic_map_ros semantic_map.launch.py
    ros2 launch semantic_map_ros semantic_map.launch.py use_fp16:=true   # Thor 上用 GPU
    ros2 launch semantic_map_ros semantic_map.launch.py text_in_action:=search   # ASR 话题输入只搜索不导航（默认 go）
    HF_HUB_OFFLINE=1 ros2 launch semantic_map_ros semantic_map.launch.py use_fp16:=true \
        model_name:=$HOME/models/bge-m3                                  # 用本地模型，不联网

前提：Qdrant 已启动（docker/docker-compose.yml），map_manager_server / waypoint_manage 在运行。
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {
    "qdrant_host": "localhost",
    "qdrant_port": "6333",
    "qdrant_path": "",            # 非空 = 本地文件模式（没有 Docker/Qdrant 服务的机器用）
    "use_fp16": "false",
    "score_threshold": "0.52",
    "text_in_action": "go",       # ASR 话题输入：go 找到就导航；search 只搜索（launch 参数会覆盖 yaml，两处保持一致）
    # 本地模型目录可跳过联网检查（配合 HF_HUB_OFFLINE=1），如 ~/.cache/huggingface/hub/models--BAAI--bge-m3/snapshots/<commit>
    "model_name": "BAAI/bge-m3",
}
TYPES = {"qdrant_port": int, "score_threshold": float, "use_fp16": lambda v: v.strip().lower() in ("true", "1", "yes")}


def _node(context):
    # 把 yaml 和 launch 参数合成一份再传给节点。分成"yaml 文件 + launch 参数"两份传的话，Foxy 上 yaml 里按节点名写的
    # 参数会盖过 launch 参数（Humble / Jazzy 上是 launch 参数生效），同一个 launch 在不同版本上行为不一样。
    config = os.path.join(get_package_share_directory("semantic_map_ros"), "config", "semantic_map.yaml")
    with open(config, encoding="utf-8") as f:
        params = yaml.safe_load(f)["semantic_map_server"]["ros__parameters"]
    for k in ARGS:
        params[k] = TYPES.get(k, str)(LaunchConfiguration(k).perform(context))
    return [Node(package="semantic_map_ros", executable="semantic_map_server", name="semantic_map_server",
                 output="screen", parameters=[params])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()]
                             + [OpaqueFunction(function=_node)])
