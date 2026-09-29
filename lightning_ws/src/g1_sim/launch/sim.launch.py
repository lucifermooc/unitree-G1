"""无硬件仿真：在没有 G1 的电脑/云主机上把前端和语义地图全流程跑起来。

    ros2 launch g1_sim sim.launch.py
    浏览器打开 http://localhost:8080 ，连接地址填 localhost

真实代码（原样启动，不做任何修改）：
  rosbridge_server        前端通信（ws://<ip>:9090），参数与 robot_bringup/rosbridge_websocket_launch.py 相同
  aid_robot_py map_transform_node    建图预览 /map → /map_base64
  g1_web 网页服务                    前端网页（新增包，真机同样使用）
  aid_robot_py map_manager_node      地图 / 位置点 / 禁行线数据库
  aid_robot_py waypoint_manage_node  单点导航、巡逻、/task_status
  semantic_map_ros semantic_map_server   语义地图（需要 Qdrant）
模拟（只在仿真里有）：
  g1_sim mock_robot       代替 robot_status_manager / lightning / 传感器 / Nav2 / 地图编辑

sim_home：仿真用的"家目录"，地图数据库和地图文件都放在 <sim_home>/maps 下，
不会碰到真实的 ~/maps。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    sim_home = os.path.expanduser(LaunchConfiguration("sim_home").perform(context))
    os.makedirs(os.path.join(sim_home, "maps"), exist_ok=True)
    env = {"HOME": sim_home}
    web_dir = LaunchConfiguration("web_dir").perform(context) or \
        os.path.join(get_package_share_directory("g1_web"), "www")
    actions = [
        # 与真机 rosbridge_websocket_launch.py 的默认值一致：服务调用不设超时（0.0）、不开新线程
        Node(package="rosbridge_server", executable="rosbridge_websocket", name="rosbridge_websocket",
             output="screen", parameters=[{"port": int(LaunchConfiguration("ws_port").perform(context)),
                                           "default_call_service_timeout": 0.0,
                                           "call_services_in_new_thread": False}]),
        Node(package="aid_robot_py", executable="map_transform_node", name="map_transform_node", output="screen"),
        Node(package="aid_robot_py", executable="map_manager_node", name="map_manager_server",
             output="screen", additional_env=env),
        Node(package="aid_robot_py", executable="waypoint_manage_node", name="waypoint_mange",
             output="screen"),
        Node(package="g1_sim", executable="mock_robot", name="mock_robot", output="screen",
             additional_env=env),
        Node(package="semantic_map_ros", executable="semantic_map_server", name="semantic_map_server",
             output="screen", condition=IfCondition(LaunchConfiguration("semantic")),
             parameters=[{"collection_prefix": "sim_semantic_map",
                          "log_file": os.path.join(sim_home, "maps", "semantic_map_log.jsonl")}]),
    ]
    actions.append(ExecuteProcess(
        cmd=["python3", "-m", "http.server", LaunchConfiguration("web_port").perform(context),
             "--directory", os.path.expanduser(web_dir)],
        output="log"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sim_home", default_value="~/g1_sim_home"),
        DeclareLaunchArgument("ws_port", default_value="9090"),
        DeclareLaunchArgument("web_dir", default_value="", description="前端目录；默认用 g1_web 包里装好的"),
        DeclareLaunchArgument("web_port", default_value="8080"),
        DeclareLaunchArgument("semantic", default_value="true", description="是否启动语义地图节点"),
        OpaqueFunction(function=_nodes),
    ])
