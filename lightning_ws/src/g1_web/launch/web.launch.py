"""网页控制台：静态网页服务 + 建图预览转换节点。和 robot.launch.py 分开启动，不影响原有后端。

    ros2 launch g1_web web.launch.py            # 浏览器打开 http://<机器人IP>:8080
    ros2 launch g1_web web.launch.py port:=80

网页通过 rosbridge（robot.launch.py 默认已启动，ws://<机器人IP>:9090）和机器人通信。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    www = os.path.join(get_package_share_directory("g1_web"), "www")
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("www_dir", default_value=www, description="前端文件目录"),
        DeclareLaunchArgument("serve_web", default_value="true"),
        DeclareLaunchArgument("map_preview", default_value="true", description="是否启动 /map → /map_base64"),
        ExecuteProcess(
            cmd=["python3", "-m", "http.server", LaunchConfiguration("port"),
                 "--directory", LaunchConfiguration("www_dir")],
            condition=IfCondition(LaunchConfiguration("serve_web")), output="log"),
        Node(package="g1_web", executable="map_preview_bridge", name="map_preview_bridge", output="screen",
             condition=IfCondition(LaunchConfiguration("map_preview"))),
    ])
