"""网页控制台的静态网页服务。和 robot.launch.py 分开启动，不影响原有后端。

    ros2 launch g1_web web.launch.py            # 浏览器打开 http://<机器人IP>:8080
    ros2 launch g1_web web.launch.py port:=80

网页通过 rosbridge（robot.launch.py 默认已启动，ws://<机器人IP>:9090）和机器人通信；
建图预览用的 /map_base64 由原有的 aid_robot_py/map_transform_node 发布（robot.launch.py 已启动）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("g1_web")
    www = os.path.join(share, "www")
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument("www_dir", default_value=www, description="前端文件目录"),
        ExecuteProcess(
            cmd=["python3", os.path.join(share, "serve.py"), LaunchConfiguration("port"),
                 "--directory", LaunchConfiguration("www_dir")],
            output="log"),
    ])
