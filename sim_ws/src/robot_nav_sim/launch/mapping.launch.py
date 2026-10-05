"""2D 建图：仿真 + slam_toolbox（在线异步）+ RViz。

    ros2 launch robot_nav_sim mapping.launch.py robot:=diff world:=flat
    另开终端遥控：ros2 run teleop_twist_keyboard teleop_twist_keyboard
    保存地图：  ros2 run nav2_map_server map_saver_cli -f ~/my_map

两层楼分层建图：先在一楼建图并保存；让 Go2 上到二楼后，
用 sim:=false 重新启动本文件（重新开始一张地图），在二楼建图并另存。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("robot_nav_sim")
    nav_rviz = os.path.join(pkg, "rviz", "nav.rviz")
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "sim.launch.py")),
        condition=IfCondition(LaunchConfiguration("sim")),
        launch_arguments={
            "robot": LaunchConfiguration("robot"),
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
            "rviz": LaunchConfiguration("rviz"),
            "rviz_config": nav_rviz,
        }.items())
    slam = Node(package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox",
                output="screen", parameters=[os.path.join(pkg, "config", "slam_toolbox.yaml")])
    rviz_only = Node(package="rviz2", executable="rviz2", output="log", arguments=["-d", nav_rviz],
                     parameters=[{"use_sim_time": True}],
                     condition=UnlessCondition(LaunchConfiguration("sim")))
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="go2"),
        DeclareLaunchArgument("world", default_value="flat"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("sim", default_value="true", description="false：仿真已经在运行，只启动建图"),
        sim, slam, rviz_only,
    ])
