"""统一导航入口；不启动 SLAM、URDF 或任何 TF 发布器。

Nav2 与碰撞停车的全部参数写在 yaml 里（aid_navigation2/param/nav2_params.yaml、
robot_bringup/param/collision_monitor_params.yaml），这里只透传地图和三个开关。
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    bridge = Path(get_package_share_directory('g1_nav_bridge'))
    nav = Path(get_package_share_directory('aid_navigation2'))
    args = ['map', 'use_collision_monitor', 'use_keepout', 'use_realsense_obstacles',
            'use_d435_mark_filter']
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/opt/G1/lighting_ws/data/new_map/map.yaml',
                              description='导航栅格地图 YAML 的绝对路径，由 map_server 加载'),
        DeclareLaunchArgument('use_collision_monitor', default_value='true',
                              choices=['true', 'false'],
                              description='false：跳过额外碰撞停车，桥接直接订阅 /cmd_vel'),
        DeclareLaunchArgument('use_keepout', default_value='true', choices=['true', 'false'],
                              description='有真实禁行区 mask 发布端时才开启'),
        DeclareLaunchArgument('use_realsense_obstacles', default_value='true',
                              choices=['true', 'false'],
                              description='将 D435 点云经 STVL 加入 Nav2 局部障碍层'),
        DeclareLaunchArgument('use_d435_mark_filter', default_value='true',
                              choices=['true', 'false'],
                              description='D435 标记源吃 d435_mark_filter 输出（4 邻域剔除孤立飞点，参数见 g1_nav_bridge/config/d435_mark_filter.yaml）；false 改回原始点云'),
        DeclareLaunchArgument('start_bridge', default_value='true',
                              description='robot.launch 已启动 bridge 时必须设为 false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bridge/'launch/nav_bridge.launch.py')),
                                 launch_arguments={'cmd_vel_topic': PythonExpression([
                                     "'/cmd_vel_safe' if '", LaunchConfiguration('use_collision_monitor'),
                                     "' == 'true' else '/cmd_vel'"]) }.items(),
                                 condition=IfCondition(LaunchConfiguration('start_bridge'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav/'launch/g1_navigation_direct.launch.py')),
                                 launch_arguments={k: LaunchConfiguration(k) for k in args}.items()),
    ])
