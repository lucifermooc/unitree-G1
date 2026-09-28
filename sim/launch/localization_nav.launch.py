"""Cartographer 纯定位 + Nav2 导航。

- map_server：发布建图时保存的 maps/tb3_world.yaml（给 Nav2 的全局代价地图用）
- cartographer_node：加载 maps/tb3_world.pbstream，只做定位，发布 map -> odom
- nav2 navigation_launch.py：路径规划 + 控制（不启动 AMCL，定位完全交给 Cartographer）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAPS_DIR = os.path.join(SIM_DIR, 'maps')
CONFIG_DIR = os.path.join(SIM_DIR, 'config')


def generate_launch_description():
    nav2_bringup = get_package_share_directory('nav2_bringup')
    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        Node(
            package='nav2_map_server', executable='map_server', name='map_server', output='screen',
            parameters=[sim_time, {'yaml_filename': os.path.join(MAPS_DIR, 'tb3_world.yaml')}],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map', output='screen',
            parameters=[sim_time, {'autostart': True, 'node_names': ['map_server']}],
        ),
        Node(
            package='cartographer_ros', executable='cartographer_node', name='cartographer_node',
            output='screen', parameters=[sim_time],
            arguments=['-configuration_directory', CONFIG_DIR,
                       '-configuration_basename', 'tb3_localization.lua',
                       '-load_state_filename', os.path.join(MAPS_DIR, 'tb3_world.pbstream'),
                       '-load_frozen_state=true'],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
            launch_arguments={'use_sim_time': 'true', 'autostart': 'true',
                              'params_file': os.path.join(CONFIG_DIR, 'nav2_params.yaml')}.items(),
        ),
    ])
