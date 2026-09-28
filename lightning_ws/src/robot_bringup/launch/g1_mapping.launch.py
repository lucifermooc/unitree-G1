#!/usr/bin/env python3
"""原子化 G1 建图入口；Livox 驱动由 robot.launch.py 统一持有。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    lightning_launch = os.path.join(
        get_package_share_directory('lightning'), 'launch', 'g1_mapping.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('map_dir', default_value=os.path.expanduser('~/maps/new_map')),
        DeclareLaunchArgument('map_save_root', default_value=os.path.expanduser('~/maps')),
        DeclareLaunchArgument('with_ui', default_value='false'),
        DeclareLaunchArgument('with_2dui', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('pub_registered_scan', default_value='false',
                              description='发布 map 系配准点云 /lightning/registered_scan（诊断用）'),
        DeclareLaunchArgument('start_map_transform', default_value='true'),
        DeclareLaunchArgument('floor_height', default_value=''),
        DeclareLaunchArgument('min_obstacle_height', default_value=''),
        DeclareLaunchArgument('max_obstacle_height', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lightning_launch),
            launch_arguments={
                'start_livox': 'false',
                'map_path': LaunchConfiguration('map_dir'),
                'map_save_root': LaunchConfiguration('map_save_root'),
                'with_ui': LaunchConfiguration('with_ui'),
                'with_2dui': LaunchConfiguration('with_2dui'),
                'start_rviz': LaunchConfiguration('start_rviz'),
                'pub_registered_scan': LaunchConfiguration('pub_registered_scan'),
                'start_map_transform': LaunchConfiguration('start_map_transform'),
                'floor_height': LaunchConfiguration('floor_height'),
                'min_obstacle_height': LaunchConfiguration('min_obstacle_height'),
                'max_obstacle_height': LaunchConfiguration('max_obstacle_height'),
            }.items(),
        ),
    ])
