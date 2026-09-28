import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('g1_nav_bridge'), 'config', 'nav_bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='g1_nav_bridge',
            executable='battery_state_bridge',
            name='g1_battery_state_bridge',
            output='screen',
            parameters=[config, {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
