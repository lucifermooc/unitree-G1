"""Stop all current-user ROS processes belonging to the G1 workspace."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    script = os.path.join(
        get_package_share_directory('robot_bringup'),
        'script', 'stop_ros_processes.py')
    return LaunchDescription([
        DeclareLaunchArgument(
            'workspace', default_value='/opt/G1/lighting_ws',
            description='Only kill workspace processes and current-user ROS executables'),
        ExecuteProcess(
            cmd=['python3', script, '--workspace', LaunchConfiguration('workspace')],
            output='screen'),
    ])
