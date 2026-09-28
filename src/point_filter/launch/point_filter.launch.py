"""单独启动 point_filter（整套栈里由 robot_bringup/sensor_driver.launch.py 启动）。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = os.path.join(get_package_share_directory('point_filter'), 'config', 'point_filter.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default),
        Node(package='point_filter', executable='point_filter_node', name='point_filter',
             parameters=[LaunchConfiguration('params_file')], output='screen'),
    ])
