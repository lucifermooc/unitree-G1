"""开机全局定位初始化。map_dir 必填（与 g1_localization.launch.py 的 map_dir 相同）。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('loc_global_init'), 'config', 'loc_global_init.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map_dir'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        Node(package='loc_global_init', executable='loc_global_init_node', name='loc_global_init', output='screen',
             parameters=[cfg, {'map_dir': LaunchConfiguration('map_dir'),
                               'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool)}]),
    ])
