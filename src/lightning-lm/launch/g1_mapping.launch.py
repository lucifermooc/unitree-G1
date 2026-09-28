from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_map_transform', default_value='true'),
        DeclareLaunchArgument('map_save_root', default_value=os.path.expanduser('~/maps')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('lightning'),
                                                       'launch', 'g1_online.launch.py')),
            launch_arguments={
                'mode': 'mapping',
                'map_save_root': LaunchConfiguration('map_save_root'),
            }.items()),
        Node(
            package='aid_robot_py',
            executable='map_transform_node',
            name='map_transform_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_map_transform')),
        ),
    ])
