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
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_safe'),
        # tf_to_current_pose 按 2026-09-22 决定不再启动（前端不需要它发的 PoseStamped）。
        # 文件本身已补上 __main__ 守卫，需要时可手动 ros2 run 起来。
        Node(package='g1_nav_bridge', executable='sport_to_odom',
             name='g1_sport_to_odom', output='screen', parameters=[config]),
        Node(
            package='g1_nav_bridge',
            executable='cmdvel_to_sport',
            name='g1_cmdvel_to_sport',
            output='screen',
            parameters=[config, {
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            }],
        )
    ])
