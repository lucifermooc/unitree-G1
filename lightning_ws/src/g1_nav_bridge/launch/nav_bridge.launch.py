import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    config = os.path.join(
        get_package_share_directory('g1_nav_bridge'), 'config', 'nav_bridge.yaml')
    # 临时试参数用的覆盖文件：叠在 nav_bridge.yaml 之后，同名参数以它为准（常驻值直接改 nav_bridge.yaml）。
    overlay = LaunchConfiguration('bridge_params_overlay').perform(context)
    params = [config] + ([overlay] if overlay else [])
    return [
        # tf_to_current_pose 按 2026-09-22 决定不再启动（前端不需要它发的 PoseStamped）。
        # 文件本身已补上 __main__ 守卫，需要时可手动 ros2 run 起来。
        Node(package='g1_nav_bridge', executable='sport_to_odom',
             name='g1_sport_to_odom', output='screen', parameters=params),
        # 走路蟹行补偿：发 TF base_link -> base_link_nav（crab_deg 0 = 恒等），nav2_params.yaml 的 local costmap 用它，见 crab_comp.hpp。
        Node(package='g1_nav_bridge', executable='crab_comp_frame',
             name='g1_crab_comp_frame', output='screen', parameters=params),
        Node(
            package='g1_nav_bridge',
            executable='cmdvel_to_sport',
            name='g1_cmdvel_to_sport',
            output='screen',
            parameters=params + [{
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_safe'),
        DeclareLaunchArgument('bridge_params_overlay', default_value='',
                              description='叠加在 nav_bridge.yaml 之后的本机参数文件（空 = 不叠加）'),
        OpaqueFunction(function=_nodes),
    ])
