#!/usr/bin/env python3
"""G1 唯一顶层入口。

本文件只负责编排已有的原子 launch，不在这里重复创建 Livox、Lightning、
Nav2、运动桥或 collision_monitor 节点。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _is_mode(*modes):
    """顶层直接拥有 SLAM/Nav2 的条件：模式匹配且未启动后台。

    start_backend:=true（默认，前端 UI 使用）时，定位/导航/建图统一由
    robot_status_manager_node 经 launch_manager 启停，前端才能切换模式；
    这里不再重复创建，避免两套同名进程。
    """
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration('requested_mode'), "' in ", repr(tuple(modes)),
        " and '", LaunchConfiguration('start_backend'), "'.lower() != 'true'",
    ]))


def _launch_args(context, *names, **fixed):
    """把顶层参数拼成子 launch 参数串（空值跳过，交给子 launch 默认值）。"""
    pairs = [(n, LaunchConfiguration(n).perform(context)) for n in names]
    pairs += list(fixed.items())
    return ' '.join(f'{k}:={v}' for k, v in pairs if v != '')


def _status_manager(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    return [Node(
        package='robot_bringup', executable='robot_status_manager_node',
        name='robot_status_manager_node', output='screen',
        # 本节点是 SLAM/Nav2 的唯一所有者：按 startup_mode 拉起，之后由前端
        # mode_set 切换建图/定位。参数透传保证与顶层直接启动时配置一致。
        parameters=[{
            'use_sim_time': use_sim_time,
            'manage_stack': True,
            'startup_mode': LaunchConfiguration('requested_mode').perform(context),
            'default_map_dir': LaunchConfiguration('map_dir').perform(context),
            'localization_launch_args': _launch_args(
                context, 'with_ui', 'with_2dui', 'start_rviz', 'pub_registered_scan'),
            # Nav2 的数值参数全在 aid_navigation2/param/nav2_params.yaml 里，只透传开关。
            'navigation_launch_args': _launch_args(
                context, 'use_realsense_obstacles', 'use_d435_mark_filter',
                'use_collision_monitor', 'use_keepout', start_bridge='true'),
            'mapping_launch_args': _launch_args(
                context, 'map_save_root', 'with_ui', 'with_2dui', 'start_rviz',
                'pub_registered_scan', 'floor_height', 'min_obstacle_height',
                'max_obstacle_height',
                # 后台已有 map_transform_node
                start_map_transform='false'),
        }])]


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')
    bridge_share = get_package_share_directory('g1_nav_bridge')

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_livox = LaunchConfiguration('start_livox')
    start_realsense = LaunchConfiguration('start_realsense')
    map_dir = LaunchConfiguration('map_dir')
    map_save_root = LaunchConfiguration('map_save_root')
    with_ui = LaunchConfiguration('with_ui')
    with_2dui = LaunchConfiguration('with_2dui')
    start_rviz = LaunchConfiguration('start_rviz')

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'sensor_driver.launch.py')),
        launch_arguments={
            'start_livox': start_livox,
            'livox_config': LaunchConfiguration('livox_config'),
            'start_realsense': start_realsense,
            'realsense_serial_no': LaunchConfiguration('realsense_serial_no'),
            'realsense_initial_reset': LaunchConfiguration('realsense_initial_reset'),
            'realsense_enable_color': LaunchConfiguration('realsense_enable_color'),
            'realsense_depth_filters': LaunchConfiguration('realsense_depth_filters'),
        }.items(),
    )

    battery_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, 'launch', 'battery_bridge.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_battery_bridge')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    rosbridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'rosbridge_websocket_launch.py')),
        condition=IfCondition(LaunchConfiguration('start_rosbridge')),
    )

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'g1_mapping.launch.py')),
        condition=_is_mode('mapping'),
        launch_arguments={
            'map_dir': map_dir,
            'map_save_root': map_save_root,
            'with_ui': with_ui,
            'with_2dui': with_2dui,
            'start_rviz': start_rviz,
            'pub_registered_scan': LaunchConfiguration('pub_registered_scan'),
            'start_map_transform': PythonExpression([
                "'false' if '", LaunchConfiguration('start_backend'), "' == 'true' else 'true'"
            ]),
            'floor_height': LaunchConfiguration('floor_height'),
            'min_obstacle_height': LaunchConfiguration('min_obstacle_height'),
            'max_obstacle_height': LaunchConfiguration('max_obstacle_height'),
        }.items(),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'g1_localization.launch.py')),
        condition=_is_mode('localization', 'navigation'),
        launch_arguments={
            'map_dir': map_dir,
            'with_ui': with_ui,
            'with_2dui': with_2dui,
            'start_rviz': start_rviz,
            'pub_registered_scan': LaunchConfiguration('pub_registered_scan'),
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, 'launch', 'g1_navigation.launch.py')),
        condition=_is_mode('navigation'),
        launch_arguments={
            'map': LaunchConfiguration('nav_map'),
            'use_realsense_obstacles': LaunchConfiguration('use_realsense_obstacles'),
            'use_d435_mark_filter': LaunchConfiguration('use_d435_mark_filter'),
            'use_collision_monitor': LaunchConfiguration('use_collision_monitor'),
            'use_keepout': LaunchConfiguration('use_keepout'),
            # 运动桥只由 g1_navigation.launch.py 创建一次。
            'start_bridge': 'true',
        }.items(),
    )

    backend_nodes = GroupAction(
        condition=IfCondition(LaunchConfiguration('start_backend')),
        actions=[
            Node(package='robot_bringup', executable='robot_pose_pub_node',
                 name='robot_pose_pub_node', output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
            Node(package='robot_bringup', executable='editor_map_node',
                 name='editor_map_node', output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
            Node(package='aid_robot_py', executable='map_manager_node',
                 name='map_manager_node', output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
            OpaqueFunction(function=_status_manager),
            Node(package='aid_robot_py', executable='map_transform_node',
                 name='map_transform_node', output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
            # 不要加 taskset 绑核：launch_manager 拉起的建图/定位/导航/RViz 都继承其亲和性，
            # 曾限制在 3-7 时建图进程与 RViz、Nav2 挤 5 核，接收线程被饿住，IMU 丢数据，地图多层墙。
            Node(package='aid_robot_py', executable='launch_manager_node',
                 name='launch_manager_node', output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
            # 禁行区地图节点只在 use_keepout:=true（默认）时启动：前端画禁行线走
            # /aid_draw_forbidden_line，由它生成 /keepout_filter_map 给 Nav2 keepout 层。
            Node(package='robot_bringup', executable='forbidden_map_create_node',
                 condition=IfCondition(LaunchConfiguration('use_keepout')),
                 name='forbidden_map_create_node', output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
            Node(package='aid_robot_py', executable='waypoint_manage_node',
                 name='waypoint_manage_node', output='screen',
                 parameters=[{'use_sim_time': use_sim_time}]),
        ],
    )

    # Thor 当前经过验证并实际使用的 Lightning/2D 地图目录。
    default_map_dir = '/opt/G1/lighting_ws/data/new_map'
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='navigation',
            choices=['base', 'mapping', 'localization', 'navigation'],
            description='互斥运行模式；默认启动定位、Nav2 与运动桥'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_livox', default_value='true'),
        DeclareLaunchArgument('start_realsense', default_value='true'),
        DeclareLaunchArgument('start_battery_bridge', default_value='true'),
        # 使用序列号绑定相机，与USB端口和/dev/video*编号无关。
        DeclareLaunchArgument(
            'realsense_serial_no', default_value="'347622073141'"),
        DeclareLaunchArgument('realsense_initial_reset', default_value='false'),
        DeclareLaunchArgument(
            'realsense_enable_color', default_value='true', choices=['true', 'false'],
            description='点云不再需要 color；纯避障场景可设 false 省带宽'),
        DeclareLaunchArgument('start_rosbridge', default_value='true'),
        DeclareLaunchArgument('start_backend', default_value='true'),
        DeclareLaunchArgument('with_ui', default_value='false'),
        DeclareLaunchArgument('with_2dui', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='false'),
        DeclareLaunchArgument('map_dir', default_value=default_map_dir),
        DeclareLaunchArgument('map_save_root', default_value=os.path.expanduser('~/maps')),
        DeclareLaunchArgument('nav_map', default_value=default_map_dir + '/map.yaml'),
        DeclareLaunchArgument('floor_height', default_value=''),
        DeclareLaunchArgument('min_obstacle_height', default_value=''),
        DeclareLaunchArgument('max_obstacle_height', default_value=''),
        # floor_z / base_floor_z / cloud_topic / sensor_frame / realsense_topic / robot_radius
        # 已于 2026-09-21 从导航链路移除：这些值直接写在 aid_navigation2/param/nav2_params.yaml
        # （雷达障碍源固定为 /lightning/registered_scan，需 pub_registered_scan:=true）。
        DeclareLaunchArgument(
            'use_realsense_obstacles', default_value='true',
            choices=['true', 'false'],
            description='将D435点云经STVL加入Nav2局部障碍层（话题/阈值见 nav2_params.yaml）'),
        DeclareLaunchArgument(
            'use_d435_mark_filter', default_value='true', choices=['true', 'false'],
            description='D435 标记源吃 g1_nav_bridge/d435_mark_filter 输出（4 邻域剔除孤立飞点，'
                        '参数见 g1_nav_bridge/config/d435_mark_filter.yaml）；false 改回原始点云'),
        DeclareLaunchArgument(
            'realsense_depth_filters', default_value='false', choices=['true', 'false'],
            description='D435 驱动内 temporal 深度滤波（spatial 已默认开）；temporal 会用历史深度制造残影，默认关'),
        DeclareLaunchArgument(
            'use_collision_monitor', default_value='false', choices=['true', 'false'],
            description='仅 navigation 模式使用；完成独立验收前默认关闭'),
        DeclareLaunchArgument('use_keepout', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument(
            'pub_registered_scan', default_value='true', choices=['true', 'false'],
            description='发布 map 系配准点云 /lightning/registered_scan（默认开启，便于在 RViz 直接看配准结果）'),
        DeclareLaunchArgument(
            'livox_config',
            default_value=os.path.join(
                get_package_share_directory('livox_ros_driver2'), 'config',
                'G1_MID360s_config.json')),
        # 子 launch（Lightning）内部也使用名为 mode 的参数，并会把它改成
        # localization/mapping。先保存用户请求的顶层模式，避免后续 Nav2
        # 条件和后台节点被子 launch 的同名参数污染。
        SetLaunchConfiguration('requested_mode', LaunchConfiguration('mode')),
        rosbridge_launch,
        sensor_launch,
        battery_launch,
        mapping_launch,
        localization_launch,
        navigation_launch,
        backend_nodes,
    ])
