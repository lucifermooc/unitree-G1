"""G1 导航入口：直接使用 map -> base_link，不启动 SLAM / URDF / TF 发布器。

参数全部写在两个 yaml 里，它们就是真机运行值：
  aid_navigation2/param/nav2_params.yaml
  robot_bringup/param/collision_monitor_params.yaml
本文件不再计算/改写任何数值（2026-09-21 起取消 g1_config.py 注入层），
只按开关删层：use_keepout=false 去掉 keepout_layer，
use_realsense_obstacles=false 去掉两张图的 stvl_voxel_layer（D435 专用层；MID360 在 mid360_voxel_layer，保留）。
D435 两个标记源（realsense_mark / realsense_mark_tall）吃 g1_nav_bridge/d435_mark_filter 的输出（默认开）：
读深度图，按 4 邻域一致性剔除孤立飞点（移植自 Haier Aurora 驱动，参数见 g1_nav_bridge/config/d435_mark_filter.yaml）。
use_d435_mark_filter=false 时两个标记源改回原始点云（A/B 用）。realsense_clear 始终吃原始点云。
MID360 障碍在 mid360_voxel_layer（STVL，2026-09-24 从 2D obstacle_layer 迁移），STVL 没有 obstacle_min_range，
所以始终另起 g1_nav_bridge/scan_range_filter 剔除离雷达 0.25 m 内的机身回波，以及 base_link 水平 0.5 m（足迹半径）内的点。
开关都是默认值时，yaml 原封不动交给 nav2，`ros2 param get` 与文件一一对应。
"""
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RAW_D435 = '/camera/camera/depth/color/points'
# D435 两个标记源的输入：d435_mark_filter 按 4 邻域一致性剔除孤立飞点后的点云（清除源仍吃 RAW_D435）。
# 2026-09-24 move_0924：base_link 0.5 m 内的 D435 标记 109 次，61 次来自一致邻居 <3 个的孤立像素（镜头前 ~0.2 m、
# 高 ~1.0 m），落在清除视锥外，global 里贴着 base_link 挂满 voxel_decay；剔除后孤立点 0 个，真实物体 2D 格召回 97~100%。
# 飞点高 ~1.0 m，高物体源（0.7~1.8 m）也会标它，所以两个标记源都要吃过滤后的点云。
FILTERED_D435 = '/camera/camera/depth/mark_points'
MARK_SOURCES = ('realsense_mark', 'realsense_mark_tall')
MID360_RAW = '/lightning/registered_scan'
MID360_NAV = '/lightning/registered_scan_nav'
MID360_MIN_RANGE = 0.25   # 原 obstacle_layer.obstacle_min_range：更近的是打在头上的机身回波
# base_link 水平半径内的点在进 STVL 前删掉（= yaml 里的足迹半径，测试守着）：footprint 自清只遮当前足迹、不删体素，
# 身边每帧都有的回波会在机器人走过的位置留下一串障碍（2026-09-24 实测）。
MID360_CROP_RADIUS = 0.50
COSTMAPS = ('local_costmap', 'global_costmap')


def drop_layers(nav, layers):
    """从两张代价地图里摘掉指定层；不改源文件，返回是否发生改动。"""
    changed = False
    for name in ('local_costmap', 'global_costmap'):
        params = nav[name][name]['ros__parameters']
        for layer in layers:
            if layer in params.get('plugins', []):
                params['plugins'] = [p for p in params['plugins'] if p != layer]
                params.pop(layer, None)
                changed = True
    return changed


def raw_mark(nav):
    """use_d435_mark_filter:=false：两张图的 D435 标记源改回原始点云（A/B 用，标记距离不变）；返回是否改动。"""
    changed = False
    for name in COSTMAPS:
        stvl = nav[name][name]['ros__parameters'].get('stvl_voxel_layer', {})
        for src in MARK_SOURCES:
            mark = stvl.get(src)
            if mark and mark['topic'] != RAW_D435:
                mark['topic'] = RAW_D435
                changed = True
    return changed


def setup(context):
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    nav_share = Path(get_package_share_directory('aid_navigation2'))
    robot_share = Path(get_package_share_directory('robot_bringup'))

    map_file = Path(get('map')).expanduser().resolve()
    if not map_file.is_file():
        raise ValueError(f'导航地图不存在：{map_file}')

    nav_file = Path(get('params_file') or nav_share / 'param/nav2_params.yaml')
    safety_file = Path(get('collision_params_file')
                       or robot_share / 'param/collision_monitor_params.yaml')

    use_d435 = get('use_realsense_obstacles') == 'true'
    layers = (([] if get('use_keepout') == 'true' else ['keepout_layer'])
              + ([] if use_d435 else ['stvl_voxel_layer']))
    use_filter = use_d435 and get('use_d435_mark_filter') == 'true'
    use_raw = use_d435 and not use_filter
    if layers or use_raw:
        nav = yaml.safe_load(nav_file.read_text())
        changed = drop_layers(nav, layers)
        changed = (use_raw and raw_mark(nav)) or changed
        if changed:
            nav_file = Path(tempfile.mkdtemp(prefix='g1_nav_')) / 'nav.yaml'
            nav_file.write_text(yaml.safe_dump(nav, sort_keys=False, allow_unicode=True))

    filter_params = Path(get_package_share_directory('g1_nav_bridge')) / 'config/d435_mark_filter.yaml' \
        if use_filter else None
    mark_filter = [Node(
        package='g1_nav_bridge', executable='d435_mark_filter', name='d435_mark_filter',
        output='screen',
        # 节点一退出，两个 D435 标记源整体断流（含 1.5 m 内近场，清除不受影响）；respawn 只缩短断流时间，不是回退。
        respawn=True, respawn_delay=1.0,
        parameters=[str(filter_params)])] if use_filter else []

    # 两张图的 MID360 源都吃它：节点一退出 MID360 障碍整体断流，respawn 只缩短断流时间
    scan_filter = Node(
        package='g1_nav_bridge', executable='scan_range_filter', name='scan_range_filter',
        output='screen', respawn=True, respawn_delay=1.0,
        parameters=[{'input_topic': MID360_RAW, 'output_topic': MID360_NAV,
                     'min_range': MID360_MIN_RANGE,
                     'base_frame': 'base_link', 'crop_radius': MID360_CROP_RADIUS}])

    return mark_filter + [
        scan_filter,
        LogInfo(msg=f'G1 nav2 参数：{nav_file}（已去掉的层：{layers or "无"}，'
                    f'D435：{"开" if use_d435 else "关"}，D435 标记过滤：{"开" if use_filter else "关"}）'),
        LogInfo(msg='注意：collision_monitor 已关闭，仍保留 Nav2 障碍层；运动必须手动使能。',
                condition=IfCondition('true' if get('use_collision_monitor') == 'false' else 'false')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav_share / 'launch/navigation2.launch.py')),
            launch_arguments={'map': str(map_file), 'params_file': str(nav_file),
                              'use_sim_time': 'false', 'use_composition': 'False'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(robot_share / 'launch/collision_monitor.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_collision_monitor')),
            launch_arguments={'params_file': str(safety_file), 'use_sim_time': 'false'}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', description='导航栅格地图 YAML 的绝对路径'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='留空用 aid_navigation2/param/nav2_params.yaml'),
        DeclareLaunchArgument('collision_params_file', default_value='',
                              description='留空用 robot_bringup/param/collision_monitor_params.yaml'),
        DeclareLaunchArgument('use_collision_monitor', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_keepout', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_realsense_obstacles', default_value='true', choices=['true', 'false'],
                              description='false：两张图的 STVL 去掉 D435 源，只靠 MID360'),
        DeclareLaunchArgument('use_d435_mark_filter', default_value='true', choices=['true', 'false'],
                              description='false：D435 标记源改回原始点云（不剔除孤立飞点，A/B 用）'),
        OpaqueFunction(function=setup)])
