#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import ResetLaunchConfigurations
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():


    LDS_LAUNCH_FILE = '/msg_MID360_launch.py'
    lidar_pkg_dir = LaunchConfiguration(
        'lidar_pkg_dir',
        default=os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch_ROS2'))
    start_livox = LaunchConfiguration('start_livox')
    start_realsense = LaunchConfiguration('start_realsense')

    return LaunchDescription([

        DeclareLaunchArgument('start_livox', default_value='true'),
        DeclareLaunchArgument('start_realsense', default_value='true'),
        # 使用序列号绑定相机，与USB端口和/dev/video*编号无关。
        DeclareLaunchArgument(
            'realsense_serial_no', default_value="'347622073141'"),
        DeclareLaunchArgument('realsense_initial_reset', default_value='false'),
        DeclareLaunchArgument(
            'realsense_enable_color', default_value='true', choices=['true', 'false'],
            description='点云不再需要 color；纯避障场景可设 false 省带宽'),
        DeclareLaunchArgument(
            'realsense_depth_filters', default_value='false', choices=['true', 'false'],
            description='temporal 深度滤波（spatial 已默认开）；temporal 会用历史深度制造残影，默认关'),
        DeclareLaunchArgument('realsense_config', default_value=os.path.join(
            get_package_share_directory('robot_bringup'), 'param',
            'realsense_g1.yaml')),
        DeclareLaunchArgument('livox_config', default_value=os.path.join(
            get_package_share_directory('livox_ros_driver2'), 'config',
            'G1_MID360s_config.json')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            condition=IfCondition(start_livox),
            launch_arguments={'livox_config': LaunchConfiguration('livox_config')}.items(),
        ),
        # 该版本 rs_launch.py 会把上下文中所有 launch 参数当作相机参数检查。
        # 在局部作用域中清除 robot.launch.py 的 mode/use_sim_time 等参数，
        # 只传入 RealSense 官方支持的参数，避免无意义的 unsupported 警告。
        GroupAction(
            condition=IfCondition(start_realsense),
            actions=[
                ResetLaunchConfigurations({
                'camera_namespace': 'camera',
                'camera_name': 'camera',
                'serial_no': LaunchConfiguration('realsense_serial_no'),
                'initial_reset': LaunchConfiguration('realsense_initial_reset'),
                'config_file': LaunchConfiguration('realsense_config'),
                'enable_depth': 'true',
                # 点云已在 realsense_g1.yaml 里关闭纹理映射（stream_filter=0），
                # 因此点云不再依赖 color 流。这里保留 color 开关供调试使用，
                # 只做避障时可以 realsense_enable_color:=false 省一半 USB 带宽。
                'enable_color': LaunchConfiguration('realsense_enable_color'),
                'pointcloud.enable': 'true',
                'pointcloud.allow_no_texture_points': 'true',
                # 848x480 全分辨率点云约 34 万点/帧（~7MB，9Hz），best-effort 订阅大量丢帧、
                # STVL 体素滤波吃满 CPU，实测 2~3m 障碍几乎标不上。
                # 2026-09-24 由 4 倍改 6 倍降采样（212x120 → 144x80，点数约为原来的 44%）：
                # ≥4 倍时 librealsense 按块内非零深度取均值，孤立噪点被周围像素稀释。
                # 代价：2.5 m 处点距 ~3.5 cm，5 cm 体素里的点数可能不够 voxel_min_points 4。
                'decimation_filter.enable': 'true',
                'decimation_filter.filter_magnitude': '6',
                # 实测深度 >4m 的点 100% 落在地面以下 2.6~3.4m（地面反光导致深度偏大，
                # 视野上沿约 3.9m 就已打到地面），3~4m 也有 15%。只保留 3m 内，远处交给雷达。
                'clip_distance': '3.0',
                # 地面"不平、有空洞、打穿"是原始深度质量问题，STVL 参数管不到。
                # 顺序由 realsense-ros 固定：decimation -> spatial -> temporal -> pointcloud。
                # 不开 hole_filling_filter：它会编造深度，就是我们要消灭的那种假点。
                # 调参值在 realsense_g1.yaml（rs_launch 只转发白名单里的 launch 参数，其余静默丢弃）。
                # spatial（保边平滑）默认开；temporal 仍由 realsense_depth_filters 控制、默认关：
                # 它用历史帧深度维持当前像素，人走开后原位置还留着深度，正好制造残影。
                'spatial_filter.enable': 'true',
                'temporal_filter.enable': LaunchConfiguration('realsense_depth_filters'),
                'align_depth.enable': 'false',
                'enable_gyro': 'false',
                'enable_accel': 'false',
                'publish_tf': 'true',
                'output': 'screen',
                }),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        get_package_share_directory('realsense2_camera'),
                        'launch', 'rs_launch.py')),
                ),
            ],
        ),
        # /livox/lidar（CustomMsg）-> /livox/points：按 MID360 tag 剔除低置信度噪点，并做与 lightning-lm 预处理相同的
        # 抽点/盲区/雷达系高度裁剪（参数见 point_filter/config/point_filter.yaml）。LIO 仍直接订阅 /livox/lidar。
        Node(
            package='point_filter',
            executable='point_filter_node',
            name='point_filter',
            parameters=[os.path.join(
                get_package_share_directory('point_filter'), 'config', 'point_filter.yaml')],
            output='screen',
        )
    ])
