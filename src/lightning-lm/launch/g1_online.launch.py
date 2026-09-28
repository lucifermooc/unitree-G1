"""Shared G1 launch: URDF, optional Livox driver, and online SLAM/localization."""
import os
import tempfile
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    enabled = lambda key: value(key).lower() in ('1', 'true', 'yes', 'on')
    mode = value('mode')
    if mode not in ('mapping', 'localization'):
        raise RuntimeError('mode must be mapping or localization')
    with open(value('urdf'), encoding='utf-8') as stream:
        description = stream.read()
    ET.fromstring(description)  # Validate XML; robot_state_publisher owns static TF.
    with open(value('config'), encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    config['common']['livox_lidar_topic'] = value('lidar_topic')
    config['common']['imu_topic'] = value('imu_topic')
    # G2P5 uses height above the configured floor, not height below the LiDAR.
    # Keep these as launch arguments so the humanoid posture/site can be tuned
    # without rebuilding Lightning-LM.
    for argument, key in [('floor_height', 'floor_height'),
                          ('min_obstacle_height', 'min_th_floor'),
                          ('max_obstacle_height', 'max_th_floor')]:
        if value(argument):
            config['g2p5'][key] = float(value(argument))
    # 诊断话题开关只能经 yaml 下发：lightning 可执行文件用 gflags 解析 argv，
    # 传 --ros-args 会被当成未知 flag 直接报错退出。
    config['system']['pub_registered_scan'] = enabled('pub_registered_scan')
    config['system'].update(with_ui=enabled('with_ui'), with_2dui=enabled('with_2dui'),
                            map_path=os.path.abspath(os.path.expanduser(value('map_path'))),
                            map_save_root=os.path.abspath(
                                os.path.expanduser(value('map_save_root'))))
    if mode == 'localization':
        map_index = os.path.join(config['system']['map_path'], 'index.txt')
        if not os.path.isfile(map_index):
            raise RuntimeError(
                'Localization map is incomplete; expected map index: ' + map_index)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix='g1_lightning_', delete=False) as stream:
        yaml.safe_dump(config, stream)
        config_path = stream.name

    def cleanup(_context):
        if os.path.exists(config_path):
            os.unlink(config_path)
        return []

    executable = 'run_slam_online' if mode == 'mapping' else 'run_loc_online'
    actions = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='g1_robot_state_publisher', parameters=[{'robot_description': description}]),
        # gflags parses argv before ROS; ExecuteProcess avoids injected ROS arguments.
        ExecuteProcess(cmd=[os.path.join(get_package_prefix('lightning'), 'lib', 'lightning', executable),
                            '--config=' + config_path], output='screen'),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
    ]
    if enabled('start_rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', name='g1_mapping_rviz',
                            arguments=(['-d', value('rviz_config')] if value('rviz_config') else []), output='screen'))
    if value('start_livox').lower() == 'true':
        driver_config = value('livox_config')
        if not os.path.isfile(driver_config):
            raise RuntimeError('Livox config does not exist: ' + driver_config)
        actions.insert(1, Node(
            package='livox_ros_driver2', executable='livox_ros_driver2_node',
            parameters=[dict(xfer_format=1, multi_topic=0, data_src=0, publish_freq=10.0,
                             output_data_type=0, frame_id='mid360_link',
                             user_config_path=driver_config)],
            remappings=[('/livox/lidar', value('lidar_topic')), ('/livox/imu', value('imu_topic'))],
            output='screen'))
    return actions


def generate_launch_description():
    share = get_package_share_directory('lightning')
    defaults = dict(mode='mapping', urdf=os.path.join(share, 'urdf', 'g1.urdf'),
                    config=os.path.join(share, 'config', 'default_livox.yaml'),
                    map_path=os.path.expanduser('~/maps/new_map'),
                    map_save_root=os.path.expanduser('~/maps'),
                    lidar_topic='/livox/lidar', imu_topic='/livox/imu', start_livox='false',
                    with_ui='false', with_2dui='false', start_rviz='false',
                    pub_registered_scan='false',
                    rviz_config='',
                    floor_height='', min_obstacle_height='',
                    max_obstacle_height='',
                    livox_config=os.path.join(get_package_share_directory('livox_ros_driver2'),
                                              'config', 'G1_MID360s_config.json'))
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=setup)])
