#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('robot_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')
    autostart = LaunchConfiguration('autostart')
    lifecycle_nodes = ['collision_monitor']

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'param', 'collision_monitor_params.yaml'),
        description='Full path to the ROS2 parameters file to use')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', 
        default_value='info',
        description='log level')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically configure & activate lifecycle nodes')

    # Collision Monitor is a lifecycle node; it must be configured & activated
    collision_monitor_node = LifecycleNode(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        respawn=True,
        respawn_delay=2.0,
        namespace='',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
        ],
        arguments=['--ros-args', '--log-level', log_level])

    # Automatically bring collision_monitor through lifecycle transitions
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            {'autostart': ParameterValue(autostart, value_type=bool)},
            {'node_names': lifecycle_nodes},
        ],
    )

    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_autostart_cmd)
    
    # Add collision monitor node
    ld.add_action(collision_monitor_node)
    ld.add_action(lifecycle_manager_node)
    
    return ld
