from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 声明所有启动参数
        DeclareLaunchArgument(
            'port',
            default_value='9090',
            description='Port for rosbridge websocket server'
        ),
        DeclareLaunchArgument(
            'address',
            default_value='',
            description='Address for rosbridge websocket server'
        ),
        DeclareLaunchArgument(
            'url_path',
            default_value='/',
            description='URL path for rosbridge websocket server'
        ),
        DeclareLaunchArgument(
            'ssl',
            default_value='false',
            description='Enable SSL encryption'
        ),
        DeclareLaunchArgument(
            'certfile',
            default_value='',
            description='SSL certificate file path'
        ),
        DeclareLaunchArgument(
            'keyfile',
            default_value='',
            description='SSL private key file path'
        ),
        DeclareLaunchArgument(
            'retry_startup_delay',
            default_value='5.0',
            description='Delay before retrying startup on failure'
        ),
        DeclareLaunchArgument(
            'fragment_timeout',
            default_value='600',
            description='Fragment timeout in seconds'
        ),
        DeclareLaunchArgument(
            'delay_between_messages',
            default_value='0.0',
            description='Delay between messages in seconds'
        ),
        DeclareLaunchArgument(
            'max_message_size',
            default_value='10000000',
            description='Maximum message size in bytes'
        ),
        DeclareLaunchArgument(
            'unregister_timeout',
            default_value='10.0',
            description='Unregister timeout in seconds'
        ),
        DeclareLaunchArgument(
            'use_compression',
            default_value='false',
            description='Enable message compression'
        ),
        DeclareLaunchArgument(
            'call_services_in_new_thread',
            default_value='false',
            description='Call services in new thread'
        ),
        DeclareLaunchArgument(
            'default_call_service_timeout',
            default_value='0.0',
            description='Default service call timeout in seconds'
        ),
        DeclareLaunchArgument(
            'send_action_goals_in_new_thread',
            default_value='false',
            description='Send action goals in new thread'
        ),
        DeclareLaunchArgument(
            'topics_glob',
            default_value='',
            description='Topics glob pattern'
        ),
        DeclareLaunchArgument(
            'services_glob',
            default_value='',
            description='Services glob pattern'
        ),
        DeclareLaunchArgument(
            'params_glob',
            default_value='',
            description='Parameters glob pattern'
        ),
        DeclareLaunchArgument(
            'params_timeout',
            default_value='5.0',
            description='Parameters timeout in seconds'
        ),
        DeclareLaunchArgument(
            'bson_only_mode',
            default_value='false',
            description='BSON only mode'
        ),

        # rosbridge_websocket SSL 版本
        GroupAction(
            condition=IfCondition(LaunchConfiguration('ssl')),
            actions=[
                Node(
                    package='rosbridge_server',
                    executable='rosbridge_websocket',
                    name='rosbridge_websocket',
                    output='screen',
                    parameters=[
                        {'certfile': LaunchConfiguration('certfile')},
                        {'keyfile': LaunchConfiguration('keyfile')},
                        {'port': LaunchConfiguration('port')},
                        {'address': LaunchConfiguration('address')},
                        {'url_path': LaunchConfiguration('url_path')},
                        {'retry_startup_delay': LaunchConfiguration('retry_startup_delay')},
                        {'fragment_timeout': LaunchConfiguration('fragment_timeout')},
                        {'delay_between_messages': LaunchConfiguration('delay_between_messages')},
                        {'max_message_size': LaunchConfiguration('max_message_size')},
                        {'unregister_timeout': LaunchConfiguration('unregister_timeout')},
                        {'use_compression': LaunchConfiguration('use_compression')},
                        {'call_services_in_new_thread': LaunchConfiguration('call_services_in_new_thread')},
                        {'default_call_service_timeout': LaunchConfiguration('default_call_service_timeout')},
                        {'send_action_goals_in_new_thread': LaunchConfiguration('send_action_goals_in_new_thread')},
                        {'topics_glob': LaunchConfiguration('topics_glob')},
                        {'services_glob': LaunchConfiguration('services_glob')},
                        {'params_glob': LaunchConfiguration('params_glob')}
                    ]
                )
            ]
        ),

        # rosbridge_websocket 非SSL 版本
        GroupAction(
            condition=UnlessCondition(LaunchConfiguration('ssl')),
            actions=[
                Node(
                    package='rosbridge_server',
                    executable='rosbridge_websocket',
                    name='rosbridge_websocket',
                    output='screen',
                    parameters=[
                        {'port': LaunchConfiguration('port')},
                        {'address': LaunchConfiguration('address')},
                        {'url_path': LaunchConfiguration('url_path')},
                        {'retry_startup_delay': LaunchConfiguration('retry_startup_delay')},
                        {'fragment_timeout': LaunchConfiguration('fragment_timeout')},
                        {'delay_between_messages': LaunchConfiguration('delay_between_messages')},
                        {'max_message_size': LaunchConfiguration('max_message_size')},
                        {'unregister_timeout': LaunchConfiguration('unregister_timeout')},
                        {'use_compression': LaunchConfiguration('use_compression')},
                        {'call_services_in_new_thread': LaunchConfiguration('call_services_in_new_thread')},
                        {'default_call_service_timeout': LaunchConfiguration('default_call_service_timeout')},
                        {'send_action_goals_in_new_thread': LaunchConfiguration('send_action_goals_in_new_thread')},
                        {'topics_glob': LaunchConfiguration('topics_glob')},
                        {'services_glob': LaunchConfiguration('services_glob')},
                        {'params_glob': LaunchConfiguration('params_glob')},
                        {'bson_only_mode': LaunchConfiguration('bson_only_mode')}
                    ]
                )
            ]
        ),

        # rosapi 节点
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            parameters=[
                {'topics_glob': LaunchConfiguration('topics_glob')},
                {'services_glob': LaunchConfiguration('services_glob')},
                {'params_glob': LaunchConfiguration('params_glob')},
                {'params_timeout': LaunchConfiguration('params_timeout')}
            ]
        )
    ])