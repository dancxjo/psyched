#!/usr/bin/env python3
"""Launch file for the pilot web interface node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for pilot node."""
    
    # Launch arguments
    web_port_arg = DeclareLaunchArgument(
        'web_port', 
        default_value=EnvironmentVariable(name='PILOT_WEB_PORT', default_value='8080'),
        description='Port for the web interface'
    )
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value=EnvironmentVariable(name='PILOT_IMU_TOPIC', default_value='/imu'),
        description='IMU topic (sensor_msgs/Imu) for UI overlay'
    )
    gps_fix_topic_arg = DeclareLaunchArgument(
        'gps_fix_topic',
        default_value=EnvironmentVariable(name='PILOT_GPS_FIX_TOPIC', default_value='/gps/fix'),
        description='GPS NavSatFix topic to display on Pilot UI'
    )
    
    websocket_port_arg = DeclareLaunchArgument(
        'websocket_port',
        default_value=EnvironmentVariable(name='PILOT_WS_PORT', default_value='8081'), 
        description='Port for WebSocket connections'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value=EnvironmentVariable(name='PILOT_CMD_VEL_TOPIC', default_value='/cmd_vel'),
        description='Topic name for publishing cmd_vel messages'
    )
    conversation_topic_arg = DeclareLaunchArgument(
        'conversation_topic',
        default_value=EnvironmentVariable(name='PILOT_CONVERSATION_TOPIC', default_value='/conversation'),
        description='Topic name for chat conversation messages'
    )

    voice_topic_arg = DeclareLaunchArgument(
        'voice_topic',
        default_value=EnvironmentVariable(name='PILOT_VOICE_TOPIC', default_value='/voice'),
        description='Topic name for publishing voice text'
    )
    
    host_arg = DeclareLaunchArgument(
        'host',
        default_value=EnvironmentVariable(name='PILOT_HOST', default_value='0.0.0.0'),
        description='Host address to bind the web server (0.0.0.0 for all interfaces)'
    )

    enable_http_arg = DeclareLaunchArgument(
        'enable_http',
        default_value=EnvironmentVariable(name='PILOT_ENABLE_HTTP', default_value='true'),
        description='Enable the HTTP static server'
    )

    enable_ws_arg = DeclareLaunchArgument(
        'enable_websocket',
        default_value=EnvironmentVariable(name='PILOT_ENABLE_WS', default_value='false'),
        description='Enable the WebSocket inside pilot_node (set false when using separate websocket node)'
    )

    run_separate_ws_arg = DeclareLaunchArgument(
        'run_separate_websocket',
        default_value=EnvironmentVariable(name='PILOT_RUN_SEPARATE_WS', default_value='true'),
        description='Launch the separate websocket node (recommended)'
    )
    host_health_enable_arg = DeclareLaunchArgument(
        'enable_host_health',
        default_value=EnvironmentVariable(name='PILOT_ENABLE_HOST_HEALTH', default_value='true'),
        description='Enable the host health publisher node'
    )
    host_health_period_arg = DeclareLaunchArgument(
        'host_health_period_sec',
        default_value=EnvironmentVariable(name='PILOT_HOST_HEALTH_PERIOD', default_value='2.0'),
        description='Publish period for host health metrics'
    )
    host_health_topic_arg = DeclareLaunchArgument(
        'host_health_topic',
        default_value=EnvironmentVariable(name='PILOT_HOST_HEALTH_TOPIC', default_value='auto'),
        description='Topic for host health metrics'
    )
    
    # Pilot node
    pilot_node = Node(
        package='pilot',
        executable='pilot_node',
        name='pilot_node',
        output='screen',
        parameters=[{
            'web_port': LaunchConfiguration('web_port'),
            'websocket_port': LaunchConfiguration('websocket_port'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'voice_topic': LaunchConfiguration('voice_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'gps_fix_topic': LaunchConfiguration('gps_fix_topic'),
            'conversation_topic': LaunchConfiguration('conversation_topic'),
            'host': LaunchConfiguration('host'),
            'enable_http': LaunchConfiguration('enable_http'),
            'enable_websocket': LaunchConfiguration('enable_websocket'),
        }]
    )

    # Separate websocket node (recommended)
    websocket_node = Node(
        package='pilot',
        executable='pilot_websocket_node',
        name='pilot_websocket_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_separate_websocket')),
        parameters=[{
            'websocket_port': LaunchConfiguration('websocket_port'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'voice_topic': LaunchConfiguration('voice_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'gps_fix_topic': LaunchConfiguration('gps_fix_topic'),
            'conversation_topic': LaunchConfiguration('conversation_topic'),
            'host': LaunchConfiguration('host'),
            'host_health_topic': LaunchConfiguration('host_health_topic'),
        }]
    )

    host_health_node = Node(
        package='pilot',
        executable='host_health',
        name='host_health',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_host_health')),
        parameters=[{
            'publish_topic': LaunchConfiguration('host_health_topic'),
            'period_sec': LaunchConfiguration('host_health_period_sec'),
        }]
    )
    
    return LaunchDescription([
        web_port_arg,
        websocket_port_arg,
        cmd_vel_topic_arg,
        voice_topic_arg,
    conversation_topic_arg,
        host_arg,
    imu_topic_arg,
        gps_fix_topic_arg,
        enable_http_arg,
        enable_ws_arg,
        run_separate_ws_arg,
        host_health_enable_arg,
        host_health_period_arg,
        host_health_topic_arg,
        pilot_node,
        websocket_node,
        host_health_node,
    ])