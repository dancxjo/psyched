#!/usr/bin/env python3
"""Launch file for the pilot web interface node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
            'host': LaunchConfiguration('host'),
        }]
    )
    
    return LaunchDescription([
        web_port_arg,
        websocket_port_arg,
        cmd_vel_topic_arg,
        voice_topic_arg,
        host_arg,
        pilot_node,
    ])