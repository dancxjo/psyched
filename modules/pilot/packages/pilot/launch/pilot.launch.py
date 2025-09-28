#!/usr/bin/env python3
"""Launch file for the pilot web interface node."""


from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    params_file = os.environ.get('PSH_MODULE_CONFIG', None)
    params = [params_file] if params_file else []
    pilot_node = Node(
        package='pilot',
        executable='pilot_backend',
        name='pilot_backend',
        output='screen',
        parameters=params
    )
    websocket_node = Node(
        package='pilot',
        executable='pilot_websocket_node',
        name='pilot_websocket_node',
        output='screen',
        parameters=params
    )
    host_health_node = Node(
        package='pilot',
        executable='host_health',
        name='host_health',
        output='screen',
        parameters=params
    )
    return LaunchDescription([
        pilot_node,
        websocket_node,
        host_health_node,
    ])