from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    debounce_arg = DeclareLaunchArgument('debounce_seconds', default_value='3.0')
    window_arg = DeclareLaunchArgument('window_seconds', default_value='3.0')

    pilot_node = Node(
        package='pilot',
        executable='pilot_node',
        name='pilot',
        output='screen',
        parameters=[
            {
                'debounce_seconds': LaunchConfiguration('debounce_seconds'),
                'window_seconds': LaunchConfiguration('window_seconds'),
            }
        ],
    )

    return LaunchDescription([debounce_arg, window_arg, pilot_node])
