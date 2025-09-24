"""Launch file that starts the Psyched regime manager."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package='psyched_bt',
                executable='regime_manager',
                name='psyched_regime_manager',
                output='screen',
            )
        ]
    )
