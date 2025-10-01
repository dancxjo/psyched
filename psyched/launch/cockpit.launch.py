from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="psyched",
                executable="cockpit",
                output="screen",
                name="cockpit",
            )
        ]
    )
