
import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.environ.get("PSH_MODULE_CONFIG")
    params = [params_file] if params_file else []
    node = Node(
        package="psyched_gps",
        executable="psyched_gps_node",
        name="psyched_gps",
        output="screen",
        parameters=params,
        remappings=[
            ("fix", "/gps/fix"),
            ("time_reference", "/gps/time_reference"),
        ],
    )
    return LaunchDescription([node])
