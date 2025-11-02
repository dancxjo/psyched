"""Launch the Kinect pipeline for the eye module."""

from __future__ import annotations

from pathlib import Path
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str, default: bool = False) -> bool:
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _launch_setup(context, *args, **kwargs):
    nodes: List[Node] = []

    use_kinect = _as_bool(LaunchConfiguration("use_kinect").perform(context), True)

    kinect_rgb_topic = LaunchConfiguration("kinect_rgb_topic").perform(context)
    kinect_rgb_info_topic = LaunchConfiguration("kinect_rgb_info_topic").perform(context)
    kinect_depth_topic = LaunchConfiguration("kinect_depth_topic").perform(context)
    kinect_depth_info_topic = LaunchConfiguration("kinect_depth_info_topic").perform(context)

    if use_kinect:
        params_file = LaunchConfiguration("kinect_params_file").perform(context)
        node_kwargs = {
            "package": "kinect_ros2",
            "executable": "kinect_ros2_node",
            "name": "psyched_kinect",
            "output": "screen",
            "remappings": [
                ("image_raw", kinect_rgb_topic),
                ("camera_info", kinect_rgb_info_topic),
                ("depth/image_raw", kinect_depth_topic),
                ("depth/camera_info", kinect_depth_info_topic),
            ],
        }
        if params_file:
            node_kwargs["parameters"] = [Path(params_file).as_posix()]
        nodes.append(Node(**node_kwargs))

    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_kinect", default_value="true"),
            DeclareLaunchArgument("kinect_params_file", default_value=""),
            DeclareLaunchArgument("kinect_rgb_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("kinect_rgb_info_topic", default_value="/camera/color/camera_info"),
            DeclareLaunchArgument("kinect_depth_topic", default_value="/camera/depth/image_raw"),
            DeclareLaunchArgument("kinect_depth_info_topic", default_value="/camera/depth/camera_info"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
