"""Launch Kinect and USB camera pipelines for the eye module."""

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


def _as_int(value: str, default: int) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def _as_float(value: str, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _launch_setup(context, *args, **kwargs):
    nodes: List[Node] = []

    use_kinect = _as_bool(LaunchConfiguration("use_kinect").perform(context), True)
    use_usb = _as_bool(LaunchConfiguration("use_usb_camera").perform(context), False)
    enable_faces_router = _as_bool(LaunchConfiguration("enable_faces_router").perform(context), True)

    kinect_rgb_topic = LaunchConfiguration("kinect_rgb_topic").perform(context)
    kinect_rgb_info_topic = LaunchConfiguration("kinect_rgb_info_topic").perform(context)
    kinect_depth_topic = LaunchConfiguration("kinect_depth_topic").perform(context)
    kinect_depth_info_topic = LaunchConfiguration("kinect_depth_info_topic").perform(context)

    usb_image_topic = LaunchConfiguration("usb_image_topic").perform(context)
    usb_info_topic = LaunchConfiguration("usb_camera_info_topic").perform(context)

    faces_image_topic = LaunchConfiguration("faces_output_image_topic").perform(context)
    faces_info_topic = LaunchConfiguration("faces_output_camera_info_topic").perform(context)

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

    if use_usb:
        usb_params = {
            "device": LaunchConfiguration("usb_device").perform(context),
            "frame_id": LaunchConfiguration("usb_frame_id").perform(context),
            "width": _as_int(LaunchConfiguration("usb_width").perform(context), 640),
            "height": _as_int(LaunchConfiguration("usb_height").perform(context), 480),
            "fps": _as_float(LaunchConfiguration("usb_fps").perform(context), 30.0),
            "encoding": LaunchConfiguration("usb_encoding").perform(context),
            "image_topic": usb_image_topic,
            "camera_info_topic": usb_info_topic,
        }
        nodes.append(
            Node(
                package="psyched_eye",
                executable="usb_camera_node",
                name="psyched_usb_camera",
                output="screen",
                parameters=[usb_params],
            )
        )

    if enable_faces_router and (faces_image_topic or faces_info_topic):
        faces_params = {
            "faces_source": LaunchConfiguration("faces_source").perform(context),
            "fallback_source": LaunchConfiguration("faces_fallback_source").perform(context),
            "kinect_enabled": use_kinect,
            "usb_enabled": use_usb,
            "kinect_image_topic": kinect_rgb_topic,
            "kinect_camera_info_topic": kinect_rgb_info_topic,
            "usb_image_topic": usb_image_topic,
            "usb_camera_info_topic": usb_info_topic,
            "output_image_topic": faces_image_topic,
            "output_camera_info_topic": faces_info_topic,
        }
        nodes.append(
            Node(
                package="psyched_eye",
                executable="faces_router_node",
                name="psyched_faces_router",
                output="screen",
                parameters=[faces_params],
            )
        )

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
            DeclareLaunchArgument("use_usb_camera", default_value="false"),
            DeclareLaunchArgument("usb_device", default_value="/dev/video0"),
            DeclareLaunchArgument("usb_frame_id", default_value="usb_camera"),
            DeclareLaunchArgument("usb_width", default_value="640"),
            DeclareLaunchArgument("usb_height", default_value="480"),
            DeclareLaunchArgument("usb_fps", default_value="30.0"),
            DeclareLaunchArgument("usb_encoding", default_value="bgr8"),
            DeclareLaunchArgument("usb_image_topic", default_value="/eye/usb/image_raw"),
            DeclareLaunchArgument("usb_camera_info_topic", default_value="/eye/usb/camera_info"),
            DeclareLaunchArgument("enable_faces_router", default_value="true"),
            DeclareLaunchArgument("faces_source", default_value="kinect"),
            DeclareLaunchArgument("faces_fallback_source", default_value="auto"),
            DeclareLaunchArgument("faces_output_image_topic", default_value="/faces/camera/image_raw"),
            DeclareLaunchArgument("faces_output_camera_info_topic", default_value="/faces/camera/camera_info"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
