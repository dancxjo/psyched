"""Launch the Psyched face detector node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    camera_topic = LaunchConfiguration("camera_topic", default="/image_raw")
    faces_topic = LaunchConfiguration("faces_topic", default="/vision/faces")
    face_detected_topic = LaunchConfiguration("face_detected_topic", default="/vision/face_detected")
    trigger_cooldown = LaunchConfiguration("trigger_cooldown_sec", default="2.0")

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_topic", default_value=camera_topic),
            DeclareLaunchArgument("faces_topic", default_value=faces_topic),
            DeclareLaunchArgument("face_detected_topic", default_value=face_detected_topic),
            DeclareLaunchArgument("trigger_cooldown_sec", default_value=trigger_cooldown),
            Node(
                package="faces",
                executable="face_detector",
                name="psyched_faces",
                parameters=[
                    {
                        "camera_topic": camera_topic,
                        "faces_topic": faces_topic,
                        "face_detected_topic": face_detected_topic,
                        "trigger_cooldown_sec": trigger_cooldown,
                    }
                ],
                output="screen",
            ),
        ]
    )
