"""Launch configuration for the pilot web interface."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _declare_launch_arguments():
    return [
        DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_port", default_value="8080"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("health_publish_topic", default_value="auto"),
        DeclareLaunchArgument("health_period_sec", default_value="2.0"),
    ]


def generate_launch_description() -> LaunchDescription:
    """Return the launch description for the pilot backend and health monitor."""

    backend = Node(
        package="pilot",
        executable="pilot_backend",
        name="pilot_backend",
        output="screen",
        arguments=[
            "--host",
            LaunchConfiguration("web_host"),
            "--port",
            LaunchConfiguration("web_port"),
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
    )

    host_health = Node(
        package="pilot",
        executable="host_health",
        name="host_health",
        output="screen",
        parameters=[
            {
                "publish_topic": LaunchConfiguration("health_publish_topic"),
                "period_sec": LaunchConfiguration("health_period_sec"),
            }
        ],
    )

    return LaunchDescription([*_declare_launch_arguments(), backend, host_health])
