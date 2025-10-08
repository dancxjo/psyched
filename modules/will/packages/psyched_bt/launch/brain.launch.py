"""Launch file that starts the Psyched regime manager."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_regime_arg = DeclareLaunchArgument(
        "default_regime", default_value="idle", description="Initial behavioural regime"
    )

    return LaunchDescription(
        [
            default_regime_arg,
            Node(
                package='psyched_bt',
                executable='regime_manager',
                name='psyched_regime_manager',
                output='screen',
                parameters=[{"default_regime": LaunchConfiguration("default_regime")}],
            ),
        ]
    )
