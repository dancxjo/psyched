from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic = LaunchConfiguration('topic', default='/audio')
    device = LaunchConfiguration('device', default='0')  # Integer for device ID
    sample_rate = LaunchConfiguration('sample_rate', default='44100')  # More common sample rate
    channels = LaunchConfiguration('channels', default='1')
    chunk = LaunchConfiguration('chunk', default='1024')  # Smaller chunk size

    return LaunchDescription([
        DeclareLaunchArgument('topic', default_value=topic),
        DeclareLaunchArgument('device', default_value=device),
        DeclareLaunchArgument('sample_rate', default_value=sample_rate),
        DeclareLaunchArgument('channels', default_value=channels),
        DeclareLaunchArgument('chunk', default_value=chunk),
        Node(
            package='audio_common',
            executable='audio_capturer_node',
            name='ear',
            parameters=[{
                'topic': topic,
                'device': device,
                'sample_rate': sample_rate,
                'channels': channels,
                'chunk': chunk,
            }],
            output='screen',
        )
    ])
