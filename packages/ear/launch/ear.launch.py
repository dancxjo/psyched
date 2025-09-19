from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic = LaunchConfiguration('topic', default='/audio/pcm')
    device = LaunchConfiguration('device', default='default')
    rate = LaunchConfiguration('rate', default='16000')
    channels = LaunchConfiguration('channels', default='1')
    chunk = LaunchConfiguration('chunk', default='2048')

    return LaunchDescription([
        DeclareLaunchArgument('topic', default_value=topic),
        DeclareLaunchArgument('device', default_value=device),
        DeclareLaunchArgument('rate', default_value=rate),
        DeclareLaunchArgument('channels', default_value=channels),
        DeclareLaunchArgument('chunk', default_value=chunk),
        Node(
            package='ear',
            executable='ear_node',
            name='ear',
            parameters=[{
                'topic': topic,
                'device': device,
                'rate': rate,
                'channels': channels,
                'chunk': chunk,
            }],
            output='screen',
        )
    ])
