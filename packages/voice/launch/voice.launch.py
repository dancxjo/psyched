from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    topic_arg = DeclareLaunchArgument('topic', default_value='/voice')
    voice_path_arg = DeclareLaunchArgument(
        'voice_path',
        default_value=EnvironmentVariable(name='PIPER_VOICE', default_value=''),
        description='Path to Piper .onnx voice model (can also set PIPER_VOICE env var)'
    )

    use_cuda_arg = DeclareLaunchArgument('use_cuda', default_value='false')

    node = Node(
        package='voice',
        executable='voice_node',
        name='voice_node',
        output='screen',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'voice_path': LaunchConfiguration('voice_path'),
            'use_cuda': LaunchConfiguration('use_cuda'),
        }]
    )

    return LaunchDescription([
        topic_arg,
        voice_path_arg,
        use_cuda_arg,
        node
    ])
