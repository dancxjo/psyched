from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    topic_arg = DeclareLaunchArgument('topic', default_value='/voice')
    engine_arg = DeclareLaunchArgument(
        'engine',
        default_value=EnvironmentVariable(name='VOICE_ENGINE', default_value='piper'),
        description="TTS engine to use: 'piper' or 'espeak'"
    )
    voice_path_arg = DeclareLaunchArgument(
        'voice_path',
        default_value=EnvironmentVariable(name='PIPER_VOICE', default_value=''),
        description='Path to Piper .onnx voice model (can also set PIPER_VOICE env var)'
    )

    use_cuda_arg = DeclareLaunchArgument('use_cuda', default_value='false')

    # espeak-ng args
    espeak_voice_arg = DeclareLaunchArgument(
        'espeak_voice',
        default_value=EnvironmentVariable(name='ESPEAK_VOICE', default_value='en-us'),
        description='espeak-ng voice name, e.g., en-us or mb-us1 (MBROLA)'
    )
    espeak_rate_arg = DeclareLaunchArgument('espeak_rate', default_value='170')
    espeak_pitch_arg = DeclareLaunchArgument('espeak_pitch', default_value='50')
    espeak_volume_arg = DeclareLaunchArgument('espeak_volume', default_value='1.0')
    espeak_extra_args_arg = DeclareLaunchArgument('espeak_extra_args', default_value='')

    node = Node(
        package='voice',
        executable='voice_node',
        name='voice_node',
        output='screen',
        parameters=[{
            'topic': LaunchConfiguration('topic'),
            'engine': LaunchConfiguration('engine'),
            'voice_path': LaunchConfiguration('voice_path'),
            'use_cuda': LaunchConfiguration('use_cuda'),
            # espeak-ng
            'espeak_voice': LaunchConfiguration('espeak_voice'),
            'espeak_rate': LaunchConfiguration('espeak_rate'),
            'espeak_pitch': LaunchConfiguration('espeak_pitch'),
            'espeak_volume': LaunchConfiguration('espeak_volume'),
            'espeak_extra_args': LaunchConfiguration('espeak_extra_args'),
        }]
    )

    return LaunchDescription([
        topic_arg,
        engine_arg,
        voice_path_arg,
        use_cuda_arg,
        espeak_voice_arg,
        espeak_rate_arg,
        espeak_pitch_arg,
        espeak_volume_arg,
        espeak_extra_args_arg,
        node
    ])
