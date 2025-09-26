from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_id = LaunchConfiguration('device_id', default='0')
    sample_rate = LaunchConfiguration('sample_rate', default='44100')
    channels = LaunchConfiguration('channels', default='1')
    chunk_size = LaunchConfiguration('chunk_size', default='1024')
    silence_threshold = LaunchConfiguration('silence_threshold', default='500.0')
    model = LaunchConfiguration('model', default='base')
    device = LaunchConfiguration('device', default='cpu')
    compute_type = LaunchConfiguration('compute_type', default='int8')
    language = LaunchConfiguration('language', default='')
    beam_size = LaunchConfiguration('beam_size', default='5')

    return LaunchDescription([
        DeclareLaunchArgument('device_id', default_value=device_id),
        DeclareLaunchArgument('sample_rate', default_value=sample_rate),
        DeclareLaunchArgument('channels', default_value=channels),
        DeclareLaunchArgument('chunk_size', default_value=chunk_size),
        DeclareLaunchArgument('silence_threshold', default_value=silence_threshold),
        DeclareLaunchArgument('model', default_value=model),
        DeclareLaunchArgument('device', default_value=device),
        DeclareLaunchArgument('compute_type', default_value=compute_type),
        DeclareLaunchArgument('language', default_value=language),
        DeclareLaunchArgument('beam_size', default_value=beam_size),
        Node(
            package='ear',
            executable='ear_node',
            name='ear',
            parameters=[{
                'device_id': device_id,
                'sample_rate': sample_rate,
                'channels': channels,
                'chunk_size': chunk_size,
                'silence_threshold': silence_threshold,
            }],
            output='screen',
        ),
        Node(
            package='ear',
            executable='vad_node',
            name='vad',
            output='screen',
        ),
        Node(
            package='ear',
            executable='transcriber_node',
            name='transcriber',
            parameters=[{
                'segment_topic': '/audio/speech_segment',
                'transcript_topic': '/audio/transcription',
                'model': model,
                'device': device,
                'compute_type': compute_type,
                'language': language,
                'beam_size': beam_size,
            }],
            output='screen',
        ),
    ])
