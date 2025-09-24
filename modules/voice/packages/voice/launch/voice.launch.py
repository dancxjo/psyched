from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    topic_arg = DeclareLaunchArgument('topic', default_value='/voice')
    engine_arg = DeclareLaunchArgument(
        'engine',
    default_value=EnvironmentVariable(name='VOICE_ENGINE', default_value='espeak-ng'),
    description="TTS engine to use: 'espeak-ng' only"
    )
    voice_path_arg = DeclareLaunchArgument(
        'voice_path',
        default_value='',
        description='Path to specific voice model (optional, usually unused)'
    )

    use_cuda_arg = DeclareLaunchArgument('use_cuda', default_value='false')

    # espeak-ng args
    espeak_voice_arg = DeclareLaunchArgument(
        'espeak_voice',
        default_value=EnvironmentVariable(name='ESPEAK_VOICE', default_value='mb-en1'),
        description='espeak-ng voice name, e.g., en-us or mb-us1 (MBROLA)'
    )
    espeak_rate_arg = DeclareLaunchArgument('espeak_rate', default_value='100')
    espeak_pitch_arg = DeclareLaunchArgument('espeak_pitch', default_value='50')
    espeak_volume_arg = DeclareLaunchArgument('espeak_volume', default_value='1.0')
    espeak_extra_args_arg = DeclareLaunchArgument('espeak_extra_args', default_value='')

    # Startup greeting and heartbeat ping
    startup_greeting_arg = DeclareLaunchArgument(
        'startup_greeting',
        default_value=EnvironmentVariable(name='VOICE_STARTUP_GREETING', default_value='Hello world!'),
        description='Greeting spoken when the node starts'
    )
    enable_ping_arg = DeclareLaunchArgument('enable_ping', default_value='false', description='Enable periodic ping')
    ping_interval_arg = DeclareLaunchArgument('ping_interval_sec', default_value='30', description='Ping interval in seconds')

    # Control topics
    pause_topic_arg = DeclareLaunchArgument('pause_topic', default_value='/voice/interrupt')
    resume_topic_arg = DeclareLaunchArgument('resume_topic', default_value='/voice/resume')
    clear_topic_arg = DeclareLaunchArgument('clear_topic', default_value='/voice/clear')
    interrupt_topic_arg = DeclareLaunchArgument('interrupt_topic', default_value='/voice/interrupt')

    # Piper model args
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=EnvironmentVariable(name='PIPER_MODEL', default_value='en_US-ryan-high'),
        description='Piper model name (e.g., en_US-ryan-high)'
    )
    voices_dir_arg = DeclareLaunchArgument(
        'voices_dir',
        default_value=EnvironmentVariable(name='PIPER_VOICES_DIR', default_value='~/.local/share/piper/voices'),
        description='Directory for Piper voice models'
    )

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
            # control topics
            'pause_topic': LaunchConfiguration('pause_topic'),
            'resume_topic': LaunchConfiguration('resume_topic'),
            'clear_topic': LaunchConfiguration('clear_topic'),
            'interrupt_topic': LaunchConfiguration('interrupt_topic'),
            # startup/heartbeat
            'startup_greeting': LaunchConfiguration('startup_greeting'),
            'enable_ping': LaunchConfiguration('enable_ping'),
            'ping_interval_sec': LaunchConfiguration('ping_interval_sec'),
        }],
        arguments=['--model', LaunchConfiguration('model'), '--voices-dir', LaunchConfiguration('voices_dir')]
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
        pause_topic_arg,
        resume_topic_arg,
        clear_topic_arg,
        interrupt_topic_arg,
        startup_greeting_arg,
        enable_ping_arg,
        ping_interval_arg,
        model_arg,
        voices_dir_arg,
        node
    ])
