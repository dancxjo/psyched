"""Launch configuration for the voice module."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


VOICE_DEFAULTS = {
    "topic": "/voice",
    "engine": "websocket",
    "volume": "1.0",
    "startup_greeting": "Hello, voice is online",
    "enable_ping": "true",
    "ping_interval_sec": "30",
    "conversation_topic": "/conversation",
    "pause_topic": "/voice/interrupt",
    "resume_topic": "/voice/resume",
    "clear_topic": "/voice/clear",
    "interrupt_topic": "/voice/interrupt",
    "tts_ws_url": "ws://localhost:5002/tts",
    "tts_ws_speaker": "",
    "tts_ws_language": "",
    "tts_ws_open_timeout": "5.0",
    "tts_ws_close_timeout": "5.0",
    "tts_ws_metadata_timeout": "2.0",
    "tts_ws_chunk_timeout": "1.0",
    "tts_ws_ping_interval": "20.0",
    "tts_ws_ping_timeout": "20.0",
    "playback_backend": "ffplay",
    "playback_device": "",
    "aplay": "true",
    "aplay_device": "",
    "espeak_voice": "mb-en1",
    "espeak_rate": "170",
    "espeak_pitch": "50",
    "espeak_volume": "1.0",
    "espeak_extra_args": "",
    "model": "",
    "voices_dir": "",
    "wav_out_dir": "",
    "length_scale": "1.0",
    "noise_scale": "0.667",
    "noise_w_scale": "0.8",
    "normalize_audio": "true",
}


def generate_launch_description() -> LaunchDescription:
    """Return the launch description for the voice node."""

    arguments = [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in VOICE_DEFAULTS.items()
    ]

    voice_node = Node(
        package="voice",
        executable="voice_node",
        name="voice",
        output="screen",
        parameters=[
            {name: LaunchConfiguration(name) for name in VOICE_DEFAULTS}
        ],
    )

    return LaunchDescription(arguments + [voice_node])
