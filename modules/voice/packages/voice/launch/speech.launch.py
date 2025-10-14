"""Launch description for the Psyched voice speech service."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _declare_argument(name: str, default: str = "", description: str = "") -> DeclareLaunchArgument:
    return DeclareLaunchArgument(
        name,
        default_value=default,
        description=description,
    )


def generate_launch_description() -> LaunchDescription:
    """Create the launch description used to run the voice speech service."""

    backend = LaunchConfiguration("backend")
    input_topic = LaunchConfiguration("input_topic")
    spoken_topic = LaunchConfiguration("spoken_topic")
    pause_topic = LaunchConfiguration("pause_topic")
    resume_topic = LaunchConfiguration("resume_topic")
    clear_topic = LaunchConfiguration("clear_topic")
    tts_url = LaunchConfiguration("tts_url")
    tts_scheme = LaunchConfiguration("tts_scheme")
    tts_host = LaunchConfiguration("tts_host")
    tts_port = LaunchConfiguration("tts_port")
    tts_path = LaunchConfiguration("tts_path")
    tts_speaker = LaunchConfiguration("tts_speaker")
    tts_language = LaunchConfiguration("tts_language")

    declare_arguments = [
        _declare_argument("backend", "print", "Voice playback backend ('print', 'espeak', or 'websocket')."),
        _declare_argument("input_topic", "/voice", "Input topic that provides text to speak."),
        _declare_argument("spoken_topic", "/voice/spoken", "Topic used to publish spoken acknowledgements."),
        _declare_argument("pause_topic", "/voice/pause", "Topic that pauses queued speech when a message is received."),
        _declare_argument("resume_topic", "/voice/resume", "Topic that resumes queued speech."),
        _declare_argument("clear_topic", "/voice/clear", "Topic that clears pending speech."),
        _declare_argument("tts_url", "", "Full websocket URL for the TTS service."),
        _declare_argument("tts_scheme", "ws", "Websocket scheme when building the TTS URL."),
        _declare_argument("tts_host", "127.0.0.1", "Host used when building the TTS URL."),
        _declare_argument("tts_port", "5002", "Port used when building the TTS URL."),
        _declare_argument("tts_path", "/tts", "Path used when building the TTS URL."),
        _declare_argument("tts_speaker", "", "Optional speaker identifier for the TTS service."),
        _declare_argument("tts_language", "", "Optional language hint for the TTS service."),
    ]

    parameters = [
        {
            "backend": backend,
            "input_topic": input_topic,
            "spoken_topic": spoken_topic,
            "pause_topic": pause_topic,
            "resume_topic": resume_topic,
            "clear_topic": clear_topic,
            "tts_url": tts_url,
            "tts_scheme": tts_scheme,
            "tts_host": tts_host,
            "tts_port": ParameterValue(tts_port, value_type=int),
            "tts_path": tts_path,
            "tts_speaker": tts_speaker,
            "tts_language": tts_language,
        }
    ]

    node = Node(
        package="voice",
        executable="speech_service",
        name="voice",
        output="screen",
        parameters=parameters,
    )

    return LaunchDescription([*declare_arguments, node])
