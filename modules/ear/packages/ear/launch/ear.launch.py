"""Launch description for the ear module audio processing pipeline."""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    backend = LaunchConfiguration("backend")
    audio_topic = LaunchConfiguration("audio_topic")
    sample_rate = LaunchConfiguration("sample_rate")
    channels = LaunchConfiguration("channels")
    transcript_topic = LaunchConfiguration("transcript_topic")
    text_input_topic = LaunchConfiguration("text_input_topic")
    service_uri = LaunchConfiguration("service_uri")
    whisper_model = LaunchConfiguration("faster_whisper_model")
    whisper_device = LaunchConfiguration("faster_whisper_device")
    whisper_compute = LaunchConfiguration("faster_whisper_compute_type")
    whisper_language = LaunchConfiguration("faster_whisper_language")
    whisper_beam_size = LaunchConfiguration("faster_whisper_beam_size")
    audio_source_command = LaunchConfiguration("audio_source_command")
    chunk_duration_ms = LaunchConfiguration("chunk_duration_ms")
    chunk_size = LaunchConfiguration("chunk_size")
    restart_delay = LaunchConfiguration("restart_delay")
    enable_audio_capture = LaunchConfiguration("enable_audio_capture")
    reliability = LaunchConfiguration("audio_reliability")
    speech_topic = LaunchConfiguration("speech_topic")
    vad_frame_duration_ms = LaunchConfiguration("vad_frame_duration_ms")
    vad_aggressiveness = LaunchConfiguration("vad_aggressiveness")
    vad_smoothing_window = LaunchConfiguration("vad_smoothing_window")
    vad_publish_on_change = LaunchConfiguration("vad_publish_on_change")
    silence_topic = LaunchConfiguration("silence_topic")
    silence_rms_threshold = LaunchConfiguration("silence_rms_threshold")
    silence_average_window = LaunchConfiguration("silence_average_window")
    silence_publish_on_change = LaunchConfiguration("silence_publish_on_change")

    audio_capture_node = Node(
        package="ear",
        executable="ear_audio_capture",
        name="ear_audio_capture",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "audio_topic": audio_topic,
                "sample_rate": ParameterValue(sample_rate, value_type=int),
                "channels": ParameterValue(channels, value_type=int),
                "chunk_duration_ms": ParameterValue(chunk_duration_ms, value_type=float),
                "chunk_size": ParameterValue(chunk_size, value_type=int),
                "restart_delay": ParameterValue(restart_delay, value_type=float),
                "audio_source_command": audio_source_command,
                "reliability": reliability,
            }
        ],
        condition=IfCondition(enable_audio_capture),
    )

    vad_node = Node(
        package="ear",
        executable="ear_vad",
        name="ear_vad",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "audio_topic": audio_topic,
                "sample_rate": ParameterValue(sample_rate, value_type=int),
                "channels": ParameterValue(channels, value_type=int),
                "speech_topic": speech_topic,
                "frame_duration_ms": ParameterValue(vad_frame_duration_ms, value_type=int),
                "aggressiveness": ParameterValue(vad_aggressiveness, value_type=int),
                "smoothing_window": ParameterValue(vad_smoothing_window, value_type=int),
                "publish_on_change": ParameterValue(vad_publish_on_change, value_type=bool),
            }
        ],
    )

    silence_node = Node(
        package="ear",
        executable="ear_silence",
        name="ear_silence_detector",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "audio_topic": audio_topic,
                "silence_topic": silence_topic,
                "sample_width": ParameterValue(LaunchConfiguration("sample_width"), value_type=int),
                "rms_threshold": ParameterValue(silence_rms_threshold, value_type=float),
                "average_window": ParameterValue(silence_average_window, value_type=int),
                "publish_on_change": ParameterValue(silence_publish_on_change, value_type=bool),
            }
        ],
    )

    transcriber_node = Node(
        package="ear",
        executable="ear_transcriber",
        name="ear_transcriber",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "backend": backend,
                "transcript_topic": transcript_topic,
                "text_input_topic": text_input_topic,
                "audio_topic": audio_topic,
                "audio_sample_rate": ParameterValue(sample_rate, value_type=int),
                "audio_channels": ParameterValue(channels, value_type=int),
                "service_uri": service_uri,
                "faster_whisper_model": whisper_model,
                "faster_whisper_device": whisper_device,
                "faster_whisper_compute_type": whisper_compute,
                "faster_whisper_language": whisper_language,
                "faster_whisper_beam_size": ParameterValue(whisper_beam_size, value_type=int),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="console"),
            DeclareLaunchArgument("audio_topic", default_value="/audio/raw"),
            DeclareLaunchArgument("sample_rate", default_value="16000"),
            DeclareLaunchArgument("channels", default_value="1"),
            DeclareLaunchArgument("transcript_topic", default_value="/ear/hole"),
            DeclareLaunchArgument("text_input_topic", default_value=""),
            DeclareLaunchArgument("service_uri", default_value="ws://127.0.0.1:8089/ws"),
            DeclareLaunchArgument("faster_whisper_model", default_value="base"),
            DeclareLaunchArgument("faster_whisper_device", default_value="cpu"),
            DeclareLaunchArgument("faster_whisper_compute_type", default_value="int8"),
            DeclareLaunchArgument("faster_whisper_language", default_value=""),
            DeclareLaunchArgument("faster_whisper_beam_size", default_value="5"),
            DeclareLaunchArgument("audio_source_command", default_value=""),
            DeclareLaunchArgument("chunk_duration_ms", default_value="20.0"),
            DeclareLaunchArgument("chunk_size", default_value="0"),
            DeclareLaunchArgument("restart_delay", default_value="1.0"),
            DeclareLaunchArgument("enable_audio_capture", default_value="true"),
            DeclareLaunchArgument("audio_reliability", default_value="best_effort"),
            DeclareLaunchArgument("speech_topic", default_value="/ear/speech_active"),
            DeclareLaunchArgument("vad_frame_duration_ms", default_value="20"),
            DeclareLaunchArgument("vad_aggressiveness", default_value="2"),
            DeclareLaunchArgument("vad_smoothing_window", default_value="1"),
            DeclareLaunchArgument("vad_publish_on_change", default_value="true"),
            DeclareLaunchArgument("silence_topic", default_value="/ear/silence"),
            DeclareLaunchArgument("sample_width", default_value="2"),
            DeclareLaunchArgument("silence_rms_threshold", default_value="500.0"),
            DeclareLaunchArgument("silence_average_window", default_value="3"),
            DeclareLaunchArgument("silence_publish_on_change", default_value="true"),
            audio_capture_node,
            vad_node,
            silence_node,
            transcriber_node,
        ]
    )
