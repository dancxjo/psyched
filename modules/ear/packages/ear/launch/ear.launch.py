"""Launch configuration for the ear module."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


EAR_DEFAULTS = {
    "ear_topic": "/audio/pcm",
    "ear_device": "default",
    "ear_rate": "16000",
    "ear_channels": "1",
    "ear_chunk": "2048",
}

TRANSCRIBER_DEFAULTS = {
    "transcriber_segment_topic": "/audio/speech_segment",
    "transcriber_segment_accum_topic": "/audio/speech_segment_accumulating",
    "transcriber_speech_accum_topic": "/audio/speech_accumulating",
    "transcriber_transcript_topic": "/audio/transcription",
    "transcriber_transcript_short_topic": "/audio/transcript/short",
    "transcriber_transcript_medium_topic": "/audio/transcript/medium",
    "transcriber_transcript_long_topic": "/audio/transcript/long",
    "transcriber_fast_remote_ws_url": "ws://forebrain.local:8082/ws",
    "transcriber_medium_remote_ws_url": "ws://forebrain.local:8083/ws",
    "transcriber_long_remote_ws_url": "ws://forebrain.local:8084/ws",
    "transcriber_speaker": "user",
    "transcriber_segment_sample_rate": "16000",
    "transcriber_model": "small",
    "transcriber_device": "cpu",
    "transcriber_compute_type": "int8",
    "transcriber_language": "en",
    "transcriber_beam_size": "5",
}

ACCUMULATOR_DEFAULTS = {
    "speech_accumulator_segment_topic": "/audio/speech_segment",
    "speech_accumulator_accum_topic": "/audio/speech_accumulating",
    "speech_accumulator_reset_timeout": "12.0",
    "speech_accumulator_max_segments": "8",
}


def _declare_arguments(defaults):
    return [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]


def generate_launch_description() -> LaunchDescription:
    """Return the launch description for audio capture, VAD, and transcription."""

    args = [
        *_declare_arguments(EAR_DEFAULTS),
        *_declare_arguments(TRANSCRIBER_DEFAULTS),
        *_declare_arguments(ACCUMULATOR_DEFAULTS),
    ]

    ear_node = Node(
        package="ear",
        executable="ear_node",
        name="ear",
        parameters=[
            {
                "topic": LaunchConfiguration("ear_topic"),
                "device": LaunchConfiguration("ear_device"),
                "rate": LaunchConfiguration("ear_rate"),
                "channels": LaunchConfiguration("ear_channels"),
                "chunk": LaunchConfiguration("ear_chunk"),
            }
        ],
        output="screen",
    )

    vad_node = Node(
        package="ear",
        executable="vad_node",
        name="vad",
        output="screen",
    )

    accumulator_node = Node(
        package="ear",
        executable="speech_accumulator_node",
        name="speech_accumulator",
        parameters=[
            {
                "segment_topic": LaunchConfiguration("speech_accumulator_segment_topic"),
                "accum_topic": LaunchConfiguration("speech_accumulator_accum_topic"),
                "reset_timeout": LaunchConfiguration("speech_accumulator_reset_timeout"),
                "max_segments": LaunchConfiguration("speech_accumulator_max_segments"),
            }
        ],
        output="screen",
    )

    transcriber_node = Node(
        package="ear",
        executable="transcriber_node",
        name="transcriber",
        parameters=[
            {
                "segment_topic": LaunchConfiguration("transcriber_segment_topic"),
                "segment_accumulating_topic": LaunchConfiguration("transcriber_segment_accum_topic"),
                "speech_accumulating_topic": LaunchConfiguration("transcriber_speech_accum_topic"),
                "transcript_topic": LaunchConfiguration("transcriber_transcript_topic"),
                "transcript_short_topic": LaunchConfiguration("transcriber_transcript_short_topic"),
                "transcript_medium_topic": LaunchConfiguration("transcriber_transcript_medium_topic"),
                "transcript_long_topic": LaunchConfiguration("transcriber_transcript_long_topic"),
                "fast_remote_ws_url": LaunchConfiguration("transcriber_fast_remote_ws_url"),
                "medium_remote_ws_url": LaunchConfiguration("transcriber_medium_remote_ws_url"),
                "long_remote_ws_url": LaunchConfiguration("transcriber_long_remote_ws_url"),
                "speaker": LaunchConfiguration("transcriber_speaker"),
                "segment_sample_rate": LaunchConfiguration("transcriber_segment_sample_rate"),
                "model": LaunchConfiguration("transcriber_model"),
                "device": LaunchConfiguration("transcriber_device"),
                "compute_type": LaunchConfiguration("transcriber_compute_type"),
                "language": LaunchConfiguration("transcriber_language"),
                "beam_size": LaunchConfiguration("transcriber_beam_size"),
            }
        ],
        output="screen",
    )

    return LaunchDescription(args + [ear_node, vad_node, accumulator_node, transcriber_node])
