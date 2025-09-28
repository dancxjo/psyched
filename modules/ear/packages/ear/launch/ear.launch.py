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
    "transcriber_transcript_topic": "/audio/transcription",
    "transcriber_speaker": "user",
    "transcriber_segment_sample_rate": "16000",
    "transcriber_model": "small",
    "transcriber_device": "cpu",
    "transcriber_compute_type": "int8",
    "transcriber_language": "en",
    "transcriber_beam_size": "5",
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

    transcriber_node = Node(
        package="ear",
        executable="transcriber_node",
        name="transcriber",
        parameters=[
            {
                "segment_topic": LaunchConfiguration("transcriber_segment_topic"),
                "transcript_topic": LaunchConfiguration("transcriber_transcript_topic"),
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

    return LaunchDescription(args + [ear_node, vad_node, transcriber_node])
