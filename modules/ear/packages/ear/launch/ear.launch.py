"""Launch configuration for the ear module."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


EAR_DEFAULTS = {
    "ear_device_id": "0",
    "ear_sample_rate": "44100",
    "ear_channels": "1",
    "ear_chunk_size": "2048",
}

SILENCE_DEFAULTS = {
    "silence_threshold": "500.0",
    "silence_input_topic": "/audio/raw",
}

VAD_DEFAULTS = {
    "vad_input_topic": "/audio/raw",
}

SEGMENTER_DEFAULTS = {
    "segmenter_frame_topic": "/audio/vad_frames",
    "segmenter_segment_topic": "/audio/speech_segment",
    "segmenter_accum_topic": "/audio/speech_segment_accumulating",
    "segmenter_duration_topic": "/audio/speech_duration",
    "segmenter_silence_release_ms": "450.0",
    "segmenter_max_segment_ms": "12000.0",
}

TRANSCRIBER_DEFAULTS = {
    "transcriber_segment_topic": "/audio/speech_segment",
    "transcriber_segment_accum_topic": "/audio/speech_segment_accumulating",
    "transcriber_speech_accum_topic": "/audio/speech_accumulating",
    "transcriber_transcript_topic": "/audio/transcription",
    "transcriber_transcript_short_topic": "/audio/transcript/short",
    "transcriber_transcript_medium_topic": "/audio/transcript/medium",
    "transcriber_transcript_long_topic": "/audio/transcript/long",
    # local transcriber output (new): default mirrors the medium-tier compatibility topic
    "transcriber_local_transcript_topic": "/audio/transcription",
    "transcriber_fast_remote_ws_url": "ws://forebrain.local:8082/ws",
    "transcriber_medium_remote_ws_url": "ws://forebrain.local:8082/ws",
    "transcriber_long_remote_ws_url": "ws://forebrain.local:8082/ws",
    "transcriber_fast_remote_audio_dump_dir": "log/remote_asr/fast",
    "transcriber_medium_remote_audio_dump_dir": "log/remote_asr/medium",
    "transcriber_long_remote_audio_dump_dir": "log/remote_asr/long",
    "transcriber_remote_connect_timeout": "3000",
    "transcriber_remote_response_timeout": "3000",
    "transcriber_speaker": "user",
    "transcriber_segment_sample_rate": "16000",
    "transcriber_model": "small",
    "transcriber_device": "gpu",
    "transcriber_compute_type": "int8",
    "transcriber_language": "en",
    "transcriber_beam_size": "5",
}

ACCUMULATOR_DEFAULTS = {
    "speech_accumulator_segment_topic": "/audio/speech_segment",
    "speech_accumulator_accum_topic": "/audio/speech_accumulating",
    "speech_accumulator_reset_timeout": "12000",
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
        *_declare_arguments(SILENCE_DEFAULTS),
        *_declare_arguments(VAD_DEFAULTS),
        *_declare_arguments(SEGMENTER_DEFAULTS),
        *_declare_arguments(TRANSCRIBER_DEFAULTS),
        *_declare_arguments(ACCUMULATOR_DEFAULTS),
    ]

    ear_node = Node(
        package="ear",
        executable="ear_node",
        name="ear",
        parameters=[
            {
                "device_id": ParameterValue(LaunchConfiguration("ear_device_id"), value_type=int),
                "sample_rate": ParameterValue(LaunchConfiguration("ear_sample_rate"), value_type=int),
                "channels": ParameterValue(LaunchConfiguration("ear_channels"), value_type=int),
                "chunk_size": ParameterValue(LaunchConfiguration("ear_chunk_size"), value_type=int),
            }
        ],
        output="screen",
    )

    silence_node = Node(
        package="ear",
        executable="silence_node",
        name="ear_silence",
        parameters=[
            {
                "silence_threshold": ParameterValue(LaunchConfiguration("silence_threshold"), value_type=float),
                "input_topic": ParameterValue(LaunchConfiguration("silence_input_topic"), value_type=str),
            }
        ],
        output="screen",
    )

    vad_node = Node(
        package="ear",
        executable="vad_node",
        name="vad",
        parameters=[
            {
                "input_topic": ParameterValue(LaunchConfiguration("vad_input_topic"), value_type=str),
                "frame_topic": ParameterValue(LaunchConfiguration("segmenter_frame_topic"), value_type=str),
            }
        ],
        output="screen",
    )

    segmenter_node = Node(
        package="ear",
        executable="segmenter_node",
        name="speech_segmenter",
        parameters=[
            {
                "frame_topic": ParameterValue(LaunchConfiguration("segmenter_frame_topic"), value_type=str),
                "segment_topic": ParameterValue(LaunchConfiguration("segmenter_segment_topic"), value_type=str),
                "accumulating_topic": ParameterValue(LaunchConfiguration("segmenter_accum_topic"), value_type=str),
                "duration_topic": ParameterValue(LaunchConfiguration("segmenter_duration_topic"), value_type=str),
                "silence_release_ms": ParameterValue(
                    LaunchConfiguration("segmenter_silence_release_ms"), value_type=float
                ),
                "max_segment_ms": ParameterValue(
                    LaunchConfiguration("segmenter_max_segment_ms"), value_type=float
                ),
            }
        ],
        output="screen",
    )

    accumulator_node = Node(
        package="ear",
        executable="segment_accumulator_node",
        name="segment_accumulator",
        parameters=[
            {
                "segment_topic": ParameterValue(
                    LaunchConfiguration("speech_accumulator_segment_topic"), value_type=str
                ),
                "accum_topic": ParameterValue(
                    LaunchConfiguration("speech_accumulator_accum_topic"), value_type=str
                ),
                "reset_timeout": ParameterValue(
                    LaunchConfiguration("speech_accumulator_reset_timeout"), value_type=float
                ),
                "max_segments": ParameterValue(
                    LaunchConfiguration("speech_accumulator_max_segments"), value_type=int
                ),
            }
        ],
        output="screen",
    )

    transcriber_short = Node(
        package="ear",
        executable="transcriber_short_node",
        name="transcriber_short",
        parameters=[
            {
                "segment_accumulating_topic": LaunchConfiguration("transcriber_segment_accum_topic"),
                "transcript_short_topic": LaunchConfiguration("transcriber_transcript_short_topic"),
                "fast_remote_ws_url": LaunchConfiguration("transcriber_fast_remote_ws_url"),
                "fast_remote_audio_dump_dir": LaunchConfiguration("transcriber_fast_remote_audio_dump_dir"),
                "speaker": LaunchConfiguration("transcriber_speaker"),
                "segment_sample_rate": ParameterValue(
                    LaunchConfiguration("transcriber_segment_sample_rate"), value_type=int
                ),
                "model": LaunchConfiguration("transcriber_model"),
                "device": LaunchConfiguration("transcriber_device"),
                "compute_type": LaunchConfiguration("transcriber_compute_type"),
                "language": LaunchConfiguration("transcriber_language"),
                "beam_size": ParameterValue(
                    LaunchConfiguration("transcriber_beam_size"), value_type=int
                ),
                "remote_connect_timeout": ParameterValue(
                    LaunchConfiguration("transcriber_remote_connect_timeout"), value_type=float
                ),
                "remote_response_timeout": ParameterValue(
                    LaunchConfiguration("transcriber_remote_response_timeout"), value_type=float
                ),
            }
        ],
        output="screen",
    )

    transcriber_medium = Node(
        package="ear",
        executable="transcriber_medium_node",
        name="transcriber_medium",
        parameters=[
            {
                "segment_topic": LaunchConfiguration("transcriber_segment_topic"),
                "transcript_medium_topic": LaunchConfiguration("transcriber_transcript_medium_topic"),
                "transcript_topic": LaunchConfiguration("transcriber_transcript_topic"),
                "medium_remote_ws_url": LaunchConfiguration("transcriber_medium_remote_ws_url"),
                "medium_remote_audio_dump_dir": LaunchConfiguration("transcriber_medium_remote_audio_dump_dir"),
                "speaker": LaunchConfiguration("transcriber_speaker"),
                "segment_sample_rate": ParameterValue(
                    LaunchConfiguration("transcriber_segment_sample_rate"), value_type=int
                ),
                "model": LaunchConfiguration("transcriber_model"),
                "device": LaunchConfiguration("transcriber_device"),
                "compute_type": LaunchConfiguration("transcriber_compute_type"),
                "language": LaunchConfiguration("transcriber_language"),
                "beam_size": ParameterValue(
                    LaunchConfiguration("transcriber_beam_size"), value_type=int
                ),
                "remote_connect_timeout": ParameterValue(
                    LaunchConfiguration("transcriber_remote_connect_timeout"), value_type=float
                ),
                "remote_response_timeout": ParameterValue(
                    LaunchConfiguration("transcriber_remote_response_timeout"), value_type=float
                ),
            }
        ],
        output="screen",
    )

    transcriber_long = Node(
        package="ear",
        executable="transcriber_long_node",
        name="transcriber_long",
        parameters=[
            {
                "speech_accumulating_topic": LaunchConfiguration("transcriber_speech_accum_topic"),
                "transcript_long_topic": LaunchConfiguration("transcriber_transcript_long_topic"),
                "long_remote_ws_url": LaunchConfiguration("transcriber_long_remote_ws_url"),
                "long_remote_audio_dump_dir": LaunchConfiguration("transcriber_long_remote_audio_dump_dir"),
                "speaker": LaunchConfiguration("transcriber_speaker"),
                "segment_sample_rate": ParameterValue(
                    LaunchConfiguration("transcriber_segment_sample_rate"), value_type=int
                ),
                "model": LaunchConfiguration("transcriber_model"),
                "device": LaunchConfiguration("transcriber_device"),
                "compute_type": LaunchConfiguration("transcriber_compute_type"),
                "language": LaunchConfiguration("transcriber_language"),
                "beam_size": ParameterValue(
                    LaunchConfiguration("transcriber_beam_size"), value_type=int
                ),
                "remote_connect_timeout": ParameterValue(
                    LaunchConfiguration("transcriber_remote_connect_timeout"), value_type=float
                ),
                "remote_response_timeout": ParameterValue(
                    LaunchConfiguration("transcriber_remote_response_timeout"), value_type=float
                ),
            }
        ],
        output="screen",
    )

    # Local transcription node: runs small faster-whisper locally on incoming speech segments
    # Subscribes to the same segment topic (default: /audio/speech_segment) and
    # publishes short transcripts to the short transcript topic.
    transcriber_local = Node(
        package="ear",
        executable="transcriber_local_node",
        name="local_transcription",
        parameters=[
            {
                # listen for completed segments
                "segment_topic": LaunchConfiguration("transcriber_segment_topic"),
                # local transcriber publishes to its own compatibility topic by default
                "transcript_topic": LaunchConfiguration("transcriber_local_transcript_topic"),
                # older short-tier still uses /audio/transcript/short (left commented out)
                # "transcript_short_topic": LaunchConfiguration("transcriber_transcript_short_topic"),
                "speaker": LaunchConfiguration("transcriber_speaker"),
                "segment_sample_rate": ParameterValue(
                    LaunchConfiguration("transcriber_segment_sample_rate"), value_type=int
                ),
                # model and compute settings (model default is 'small')
                "model": LaunchConfiguration("transcriber_model"),
                "device": "cpu",
                "compute_type": LaunchConfiguration("transcriber_compute_type"),
                "language": LaunchConfiguration("transcriber_language"),
                "beam_size": ParameterValue(
                    LaunchConfiguration("transcriber_beam_size"), value_type=int
                ),
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        args
        + [
            ear_node,
            silence_node,
            vad_node,
            segmenter_node,
            accumulator_node,
            # transcriber_local,
            # transcriber_short,
            transcriber_medium,
            # transcriber_long,
        ]
    )
