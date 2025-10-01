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

RAW_DEFAULTS = {
    "ear_raw_topic": "/audio/raw",
}

SEGMENTER_DEFAULTS = {
    "segmenter_segment_topic": "/audio/speech_segment",
    "segmenter_accum_topic": "/audio/speech_segment_accumulating",
    "segmenter_silence_release_ms": "450.0",
    "segmenter_lead_silence_ms": "120.0",
    "segmenter_min_speech_ms": "300.0",
    "segmenter_max_segment_ms": "12000.0",
    "segmenter_trim_window_ms": "30.0",
    "segmenter_trim_keep_ms": "60.0",
    "segmenter_trim_rms_ratio": "0.12",
    "segmenter_trim_rms_floor": "200.0",
}

TRANSCRIBER_DEFAULTS = {
    "transcriber_segment_topic": "/audio/speech_segment",
    "transcriber_segment_accum_topic": "/audio/speech_segment_accumulating",
    "transcriber_speech_accum_topic": "/audio/speech_accumulating",
    "transcriber_transcript_topic": "/audio/transcription",
    "transcriber_transcript_short_topic": "/audio/transcript/short",
    "transcriber_transcript_medium_topic": "/audio/transcript/medium",
    "transcriber_transcript_long_topic": "/audio/transcript/long",
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

SEGMENT_LOG_DEFAULTS = {
    "segment_dump_dir": "log/ear_segments",
    "segment_dump_enabled": "true",
}


def _declare_arguments(defaults):
    return [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in defaults.items()
    ]


def generate_launch_description() -> LaunchDescription:
    """Return the launch description for audio capture and transcription."""

    args = [
        *_declare_arguments(EAR_DEFAULTS),
        *_declare_arguments(RAW_DEFAULTS),
        *_declare_arguments(SEGMENTER_DEFAULTS),
        *_declare_arguments(TRANSCRIBER_DEFAULTS),
        *_declare_arguments(ACCUMULATOR_DEFAULTS),
        *_declare_arguments(SEGMENT_LOG_DEFAULTS),
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
                "topic": ParameterValue(LaunchConfiguration("ear_raw_topic"), value_type=str),
                "segmenter_segment_topic": ParameterValue(
                    LaunchConfiguration("segmenter_segment_topic"), value_type=str
                ),
                "segmenter_accum_topic": ParameterValue(
                    LaunchConfiguration("segmenter_accum_topic"), value_type=str
                ),
                "speech_accumulator_accum_topic": ParameterValue(
                    LaunchConfiguration("speech_accumulator_accum_topic"), value_type=str
                ),
                "segment_topic": ParameterValue(LaunchConfiguration("segmenter_segment_topic"), value_type=str),
                "segment_accum_topic": ParameterValue(
                    LaunchConfiguration("segmenter_accum_topic"), value_type=str
                ),
                "speech_accum_topic": ParameterValue(
                    LaunchConfiguration("speech_accumulator_accum_topic"), value_type=str
                ),
                "segment_sample_rate": ParameterValue(
                    LaunchConfiguration("transcriber_segment_sample_rate"), value_type=int
                ),
                "segmenter_silence_release_ms": ParameterValue(
                    LaunchConfiguration("segmenter_silence_release_ms"), value_type=float
                ),
                "segmenter_lead_silence_ms": ParameterValue(
                    LaunchConfiguration("segmenter_lead_silence_ms"), value_type=float
                ),
                "segmenter_min_speech_ms": ParameterValue(
                    LaunchConfiguration("segmenter_min_speech_ms"), value_type=float
                ),
                "segmenter_max_segment_ms": ParameterValue(
                    LaunchConfiguration("segmenter_max_segment_ms"), value_type=float
                ),
                "segmenter_trim_window_ms": ParameterValue(
                    LaunchConfiguration("segmenter_trim_window_ms"), value_type=float
                ),
                "segmenter_trim_keep_ms": ParameterValue(
                    LaunchConfiguration("segmenter_trim_keep_ms"), value_type=float
                ),
                "segmenter_trim_rms_ratio": ParameterValue(
                    LaunchConfiguration("segmenter_trim_rms_ratio"), value_type=float
                ),
                "segmenter_trim_rms_floor": ParameterValue(
                    LaunchConfiguration("segmenter_trim_rms_floor"), value_type=float
                ),
                "speech_accumulator_reset_timeout": ParameterValue(
                    LaunchConfiguration("speech_accumulator_reset_timeout"), value_type=float
                ),
                "speech_accumulator_max_segments": ParameterValue(
                    LaunchConfiguration("speech_accumulator_max_segments"), value_type=int
                ),
                "segment_dump_dir": ParameterValue(
                    LaunchConfiguration("segment_dump_dir"), value_type=str
                ),
                "segment_dump_enabled": ParameterValue(
                    LaunchConfiguration("segment_dump_enabled"), value_type=bool
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
                "input_topic": ParameterValue(LaunchConfiguration("transcriber_segment_accum_topic"), value_type=str),
                "transcript_topic": ParameterValue(
                    LaunchConfiguration("transcriber_transcript_short_topic"), value_type=str
                ),
                "segment_topic": ParameterValue(
                    LaunchConfiguration("transcriber_segment_topic"), value_type=str
                ),
                "segment_accum_topic": ParameterValue(
                    LaunchConfiguration("transcriber_segment_accum_topic"), value_type=str
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
                "input_topic": ParameterValue(LaunchConfiguration("transcriber_segment_topic"), value_type=str),
                "transcript_topic": ParameterValue(
                    LaunchConfiguration("transcriber_transcript_medium_topic"), value_type=str
                ),
                "segment_topic": ParameterValue(
                    LaunchConfiguration("transcriber_segment_topic"), value_type=str
                ),
                "segment_accum_topic": ParameterValue(
                    LaunchConfiguration("transcriber_segment_accum_topic"), value_type=str
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
                "input_topic": ParameterValue(LaunchConfiguration("transcriber_speech_accum_topic"), value_type=str),
                "transcript_topic": ParameterValue(
                    LaunchConfiguration("transcriber_transcript_long_topic"), value_type=str
                ),
            }
        ],
        output="screen",
    )

    transcriber_local = Node(
        package="ear",
        executable="transcriber_local_node",
        name="transcriber_local",
        parameters=[
            {
                "segment_topic": ParameterValue(LaunchConfiguration("transcriber_segment_topic"), value_type=str),
                "transcript_topic": ParameterValue(
                    LaunchConfiguration("transcriber_local_transcript_topic"), value_type=str
                ),
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        args
        + [
            ear_node,
            transcriber_short,
            transcriber_medium,
            transcriber_long,
            transcriber_local,
        ]
    )
