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

HOLE_DEFAULTS = {
    "hole_partial_text_topic": "/audio/transcript/partial",
    "hole_partial_detailed_topic": "/audio/transcript/partial/timing",
    "hole_final_text_topic": "/audio/transcript/final",
    "hole_final_detailed_topic": "/audio/transcript/final/timing",
    "hole_final_audio_topic": "/audio/transcript/final/audio",
    "hole_silence_flush_ms": "1200.0",
    "hole_silence_padding_ms": "240.0",
    "hole_head_stability_window": "3",
    "hole_head_min_words": "3",
    "hole_head_min_duration_ms": "450.0",
    "hole_request_interval_ms": "250.0",
    "hole_remote_ws_url": "ws://forebrain.local:8082/ws",
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
        *_declare_arguments(HOLE_DEFAULTS),
    ]

    # ear_node = Node(
    #     package="ear",
    #     executable="ear_node",
    #     name="ear",
    #     parameters=[
    #         {
    #             "device_id": ParameterValue(LaunchConfiguration("ear_device_id"), value_type=int),
    #             "sample_rate": ParameterValue(LaunchConfiguration("ear_sample_rate"), value_type=int),
    #             "channels": ParameterValue(LaunchConfiguration("ear_channels"), value_type=int),
    #             "chunk_size": ParameterValue(LaunchConfiguration("ear_chunk_size"), value_type=int),
    #             "topic": ParameterValue(LaunchConfiguration("ear_raw_topic"), value_type=str),
    #             "segmenter_segment_topic": ParameterValue(
    #                 LaunchConfiguration("segmenter_segment_topic"), value_type=str
    #             ),
    #             "segmenter_accum_topic": ParameterValue(
    #                 LaunchConfiguration("segmenter_accum_topic"), value_type=str
    #             ),
    #             "speech_accumulator_accum_topic": ParameterValue(
    #                 LaunchConfiguration("speech_accumulator_accum_topic"), value_type=str
    #             ),
    #             "segment_topic": ParameterValue(LaunchConfiguration("segmenter_segment_topic"), value_type=str),
    #             "segment_accum_topic": ParameterValue(
    #                 LaunchConfiguration("segmenter_accum_topic"), value_type=str
    #             ),
    #             "speech_accum_topic": ParameterValue(
    #                 LaunchConfiguration("speech_accumulator_accum_topic"), value_type=str
    #             ),
    #             "segment_sample_rate": ParameterValue(
    #                 LaunchConfiguration("transcriber_segment_sample_rate"), value_type=int
    #             ),
    #             "segmenter_silence_release_ms": ParameterValue(
    #                 LaunchConfiguration("segmenter_silence_release_ms"), value_type=float
    #             ),
    #             "segmenter_lead_silence_ms": ParameterValue(
    #                 LaunchConfiguration("segmenter_lead_silence_ms"), value_type=float
    #             ),
    #             "segmenter_min_speech_ms": ParameterValue(
    #                 LaunchConfiguration("segmenter_min_speech_ms"), value_type=float
    #             ),
    #             "segmenter_max_segment_ms": ParameterValue(
    #                 LaunchConfiguration("segmenter_max_segment_ms"), value_type=float
    #             ),
    #             "segmenter_trim_window_ms": ParameterValue(
    #                 LaunchConfiguration("segmenter_trim_window_ms"), value_type=float
    #             ),
    #             "segmenter_trim_keep_ms": ParameterValue(
    #                 LaunchConfiguration("segmenter_trim_keep_ms"), value_type=float
    #             ),
    #             "segmenter_trim_rms_ratio": ParameterValue(
    #                 LaunchConfiguration("segmenter_trim_rms_ratio"), value_type=float
    #             ),
    #             "segmenter_trim_rms_floor": ParameterValue(
    #                 LaunchConfiguration("segmenter_trim_rms_floor"), value_type=float
    #             ),
    #             "speech_accumulator_reset_timeout": ParameterValue(
    #                 LaunchConfiguration("speech_accumulator_reset_timeout"), value_type=float
    #             ),
    #             "speech_accumulator_max_segments": ParameterValue(
    #                 LaunchConfiguration("speech_accumulator_max_segments"), value_type=int
    #             ),
    #             "segment_dump_dir": ParameterValue(
    #                 LaunchConfiguration("segment_dump_dir"), value_type=str
    #             ),
    #             "segment_dump_enabled": ParameterValue(
    #                 LaunchConfiguration("segment_dump_enabled"), value_type=bool
    #             ),
    #         }
    #     ],
    #     output="screen",
    # )

    # transcriber_short = Node(
    #     package="ear",
    #     executable="transcriber_short_node",
    #     name="transcriber_short",
    #     parameters=[
    #         {
    #             "input_topic": ParameterValue(LaunchConfiguration("transcriber_segment_accum_topic"), value_type=str),
    #             "transcript_topic": ParameterValue(
    #                 LaunchConfiguration("transcriber_transcript_short_topic"), value_type=str
    #             ),
    #             "segment_topic": ParameterValue(
    #                 LaunchConfiguration("transcriber_segment_topic"), value_type=str
    #             ),
    #             "segment_accum_topic": ParameterValue(
    #                 LaunchConfiguration("transcriber_segment_accum_topic"), value_type=str
    #             ),
    #         }
    #     ],
    #     output="screen",
    # )

    # transcriber_medium = Node(
    #     package="ear",
    #     executable="transcriber_medium_node",
    #     name="transcriber_medium",
    #     parameters=[
    #         {
    #             "input_topic": ParameterValue(LaunchConfiguration("transcriber_segment_topic"), value_type=str),
    #             "transcript_topic": ParameterValue(
    #                 LaunchConfiguration("transcriber_transcript_medium_topic"), value_type=str
    #             ),
    #             "segment_topic": ParameterValue(
    #                 LaunchConfiguration("transcriber_segment_topic"), value_type=str
    #             ),
    #             "segment_accum_topic": ParameterValue(
    #                 LaunchConfiguration("transcriber_segment_accum_topic"), value_type=str
    #             ),
    #         }
    #     ],
    #     output="screen",
    # )

    # transcriber_long = Node(
    #     package="ear",
    #     executable="transcriber_long_node",
    #     name="transcriber_long",
    #     parameters=[
    #         {
    #             "input_topic": ParameterValue(LaunchConfiguration("transcriber_speech_accum_topic"), value_type=str),
    #             "transcript_topic": ParameterValue(
    #                 LaunchConfiguration("transcriber_transcript_long_topic"), value_type=str
    #             ),
    #         }
    #     ],
    #     output="screen",
    # )

    # transcriber_local = Node(
    #     package="ear",
    #     executable="transcriber_local_node",
    #     name="transcriber_local",
    #     parameters=[
    #         {
    #             "segment_topic": ParameterValue(LaunchConfiguration("transcriber_segment_topic"), value_type=str),
    #             "transcript_topic": ParameterValue(
    #                 LaunchConfiguration("transcriber_local_transcript_topic"), value_type=str
    #             ),
    #         }
    #     ],
    #     output="screen",
    # )

    hole_node = Node(
        package="ear",
        executable="hole_node",
        name="hole",
        parameters=[
            {
                "device_id": ParameterValue(LaunchConfiguration("ear_device_id"), value_type=int),
                "sample_rate": ParameterValue(LaunchConfiguration("ear_sample_rate"), value_type=int),
                "channels": ParameterValue(LaunchConfiguration("ear_channels"), value_type=int),
                "chunk_size": ParameterValue(LaunchConfiguration("ear_chunk_size"), value_type=int),
                "raw_topic": ParameterValue(LaunchConfiguration("ear_raw_topic"), value_type=str),
                "segment_sample_rate": ParameterValue(
                    LaunchConfiguration("transcriber_segment_sample_rate"), value_type=int
                ),
                "speaker": ParameterValue(LaunchConfiguration("transcriber_speaker"), value_type=str),
                "model": ParameterValue(LaunchConfiguration("transcriber_model"), value_type=str),
                "device": ParameterValue(LaunchConfiguration("transcriber_device"), value_type=str),
                "compute_type": ParameterValue(LaunchConfiguration("transcriber_compute_type"), value_type=str),
                "language": ParameterValue(LaunchConfiguration("transcriber_language"), value_type=str),
                "beam_size": ParameterValue(LaunchConfiguration("transcriber_beam_size"), value_type=int),
                "remote_ws_url": ParameterValue(LaunchConfiguration("hole_remote_ws_url"), value_type=str),
                "remote_connect_timeout": ParameterValue(
                    LaunchConfiguration("transcriber_remote_connect_timeout"), value_type=float
                ),
                "remote_response_timeout": ParameterValue(
                    LaunchConfiguration("transcriber_remote_response_timeout"), value_type=float
                ),
                "remote_audio_dump_dir": ParameterValue(
                    LaunchConfiguration("transcriber_fast_remote_audio_dump_dir"), value_type=str
                ),
                "partial_text_topic": ParameterValue(
                    LaunchConfiguration("hole_partial_text_topic"), value_type=str
                ),
                "partial_detailed_topic": ParameterValue(
                    LaunchConfiguration("hole_partial_detailed_topic"), value_type=str
                ),
                "final_text_topic": ParameterValue(
                    LaunchConfiguration("hole_final_text_topic"), value_type=str
                ),
                "final_detailed_topic": ParameterValue(
                    LaunchConfiguration("hole_final_detailed_topic"), value_type=str
                ),
                "final_audio_topic": ParameterValue(
                    LaunchConfiguration("hole_final_audio_topic"), value_type=str
                ),
                "silence_flush_ms": ParameterValue(
                    LaunchConfiguration("hole_silence_flush_ms"), value_type=float
                ),
                "silence_padding_ms": ParameterValue(
                    LaunchConfiguration("hole_silence_padding_ms"), value_type=float
                ),
                "head_stability_window": ParameterValue(
                    LaunchConfiguration("hole_head_stability_window"), value_type=int
                ),
                "head_min_words": ParameterValue(
                    LaunchConfiguration("hole_head_min_words"), value_type=int
                ),
                "head_min_duration_ms": ParameterValue(
                    LaunchConfiguration("hole_head_min_duration_ms"), value_type=float
                ),
                "request_interval_ms": ParameterValue(
                    LaunchConfiguration("hole_request_interval_ms"), value_type=float
                ),
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        args
        + [
            hole_node,
        ]
    )
