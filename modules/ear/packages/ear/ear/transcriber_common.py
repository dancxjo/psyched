"""Shared utilities for tiered transcription nodes."""
from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from .audio_utils import coerce_pcm_bytes
from .transcription_backends import initialise_remote_backend, load_backend
from .transcription_pipeline import Transcript, TranscriptPublisher, TranscriptionPipeline
from .qos import best_effort_qos, sensor_data_qos

try:  # pragma: no cover - available only inside a ROS 2 environment
    from std_msgs.msg import ByteMultiArray
except ImportError:  # pragma: no cover - provide a stub for unit tests
    @dataclass
    class ByteMultiArray:
        data: Sequence[int]


__all__ = ["BaseTranscriberApp", "ByteMultiArray"]


class BaseTranscriberApp:
    """Wire a ROS-like node into the shared transcription pipeline."""

    def __init__(
        self,
        node,
        *,
        tier_name: str,
        input_topic_param: str,
        input_topic_default: str,
        transcript_topic_params: Sequence[Tuple[str, str]],
        include_timing: bool,
        include_words: bool,
        remote_param: str,
        remote_env_var: str,
        remote_default: str,
        backend_override: Optional[object] = None,
        start_worker: bool = True,
        keep_latest: bool = False,
        remote_trace_param: Optional[str] = None,
        remote_trace_default: bool = False,
        remote_audio_dir_param: Optional[str] = None,
        remote_audio_dir_default: Optional[str] = None,
    ) -> None:
        self._node = node
        self._logger = getattr(node, "get_logger", lambda: None)()
        self._speaker_label = str(node.declare_parameter("speaker", "user").value)
        self._sample_rate = int(node.declare_parameter("segment_sample_rate", 16000).value)
        self._model_name = str(node.declare_parameter("model", "base").value)
        self._device = str(node.declare_parameter("device", "cpu").value)
        self._compute_type = str(node.declare_parameter("compute_type", "int8").value)
        language_value = node.declare_parameter("language", "").value
        self._language = str(language_value).strip() or None
        self._beam_size = int(node.declare_parameter("beam_size", 5).value)
        self._connect_timeout = float(node.declare_parameter("remote_connect_timeout", 0.6).value)
        self._response_timeout = float(node.declare_parameter("remote_response_timeout", 1.5).value)

        remote_default_value = os.getenv(remote_env_var, remote_default)
        remote_param_value = node.declare_parameter(remote_param, remote_default_value).value
        self._remote_url = str(remote_param_value or "").strip()

        trace_param_name = remote_trace_param or f"{tier_name}_remote_trace_debug"
        trace_default_value = bool(remote_trace_default)
        trace_param_value = node.declare_parameter(trace_param_name, trace_default_value).value
        self._remote_trace_debug = bool(trace_param_value) if isinstance(trace_param_value, bool) else str(trace_param_value).lower() in {"1", "true", "yes", "on"}

        audio_dir_param_name = remote_audio_dir_param or f"{tier_name}_remote_audio_dump_dir"
        audio_dir_default_value = remote_audio_dir_default or ""
        audio_dir_param_value = node.declare_parameter(audio_dir_param_name, audio_dir_default_value).value
        self._remote_audio_dump_dir = str(audio_dir_param_value or "").strip()

        self._input_topic = str(node.declare_parameter(input_topic_param, input_topic_default).value)
        self._publishers = []
        for name, default in transcript_topic_params:
            topic = str(node.declare_parameter(name, default).value)
            publisher = node.create_publisher(Transcript, topic, best_effort_qos(depth=10))
            self._publishers.append(publisher)

        self._transcript_publisher = TranscriptPublisher(
            publishers=self._publishers,
            speaker_label=self._speaker_label,
            include_timing=include_timing,
            include_words=include_words,
        )

        backend = backend_override if backend_override is not None else self._build_backend()
        self._pipeline = TranscriptionPipeline(
            tier=tier_name,
            backend=backend,
            sample_rate=self._sample_rate,
            publisher=self._transcript_publisher,
            logger=self._logger,
            keep_latest=keep_latest,
            start_worker=start_worker,
        )

        self._subscription = node.create_subscription(
            ByteMultiArray,
            self._input_topic,
            self._on_audio_segment,
            sensor_data_qos(),
        )

    def destroy(self) -> None:
        self._pipeline.stop()

    def _build_backend(self) -> Optional[object]:
        remote = initialise_remote_backend(
            uri=self._remote_url,
            language=self._language,
            connect_timeout=self._connect_timeout,
            response_timeout=self._response_timeout,
            logger=self._logger,
            trace=self._remote_trace_debug,
            dump_audio_dir=self._remote_audio_dump_dir or None,
        )
        if remote is not None:
            return remote
        return load_backend(
            self._model_name,
            self._device,
            self._compute_type,
            self._language,
            self._beam_size,
            logger=self._logger,
        )

    def _on_audio_segment(self, msg: ByteMultiArray) -> None:
        pcm = coerce_pcm_bytes(getattr(msg, "data", b""))
        if not pcm:
            return
        self._pipeline.enqueue(pcm)
