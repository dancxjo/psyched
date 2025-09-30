"""ROS entry point for the medium-tier transcription pipeline."""
from __future__ import annotations

from typing import Optional

from .transcriber_common import BaseTranscriberApp

try:  # pragma: no cover - ROS imports only available at runtime
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
except ImportError:  # pragma: no cover - allow unit tests to run without ROS
    rclpy = None  # type: ignore
    Node = object  # type: ignore
    SingleThreadedExecutor = object  # type: ignore


__all__ = ["MediumTranscriberApp", "TranscriberMediumNode", "main"]


class MediumTranscriberApp(BaseTranscriberApp):
    """Configure the medium-tier transcription pipeline for a ROS-like node."""

    def __init__(self, node, backend: Optional[object] = None, *, start_worker: bool = True) -> None:
        super().__init__(
            node,
            tier_name="medium",
            input_topic_param="segment_topic",
            input_topic_default="/audio/speech_segment",
            transcript_topic_params=[
                ("transcript_medium_topic", "/audio/transcript/medium"),
                ("transcript_topic", "/audio/transcription"),
            ],
            include_timing=True,
            include_words=True,
            remote_param="medium_remote_ws_url",
            remote_env_var="EAR_ASR_MEDIUM_WS_URL",
            remote_default="ws://forebrain.local:8083/ws",
            backend_override=backend,
            start_worker=start_worker,
            keep_latest=False,
            remote_trace_param="medium_remote_trace_debug",
            remote_trace_default=True,
            remote_audio_dir_param="medium_remote_audio_dump_dir",
            remote_audio_dir_default="log/remote_asr/medium",
        )


class TranscriberMediumNode(Node):  # type: ignore[misc]
    """ROS 2 node that publishes medium-horizon transcripts."""

    def __init__(self, backend: Optional[object] = None) -> None:  # pragma: no cover - requires ROS
        super().__init__("transcriber_medium")
        self._app = MediumTranscriberApp(self, backend=backend)

    def destroy_node(self):  # pragma: no cover - requires ROS
        if hasattr(self, "_app"):
            self._app.destroy()
        destroy = getattr(super(), "destroy_node", None)
        if callable(destroy):
            return destroy()
        return None


def main(args=None):  # pragma: no cover - requires ROS
    if rclpy is None:
        raise RuntimeError("rclpy is not available; this entry point requires ROS 2")
    rclpy.init(args=args)
    node = TranscriberMediumNode()
    try:
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
