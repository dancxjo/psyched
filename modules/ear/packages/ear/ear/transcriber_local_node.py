"""ROS entry point for the local (faster-whisper) transcription pipeline."""
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


__all__ = ["LocalTranscriberApp", "LocalTranscriptionNode", "main"]


class LocalTranscriberApp(BaseTranscriberApp):
    """Configure a local faster-whisper transcription pipeline.

    This pipeline is intended to decode incoming speech segments quickly on
    the local host using faster-whisper. It subscribes to completed segments
    on the same topic as the medium tier by default.
    """

    def __init__(self, node, backend: Optional[object] = None, *, start_worker: bool = True) -> None:
        super().__init__(
            node,
            tier_name="local",
            input_topic_param="segment_topic",
            input_topic_default="/audio/speech_segment",
            transcript_topic_params=[
                ("transcript_topic", "/audio/transcription"),
            ],
            include_timing=False,
            include_words=False,
            remote_param="transcriber_fast_remote_ws_url",
            remote_env_var="EAR_ASR_FAST_WS_URL",
            remote_default="",
            backend_override=backend,
            start_worker=start_worker,
            keep_latest=True,
        )


class LocalTranscriptionNode(Node):  # type: ignore[misc]
    """ROS 2 node that publishes fast local transcripts (faster-whisper)."""

    def __init__(self, backend: Optional[object] = None) -> None:  # pragma: no cover - requires ROS
        super().__init__("local_transcription")
        self._app = LocalTranscriberApp(self, backend=backend)

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
    node = LocalTranscriptionNode()
    try:
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
