"""ROS entry point for the short-tier transcription pipeline."""
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


__all__ = ["ShortTranscriberApp", "TranscriberShortNode", "main"]


class ShortTranscriberApp(BaseTranscriberApp):
    """Configure the short-tier transcription pipeline for a ROS-like node."""

    def __init__(self, node, backend: Optional[object] = None, *, start_worker: bool = True) -> None:
        super().__init__(
            node,
            tier_name="short",
            input_topic_param="segment_accumulating_topic",
            input_topic_default="/audio/speech_segment_accumulating",
            transcript_topic_params=[
                ("transcript_short_topic", "/audio/transcript/short"),
            ],
            include_timing=False,
            include_words=False,
            remote_param="fast_remote_ws_url",
            remote_env_var="EAR_ASR_FAST_WS_URL",
            remote_default="ws://forebrain.local:8082/ws",
            backend_override=backend,
            start_worker=start_worker,
            keep_latest=True,
            remote_trace_param="fast_remote_trace_debug",
            remote_trace_default=False,
            remote_audio_dir_param="fast_remote_audio_dump_dir",
            remote_audio_dir_default="",
        )


class TranscriberShortNode(Node):  # type: ignore[misc]
    """ROS 2 node that publishes short-horizon transcripts."""

    def __init__(self, backend: Optional[object] = None) -> None:  # pragma: no cover - requires ROS
        super().__init__("transcriber_short")
        self._app = ShortTranscriberApp(self, backend=backend)

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
    node = TranscriberShortNode()
    try:
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
