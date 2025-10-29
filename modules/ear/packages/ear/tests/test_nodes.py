"""Unit tests for ear ROS node helpers."""

from __future__ import annotations

import json
import struct

import pytest
import rclpy
from std_msgs.msg import UInt8MultiArray

pytest.importorskip("webrtcvad")

from ear.silence_node import SilenceDetectorNode
from ear.transcriber_node import TranscriberNode, _thread_from_topic
from ear.vad_node import VadNode


@pytest.mark.parametrize("samples,expected", [([0] * 160, True), ([2500] * 160, False)])
def test_silence_detector_state_transitions(monkeypatch: pytest.MonkeyPatch, samples: list[int], expected: bool) -> None:
    rclpy.init()
    node = SilenceDetectorNode()
    published: list[bool] = []
    monkeypatch.setattr(node._publisher, "publish", lambda msg: published.append(bool(msg.data)))
    chunk = struct.pack("<" + "h" * len(samples), *samples)
    message = UInt8MultiArray()
    message.data = list(chunk)
    node._handle_audio(message)
    node.destroy_node()
    rclpy.shutdown()
    assert published
    assert published[-1] is expected


def test_vad_node_emits_changes(monkeypatch: pytest.MonkeyPatch) -> None:
    rclpy.init()
    node = VadNode()
    node._frame_bytes = max(node._frame_bytes, 640)
    sequence = iter([False, True, True, False])

    def fake_is_speech(frame: bytes, sample_rate: int) -> bool:
        try:
            return next(sequence)
        except StopIteration:
            return False

    monkeypatch.setattr(node._vad, "is_speech", fake_is_speech)
    node._publish_on_change = True
    published: list[bool] = []
    monkeypatch.setattr(node._publisher, "publish", lambda msg: published.append(bool(msg.data)))
    frame = bytes([0] * node._frame_bytes)
    message = UInt8MultiArray()
    message.data = list(frame * 4)
    node._handle_audio(message)
    node.destroy_node()
    rclpy.shutdown()
    assert published[:2] == [False, True]


def test_thread_from_topic_helper() -> None:
    assert _thread_from_topic("/conversation/default", prefix="/conversation", default="fallback") == "default"
    assert _thread_from_topic("/conversation/custom/thread", prefix="/conversation", default="fallback") == "custom"
    assert _thread_from_topic("solo-thread", prefix="/conversation", default="fallback") == "solo-thread"
    assert _thread_from_topic("", prefix="/conversation", default="fallback") == "fallback"


def test_transcriber_forwards_final_transcript(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("ear.transcriber_node.EarWorker.start", lambda self: None)
    monkeypatch.setattr("ear.transcriber_node.EarWorker.stop", lambda self: None)
    rclpy.init()
    node = TranscriberNode()
    published_conversant: list[str] = []
    monkeypatch.setattr(node._publisher, "publish", lambda _: None)
    monkeypatch.setattr(node._conversant_publisher, "publish", lambda msg: published_conversant.append(msg.data))
    node._active_conversation_thread = "thread-42"
    try:
        node._publish_final_text("hello world")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    assert published_conversant
    payload = json.loads(published_conversant[-1])
    assert payload["concern"] == "hello world"
    assert payload["thread_id"] == "thread-42"
