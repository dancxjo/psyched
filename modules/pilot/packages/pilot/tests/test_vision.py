from __future__ import annotations

import base64

from pilot.prompt_builder import PromptImage
from pilot.vision import summarise_image_message


class DummyCompressedImage:
    """Simplified stand-in for ``sensor_msgs.msg.CompressedImage``."""

    def __init__(self, *, fmt: str = "jpeg", data: bytes | list[int] = b"\x01\x02\x03") -> None:
        self.format = fmt
        self.data = data


def test_summarise_image_message_extracts_prompt_image() -> None:
    metadata = {"format": "jpeg", "data": [1, 2, 3]}
    msg = DummyCompressedImage(data=[1, 2, 3])

    sanitised, prompt_image = summarise_image_message(
        "/camera/color/image_raw/compressed",
        msg,
        metadata,
    )

    assert isinstance(prompt_image, PromptImage)
    assert prompt_image.topic == "/camera/color/image_raw/compressed"
    expected_b64 = base64.b64encode(bytes([1, 2, 3])).decode("ascii")
    assert prompt_image.base64_data == expected_b64
    assert "bytes" in prompt_image.description

    assert sanitised != metadata
    assert sanitised["data"].startswith("<omitted binary:")
    assert sanitised["byte_length"] == 3


def test_summarise_image_message_gracefully_handles_missing_data() -> None:
    metadata = {"format": "jpeg", "data": []}
    msg = DummyCompressedImage(data=[])

    sanitised, prompt_image = summarise_image_message("/camera", msg, metadata)

    assert prompt_image is None
    assert sanitised["data"].startswith("<omitted binary:")
