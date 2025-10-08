"""Helpers for prompting a vision-language model about scan overlays."""

from __future__ import annotations

import json
from typing import Any

VISION_PROMPT_TEMPLATE = (
    "You are a vision AI. Given an image, identify all objects that intersect the _green_ line. "
    "Return a JSON array of objects, each with 'label', 'bounding_box' (x,y,w,h), and 'confidence'. "
    "If no object is present, return an empty array."
)


def build_prompt() -> str:
    return VISION_PROMPT_TEMPLATE


class VisionLLMClient:
    def annotate(self, rgb_img: Any) -> str:
        return json.dumps([
            {"label": "box", "bounding_box": [100, 50, 40, 40], "confidence": 0.8}
        ])
"""Helpers for prompting a vision-language model about scan overlays."""

from __future__ import annotations

import json
from typing import Any

VISION_PROMPT_TEMPLATE = (
    "You are a vision AI. Given an image, identify all objects that intersect the _green_ line. "
    "Return a JSON array of objects, each with 'label', 'bounding_box' (x,y,w,h), and 'confidence'. "
    "If no object is present, return an empty array."
)


def build_prompt() -> str:
    """Return the reusable system prompt for annotating scan overlays."""

    return VISION_PROMPT_TEMPLATE


class VisionLLMClient:
    """Very small stub mimicking a vision-language model client."""

    def annotate(self, rgb_img: Any) -> str:
        """Return a canned annotation payload.

        Parameters
        ----------
        rgb_img:
            Placeholder for the RGB image that would be sent to the real model.

        Returns
        -------
        str
            JSON string containing a mocked detection result.
        """

        return json.dumps([
            {"label": "box", "bounding_box": [100, 50, 40, 40], "confidence": 0.8}
        ])
