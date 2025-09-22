"""
vision_prompt.py

Builds prompts for vision LLM to annotate green line objects in RGB images.
Stub LLM client for structured annotation.
"""
import json

VISION_PROMPT_TEMPLATE = (
    "You are a vision AI. Given an image, identify all objects that intersect the _green_ line. "
    "Return a JSON array of objects, each with 'label', 'bounding_box' (x,y,w,h), and 'confidence'. "
    "If no object is present, return an empty array."
)

def build_prompt():
    return VISION_PROMPT_TEMPLATE

class VisionLLMClient:
    def __init__(self):
        pass
    def annotate(self, rgb_img):
        # Stub: Replace with actual LLM call
        # For now, returns dummy annotation
        return json.dumps([
            {"label": "box", "bounding_box": [100, 50, 40, 40], "confidence": 0.8}
        ])
