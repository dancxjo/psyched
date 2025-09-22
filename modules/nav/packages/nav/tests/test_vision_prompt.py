from nav.vision_prompt import build_prompt, VisionLLMClient

def test_build_prompt():
    prompt = build_prompt()
    assert "green" in prompt
    assert "JSON" in prompt

def test_vision_llm_client():
    client = VisionLLMClient()
    result = client.annotate(None)
    assert "label" in result
