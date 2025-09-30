from pilot.app import _merge_qos


def test_merge_qos_overrides_and_preserves_defaults():
    module = {"history": "keep_last", "depth": 5, "reliability": "best_effort", "durability": "volatile"}
    provided = {"depth": 10}
    merged = _merge_qos(module, provided)
    assert merged["depth"] == 10
    assert merged["reliability"] == "best_effort"

    merged_none = _merge_qos(module, None)
    assert merged_none == module
