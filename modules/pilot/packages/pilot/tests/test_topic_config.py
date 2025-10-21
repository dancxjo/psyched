import json
from pathlib import Path
from typing import Any

from pilot.node import (
    _default_context_topics,
    _default_sensation_topics,
    _discover_topic_suggestions,
    _normalise_topic_entries,
    _parse_topic_parameter,
    _resolve_host_names,
)


class _StubLogger:
    def __init__(self) -> None:
        self.messages: list[str] = []

    def warning(self, message: str, *args: Any) -> None:
        if args:
            self.messages.append(message % args)
        else:
            self.messages.append(message)


def test_resolve_host_names_prefers_env() -> None:
    host_full, host_short = _resolve_host_names({"PILOT_HOST": "pete.example.com"})
    assert host_full == "pete.example.com"
    assert host_short == "pete"


def test_default_context_topics_include_host_short() -> None:
    topics = _default_context_topics("motherbrain")
    assert any(entry["topic"] == "/hosts/health/motherbrain" for entry in topics)
    assert any(entry["topic"] == "/status" for entry in topics)


def test_default_sensation_topics_contains_primary_stream() -> None:
    topics = _default_sensation_topics()
    assert {"topic": "/sensations", "type": "psyched_msgs/msg/SensationStamped"} in topics


def test_normalise_topic_entries_keep_first() -> None:
    entries = [
        {"topic": "/foo", "type": "std_msgs/msg/String", "prompt_template": "value={{data}}"},
        {"topic": "/foo", "type": "std_msgs/msg/Int64"},
        {"topic": "/bar", "type": "std_msgs/msg/String"},
    ]
    result = _normalise_topic_entries(entries, keep_first=True)
    assert result == [
        {"topic": "/foo", "type": "std_msgs/msg/String", "prompt_template": "value={{data}}"},
        {"topic": "/bar", "type": "std_msgs/msg/String"},
    ]


def test_parse_topic_parameter_invalid_json_falls_back() -> None:
    fallback = [{"topic": "/foo", "type": "std_msgs/msg/String"}]
    logger = _StubLogger()
    result = _parse_topic_parameter("not-json", param_name="context_topics", fallback=fallback, logger=logger)
    assert result == fallback
    assert logger.messages, "Expected warning to be recorded for invalid JSON"


def test_discover_topic_suggestions_expands_templates(tmp_path: Path) -> None:
    module_dir = tmp_path / "viscera" / "pilot"
    module_dir.mkdir(parents=True)
    payload = {
        "context_topics": [
            {
                "topic": "/hosts/health/{HOST_SHORT}",
                "type": "psyched_msgs/msg/HostHealth",
                "prompt_template": "host={{data.status}}",
            }
        ],
        "sensation_topics": [
            {"topic": "/custom/{HOST}", "type": "std_msgs/msg/String"}
        ],
    }
    (module_dir / "topic_suggestions.json").write_text(json.dumps(payload), encoding="utf-8")
    suggestions = _discover_topic_suggestions(
        tmp_path,
        host_full="motherbrain.local",
        host_short="motherbrain",
        logger=None,
    )
    assert suggestions["context_topics"] == [
        {
            "topic": "/hosts/health/motherbrain",
            "type": "psyched_msgs/msg/HostHealth",
            "prompt_template": "host={{data.status}}",
        }
    ]
    assert suggestions["sensation_topics"] == [
        {"topic": "/custom/motherbrain.local", "type": "std_msgs/msg/String"}
    ]
