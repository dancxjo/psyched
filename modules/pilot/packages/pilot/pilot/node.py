from __future__ import annotations

import json
import logging
import os
import socket
import threading
import time
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, List, Mapping, Optional, Sequence
from urllib import error, request
from uuid import uuid4

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

from psyched_msgs.msg import FeelingIntent, SensationStamped
from std_msgs.msg import String as StdString

from .cockpit_helpers import (
    load_local_cockpit_actions,
    normalise_cockpit_modules_payload,
    render_action_signature,
)
from .command_script import (
    CommandInvocation,
    CommandScriptError,
    CommandScriptInterpreter,
)
from .memory_pipeline import MemoryEventDraft, prepare_memory_batch
from .models import FeelingIntentData, SensationRecord
from .prompt_builder import PilotPromptContext, PromptImage, build_prompt
from .topic_translation import (
    TranslatorRegistry,
    apply_topic_translation,
    discover_topic_translators,
)

if TYPE_CHECKING:
    from memory_interfaces.msg import MemoryEvent
from .validators import FeelingIntentValidationError, parse_feeling_intent_json
from .vision import summarise_image_message


_LOGGER = logging.getLogger("psyched.pilot.node")
if not _LOGGER.handlers:
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s [pilot.node] %(message)s"))
    _LOGGER.addHandler(handler)
_LOGGER.setLevel(logging.INFO)


_FEEDBACK_FIELD_TOPICS: Dict[str, Dict[str, str]] = {
    "situation_overview": {
        "topic": "/pilot/context/situation_overview",
        "type": "std_msgs/msg/String",
        "prompt_template": "{{data.data}}",
    },
    "attitude_emoji": {
        "topic": "/pilot/context/attitude_emoji",
        "type": "std_msgs/msg/String",
        "prompt_template": "{{data.data}}",
    },
    "thought_sentence": {
        "topic": "/pilot/context/thought_sentence",
        "type": "std_msgs/msg/String",
        "prompt_template": "{{data.data}}",
    },
    "spoken_sentence": {
        "topic": "/pilot/context/spoken_sentence",
        "type": "std_msgs/msg/String",
        "prompt_template": "{{data.data}}",
    },
    "command_script": {
        "topic": "/pilot/context/command_script",
        "type": "std_msgs/msg/String",
        "prompt_template": "{{data.data}}",
    },
    "goals": {
        "topic": "/pilot/context/goals",
        "type": "std_msgs/msg/String",
        "prompt_template": "{{data.data}}",
    },
    "mood_delta": {
        "topic": "/pilot/context/mood_delta",
        "type": "std_msgs/msg/String",
        "prompt_template": "{{data.data}}",
    },
    "memory_collection_raw": {
        "topic": "/pilot/context/memory_collection_raw",
        "type": "std_msgs/msg/String",
        "prompt_template": "{{data.data}}",
    },
    "memory_collection_text": {
        "topic": "/pilot/context/memory_collection_text",
        "type": "std_msgs/msg/String",
        "prompt_template": "{{data.data}}",
    },
    "memory_collection_emoji": {
        "topic": "/pilot/context/memory_collection_emoji",
        "type": "std_msgs/msg/String",
        "prompt_template": "{{data.data}}",
    },
    "episode_id": {
        "topic": "/pilot/context/episode_id",
        "type": "std_msgs/msg/String",
        "prompt_template": "{{data.data}}",
    },
    "situation_id": {
        "topic": "/pilot/context/situation_id",
        "type": "std_msgs/msg/String",
        "prompt_template": "{{data.data}}",
    },
}

_IGNORED_ACTION_NAMES = {"call_service", "stream_topic"}


def _feedback_context_topics() -> List[Dict[str, str]]:
    """Return context topic entries for pilot feedback publications."""

    entries: List[Dict[str, str]] = []
    for meta in _FEEDBACK_FIELD_TOPICS.values():
        item = {"topic": meta["topic"], "type": meta["type"]}
        template = meta.get("prompt_template")
        if isinstance(template, str) and template:
            item["prompt_template"] = template
        entries.append(item)
    return entries


def _resolve_host_names(env: Optional[Mapping[str, str]] = None) -> tuple[str, str]:
    """Return ``(hostname, shortname)`` derived from *env* or the local host."""

    source: Mapping[str, str] = env if env is not None else os.environ
    raw = (
        str(
            source.get("PILOT_HOST")
            or source.get("HOST")
            or source.get("HOSTNAME")
            or socket.gethostname()
            or ""
        ).strip()
    )
    if not raw:
        raw = "host"
    short = raw.split(".")[0] or raw
    return raw, short


def _default_context_topics(host_short: str) -> List[Dict[str, str]]:
    """Return the minimal default context topic subscriptions."""

    return [
        {
            "topic": f"/hosts/health/{host_short}",
            "type": "psyched_msgs/msg/HostHealth",
            "prompt_template": "{{data.prompt_summary}}",
        },
        {"topic": "/instant", "type": "std_msgs/msg/String"},
        {"topic": "/situation", "type": "std_msgs/msg/String"},
        {"topic": "/status", "type": "std_msgs/msg/String"},
    ]


def _default_sensation_topics() -> List[Dict[str, str]]:
    """Return the minimal default sensation topic subscriptions."""

    return [{"topic": "/sensations", "type": "psyched_msgs/msg/SensationStamped"}]


def _normalise_topic_entries(
    entries: Sequence[Mapping[str, Any]],
    *,
    keep_first: bool,
) -> List[Dict[str, str]]:
    """Return topic/type entries with duplicates removed while preserving order."""

    dedup: Dict[str, Dict[str, Any]] = {}
    for entry in entries:
        if not isinstance(entry, Mapping):
            continue
        topic = str(entry.get("topic") or "").strip()
        type_name = str(entry.get("type") or "").strip()
        if not topic or not type_name:
            continue
        if keep_first and topic in dedup:
            existing = dedup[topic]
            template = entry.get("prompt_template")
            if isinstance(template, str) and template and not existing.get("prompt_template"):
                existing["prompt_template"] = template
            continue
        cleaned = dict(entry)
        cleaned["topic"] = topic
        cleaned["type"] = type_name
        dedup[topic] = cleaned
    return list(dedup.values())


def _expand_topic_template(value: str, *, host_full: str, host_short: str) -> str:
    """Return *value* with host placeholders substituted."""

    result = value.replace("{HOST_SHORT}", host_short).replace("${HOST_SHORT}", host_short)
    result = result.replace("{HOST}", host_full).replace("${HOST}", host_full)
    return result


def _discover_topic_suggestions(
    root: Optional[Path],
    *,
    host_full: str,
    host_short: str,
    logger: Optional[Any] = None,
) -> Dict[str, List[Dict[str, str]]]:
    """Return topic suggestions discovered under *root*."""

    suggestions: Dict[str, List[Dict[str, str]]] = {
        "context_topics": [],
        "sensation_topics": [],
    }
    if root is None:
        return suggestions
    try:
        base = Path(root)
    except TypeError:
        return suggestions
    if not base.exists():
        return suggestions

    for module_dir in sorted(p for p in base.iterdir() if p.is_dir()):
        candidate = module_dir / "pilot" / "topic_suggestions.json"
        if not candidate.exists():
            continue
        try:
            payload = json.loads(candidate.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            if logger:
                logger.warning("Failed to parse %s: %s", candidate, exc)
            continue
        for key in ("context_topics", "sensation_topics"):
            items = payload.get(key)
            if not isinstance(items, list):
                continue
            for entry in items:
                if not isinstance(entry, Mapping):
                    continue
                topic = str(entry.get("topic") or "").strip()
                type_name = str(entry.get("type") or "").strip()
                if not topic or not type_name:
                    continue
                expanded_topic = _expand_topic_template(topic, host_full=host_full, host_short=host_short)
                suggestion = {"topic": expanded_topic, "type": type_name}
                template = entry.get("prompt_template")
                if isinstance(template, str) and template:
                    suggestion["prompt_template"] = template
                suggestions[key].append(suggestion)
    return suggestions


def _suggestions_root(env: Optional[Mapping[str, str]] = None) -> Optional[Path]:
    """Return the directory containing module topic suggestion manifests."""

    source: Mapping[str, str] = env if env is not None else os.environ
    explicit = source.get("PILOT_TOPIC_SUGGESTIONS_ROOT")
    if explicit:
        return Path(explicit)
    repo_dir = source.get("REPO_DIR")
    if repo_dir:
        return Path(repo_dir) / "modules"
    return None


def _parse_topic_parameter(
    raw_value: Any,
    *,
    param_name: str,
    fallback: List[Dict[str, str]],
    logger: Any,
) -> List[Dict[str, str]]:
    """Return the configured topic entries parsed from ROS parameter storage."""

    if isinstance(raw_value, str):
        try:
            loaded = json.loads(raw_value)
        except json.JSONDecodeError:
            logger.warning("Invalid JSON for %s, using fallback", param_name)
            return list(fallback)
    elif isinstance(raw_value, list):
        loaded = raw_value
    else:
        return list(fallback)

    entries = _normalise_topic_entries(
        [entry for entry in loaded if isinstance(entry, Mapping)],
        keep_first=False,
    )
    return entries


class LLMClient:
    """Abstract interface so tests can stub LLM calls."""

    def generate(
        self, prompt: str, *, images: Optional[Sequence[str]] = None
    ) -> str:  # pragma: no cover - interface
        raise NotImplementedError


class OllamaLLMClient(LLMClient):
    """HTTP client for Ollama's `/api/generate` endpoint."""

    def __init__(self, model: str, host: Optional[str] = None, timeout: float = 30.0) -> None:
        self.model = model
        self.host = (host or os.environ.get("OLLAMA_HOST") or "http://127.0.0.1:11434").rstrip("/")
        self.timeout = timeout
        self._last_llm_response: Optional[str] = None

    def generate(self, prompt: str, *, images: Optional[Sequence[str]] = None) -> str:
        """Call Ollama and stream output to stdout while assembling the full response.

        This attempts to request a streaming response from Ollama (stream: true).
        For each chunk received, we write to stdout and flush so the caller can
        observe the raw model stream. We also assemble the chunks and return
        the final response string.
        """
        body: Dict[str, Any] = {"model": self.model, "prompt": prompt, "stream": True}
        if images:
            body["images"] = list(images)
        payload = json.dumps(body).encode("utf-8")
        req = request.Request(
            self.host + "/api/generate",
            data=payload,
            headers={"Content-Type": "application/json"},
        )
        assembled: list[str] = []
        try:
            with request.urlopen(req, timeout=self.timeout) as resp:
                # Try to stream incrementally. Some servers yield chunked JSON lines.
                # We'll read in small blocks and attempt to decode partial JSON payloads.
                try:
                    # If response supports iteration by lines
                    for raw in resp:
                        try:
                            chunk = raw.decode("utf-8") if isinstance(raw, (bytes, bytearray)) else str(raw)
                        except Exception:
                            continue
                        chunk = chunk.strip()
                        if not chunk:
                            continue
                        # Print raw chunk for operator visibility
                        try:
                            print(chunk, end="", flush=True)
                        except Exception:
                            pass
                        # Try to parse as JSON and extract 'response' or 'content'
                        try:
                            data = json.loads(chunk)
                            if isinstance(data, dict) and "response" in data:
                                text = str(data.get("response") or "")
                                assembled.append(text)
                                continue
                            if isinstance(data, dict) and "content" in data:
                                # Some streaming infra uses 'content' chunks
                                assembled.append(str(data.get("content") or ""))
                                continue
                        except json.JSONDecodeError:
                            # Not JSON: append raw chunk
                            assembled.append(chunk)
                            continue
                except TypeError:
                    # resp is not iterable; fall back to reading whole body
                    body = resp.read().decode("utf-8")
                    try:
                        data = json.loads(body)
                        if isinstance(data, dict) and "response" in data:
                            assembled.append(str(data.get("response") or ""))
                        else:
                            assembled.append(body)
                    except json.JSONDecodeError:
                        assembled.append(body)
        except Exception as exc:  # pragma: no cover - network failure path
            raise RuntimeError(f"Failed to reach Ollama at {self.host}: {exc}") from exc

        final = "".join(assembled).strip()
        # store a small recent copy for debug snapshots
        try:
            self._last_llm_response = final[:4096]
        except Exception:
            self._last_llm_response = final
        # ensure newline after streamed raw output so terminal is tidy
        try:
            print(flush=True)
        except Exception:
            pass
        return final


class MemoryServiceClient:
    """Lightweight wrapper around the memory module's ROS services."""

    def __init__(self, node: Node, *, timeout: float = 2.0) -> None:
        self._node = node
        self._timeout = timeout
        self._enabled = False
        try:
            from memory_interfaces.msg import MemoryEvent as _MemoryEvent  # type: ignore[import]
            from memory_interfaces.srv import Associate as _Associate  # type: ignore[import]
            from memory_interfaces.srv import Memorize as _Memorize  # type: ignore[import]
        except ImportError:  # pragma: no cover - optional dependency for tests
            node.get_logger().warning(
                "memory_interfaces package not available; pilot memory writes disabled"
            )
            self._MemoryEvent = None
            self._Memorize = None
            self._Associate = None
            self._memorize_client = None
            self._associate_client = None
            self._service_unavailable_logged = {}
            return

        self._MemoryEvent = _MemoryEvent
        self._Memorize = _Memorize
        self._Associate = _Associate
        self._memorize_client = node.create_client(self._Memorize, "/memory/memorize")
        self._associate_client = node.create_client(self._Associate, "/memory/associate")
        self._service_unavailable_logged = {"memorize": False, "associate": False}
        self._enabled = True

    @property
    def enabled(self) -> bool:
        return self._enabled

    def build_event_message(
        self,
        *,
        stamp,
        frame_id: str,
        kind: str,
        metadata: Mapping[str, Any],
        embedding: Optional[Sequence[float]],
    ) -> Optional["MemoryEvent"]:
        if not self._enabled or self._MemoryEvent is None:
            return None
        event_msg = self._MemoryEvent()
        event_msg.header.stamp = stamp
        event_msg.header.frame_id = frame_id or "memory"
        event_msg.kind = kind or "generic"
        event_msg.json_data = json.dumps(dict(metadata), separators=(",", ":"))
        event_msg.embedding = [float(value) for value in embedding] if embedding else []
        return event_msg

    def memorize(self, event: "MemoryEvent", *, flush: bool = True) -> Optional[str]:
        if not self._enabled or self._memorize_client is None:
            return None
        if not self._memorize_client.wait_for_service(timeout_sec=0.0):
            self._log_unavailable("memorize")
            return None
        request_msg = self._Memorize.Request()
        request_msg.event = event
        request_msg.flush = flush
        future = self._memorize_client.call_async(request_msg)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=self._timeout)
        if not future.done():
            future.cancel()
            self._node.get_logger().warning("memory memorize request timed out")
            return None
        if future.exception():
            self._node.get_logger().warning(f"memory memorize request failed: {future.exception()}")
            return None
        response = future.result()
        if response is None:
            return None
        return response.memory_id or None

    def associate(
        self,
        *,
        source_id: str,
        target_id: str,
        relation_type: str = "ASSOCIATED_WITH",
        properties: Optional[Mapping[str, Any]] = None,
    ) -> bool:
        if not self._enabled or self._associate_client is None:
            return False
        if not self._associate_client.wait_for_service(timeout_sec=0.0):
            self._log_unavailable("associate")
            return False
        request_msg = self._Associate.Request()
        request_msg.source_id = source_id
        request_msg.target_id = target_id
        request_msg.relation_type = relation_type
        request_msg.json_properties = json.dumps(dict(properties or {}), separators=(",", ":"))
        future = self._associate_client.call_async(request_msg)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=self._timeout)
        if not future.done():
            future.cancel()
            self._node.get_logger().warning("memory associate request timed out")
            return False
        if future.exception():
            self._node.get_logger().warning(f"memory associate request failed: {future.exception()}")
            return False
        response = future.result()
        return bool(response and response.success)

    def _log_unavailable(self, service: str) -> None:
        if not self._enabled:
            return
        if not self._service_unavailable_logged.get(service):
            self._node.get_logger().warning(
                f"/memory/{service} service not available; memory writes will be skipped until it appears"
            )
            self._service_unavailable_logged[service] = True


def _ros_time_to_datetime(time_msg) -> datetime:
    sec = getattr(time_msg, "sec", 0)
    nanosec = getattr(time_msg, "nanosec", 0)
    return datetime.fromtimestamp(float(sec) + float(nanosec) / 1e9, tz=timezone.utc)


class PilotNode(Node):
    """ROS node orchestrating the feeling + will integration pipeline."""

    def __init__(
        self,
        *,
        llm_client: Optional[LLMClient] = None,
        memory_client: Optional[MemoryServiceClient] = None,
    ) -> None:
        super().__init__("pilot")

        self._llm_client = llm_client or OllamaLLMClient(model=os.environ.get("FELT_MODEL", "gpt-oss"))
        self._memory_client = memory_client or MemoryServiceClient(self)

        self._debounce_seconds = self._declare_float_parameter("debounce_seconds", 3.0)
        self._window_seconds = self._declare_float_parameter("window_seconds", 3.0)
        self._feedback_topics_enabled = self._declare_bool_parameter("feedback_topics_enabled", True)

        self._topic_cache: Dict[str, Dict[str, Any]] = {}
        self._sensation_records: List[tuple[float, SensationRecord]] = []
        self._conversation_threads: Dict[str, List[Dict[str, Any]]] = {}
        self._conversation_thread_order: List[str] = []
        self._conversant_topic_name: str = "/conversant/memory_event"
        self._conversation_subscription = None
        self._subscriptions: List[Any] = []
        self._dirty = False

        self._command_interpreter = CommandScriptInterpreter()
        self._script_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="pilot-script")
        self._script_runs: List[Dict[str, Any]] = []
        self._script_lock = threading.Lock()
        self._action_schemas: Dict[str, Dict[str, Any]] = {}
        self._topic_templates: Dict[str, str] = {}
        self._feedback_publishers: Dict[str, Any] = {}
        self._modules_root = self._discover_modules_root()

        host_full, host_short = _resolve_host_names()
        self._host_full = host_full
        self._host_short = host_short

        translator_registry: TranslatorRegistry = discover_topic_translators(
            self._modules_root,
            logger=self.get_logger(),
        )
        self._topic_translators = translator_registry
        raw_sections = [
            _expand_topic_template(
                section,
                host_full=self._host_full,
                host_short=self._host_short,
            )
            for section in translator_registry.static_sections
        ]
        seen_sections: Dict[str, None] = {}
        self._static_prompt_sections = []
        for section in raw_sections:
            text = section.strip()
            if not text or text in seen_sections:
                continue
            seen_sections[text] = None
            self._static_prompt_sections.append(text)
        self._actions_fallback_logged = False
        self._last_prompt: Optional[str] = None

        suggestions = _discover_topic_suggestions(
            _suggestions_root(),
            host_full=host_full,
            host_short=host_short,
            logger=self.get_logger(),
        )
        context_entries = _default_context_topics(host_short)
        if self._feedback_topics_enabled:
            context_entries.extend(_feedback_context_topics())
        context_defaults = _normalise_topic_entries(
            context_entries + suggestions["context_topics"],
            keep_first=True,
        )
        sensation_defaults = _normalise_topic_entries(
            _default_sensation_topics() + suggestions["sensation_topics"],
            keep_first=True,
        )

        qos = QoSProfile(depth=10)
        qos.history = QoSHistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.RELIABLE

        conversant_param = self.declare_parameter("conversant_memory_topic", "/conversant/memory_event").value
        conversant_topic = str(conversant_param).strip() or "/conversant/memory_event"
        self._conversant_topic_name = conversant_topic
        try:
            self._conversation_subscription = self.create_subscription(
                StdString,
                conversant_topic,
                self._handle_conversant_memory_event,
                qos,
            )
            self._subscriptions.append(self._conversation_subscription)
            self.get_logger().info(
                f"Subscribing to conversant memory events on {conversant_topic}"
            )
        except Exception as exc:
            self.get_logger().warning(
                f"Failed to subscribe to conversant memory events: {exc}"
            )
            self._conversation_subscription = None

        self.publisher = self.create_publisher(FeelingIntent, "pilot/intent", qos)

        if self._feedback_topics_enabled:
            self._initialise_feedback_publishers(qos)

        # Debug publisher: emits JSON snapshots for cockpit debugging
        try:
            self._debug_publisher = self.create_publisher(StdString, "pilot/debug", qos)
            # Emit debug snapshot every 5 seconds
            self._debug_timer = self.create_timer(5.0, self._emit_debug_snapshot)
        except Exception:
            # If std_msgs isn't available or publisher cannot be created in tests,
            # silently continue; debug snapshots are optional.
            self._debug_publisher = None
            self._debug_timer = None

        self._context_topics = self._load_topic_configs(
            "context_topics", context_defaults, self._handle_context_message, qos
        )
        self._sensation_topics = self._load_topic_configs(
            "sensation_topics", sensation_defaults, self._handle_sensation_message, qos
        )
        self._seed_minimal_context(self._host_short)

        self._timer = self.create_timer(self._debounce_seconds, self._on_timer)

    def _declare_float_parameter(self, name: str, default: float) -> float:
        param = self.declare_parameter(name, default)
        value = getattr(param, "value", default)
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    def _declare_bool_parameter(self, name: str, default: bool) -> bool:
        param = self.declare_parameter(name, bool(default))
        value = getattr(param, "value", default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        if isinstance(value, (int, float)):
            return bool(value)
        return bool(default)

    def _initialise_feedback_publishers(self, qos: QoSProfile) -> None:
        for field, meta in _FEEDBACK_FIELD_TOPICS.items():
            topic_name = meta["topic"]
            publisher = self.create_publisher(StdString, topic_name, qos)
            self._feedback_publishers[field] = publisher
            template = meta.get("prompt_template")
            if isinstance(template, str) and template:
                self._topic_templates.setdefault(topic_name, template)

    @staticmethod
    def _serialise_feedback_value(value: Any) -> str:
        if value is None:
            return ""
        if isinstance(value, str):
            return value
        if isinstance(value, (list, tuple, dict)):
            try:
                return json.dumps(value, ensure_ascii=False, sort_keys=True)
            except (TypeError, ValueError):
                return json.dumps(value, ensure_ascii=False)
        return str(value)

    def _publish_feedback_topics(self, feeling: FeelingIntentData) -> None:
        if not self._feedback_topics_enabled or not self._feedback_publishers:
            return

        payloads: Dict[str, Any] = {
            "situation_overview": feeling.situation_overview,
            "attitude_emoji": feeling.attitude_emoji,
            "thought_sentence": feeling.thought_sentence,
            "spoken_sentence": feeling.spoken_sentence,
            "command_script": feeling.command_script,
            "goals": feeling.goals,
            "mood_delta": feeling.mood_delta,
            "memory_collection_raw": feeling.memory_collection_raw,
            "memory_collection_text": feeling.memory_collection_text,
            "memory_collection_emoji": feeling.memory_collection_emoji,
            "episode_id": feeling.episode_id,
            "situation_id": feeling.situation_id,
        }

        for field, value in payloads.items():
            publisher = self._feedback_publishers.get(field)
            if publisher is None:
                continue
            msg = StdString()
            msg.data = self._serialise_feedback_value(value)
            publisher.publish(msg)

    def _load_topic_configs(
        self,
        param_name: str,
        default: List[Dict[str, str]],
        callback,
        qos: QoSProfile,
    ) -> Dict[str, Any]:
        default_payload = json.dumps(default)
        raw_param = self.declare_parameter(param_name, default_payload).value
        entries = _parse_topic_parameter(
            raw_param,
            param_name=param_name,
            fallback=default,
            logger=self.get_logger(),
        )

        mapping: Dict[str, Any] = {}
        subscriptions: List[str] = []
        for entry in entries:
            topic = entry.get("topic")
            type_name = entry.get("type")
            if not topic or not type_name:
                continue
            try:
                msg_type = get_message(type_name)
            except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                self.get_logger().error(
                    f"Failed to import message type {type_name}: {exc}"
                )
                continue

            subscription = self.create_subscription(msg_type, topic, lambda msg, t=topic: callback(t, msg), qos)
            self._subscriptions.append(subscription)
            mapping[topic] = msg_type
            subscriptions.append(f"{topic} ({type_name})")
            template = entry.get("prompt_template")
            if isinstance(template, str) and template:
                self._topic_templates[topic] = template

        if subscriptions:
            self.get_logger().info(
                f"Subscribing to {param_name}: {', '.join(subscriptions)}"
            )
        else:
            self.get_logger().warning(
                f"No subscriptions configured for {param_name}"
            )
        return mapping

    def _seed_minimal_context(self, host_short: str) -> None:
        """Ensure the prompt loop has baseline context even before ROS traffic arrives."""

        if "/pilot/system" in self._topic_cache:
            return
        self._topic_cache["/pilot/system"] = {
            "data": {
                "status": "pilot_ready",
                "host_short": host_short,
                "note": "Awaiting upstream modules to publish topics.",
            },
            "timestamp": time.time(),
        }
        self._dirty = True

    def _handle_context_message(self, topic: str, msg: Any) -> None:
        try:
            data = message_to_ordereddict(msg)
        except Exception:
            entry = {"data": str(msg), "timestamp": time.time()}
            self._topic_cache[topic] = entry
            self._dirty = True
            return

        sanitised, prompt_image = summarise_image_message(topic, msg, data)
        sanitised = apply_topic_translation(
            topic,
            sanitised,
            self._topic_translators,
            logger=self.get_logger(),
        )
        entry = {"data": sanitised, "timestamp": time.time()}
        if prompt_image is not None:
            entry["image"] = prompt_image
        self._topic_cache[topic] = entry
        self._dirty = True

    def _handle_sensation_message(self, topic: str, msg: SensationStamped) -> None:
        if not isinstance(msg, SensationStamped):
            try:
                data = message_to_ordereddict(msg)
                kind = data.get("kind", "")
                collection_hint = data.get("collection_hint", "")
                json_payload = data.get("json_payload", "")
                vector = list(data.get("vector", []))
            except Exception:
                return
        else:
            kind = msg.kind
            collection_hint = msg.collection_hint
            json_payload = msg.json_payload
            vector = list(msg.vector)
        record = SensationRecord(
            topic=topic,
            kind=kind,
            collection_hint=collection_hint,
            json_payload=json_payload,
            vector=vector,
        )
        self._sensation_records.append((time.monotonic(), record))
        self._dirty = True

    def _handle_conversant_memory_event(self, msg: StdString) -> None:
        raw_text = str(getattr(msg, "data", "") or "")
        try:
            payload = json.loads(raw_text) if raw_text else {}
        except json.JSONDecodeError:
            payload = {"text": raw_text}
        if not isinstance(payload, dict):
            payload = {"value": payload}

        kind = str(payload.get("kind") or "conversation")
        collection_hint = str(payload.get("collection_hint") or kind)
        record = SensationRecord(
            topic=self._conversant_topic_name,
            kind=kind,
            collection_hint=collection_hint,
            json_payload=json.dumps(payload, separators=(",", ":")),
            vector=[],
        )
        self._sensation_records.append((time.monotonic(), record))
        self._dirty = True

        thread_id = str(payload.get("thread_id") or "").strip()
        if not thread_id:
            return

        history = self._conversation_threads.setdefault(thread_id, [])
        snapshot = {
            "role": payload.get("role"),
            "text": payload.get("text"),
            "intent": payload.get("intent"),
            "timestamp": payload.get("timestamp"),
            "emoji": payload.get("emoji"),
        }
        history.append(snapshot)
        if len(history) > 12:
            del history[:-12]

        if thread_id in self._conversation_thread_order:
            self._conversation_thread_order.remove(thread_id)
        self._conversation_thread_order.append(thread_id)
        while len(self._conversation_thread_order) > 12:
            evicted = self._conversation_thread_order.pop(0)
            if evicted != thread_id:
                self._conversation_threads.pop(evicted, None)

        self._topic_cache["/conversation/threads"] = {
            "data": {
                "threads": {tid: list(entries) for tid, entries in self._conversation_threads.items()},
            },
            "timestamp": time.time(),
        }
        self._dirty = True

    def _recent_sensations(self) -> List[SensationRecord]:
        cutoff = time.monotonic() - self._window_seconds
        recent: List[tuple[float, SensationRecord]] = []
        for timestamp, record in self._sensation_records:
            if timestamp >= cutoff:
                recent.append((timestamp, record))
        self._sensation_records = recent
        return [record for _, record in recent]

    def destroy_node(self) -> bool:
        try:
            self._script_executor.shutdown(wait=False)
        except Exception:
            pass
        return super().destroy_node()

    def _script_status_snapshot(self) -> List[Dict[str, Any]]:
        with self._script_lock:
            return [dict(entry) for entry in self._script_runs]

    def _record_script_state(self, script_id: str, **updates: Any) -> None:
        with self._script_lock:
            for entry in self._script_runs:
                if entry.get("id") == script_id:
                    entry.update(updates)
                    break
            else:
                entry = {"id": script_id}
                entry.update(updates)
                self._script_runs.append(entry)
            if len(self._script_runs) > 12:
                del self._script_runs[:-12]

    def _record_memory_batch(self, batch: "MemoryBatch", *, stamp) -> None:
        """Persist memory events and associations via the memory services."""

        if not batch.events or not self._memory_client.enabled:
            return
        memory_ids: Dict[str, str] = {}
        total_events = len(batch.events)
        for index, draft in enumerate(batch.events):
            metadata = dict(draft.metadata)
            metadata.setdefault("source", draft.source)
            metadata.setdefault("frame_id", draft.frame_id)
            metadata.setdefault("memory_tag", draft.tag)
            event_msg = self._memory_client.build_event_message(
                stamp=stamp,
                frame_id=draft.frame_id,
                kind=draft.kind,
                metadata=metadata,
                embedding=draft.embedding,
            )
            if event_msg is None:
                continue
            memory_id = self._memory_client.memorize(event_msg, flush=index == total_events - 1)
            if memory_id:
                memory_ids[draft.tag] = memory_id
        if not batch.associations or not memory_ids:
            return
        for association in batch.associations:
            source_id = memory_ids.get(association.source_tag)
            target_id = memory_ids.get(association.target_tag)
            if not source_id or not target_id:
                continue
            self._memory_client.associate(
                source_id=source_id,
                target_id=target_id,
                relation_type=association.relation_type,
                properties=association.properties,
            )

    def _dispatch_action(
        self, invocation: CommandInvocation, available_actions: List[str]
    ) -> tuple[bool, str]:
        module = invocation.module or "pilot"
        action = invocation.action
        fq = f"{module}.{action}"
        if fq not in available_actions:
            self.get_logger().warning(
                f"Command not available in action registry: {fq}"
            )
            return False, "not_available"

        cockpit = os.environ.get("COCKPIT_URL", "http://127.0.0.1:8088").rstrip("/")
        payload = {"arguments": invocation.arguments}
        try:
            schema = self._action_schemas.get(fq)
            if schema is not None:
                try:
                    from jsonschema import validate
                    from jsonschema.exceptions import ValidationError as _ValidationError

                    try:
                        validate(instance=invocation.arguments, schema=schema)
                    except _ValidationError as verr:
                        self.get_logger().warning(
                            f"Action {module}.{action} failed validation: {verr}"
                        )
                        return False, str(verr)
                except Exception:
                    self.get_logger().debug(
                        f"jsonschema not available or validation skipped for {fq}"
                    )

            body = json.dumps(payload).encode("utf-8")
            req = request.Request(
                cockpit + f"/api/actions/{module}/{action}",
                data=body,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with request.urlopen(req, timeout=3.0) as resp:
                resp_body = resp.read().decode("utf-8")
                self.get_logger().info(
                    f"Invoked {module}.{action} => {resp_body}"
                )
                return True, resp_body
        except error.HTTPError as exc:
            self.get_logger().warning(
                f"Action invocation failed ({module}.{action}): {exc}"
            )
            return False, str(exc)
        except Exception as exc:  # pragma: no cover - network errors
            self.get_logger().warning(
                f"Failed to invoke action {module}.{action}: {exc}"
            )
            return False, str(exc)

    def _schedule_script_execution(
        self,
        *,
        script: str,
        available_actions: List[str],
        source: str,
        context: Mapping[str, Any],
    ) -> None:
        if not script.strip():
            return
        script_id = f"script-{uuid4()}"
        now_iso = datetime.now(timezone.utc).isoformat()
        preview = script if len(script) <= 2048 else script[:2048] + "…"
        self._record_script_state(
            script_id,
            source=source,
            script=preview,
            status="queued",
            requested_at=now_iso,
            actions=[],
        )

        def _runner() -> None:
            self._record_script_state(
                script_id,
                status="running",
                started_at=datetime.now(timezone.utc).isoformat(),
            )
            actions_log: List[Dict[str, Any]] = []
            try:
                result = self._command_interpreter.execute(
                    script, available_actions, context=context
                )
            except CommandScriptError as exc:
                self.get_logger().warning(f"command_script invalid: {exc}")
                self._record_script_state(
                    script_id,
                    status="invalid",
                    error=str(exc),
                    finished_at=datetime.now(timezone.utc).isoformat(),
                )
                return
            except Exception as exc:  # pragma: no cover - defensive guard
                self.get_logger().error(
                    f"command_script execution failed: {exc}"
                )
                self._record_script_state(
                    script_id,
                    status="failed",
                    error=str(exc),
                    finished_at=datetime.now(timezone.utc).isoformat(),
                )
                return

            for invocation in result.invocations:
                success, response = self._dispatch_action(invocation, available_actions)
                actions_log.append(
                    {
                        "action": f"{invocation.module}.{invocation.action}",
                        "status": "ok" if success else "error",
                        "response": response,
                        "timestamp": datetime.now(timezone.utc).isoformat(),
                    }
                )
                self._record_script_state(script_id, actions=list(actions_log))

            final_status = (
                "completed"
                if all(entry["status"] == "ok" for entry in actions_log)
                else "completed_with_errors"
            )
            self._record_script_state(
                script_id,
                status=final_status,
                finished_at=datetime.now(timezone.utc).isoformat(),
                used_actions=result.used_actions,
                actions=list(actions_log),
            )

        self._script_executor.submit(_runner)

    def _discover_modules_root(self) -> Path:
        """Return the repository modules directory for cockpit fallbacks."""

        repo_dir = os.environ.get("REPO_DIR")
        if repo_dir:
            try:
                return Path(repo_dir) / "modules"
            except TypeError:
                pass
        return Path(__file__).resolve().parents[5] / "modules"

    def _emit_debug_snapshot(self) -> None:
        try:
            if not getattr(self, "_debug_publisher", None):
                return
            snapshot = {
                "status": "running",
                "heartbeat": datetime.now(timezone.utc).isoformat(),
                "config": {
                    "debounce_seconds": self._debounce_seconds,
                    "window_seconds": self._window_seconds,
                    "context_topics": list(self._topic_cache.keys()),
                    "sensation_topics": [r.topic for _, r in self._sensation_records] if self._sensation_records else [],
                },
                "recent_sensations": [
                    {
                        "topic": r.topic,
                        "kind": r.kind,
                        "collection_hint": r.collection_hint,
                        "json_payload": r.json_payload,
                        "vector_len": len(r.vector) if getattr(r, "vector", None) is not None else 0,
                    }
                    for r in self._recent_sensations()
                ],
                # lightweight logs: sample last few records from internal logger if available
                "logs": [],
                "errors": [],
                "last_llm": self._last_llm_response if getattr(self, "_last_llm_response", None) else None,
                "last_prompt": self._last_prompt if getattr(self, "_last_prompt", None) else None,
                "scripts": self._script_status_snapshot(),
            }
            payload = StdString()
            payload.data = json.dumps(snapshot)
            self._debug_publisher.publish(payload)
        except Exception as exc:  # pragma: no cover - defensive guard
            self.get_logger().warning(f"Failed to emit debug snapshot: {exc}")

    def _fetch_actions(
        self,
        *,
        status: Mapping[str, Any] | None,
    ) -> tuple[List[str], Dict[str, Dict[str, str]]]:
        cockpit_url = os.environ.get("COCKPIT_URL", "http://127.0.0.1:8088").rstrip("/")
        active_modules = self._active_modules_from_status(status)
        metadata: Dict[str, Dict[str, Any]] = {}

        try:
            req = request.Request(cockpit_url + "/api/actions")
            with request.urlopen(req, timeout=2.0) as resp:
                body = resp.read().decode("utf-8")
            data = json.loads(body) if body else {}
            modules = data.get("modules") if isinstance(data, Mapping) else {}
            if isinstance(modules, Mapping):
                for module_name, info in modules.items():
                    actions = info.get("actions") if isinstance(info, Mapping) else []
                    if not isinstance(actions, list):
                        continue
                    for entry in actions:
                        if not isinstance(entry, Mapping):
                            continue
                        name = str(entry.get("name") or "").strip()
                        if not name:
                            continue
                        fq_name = f"{module_name}.{name}"
                        params_raw = entry.get("parameters")
                        parameters = dict(params_raw) if isinstance(params_raw, Mapping) else {}
                        metadata[fq_name] = {
                            "module": module_name,
                            "name": name,
                            "signature": str(entry.get("signature") or "").strip(),
                            "description": str(entry.get("description") or "").strip(),
                            "parameters": parameters,
                            "streaming": bool(entry.get("streaming")),
                        }
            if metadata:
                return self._assemble_action_catalogue(metadata, active_modules)
            raise ValueError("no actions from cockpit API")
        except Exception as exc:
            if not self._actions_fallback_logged:
                self.get_logger().warning(
                    f"Falling back to local cockpit actions: {exc}"
                )
                self._actions_fallback_logged = True
            else:
                self.get_logger().debug(
                    f"Falling back to local cockpit actions: {exc}"
                )

        metadata, _schemas = load_local_cockpit_actions(
            self._modules_root,
            logger=self.get_logger(),
        )
        return self._assemble_action_catalogue(metadata, active_modules)

    def _active_modules_from_status(
        self,
        status: Mapping[str, Any] | None,
    ) -> set[str] | None:
        if not isinstance(status, Mapping):
            return None
        modules = status.get("modules")
        if not isinstance(modules, Mapping) or not modules:
            return None

        active: set[str] = set()
        for name, info in modules.items():
            if not isinstance(info, Mapping):
                continue
            if self._module_is_active(info):
                active.add(str(name))
        return active if active else set()

    @staticmethod
    def _module_is_active(info: Mapping[str, Any]) -> bool:
        if bool(info.get("active")):
            return True
        status_value = info.get("status")
        if isinstance(status_value, str) and status_value.strip().lower() in {"running", "active"}:
            return True

        systemd = info.get("systemd")
        if isinstance(systemd, Mapping):
            if bool(systemd.get("active")):
                return True
            active_state = systemd.get("active_state")
            if isinstance(active_state, str) and active_state.strip().lower() in {"active", "activating", "reloading"}:
                return True
            sub_state = systemd.get("sub_state")
            if isinstance(sub_state, str) and sub_state.strip().lower() in {"running", "listening"}:
                return True
        return False

    def _assemble_action_catalogue(
        self,
        metadata: Mapping[str, Mapping[str, Any]],
        active_modules: set[str] | None,
    ) -> tuple[List[str], Dict[str, Dict[str, str]]]:
        allowed: List[str] = []
        contract: Dict[str, OrderedDict[str, str]] = {}
        self._action_schemas = {}

        for fq_name, entry in metadata.items():
            module = str(entry.get("module") or fq_name.split(".", 1)[0])
            if active_modules is not None and module not in active_modules:
                continue

            name = str(entry.get("name") or "").strip()
            if not name or name in _IGNORED_ACTION_NAMES:
                continue

            params_raw = entry.get("parameters")
            params: Dict[str, Any] = dict(params_raw) if isinstance(params_raw, Mapping) else {}

            signature = str(entry.get("signature") or "").strip()
            if not signature:
                signature = render_action_signature(name, params)

            description = str(entry.get("description") or "").strip()

            allowed.append(fq_name)
            bucket = contract.setdefault(module, OrderedDict())
            bucket[signature] = description
            if params:
                self._action_schemas[fq_name] = params

        allowed.sort()
        ordered_contract = {
            module: dict(actions)
            for module, actions in sorted(contract.items())
        }
        return allowed, ordered_contract

    def _fetch_status(self) -> Dict[str, Any]:
        cockpit_url = os.environ.get("COCKPIT_URL", "http://127.0.0.1:8088").rstrip("/")
        try:
            req = request.Request(cockpit_url + "/api/modules")
            with request.urlopen(req, timeout=2.0) as resp:
                body = resp.read().decode("utf-8")
            data = json.loads(body) if body else {}
            if isinstance(data, Mapping):
                return normalise_cockpit_modules_payload(data)
        except Exception as exc:
            self.get_logger().warning(
                f"Failed to fetch cockpit status: {exc}"
            )
        return {}

    def _parse_command(self, command: str) -> tuple[str, str, dict] | None:
        """Deprecated helper retained for compatibility; not used with scripts."""
        raise RuntimeError("_parse_command is no longer supported")

    def _build_prompt_context(
        self,
        actions: List[str],
        status: Dict[str, Any],
        *,
        action_contract: Mapping[str, Mapping[str, str]],
    ) -> Optional[PilotPromptContext]:
        if not self._topic_cache and not self._sensation_records:
            return None
        now = time.time()
        snapshot_timestamp = datetime.now(timezone.utc)

        topics: Dict[str, Dict[str, Any]] = {}
        stale_topics: List[str] = []
        for topic, entry in list(self._topic_cache.items()):
            timestamp = float(entry.get("timestamp", now))
            age = max(0.0, now - timestamp)
            if age > self._window_seconds:
                stale_topics.append(topic)
                continue
            value = entry.get("data")
            captured_at = datetime.fromtimestamp(timestamp, tz=timezone.utc).isoformat()
            snapshot = {
                "value": value,
                "age_seconds": age,
                "captured_at": captured_at,
            }
            if isinstance(value, Mapping):
                summary = value.get("prompt_summary")
                if isinstance(summary, str) and summary.strip():
                    snapshot["prompt_summary"] = summary.strip()
            topics[topic] = snapshot

        if stale_topics:
            for topic in stale_topics:
                self._topic_cache.pop(topic, None)
            self._dirty = True

        if status and "/status" not in topics:
            topics["/status"] = {
                "value": status,
                "age_seconds": 0.0,
                "captured_at": snapshot_timestamp.isoformat(),
            }

        sensations = [record.to_summary() for record in self._recent_sensations()]
        script_snapshot = self._script_status_snapshot()
        if script_snapshot:
            topics["/pilot/scripts"] = {
                "value": {
                    "recent": script_snapshot,
                    "running": [
                        entry
                        for entry in script_snapshot
                        if entry.get("status") == "running"
                    ],
                },
                "age_seconds": 0.0,
                "captured_at": snapshot_timestamp.isoformat(),
            }

        vision_images: List[PromptImage] = []
        for topic in topics:
            cache_entry = self._topic_cache.get(topic)
            if not cache_entry:
                continue
            image = cache_entry.get("image")
            if isinstance(image, PromptImage):
                vision_images.append(image)

        templates = {
            topic: template
            for topic in topics
            if (template := self._topic_templates.get(topic))
        }
        return PilotPromptContext(
            topics=topics,
            topic_templates=templates,
            status=status or topics.get("/status", {}).get("value", {}),
            sensations=sensations,
            cockpit_actions=actions,
            action_contract={module: dict(entries) for module, entries in action_contract.items()},
            window_seconds=self._window_seconds,
            vision_images=vision_images,
            snapshot_timestamp=snapshot_timestamp,
            static_sections=self._static_prompt_sections,
        )

    def _on_timer(self) -> None:
        if not self._dirty and not self._sensation_records:
            return
        status = self._fetch_status()
        actions, action_contract = self._fetch_actions(status=status)
        context = self._build_prompt_context(actions, status, action_contract=action_contract)
        if not context:
            return

        prompt = build_prompt(context)
        # Preserve the full prompt so debug snapshots reflect exactly what the LLM received.
        self._last_prompt = prompt
        try:
            images_payload = [img.base64_data for img in context.vision_images] or None
            raw_response = self._llm_client.generate(prompt, images=images_payload)
            feeling_data = parse_feeling_intent_json(raw_response, actions)
        except (RuntimeError, FeelingIntentValidationError) as exc:
            self.get_logger().error(f"Failed to generate FeelingIntent: {exc}")
            return

        recent_records = self._recent_sensations()
        script_context = {
            "topics": context.topics,
            "status": context.status or {},
            "sensations": [s.prompt_payload() for s in context.sensations],
            "window_seconds": context.window_seconds,
        }

        msg = FeelingIntent()
        msg.stamp = self.get_clock().now().to_msg()
        source_topics = sorted(set(context.topics.keys()) | {record.topic for record in recent_records})
        msg.source_topics = source_topics
        msg.situation_overview = feeling_data.situation_overview
        msg.attitude_emoji = feeling_data.attitude_emoji
        msg.thought_sentence = feeling_data.thought_sentence
        msg.spoken_sentence = feeling_data.spoken_sentence
        msg.command_script = feeling_data.command_script
        msg.goals = feeling_data.goals
        msg.mood_delta = feeling_data.mood_delta
        msg.memory_collection_raw = feeling_data.memory_collection_raw
        msg.memory_collection_text = feeling_data.memory_collection_text
        msg.memory_collection_emoji = feeling_data.memory_collection_emoji
        msg.episode_id = feeling_data.episode_id
        msg.situation_id = feeling_data.situation_id

        self.publisher.publish(msg)
        self._dirty = False

        self._publish_feedback_topics(feeling_data)

        # Execute the generated script asynchronously so the LLM loop can continue.
        try:
            self._schedule_script_execution(
                script=feeling_data.command_script,
                available_actions=actions,
                source="pilot.llm",
                context=script_context,
            )
        except Exception as exc:  # pragma: no cover - execution failures should not crash
            self.get_logger().warning(
                f"Failed to schedule command script: {exc}"
            )

        timestamp = _ros_time_to_datetime(msg.stamp)
        feeling_id = f"pilot-{uuid4()}"
        batch = prepare_memory_batch(
            feeling=feeling_data,
            sensations=recent_records,
            source_topics=source_topics,
            timestamp=timestamp,
            feeling_id=feeling_id,
        )
        try:
            self._record_memory_batch(batch, stamp=msg.stamp)
        except Exception as exc:  # pragma: no cover - external service failure
            self.get_logger().warning(f"Failed to write to memory services: {exc}")


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = PilotNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    main()
