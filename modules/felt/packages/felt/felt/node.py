from __future__ import annotations

import json
import logging
import os
import subprocess
import time
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Sequence
from urllib import request, error
from uuid import uuid4

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

from psyched_msgs.msg import FeelingIntent, SensationStamped

from .memory_pipeline import prepare_memory_batch
from .models import SensationRecord
from .prompt_builder import FeltPromptContext, build_prompt
from .validators import FeelingIntentValidationError, parse_feeling_intent_json


_LOGGER = logging.getLogger("psyched.felt.node")
if not _LOGGER.handlers:
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s [felt.node] %(message)s"))
    _LOGGER.addHandler(handler)
_LOGGER.setLevel(logging.INFO)


class LLMClient:
    """Abstract interface so tests can stub LLM calls."""

    def generate(self, prompt: str) -> str:  # pragma: no cover - interface
        raise NotImplementedError


class OllamaLLMClient(LLMClient):
    """HTTP client for Ollama's `/api/generate` endpoint."""

    def __init__(self, model: str, host: Optional[str] = None, timeout: float = 30.0) -> None:
        self.model = model
        self.host = (host or os.environ.get("OLLAMA_HOST") or "http://127.0.0.1:11434").rstrip("/")
        self.timeout = timeout

    def generate(self, prompt: str) -> str:
        payload = json.dumps({"model": self.model, "prompt": prompt, "stream": False}).encode("utf-8")
        req = request.Request(
            self.host + "/api/generate",
            data=payload,
            headers={"Content-Type": "application/json"},
        )
        try:
            with request.urlopen(req, timeout=self.timeout) as resp:
                body = resp.read().decode("utf-8")
        except Exception as exc:  # pragma: no cover - network failure path
            raise RuntimeError(f"Failed to reach Ollama at {self.host}: {exc}") from exc

        data = json.loads(body)
        if isinstance(data, dict) and "response" in data:
            return str(data["response"])
        raise RuntimeError(f"Unexpected Ollama response: {data!r}")


class RememberdClient:
    """Minimal JSON-RPC client for rememberd."""

    def __init__(self, endpoint: Optional[str] = None, timeout: float = 5.0) -> None:
        self.endpoint = (endpoint or os.environ.get("REMEMBERD_URL", "")).strip()
        self.timeout = timeout

    def memorize(self, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        if not self.endpoint:
            _LOGGER.debug("rememberd endpoint not configured; skipping memory write")
            return None
        body = json.dumps(payload).encode("utf-8")
        req = request.Request(
            self.endpoint,
            data=body,
            headers={"Content-Type": "application/json"},
        )
        try:
            with request.urlopen(req, timeout=self.timeout) as resp:
                content_type = resp.headers.get("Content-Type", "")
                if "application/json" in content_type:
                    return json.loads(resp.read().decode("utf-8"))
                return None
        except error.URLError as exc:  # pragma: no cover - network failure path
            _LOGGER.error("rememberd request failed: %s", exc)
            return None


def _ros_time_to_datetime(time_msg) -> datetime:
    sec = getattr(time_msg, "sec", 0)
    nanosec = getattr(time_msg, "nanosec", 0)
    return datetime.fromtimestamp(float(sec) + float(nanosec) / 1e9, tz=timezone.utc)


class FeltNode(Node):
    """ROS node orchestrating the feeling + will integration pipeline."""

    _DEFAULT_CONTEXT_TOPICS = [
        {"topic": "/instant", "type": "std_msgs/msg/String"},
        {"topic": "/situation", "type": "std_msgs/msg/String"},
        {"topic": "/status", "type": "std_msgs/msg/String"},
    ]
    _DEFAULT_SENSATION_TOPICS = [
        {"topic": "/sensations", "type": "psyched_msgs/msg/SensationStamped"}
    ]

    def __init__(self, *, llm_client: Optional[LLMClient] = None, rememberd: Optional[RememberdClient] = None) -> None:
        super().__init__("felt")

        self._llm_client = llm_client or OllamaLLMClient(model=os.environ.get("FELT_MODEL", "gpt-oss"))
        self._rememberd = rememberd or RememberdClient()

        self._debounce_seconds = self._declare_float_parameter("debounce_seconds", 3.0)
        self._window_seconds = self._declare_float_parameter("window_seconds", 3.0)

        self._topic_cache: Dict[str, Dict[str, Any]] = {}
        self._sensation_records: List[tuple[float, SensationRecord]] = []
        self._subscriptions: List[Any] = []
        self._dirty = False

        qos = QoSProfile(depth=10)
        qos.history = QoSHistoryPolicy.KEEP_LAST
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.publisher = self.create_publisher(FeelingIntent, "felt/intent", qos)

        self._context_topics = self._load_topic_configs(
            "context_topics", self._DEFAULT_CONTEXT_TOPICS, self._handle_context_message, qos
        )
        self._sensation_topics = self._load_topic_configs(
            "sensation_topics", self._DEFAULT_SENSATION_TOPICS, self._handle_sensation_message, qos
        )

        self._timer = self.create_timer(self._debounce_seconds, self._on_timer)

    def _declare_float_parameter(self, name: str, default: float) -> float:
        param = self.declare_parameter(name, default)
        value = getattr(param, "value", default)
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    def _load_topic_configs(
        self,
        param_name: str,
        default: List[Dict[str, Any]],
        callback,
        qos: QoSProfile,
    ) -> Dict[str, Any]:
        raw_param = self.declare_parameter(param_name, json.dumps(default)).value
        if isinstance(raw_param, str):
            try:
                configs = json.loads(raw_param)
            except json.JSONDecodeError:
                self.get_logger().warning("Invalid JSON for %s, using defaults", param_name)
                configs = default
        elif isinstance(raw_param, list):
            configs = raw_param
        else:
            configs = default

        mapping: Dict[str, Any] = {}
        for entry in configs:
            if not isinstance(entry, dict):
                continue
            topic = entry.get("topic")
            type_name = entry.get("type")
            if not topic or not type_name:
                continue
            try:
                msg_type = get_message(type_name)
            except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                self.get_logger().error("Failed to import message type %s: %s", type_name, exc)
                continue

            subscription = self.create_subscription(msg_type, topic, lambda msg, t=topic: callback(t, msg), qos)
            self._subscriptions.append(subscription)
            mapping[topic] = msg_type
        return mapping

    def _handle_context_message(self, topic: str, msg: Any) -> None:
        try:
            data = message_to_ordereddict(msg)
        except Exception:
            data = str(msg)
        self._topic_cache[topic] = {"data": data, "timestamp": time.time()}
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

    def _recent_sensations(self) -> List[SensationRecord]:
        cutoff = time.monotonic() - self._window_seconds
        recent: List[tuple[float, SensationRecord]] = []
        for timestamp, record in self._sensation_records:
            if timestamp >= cutoff:
                recent.append((timestamp, record))
        self._sensation_records = recent
        return [record for _, record in recent]

    def _run_psh(self, args: Sequence[str]) -> str:
        try:
            proc = subprocess.run(args, check=False, capture_output=True, text=True)
        except FileNotFoundError:
            self.get_logger().warning("psh command not available: %s", args[0])
            return ""
        if proc.returncode != 0:
            self.get_logger().warning("psh command failed (%s): %s", proc.returncode, proc.stderr.strip())
            return ""
        return proc.stdout

    def _fetch_actions(self) -> List[str]:
        try:
            result = self._run_psh(["psh", "actions", "export", "--json"])
            data = json.loads(result) if result else {}
            actions = data.get("actions") if isinstance(data, dict) else data
            if isinstance(actions, list):
                return [str(action) for action in actions if isinstance(action, str)]
        except Exception as exc:  # pragma: no cover - external command failure
            self.get_logger().warning("Failed to fetch cockpit actions: %s", exc)
        return []

    def _fetch_status(self) -> Dict[str, Any]:
        try:
            result = self._run_psh(["psh", "cockpit", "status", "--json"])
            data = json.loads(result) if result else {}
            return data if isinstance(data, dict) else {}
        except Exception as exc:  # pragma: no cover - external command failure
            self.get_logger().warning("Failed to fetch cockpit status: %s", exc)
            return {}

    def _build_prompt_context(self, actions: List[str], status: Dict[str, Any]) -> Optional[FeltPromptContext]:
        if not self._topic_cache and not self._sensation_records:
            return None
        topics = {topic: entry["data"] for topic, entry in self._topic_cache.items()}
        if status and "/status" not in topics:
            topics["/status"] = status
        sensations = [record.to_summary() for record in self._recent_sensations()]
        return FeltPromptContext(
            topics=topics,
            status=status or topics.get("/status", {}),
            sensations=sensations,
            cockpit_actions=actions,
            window_seconds=self._window_seconds,
        )

    def _on_timer(self) -> None:
        if not self._dirty and not self._sensation_records:
            return
        actions = self._fetch_actions()
        status = self._fetch_status()
        context = self._build_prompt_context(actions, status)
        if not context:
            return

        prompt = build_prompt(context)
        try:
            raw_response = self._llm_client.generate(prompt)
            feeling_data = parse_feeling_intent_json(raw_response, actions)
        except (RuntimeError, FeelingIntentValidationError) as exc:
            self.get_logger().error("Failed to generate FeelingIntent: %s", exc)
            return

        recent_records = self._recent_sensations()

        msg = FeelingIntent()
        msg.stamp = self.get_clock().now().to_msg()
        source_topics = sorted(set(context.topics.keys()) | {record.topic for record in recent_records})
        msg.source_topics = source_topics
        msg.attitude_emoji = feeling_data.attitude_emoji
        msg.thought_sentence = feeling_data.thought_sentence
        msg.spoken_sentence = feeling_data.spoken_sentence
        msg.commands = feeling_data.commands
        msg.goals = feeling_data.goals
        msg.mood_delta = feeling_data.mood_delta
        msg.memory_collection_raw = feeling_data.memory_collection_raw
        msg.memory_collection_text = feeling_data.memory_collection_text
        msg.memory_collection_emoji = feeling_data.memory_collection_emoji
        msg.episode_id = feeling_data.episode_id
        msg.situation_id = feeling_data.situation_id

        self.publisher.publish(msg)
        self._dirty = False

        timestamp = _ros_time_to_datetime(msg.stamp)
        feeling_id = f"felt-{uuid4()}"
        batch = prepare_memory_batch(
            feeling=feeling_data,
            sensations=recent_records,
            source_topics=source_topics,
            timestamp=timestamp,
            feeling_id=feeling_id,
        )
        try:
            self._rememberd.memorize(
                {
                    "graph_mutations": batch.graph_mutations,
                    "vectors": batch.vectors,
                }
            )
        except Exception as exc:  # pragma: no cover - external service failure
            self.get_logger().warning("Failed to write to rememberd: %s", exc)


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = FeltNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    main()
