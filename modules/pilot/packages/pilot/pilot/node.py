from __future__ import annotations

import json
import logging
import os
import subprocess
import time
from datetime import datetime, timezone
from typing import Any, Dict, List, Mapping, Optional, Sequence
from urllib import request, error
from uuid import uuid4

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

from psyched_msgs.msg import FeelingIntent, SensationStamped
from std_msgs.msg import String as StdString

from .memory_pipeline import prepare_memory_batch
from .models import SensationRecord
from .prompt_builder import PilotPromptContext, build_prompt
from .validators import FeelingIntentValidationError, parse_feeling_intent_json
import re


_LOGGER = logging.getLogger("psyched.pilot.node")
if not _LOGGER.handlers:
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s [pilot.node] %(message)s"))
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
        self._last_llm_response: Optional[str] = None

    def generate(self, prompt: str) -> str:
        """Call Ollama and stream output to stdout while assembling the full response.

        This attempts to request a streaming response from Ollama (stream: true).
        For each chunk received, we write to stdout and flush so the caller can
        observe the raw model stream. We also assemble the chunks and return
        the final response string.
        """
        payload = json.dumps({"model": self.model, "prompt": prompt, "stream": True}).encode("utf-8")
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


class PilotNode(Node):
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
        super().__init__("pilot")

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

        self.publisher = self.create_publisher(FeelingIntent, "pilot/intent", qos)

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
            }
            payload = StdString()
            payload.data = json.dumps(snapshot)
            self._debug_publisher.publish(payload)
        except Exception as exc:  # pragma: no cover - defensive guard
            self.get_logger().warning("Failed to emit debug snapshot: %s", exc)

    def _fetch_actions(self) -> List[str]:
        # Prefer fetching actions from cockpit HTTP API if available via env var
        cockpit_url = os.environ.get("COCKPIT_URL", "http://127.0.0.1:8088").rstrip("/")
        try:
            # Try HTTP first
            req = request.Request(cockpit_url + "/api/actions")
            with request.urlopen(req, timeout=2.0) as resp:
                body = resp.read().decode("utf-8")
            data = json.loads(body) if body else {}
            modules = data.get("modules") if isinstance(data, dict) else {}
            actions: List[str] = []
            # Also capture schemas mapping for potential validation
            self._action_schemas: Dict[str, Dict[str, Any]] = {}
            if isinstance(modules, dict):
                for module_name, info in modules.items():
                    for act in info.get("actions", []) or []:
                        if not isinstance(act, dict):
                            continue
                        name = act.get("name")
                        if isinstance(name, str):
                            actions.append(f"{module_name}.{name}")
                            # Store the action's parameters schema if present
                            params = act.get("parameters")
                            if isinstance(params, dict):
                                self._action_schemas[f"{module_name}.{name}"] = params
            # Fall back to psh export if cockpit didn't return a useful payload
            if not actions:
                raise ValueError("no actions from cockpit API")
            return actions
        except Exception:
            # Fall back to legacy `psh` export when cockpit is not reachable
            try:
                result = self._run_psh(["psh", "actions", "export", "--json"])
                data = json.loads(result) if result else {}
                if isinstance(data, dict):
                    modules_payload = data.get("modules")
                    if isinstance(modules_payload, Mapping):
                        actions: List[str] = []
                        self._action_schemas = {}
                        for module_name, info in modules_payload.items():
                            if not isinstance(info, Mapping):
                                continue
                            module_actions = info.get("actions")
                            if not isinstance(module_actions, list):
                                continue
                            for action_entry in module_actions:
                                if not isinstance(action_entry, Mapping):
                                    continue
                                action_name = action_entry.get("name")
                                if isinstance(action_name, str):
                                    actions.append(f"{module_name}.{action_name}")
                                    params = action_entry.get("parameters")
                                    if isinstance(params, Mapping):
                                        self._action_schemas[f"{module_name}.{action_name}"] = dict(params)
                        if actions:
                            return actions
                actions_raw = data.get("actions") if isinstance(data, dict) else data
                if isinstance(actions_raw, list):
                    return [str(action) for action in actions_raw if isinstance(action, str)]
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

        def _parse_command(self, command: str) -> tuple[str, str, dict] | None:
            """Parse a command string into (module, action, arguments).

            Accepts forms like:
            - "action"
            - "module.action"
            - "action(arg)" or "module.action(arg)"

            Returns None when the command cannot be sensibly parsed.
            """
            s = command.strip()
            if not s:
                return None

            # Extract any parentheses content as a single argument payload if present.
            m = re.match(r"^([A-Za-z0-9_.-]+)\s*(?:\((.*)\))?$", s)
            if not m:
                return None
            fullname = m.group(1)
            args_raw = m.group(2)

            if "." in fullname:
                module, action = fullname.rsplit(".", 1)
            else:
                # If no module is specified, default to 'pilot' as origin for intent
                module = "pilot"
                action = fullname

            # Try to parse args_raw as JSON, otherwise send as a single positional arg
            args: dict = {}
            if args_raw is not None and args_raw.strip():
                # Attempt JSON decode first
                try:
                    import json as _json

                    parsed = _json.loads(args_raw)
                    if isinstance(parsed, dict):
                        args = parsed
                    else:
                        # Wrap non-dict into {'_args': parsed}
                        args = {"_args": parsed}
                except Exception:
                    # Fallback: send the raw string as a single field
                    args = {"_raw": args_raw}

            return (module, action, args)

        def _invoke_commands(self, commands: list[str], available_actions: list[str]) -> None:
            """Best-effort invocation of each command via the cockpit action API.

            For each command string, parse into module/action/args and POST to
            /api/actions/{module}/{action} with JSON payload {arguments: args}.
            """
            if not commands:
                return

            cockpit = os.environ.get("COCKPIT_URL", "http://127.0.0.1:8088").rstrip("/")
            for cmd in commands:
                parsed = self._parse_command(cmd)
                if not parsed:
                    self.get_logger().warning("Skipping unrecognised command: %s", cmd)
                    continue
                module, action, args = parsed

                # Basic check: ensure module.action exists in available_actions
                fq = f"{module}.{action}"
                if fq not in available_actions and action not in [a.split(".", 1)[-1] for a in available_actions]:
                    self.get_logger().warning("Command not available in action registry: %s", fq)
                    continue

                payload = {"arguments": args}
                body = None
                try:
                    import json as _json

                    # Validate arguments against action schema if available
                    schema = getattr(self, "_action_schemas", {}).get(fq)
                    if schema is not None:
                        try:
                            # jsonschema is optional; if installed we validate
                            from jsonschema import validate
                            from jsonschema.exceptions import ValidationError as _ValidationError

                            try:
                                validate(instance=args, schema=schema)
                            except _ValidationError as verr:
                                self.get_logger().warning("Action %s.%s failed validation: %s", module, action, verr)
                                continue
                        except Exception:
                            # If jsonschema isn't available or something else goes wrong,
                            # we log and continue without blocking invocation.
                            self.get_logger().debug("jsonschema not available or validation failed to run; skipping strict validation for %s.%s", module, action)

                    body = _json.dumps(payload).encode("utf-8")
                    req = request.Request(
                        cockpit + f"/api/actions/{module}/{action}",
                        data=body,
                        headers={"Content-Type": "application/json"},
                        method="POST",
                    )
                    with request.urlopen(req, timeout=3.0) as resp:
                        resp_body = resp.read().decode("utf-8")
                        self.get_logger().info("Invoked %s.%s => %s", module, action, resp_body)
                except error.HTTPError as exc:
                    self.get_logger().warning("Action invocation failed (%s.%s): %s", module, action, exc)
                except Exception as exc:  # pragma: no cover - network errors
                    self.get_logger().warning("Failed to invoke action %s.%s: %s", module, action, exc)

    def _build_prompt_context(self, actions: List[str], status: Dict[str, Any]) -> Optional[PilotPromptContext]:
        if not self._topic_cache and not self._sensation_records:
            return None
        topics = {topic: entry["data"] for topic, entry in self._topic_cache.items()}
        if status and "/status" not in topics:
            topics["/status"] = status
        sensations = [record.to_summary() for record in self._recent_sensations()]
        return PilotPromptContext(
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

        # Attempt to invoke any commands proposed by the LLM. Invocation is
        # best-effort: we prefer the cockpit HTTP API (COCKPIT_URL) and will
        # log failures rather than preventing the feeling intent from being
        # published.
        try:
            self._invoke_commands(feeling_data.commands, actions)
        except Exception as exc:  # pragma: no cover - invocation failures should not crash
            self.get_logger().warning("Failed to invoke commands: %s", exc)

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
    node = PilotNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    main()
