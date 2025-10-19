from __future__ import annotations

import json
import logging
import os
import socket
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence
from urllib import error, request
from uuid import uuid4

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

from psyched_msgs.msg import FeelingIntent, SensationStamped
from std_msgs.msg import String as StdString

from .command_script import (
    CommandInvocation,
    CommandScriptError,
    CommandScriptInterpreter,
)
from .memory_pipeline import prepare_memory_batch
from .models import SensationRecord
from .prompt_builder import PilotPromptContext, build_prompt
from .validators import FeelingIntentValidationError, parse_feeling_intent_json


_LOGGER = logging.getLogger("psyched.pilot.node")
if not _LOGGER.handlers:
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s [pilot.node] %(message)s"))
    _LOGGER.addHandler(handler)
_LOGGER.setLevel(logging.INFO)


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
        {"topic": f"/hosts/health/{host_short}", "type": "psyched_msgs/msg/HostHealth"},
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

    dedup: Dict[str, Dict[str, str]] = {}
    for entry in entries:
        if not isinstance(entry, Mapping):
            continue
        topic = str(entry.get("topic") or "").strip()
        type_name = str(entry.get("type") or "").strip()
        if not topic or not type_name:
            continue
        if keep_first and topic in dedup:
            continue
        dedup[topic] = {"topic": topic, "type": type_name}
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
                suggestions[key].append({"topic": expanded_topic, "type": type_name})
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

        self._command_interpreter = CommandScriptInterpreter()
        self._script_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="pilot-script")
        self._script_runs: List[Dict[str, Any]] = []
        self._script_lock = threading.Lock()
        self._action_schemas: Dict[str, Dict[str, Any]] = {}
        self._last_prompt: Optional[str] = None

        host_full, host_short = _resolve_host_names()
        suggestions = _discover_topic_suggestions(
            _suggestions_root(),
            host_full=host_full,
            host_short=host_short,
            logger=self.get_logger(),
        )
        context_defaults = _normalise_topic_entries(
            _default_context_topics(host_short) + suggestions["context_topics"],
            keep_first=True,
        )
        sensation_defaults = _normalise_topic_entries(
            _default_sensation_topics() + suggestions["sensation_topics"],
            keep_first=True,
        )

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
            "context_topics", context_defaults, self._handle_context_message, qos
        )
        self._sensation_topics = self._load_topic_configs(
            "sensation_topics", sensation_defaults, self._handle_sensation_message, qos
        )
        self._seed_minimal_context(host_short)

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

    def _run_psh(self, args: Sequence[str]) -> str:
        try:
            proc = subprocess.run(args, check=False, capture_output=True, text=True)
        except FileNotFoundError:
            self.get_logger().warning(
                f"psh command not available: {args[0]}"
            )
            return ""
        if proc.returncode != 0:
            self.get_logger().warning(
                f"psh command failed ({proc.returncode}): {proc.stderr.strip()}"
            )
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
                "last_prompt": self._last_prompt if getattr(self, "_last_prompt", None) else None,
                "scripts": self._script_status_snapshot(),
            }
            payload = StdString()
            payload.data = json.dumps(snapshot)
            self._debug_publisher.publish(payload)
        except Exception as exc:  # pragma: no cover - defensive guard
            self.get_logger().warning(f"Failed to emit debug snapshot: {exc}")

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
                self.get_logger().warning(
                    f"Failed to fetch cockpit actions: {exc}"
                )
        return []

    def _fetch_status(self) -> Dict[str, Any]:
        try:
            result = self._run_psh(["psh", "cockpit", "status", "--json"])
            data = json.loads(result) if result else {}
            return data if isinstance(data, dict) else {}
        except Exception as exc:  # pragma: no cover - external command failure
            self.get_logger().warning(
                f"Failed to fetch cockpit status: {exc}"
            )
            return {}

    def _parse_command(self, command: str) -> tuple[str, str, dict] | None:
        """Deprecated helper retained for compatibility; not used with scripts."""
        raise RuntimeError("_parse_command is no longer supported")

    def _build_prompt_context(self, actions: List[str], status: Dict[str, Any]) -> Optional[PilotPromptContext]:
        if not self._topic_cache and not self._sensation_records:
            return None
        topics = {topic: entry["data"] for topic, entry in self._topic_cache.items()}
        if status and "/status" not in topics:
            topics["/status"] = status
        sensations = [record.to_summary() for record in self._recent_sensations()]
        script_snapshot = self._script_status_snapshot()
        if script_snapshot:
            topics["/pilot/scripts"] = {
                "recent": script_snapshot,
                "running": [entry for entry in script_snapshot if entry.get("status") == "running"],
            }
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
            self._last_prompt = prompt[:4096]
        except Exception:
            self._last_prompt = prompt
        try:
            raw_response = self._llm_client.generate(prompt)
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
            self._rememberd.memorize(
                {
                    "graph_mutations": batch.graph_mutations,
                    "vectors": batch.vectors,
                }
            )
        except Exception as exc:  # pragma: no cover - external service failure
            self.get_logger().warning(f"Failed to write to rememberd: {exc}")


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
