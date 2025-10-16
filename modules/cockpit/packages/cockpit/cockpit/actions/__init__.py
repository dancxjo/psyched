"""Action registry and execution primitives for cockpit modules.

The cockpit exposes module functionality to both the browser frontend and LLM
operators through a catalogue of actions. Each action is described using a
schema compatible with common LLM function-calling conventions and executed on
the backend so modules can route all ROS 2 interactions through the cockpit
server.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Dict, Iterable, Mapping, Optional

from aiohttp import web

ActionHandler = Callable[["ActionContext"], Awaitable["ActionResult"]]


class ActionError(RuntimeError):
    """Raised when an action fails to execute."""


@dataclass(slots=True)
class ActionContext:
    """Execution context passed to action handlers."""

    module: str
    arguments: Mapping[str, Any]
    app: web.Application
    request: Optional[web.Request] = None

    def get_argument(self, key: str, default: Any = None) -> Any:
        """Return an action argument with an optional default."""

        return self.arguments.get(key, default)


@dataclass(slots=True)
class ActionResult:
    """Outcome produced by an action handler."""

    payload: Optional[Mapping[str, Any]] = None
    stream: Optional[Any] = None

    @property
    def streaming(self) -> bool:
        """Return ``True`` when the action yielded a streaming result."""

        return self.stream is not None


@dataclass(slots=True)
class ModuleAction:
    """Describe and execute an action exposed by a module."""

    name: str
    description: str
    parameters: Mapping[str, Any]
    handler: ActionHandler
    returns: Optional[Mapping[str, Any]] = None
    streaming: bool = False

    def to_dict(self) -> Dict[str, Any]:
        """Serialise the action metadata for API consumers."""

        payload: Dict[str, Any] = {
            "name": self.name,
            "description": self.description,
            "parameters": dict(self.parameters),
            "streaming": self.streaming,
        }
        if self.returns is not None:
            payload["returns"] = dict(self.returns)
        return payload


class ActionRegistry:
    """Registry storing actions exposed by cockpit modules."""

    def __init__(self) -> None:
        self._actions: Dict[str, Dict[str, ModuleAction]] = {}

    def register(self, module: str, action: ModuleAction) -> None:
        """Register *action* for *module*."""

        if not module or not isinstance(module, str):
            raise ValueError("module name must be a non-empty string")
        module_actions = self._actions.setdefault(module, {})
        normalized_name = action.name.strip()
        module_actions[normalized_name] = action

    def register_many(self, module: str, actions: Iterable[ModuleAction]) -> None:
        """Register multiple actions for a module."""

        for action in actions:
            self.register(module, action)

    def get(self, module: str, name: str) -> ModuleAction:
        """Return the registered action for *module*/*name*."""

        module_actions = self._actions.get(module)
        if module_actions is None:
            raise KeyError(f"No actions registered for module {module!r}")
        try:
            return module_actions[name]
        except KeyError as exc:
            raise KeyError(f"Action {name!r} not found for module {module!r}") from exc

    def list_modules(self) -> Iterable[str]:
        """Yield the module identifiers with registered actions."""

        return list(self._actions.keys())

    def list_actions(self, module: str) -> Iterable[ModuleAction]:
        """Return the actions registered for *module*."""

        module_actions = self._actions.get(module, {})
        return list(module_actions.values())

    async def execute(
        self,
        module: str,
        name: str,
        *,
        app: web.Application,
        arguments: Mapping[str, Any],
        request: Optional[web.Request] = None,
    ) -> ActionResult:
        """Execute the requested action and return its result."""

        action = self.get(module, name)
        context = ActionContext(module=module, arguments=arguments, app=app, request=request)
        try:
            result = await action.handler(context)
        except ActionError:
            raise
        except Exception as exc:  # pragma: no cover - defensive guard for unexpected errors
            raise ActionError(f"Action {module}.{name} failed: {exc}") from exc

        if action.streaming and not result.streaming:
            raise ActionError(f"Action {module}.{name} did not create a stream as expected")
        if not action.streaming and result.streaming:
            raise ActionError(f"Action {module}.{name} unexpectedly returned a stream")
        return result

    def to_payload(self) -> Dict[str, Any]:
        """Return a JSON-serialisable snapshot of all registered actions."""

        modules: Dict[str, Any] = {}
        for module, actions in self._actions.items():
            modules[module] = {
                "actions": [action.to_dict() for action in actions.values()],
            }
        return {"modules": modules}
