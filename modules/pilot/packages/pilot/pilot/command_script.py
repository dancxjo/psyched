from __future__ import annotations

import ast
import re
import textwrap
from collections import deque
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Dict, List, Mapping, Sequence, Set, Tuple


class CommandScriptError(ValueError):
    """Raised when a command script cannot be analysed or executed safely."""


@dataclass(slots=True, eq=True)
class CommandInvocation:
    """Representation of a single cockpit action invocation."""

    module: str
    action: str
    arguments: Dict[str, Any]


@dataclass(slots=True)
class CommandScriptResult:
    """Output of executing a command script in dry-run mode."""

    invocations: List[CommandInvocation]
    used_actions: List[str]


@dataclass(slots=True)
class CommandScriptAnalysis:
    """Static analysis summary for a command script."""

    script: str
    tree: ast.Module
    registry: "_ActionRegistry"
    used_actions: Set[str]


@dataclass(slots=True)
class _ActionSpec:
    module: str
    action: str
    path: Tuple[str, ...]
    fq_name: str


_ACTION_NAME_PATTERN = re.compile(r"^([A-Za-z0-9_.-]+)")
_DISALLOWED_NODES = (
    ast.Import,
    ast.ImportFrom,
    ast.With,
    ast.AsyncFunctionDef,
    ast.Lambda,
    ast.Try,
    ast.Raise,
    ast.Global,
    ast.Nonlocal,
    ast.ClassDef,
    ast.Delete,
    ast.Await,
    ast.Yield,
    ast.YieldFrom,
)
_SAFE_CALL_NAMES = {
    "action",
    "available_actions",
    "any",
    "all",
    "bool",
    "dict",
    "enumerate",
    "float",
    "int",
    "len",
    "list",
    "max",
    "min",
    "range",
    "round",
    "set",
    "sorted",
    "str",
    "sum",
    "tuple",
    "zip",
}
_DEFAULT_SAFE_BUILTINS = MappingProxyType(
    {
        "any": any,
        "all": all,
        "bool": bool,
        "dict": dict,
        "enumerate": enumerate,
        "float": float,
        "int": int,
        "len": len,
        "list": list,
        "max": max,
        "min": min,
        "range": range,
        "round": round,
        "set": set,
        "sorted": sorted,
        "str": str,
        "sum": sum,
        "tuple": tuple,
        "zip": zip,
    }
)


def _clean_action_name(raw: str) -> str | None:
    if not isinstance(raw, str):
        return None
    candidate = raw.strip()
    if not candidate:
        return None
    match = _ACTION_NAME_PATTERN.match(candidate)
    if not match:
        return None
    return match.group(1)


class _ActionRegistry:
    """Normalised view of cockpit actions exposed to the interpreter."""

    def __init__(self, allowed_actions: Sequence[str]) -> None:
        self._spec_by_path: Dict[Tuple[str, ...], _ActionSpec] = {}
        self._valid_names: Set[str] = set()
        self._roots: Set[str] = set()
        self._singletons: Set[str] = set()

        for raw in allowed_actions:
            name = _clean_action_name(raw)
            if not name:
                continue
            parts = tuple(segment for segment in name.split(".") if segment)
            if not parts:
                continue
            module = ".".join(parts[:-1]) if len(parts) > 1 else "pilot"
            spec = _ActionSpec(module=module, action=parts[-1], path=parts, fq_name=name)
            self._spec_by_path[parts] = spec
            self._valid_names.add(name)
            self._roots.add(parts[0])
            if len(parts) == 1:
                self._singletons.add(parts[0])
                # Allow explicit pilot namespace for singletons.
                pilot_path = ("pilot", parts[0])
                self._spec_by_path.setdefault(pilot_path, spec)

    @property
    def roots(self) -> Set[str]:
        return set(self._roots)

    @property
    def singletons(self) -> Set[str]:
        return set(self._singletons)

    def list_actions(self) -> List[str]:
        return sorted(self._valid_names)

    def resolve(self, path: Tuple[str, ...]) -> _ActionSpec | None:
        if path in self._spec_by_path:
            return self._spec_by_path[path]
        joined = ".".join(path)
        if joined in self._valid_names:
            return self._spec_by_path.get(tuple(joined.split(".")))
        return None

    def parse_name(self, name: str) -> Tuple[str, ...]:
        cleaned = _clean_action_name(name)
        if not cleaned:
            raise CommandScriptError(f"Unknown action: {name!r}")
        parts = tuple(segment for segment in cleaned.split(".") if segment)
        if not parts:
            raise CommandScriptError(f"Unknown action: {name!r}")
        return parts


class _ActionProxy:
    def __init__(self, collector: "_ActionCollector", path: Tuple[str, ...]) -> None:
        self._collector = collector
        self._path = path

    def __getattr__(self, item: str) -> "_ActionProxy":
        return _ActionProxy(self._collector, self._path + (item,))

    def __call__(self, *args: Any, **kwargs: Any) -> None:
        self._collector.invoke(self._path, args, kwargs)


class _ActionCollector:
    def __init__(self, registry: _ActionRegistry, max_invocations: int) -> None:
        self._registry = registry
        self._max_invocations = max_invocations
        self._invocations: List[CommandInvocation] = []

    @property
    def invocations(self) -> List[CommandInvocation]:
        return list(self._invocations)

    @property
    def registry(self) -> _ActionRegistry:
        return self._registry

    def namespace(self) -> Dict[str, Any]:
        namespace: Dict[str, Any] = {
            "action": self.call_by_name,
            "available_actions": self._registry.list_actions,
        }
        for root in self._registry.roots:
            namespace[root] = _ActionProxy(self, (root,))
        for singleton in self._registry.singletons:
            namespace.setdefault(singleton, _ActionProxy(self, (singleton,)))
        return namespace

    def call_by_name(self, name: str, *args: Any, **kwargs: Any) -> None:
        path = self._registry.parse_name(name)
        self.invoke(path, args, kwargs)

    def invoke(self, path: Tuple[str, ...], args: Tuple[Any, ...], kwargs: Dict[str, Any]) -> None:
        spec = self._registry.resolve(path)
        if spec is None:
            raise CommandScriptError(f"Unknown action: {'.'.join(path)}")
        if len(self._invocations) >= self._max_invocations:
            raise CommandScriptError(
                f"command_script exceeds {self._max_invocations} action invocations"
            )
        arguments = _package_arguments(args, kwargs)
        self._invocations.append(
            CommandInvocation(module=spec.module, action=spec.action, arguments=arguments)
        )


def _package_arguments(args: Tuple[Any, ...], kwargs: Dict[str, Any]) -> Dict[str, Any]:
    if args and kwargs:
        payload = {"_args": list(args)}
        payload.update(kwargs)
        return payload
    if args:
        return {"_args": list(args)}
    if kwargs:
        return dict(kwargs)
    return {}


class _CommandScriptAnalyzer(ast.NodeVisitor):
    def __init__(self, registry: _ActionRegistry) -> None:
        self._registry = registry
        self.used_actions: Set[str] = set()
        self._defined_functions: Set[str] = set()
        self._known_names: Set[str] = {"context"}

    def visit(self, node: ast.AST) -> Any:
        if isinstance(node, _DISALLOWED_NODES):
            raise CommandScriptError(f"Unsupported syntax: {node.__class__.__name__}")
        return super().visit(node)

    def visit_FunctionDef(self, node: ast.FunctionDef) -> Any:
        self._defined_functions.add(node.name)
        self._known_names.add(node.name)
        return self.generic_visit(node)

    def visit_Assign(self, node: ast.Assign) -> Any:
        for target in node.targets:
            self._register_target(target)
        return self.generic_visit(node)

    def visit_AnnAssign(self, node: ast.AnnAssign) -> Any:
        self._register_target(node.target)
        return self.generic_visit(node)

    def visit_AugAssign(self, node: ast.AugAssign) -> Any:
        self._register_target(node.target)
        return self.generic_visit(node)

    def visit_For(self, node: ast.For) -> Any:
        self._register_target(node.target)
        return self.generic_visit(node)

    def visit_ListComp(self, node: ast.ListComp) -> Any:
        for generator in node.generators:
            self._register_target(generator.target)
        return self.generic_visit(node)

    def visit_SetComp(self, node: ast.SetComp) -> Any:
        for generator in node.generators:
            self._register_target(generator.target)
        return self.generic_visit(node)

    def visit_DictComp(self, node: ast.DictComp) -> Any:
        for generator in node.generators:
            self._register_target(generator.target)
        return self.generic_visit(node)

    def visit_GeneratorExp(self, node: ast.GeneratorExp) -> Any:
        for generator in node.generators:
            self._register_target(generator.target)
        return self.generic_visit(node)

    def visit_Call(self, node: ast.Call) -> Any:
        path = _attribute_path(node.func)
        if path and (len(path) > 1 or path[0] in self._registry.roots):
            root = path[0]
            if root in self._registry.roots:
                spec = self._registry.resolve(path)
                if spec is None:
                    raise CommandScriptError(f"Unknown action '{'.'.join(path)}'")
                self.used_actions.add(spec.fq_name)
            elif root in self._known_names:
                pass
            else:
                dotted = ".".join(path)
                raise CommandScriptError(f"Unknown command namespace '{root}' in call to {dotted}")
        else:
            name = path[0] if path else _callable_name(node.func)
            if name and name not in self._defined_functions:
                if name in self._registry.singletons:
                    spec = self._registry.resolve((name,))
                    if spec is None:
                        raise CommandScriptError(f"Unknown action '{name}'")
                    self.used_actions.add(spec.fq_name)
                elif name in _SAFE_CALL_NAMES or name in self._known_names:
                    pass
                else:
                    raise CommandScriptError(f"Call to unknown helper '{name}' is not allowed")
        return self.generic_visit(node)

    def _register_target(self, target: ast.AST) -> None:
        if isinstance(target, ast.Name):
            self._known_names.add(target.id)
        elif isinstance(target, (ast.Tuple, ast.List)):
            for element in target.elts:
                self._register_target(element)


def _attribute_path(node: ast.AST) -> Tuple[str, ...] | None:
    parts: deque[str] = deque()
    current = node
    while isinstance(current, ast.Attribute):
        parts.appendleft(current.attr)
        current = current.value
    if isinstance(current, ast.Name):
        parts.appendleft(current.id)
        return tuple(parts)
    return None


def _callable_name(node: ast.AST) -> str | None:
    if isinstance(node, ast.Name):
        return node.id
    return None


def _make_context_proxy(context: Mapping[str, Any] | None) -> Mapping[str, Any]:
    if context is None:
        return MappingProxyType({})
    if isinstance(context, Mapping):
        return MappingProxyType(dict(context))
    raise CommandScriptError("context must be a mapping")


class CommandScriptInterpreter:
    """Validate and execute feeling intent command scripts."""

    def __init__(self, *, max_invocations: int = 64) -> None:
        if max_invocations <= 0:
            raise ValueError("max_invocations must be positive")
        self._max_invocations = int(max_invocations)
        self._builtins = dict(_DEFAULT_SAFE_BUILTINS)

    def analyse(self, script: str, allowed_actions: Sequence[str]) -> CommandScriptAnalysis:
        if not isinstance(script, str):
            raise CommandScriptError("command_script must be a string")
        normalised = textwrap.dedent(script).strip()
        if not normalised:
            raise CommandScriptError("command_script must not be empty")
        try:
            tree = ast.parse(normalised, mode="exec")
        except SyntaxError as exc:  # pragma: no cover - syntax errors surfaced in tests
            raise CommandScriptError(f"command_script contains invalid Python: {exc.msg}") from exc
        registry = _ActionRegistry(allowed_actions)
        analyser = _CommandScriptAnalyzer(registry)
        analyser.visit(tree)
        return CommandScriptAnalysis(
            script=normalised,
            tree=tree,
            registry=registry,
            used_actions=analyser.used_actions,
        )

    def execute(
        self,
        script: str,
        allowed_actions: Sequence[str],
        context: Mapping[str, Any] | None = None,
    ) -> CommandScriptResult:
        analysis = self.analyse(script, allowed_actions)
        collector = _ActionCollector(analysis.registry, self._max_invocations)
        env: Dict[str, Any] = {"__builtins__": self._builtins}
        env.update(collector.namespace())
        env["context"] = _make_context_proxy(context)
        compiled = compile(analysis.tree, "<command_script>", "exec")
        exec(compiled, env, {})
        return CommandScriptResult(
            invocations=collector.invocations,
            used_actions=sorted(analysis.used_actions),
        )
