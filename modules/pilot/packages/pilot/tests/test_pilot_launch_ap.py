"""Tests for the pilot launch description wiring of the access point node."""

from __future__ import annotations

import importlib.util
import sys
import types
from pathlib import Path


class _FakeLaunchDescription(list):
    """Minimal stub of launch.LaunchDescription for import-time isolation.

    The real ROS 2 launch API is not available inside the CI environment, so we
    substitute the handful of behaviours required by the pilot launch file.
    The launch script treats the object as an iterable of entities via the
    ``entities`` attribute.  We mirror that contract while otherwise behaving
    like a regular list to keep assertions straightforward.
    """

    def __init__(self, entities):  # pragma: no cover - trivial wrapper
        super().__init__(entities)
        self.entities = list(entities)


class _FakeDeclareLaunchArgument:
    def __init__(self, name, **kwargs):
        self.name = name
        self.kwargs = kwargs


class _FakeIfCondition:
    def __init__(self, expression):
        self.expression = expression


class _FakeLaunchConfiguration:
    def __init__(self, name):
        self.name = name


class _FakeEnvironmentVariable:
    def __init__(self, name, default_value=None):
        self.name = name
        self.default_value = default_value


class _FakeNode:
    def __init__(self, *, package, executable, parameters=None, condition=None, **kwargs):
        self.package = package
        self.executable = executable
        self.parameters = parameters or []
        self.condition = condition
        self.kwargs = kwargs


def _load_pilot_launch(monkeypatch):
    """Load pilot.launch.py with ROS launch modules stubbed out."""

    launch_mod = types.ModuleType('launch')
    launch_mod.LaunchDescription = _FakeLaunchDescription
    actions_mod = types.ModuleType('launch.actions')
    actions_mod.DeclareLaunchArgument = _FakeDeclareLaunchArgument
    conditions_mod = types.ModuleType('launch.conditions')
    conditions_mod.IfCondition = _FakeIfCondition
    substitutions_mod = types.ModuleType('launch.substitutions')
    substitutions_mod.LaunchConfiguration = _FakeLaunchConfiguration
    substitutions_mod.EnvironmentVariable = _FakeEnvironmentVariable
    launch_ros_mod = types.ModuleType('launch_ros')
    launch_ros_actions = types.ModuleType('launch_ros.actions')
    launch_ros_actions.Node = _FakeNode

    monkeypatch.setitem(sys.modules, 'launch', launch_mod)
    monkeypatch.setitem(sys.modules, 'launch.actions', actions_mod)
    monkeypatch.setitem(sys.modules, 'launch.conditions', conditions_mod)
    monkeypatch.setitem(sys.modules, 'launch.substitutions', substitutions_mod)
    monkeypatch.setitem(sys.modules, 'launch_ros', launch_ros_mod)
    monkeypatch.setitem(sys.modules, 'launch_ros.actions', launch_ros_actions)

    launch_file = Path(__file__).resolve().parents[1] / 'launch' / 'pilot.launch.py'
    spec = importlib.util.spec_from_file_location('pilot_launch', launch_file)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_pilot_launch_declares_ap_node(monkeypatch):
    """The pilot launch description should include the AP node wiring."""

    module = _load_pilot_launch(monkeypatch)
    launch_desc = module.generate_launch_description()

    enable_args = [
        entity
        for entity in launch_desc.entities
        if isinstance(entity, _FakeDeclareLaunchArgument) and entity.name == 'enable_ap'
    ]
    assert enable_args, 'enable_ap launch argument must be declared'

    ap_nodes = [
        entity
        for entity in launch_desc.entities
        if isinstance(entity, _FakeNode) and entity.executable == 'pilot_ap'
    ]
    assert ap_nodes, 'pilot_ap node should be part of the launch description'

    ap_node = ap_nodes[0]
    assert isinstance(ap_node.condition, _FakeIfCondition)
    assert isinstance(ap_node.condition.expression, _FakeLaunchConfiguration)
    assert ap_node.condition.expression.name == 'enable_ap'

    assert ap_node.parameters, 'AP node should expose parameters'
    params = ap_node.parameters[0]
    expected_params = {
        'ap_interface',
        'ap_ssid',
        'ap_passphrase',
        'ap_ip',
        'dhcp_range',
        'dhcp_lease_time',
        'mdns_name',
        'http_port',
        'websocket_port',
        'dry_run',
    }
    assert expected_params.issubset(params.keys())
