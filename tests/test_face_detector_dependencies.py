"""Tests covering dependency validation for the faces module."""
from __future__ import annotations

import importlib
import sys
import types
from collections import deque
from pathlib import Path

import pytest


def _install_module(name: str, module: types.ModuleType) -> None:
    """Register ``module`` under ``name`` in ``sys.modules``."""

    parts = name.split(".")
    for index in range(1, len(parts)):
        parent_name = ".".join(parts[:index])
        sys.modules.setdefault(parent_name, types.ModuleType(parent_name))
    sys.modules[name] = module


def _stub_ros_graph() -> None:
    """Create lightweight stubs for ROS-related modules used by the node."""

    # rclpy core entry points
    rclpy = types.ModuleType("rclpy")

    def _noop(*_args: object, **_kwargs: object) -> None:  # pragma: no cover - trivial stub
        return None

    rclpy.init = _noop  # type: ignore[attr-defined]
    rclpy.spin = _noop  # type: ignore[attr-defined]
    rclpy.shutdown = _noop  # type: ignore[attr-defined]
    _install_module("rclpy", rclpy)

    # rclpy.node
    rclpy_node = types.ModuleType("rclpy.node")

    class _StubLogger:
        def info(self, *_args: object, **_kwargs: object) -> None:  # pragma: no cover - debug stub
            return None

        def error(self, *_args: object, **_kwargs: object) -> None:  # pragma: no cover - debug stub
            return None

    class _StubParameter:
        def __init__(self) -> None:
            self.value = None

    class _StubNode:
        """Minimal stand-in for :class:`rclpy.node.Node`."""

        def __init__(self, _name: str) -> None:
            self._logger = _StubLogger()

        def declare_parameter(self, *_args: object, **_kwargs: object) -> None:
            return None

        def get_parameter(self, _name: str) -> _StubParameter:
            return _StubParameter()

        def create_publisher(self, *_args: object, **_kwargs: object) -> object:
            return object()

        def create_subscription(self, *_args: object, **_kwargs: object) -> object:
            return object()

        def add_on_set_parameters_callback(self, *_args: object, **_kwargs: object) -> None:
            return None

        def get_logger(self) -> _StubLogger:
            return self._logger

        def destroy_subscription(self, *_args: object, **_kwargs: object) -> None:
            return None

        def destroy_node(self) -> None:
            return None

    rclpy_node.Node = _StubNode  # type: ignore[attr-defined]
    _install_module("rclpy.node", rclpy_node)

    # rclpy.qos helpers referenced in the node
    rclpy_qos = types.ModuleType("rclpy.qos")

    class _QoSProfile:
        def __init__(self, *args: object, **kwargs: object) -> None:
            self.args = args
            self.kwargs = kwargs

    class _Enum:
        def __init__(self, value: int) -> None:
            self.value = value

    rclpy_qos.QoSProfile = _QoSProfile  # type: ignore[attr-defined]
    rclpy_qos.QoSDurabilityPolicy = types.SimpleNamespace(VOLATILE=_Enum(0))
    rclpy_qos.QoSHistoryPolicy = types.SimpleNamespace(KEEP_LAST=_Enum(0))
    rclpy_qos.QoSReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=_Enum(0))

    def _sensor_data_qos() -> int:  # pragma: no cover - simple stub
        return 10

    rclpy_qos.SensorDataQoS = _sensor_data_qos  # type: ignore[attr-defined]
    _install_module("rclpy.qos", rclpy_qos)

    # rclpy.logging
    rclpy_logging = types.ModuleType("rclpy.logging")

    def _get_logger(_name: str) -> _StubLogger:  # pragma: no cover - trivial stub
        return _StubLogger()

    rclpy_logging.get_logger = _get_logger  # type: ignore[attr-defined]
    _install_module("rclpy.logging", rclpy_logging)

    # rcl_interfaces.msg
    rcl_interfaces_msg = types.ModuleType("rcl_interfaces.msg")

    class _SetParametersResult:
        def __init__(self, successful: bool = True) -> None:
            self.successful = successful

    rcl_interfaces_msg.SetParametersResult = _SetParametersResult  # type: ignore[attr-defined]
    _install_module("rcl_interfaces.msg", rcl_interfaces_msg)

    # sensor_msgs.msg
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")

    class _Image:  # pragma: no cover - minimal stub
        def __init__(self) -> None:
            self.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0, nanosec=0))
            self.width = 0
            self.height = 0

    sensor_msgs_msg.Image = _Image  # type: ignore[attr-defined]
    _install_module("sensor_msgs.msg", sensor_msgs_msg)

    # std_msgs.msg
    std_msgs_msg = types.ModuleType("std_msgs.msg")

    class _String:  # pragma: no cover - minimal stub
        def __init__(self) -> None:
            self.data = ""

    class _Header:  # pragma: no cover - minimal stub
        def __init__(self) -> None:
            self.frame_id = ""
            self.stamp = types.SimpleNamespace(sec=0, nanosec=0)

    std_msgs_msg.String = _String  # type: ignore[attr-defined]
    std_msgs_msg.Header = _Header  # type: ignore[attr-defined]
    _install_module("std_msgs.msg", std_msgs_msg)

    # faces_msgs.msg
    faces_msgs_msg = types.ModuleType("faces_msgs.msg")

    class _FaceDetection:  # pragma: no cover - minimal stub
        def __init__(self) -> None:
            self.header = None
            self.x = 0
            self.y = 0
            self.width = 0
            self.height = 0
            self.confidence = 0.0
            self.embedding = []
            self.crop = None

    class _FaceDetections:  # pragma: no cover - minimal stub
        def __init__(self) -> None:
            self.header = None
            self.faces: deque = deque()

    faces_msgs_msg.FaceDetection = _FaceDetection  # type: ignore[attr-defined]
    faces_msgs_msg.FaceDetections = _FaceDetections  # type: ignore[attr-defined]
    _install_module("faces_msgs.msg", faces_msgs_msg)

    # cv_bridge
    cv_bridge = types.ModuleType("cv_bridge")

    class _CvBridge:  # pragma: no cover - minimal stub
        def imgmsg_to_cv2(self, msg: object, desired_encoding: str = "bgr8") -> object:
            return object()

    cv_bridge.CvBridge = _CvBridge  # type: ignore[attr-defined]
    _install_module("cv_bridge", cv_bridge)

    # numpy placeholder (sufficient for imports without heavy dependency)
    numpy_module = types.ModuleType("numpy")
    numpy_module.ndarray = object  # type: ignore[attr-defined]
    numpy_module.asarray = lambda array, dtype=None: array  # type: ignore[attr-defined]
    _install_module("numpy", numpy_module)


def test_face_detector_reports_missing_cv2(monkeypatch: pytest.MonkeyPatch) -> None:
    """Ensure the faces node surfaces a helpful error when OpenCV is absent."""

    _stub_ros_graph()

    # Ensure cv2 is absent so the processing import fails similarly to runtime logs.
    monkeypatch.delitem(sys.modules, "cv2", raising=False)

    module_name = "modules.faces.packages.faces.faces.face_detector_node"
    if module_name in sys.modules:
        sys.modules.pop(module_name)

    module = importlib.import_module(module_name)

    with pytest.raises(getattr(module, "MissingDependencyError")) as exc_info:
        module._get_processing_module()

    message = str(exc_info.value)
    assert "OpenCV" in message or "cv2" in message
REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
