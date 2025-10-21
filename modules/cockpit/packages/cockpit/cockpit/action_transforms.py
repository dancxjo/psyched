"""Helper transforms for cockpit module actions."""

from __future__ import annotations

import math
import re
import time
from typing import Any, Dict, Mapping


class ActionTransformError(ValueError):
    """Error raised when a cockpit action transform fails."""


_VALUE_PATTERN = re.compile(r"^-?\d+(?:\.\d+)?$")
_UNIT_SUFFIXES = ("m/s", "rad/s", "radians/s", "deg/s", "degrees/s", "s")

_LINEAR_KEYS = {
    "linear_x": ("linear", "x"),
    "linear.y": ("linear", "y"),
    "linear_z": ("linear", "z"),
    "lx": ("linear", "x"),
    "ly": ("linear", "y"),
    "lz": ("linear", "z"),
    "vx": ("linear", "x"),
    "vy": ("linear", "y"),
    "vz": ("linear", "z"),
}

_ANGULAR_KEYS = {
    "angular_x": ("angular", "x"),
    "angular_y": ("angular", "y"),
    "angular_z": ("angular", "z"),
    "ax": ("angular", "x"),
    "ay": ("angular", "y"),
    "az": ("angular", "z"),
    "wx": ("angular", "x"),
    "wy": ("angular", "y"),
    "wz": ("angular", "z"),
}

_DIRECTION_SYNONYMS = {
    "forward": ("linear", "x", 1.0),
    "backward": ("linear", "x", -1.0),
    "reverse": ("linear", "x", -1.0),
    "strafe_left": ("linear", "y", 1.0),
    "strafe_right": ("linear", "y", -1.0),
    "left": ("angular", "z", 1.0),
    "turn_left": ("angular", "z", 1.0),
    "right": ("angular", "z", -1.0),
    "turn_right": ("angular", "z", -1.0),
    "spin_left": ("angular", "z", 1.0),
    "spin_right": ("angular", "z", -1.0),
    "up": ("linear", "z", 1.0),
    "down": ("linear", "z", -1.0),
}


def _coerce_number(value: str) -> float:
    text = value.strip()
    for suffix in _UNIT_SUFFIXES:
        if text.lower().endswith(suffix):
            text = text[: -len(suffix)].strip()
            break
    if text.lower().endswith("deg"):
        text = text[:-3].strip()
        try:
            return math.radians(float(text))
        except ValueError as exc:
            raise ActionTransformError(f"Could not parse numeric value: {value}") from exc
    if not _VALUE_PATTERN.match(text):
        raise ActionTransformError(f"Could not parse numeric value: {value}")
    try:
        return float(text)
    except ValueError as exc:  # pragma: no cover - defensive
        raise ActionTransformError(f"Could not parse numeric value: {value}") from exc


def parse_twist_text(command: str) -> Dict[str, Any]:
    """Return a geometry_msgs/msg/Twist payload parsed from *command*."""

    if not command or not command.strip():
        raise ActionTransformError("Command text must not be empty.")

    linear = {"x": 0.0, "y": 0.0, "z": 0.0}
    angular = {"x": 0.0, "y": 0.0, "z": 0.0}

    tokens = [token for token in re.split(r"[,\s]+", command.strip()) if token]
    index = 0
    while index < len(tokens):
        token = tokens[index]
        normalised = token.lower()
        if "=" in token:
            key, raw = token.split("=", 1)
            axis = key.lower().strip()
            if axis in _LINEAR_KEYS:
                linear[_LINEAR_KEYS[axis][1]] = _coerce_number(raw)
            elif axis in _ANGULAR_KEYS:
                angular[_ANGULAR_KEYS[axis][1]] = _coerce_number(raw)
            else:
                raise ActionTransformError(f"Unknown velocity component: {axis}")
            index += 1
            continue

        if normalised in {"stop", "halt", "idle"}:
            linear = {"x": 0.0, "y": 0.0, "z": 0.0}
            angular = {"x": 0.0, "y": 0.0, "z": 0.0}
            index += 1
            continue

        if normalised in _DIRECTION_SYNONYMS:
            axis_group, axis_component, multiplier = _DIRECTION_SYNONYMS[normalised]
            if index + 1 >= len(tokens):
                raise ActionTransformError(
                    f"Missing numeric value after '{token}'."
                )
            magnitude = _coerce_number(tokens[index + 1]) * multiplier
            if axis_group == "linear":
                linear[axis_component] = magnitude
            else:
                angular[axis_component] = magnitude
            index += 2
            continue

        axis_alias = normalised
        if axis_alias in _LINEAR_KEYS or axis_alias in _ANGULAR_KEYS:
            if index + 1 >= len(tokens):
                raise ActionTransformError(
                    f"Missing numeric value after '{token}'."
                )
            magnitude = _coerce_number(tokens[index + 1])
            if axis_alias in _LINEAR_KEYS:
                linear[_LINEAR_KEYS[axis_alias][1]] = magnitude
            else:
                angular[_ANGULAR_KEYS[axis_alias][1]] = magnitude
            index += 2
            continue

        raise ActionTransformError(f"Unrecognised cmd_vel token: {token}")

    return {"linear": linear, "angular": angular}


def parse_ascii_text(text: str) -> Dict[str, Any]:
    """Convert text to the UInt8MultiArray payload expected by set_ascii."""

    if text is None:
        raise ActionTransformError("ASCII display text must not be null.")
    codes = [ord(char) for char in list(text)[:4]]
    return {"data": codes}


def _current_stamp() -> Dict[str, int]:
    now = time.time()
    sec = int(now)
    nanosec = int((now - sec) * 1_000_000_000)
    return {"sec": sec, "nanosec": nanosec}


def _parse_angle(value: str) -> float:
    text = value.strip().lower()
    for suffix in ("degrees", "degree", "deg"):
        if text.endswith(suffix):
            cleaned = text[: -len(suffix)].strip()
            try:
                return math.radians(float(cleaned))
            except ValueError as exc:
                raise ActionTransformError(f"Could not parse angle value: {value}") from exc
    if text.endswith("°"):
        cleaned = text[:-1].strip()
        try:
            return math.radians(float(cleaned))
        except ValueError as exc:
            raise ActionTransformError(f"Could not parse angle value: {value}") from exc
    for suffix in ("radians", "radian", "rad"):
        if text.endswith(suffix):
            cleaned = text[: -len(suffix)].strip()
            try:
                return float(cleaned)
            except ValueError as exc:
                raise ActionTransformError(f"Could not parse angle value: {value}") from exc
    try:
        return float(text)
    except ValueError as exc:
        raise ActionTransformError(f"Could not parse angle value: {value}") from exc


def _yaw_to_quaternion(yaw: float) -> Dict[str, float]:
    half = yaw / 2.0
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half),
        "w": math.cos(half),
    }


def _parse_pose_tokens(command: str) -> Dict[str, Any]:
    if not command or not command.strip():
        raise ActionTransformError("Pose command must not be empty.")
    tokens = [token for token in re.split(r"[,\s]+", command.strip()) if token]
    frame = "map"
    x = 0.0
    y = 0.0
    yaw = 0.0
    index = 0
    while index < len(tokens):
        token = tokens[index]
        if "=" in token:
            key, raw_value = token.split("=", 1)
        else:
            if index + 1 >= len(tokens):
                raise ActionTransformError(f"Missing value after '{token}'.")
            key = token
            raw_value = tokens[index + 1]
            index += 1
        index += 1
        key_normalized = key.lower().strip()
        value = raw_value.strip()
        if not key_normalized:
            continue
        if key_normalized in {"frame", "frame_id"}:
            if not value:
                raise ActionTransformError("frame must be a non-empty string.")
            frame = value
            continue
        try:
            if key_normalized in {"x"}:
                x = float(value)
            elif key_normalized in {"y"}:
                y = float(value)
            elif key_normalized in {"yaw", "theta", "heading", "yaw_rad"}:
                yaw = _parse_angle(value)
            elif key_normalized in {"yaw_deg", "heading_deg"}:
                yaw = math.radians(float(value))
            else:
                raise ActionTransformError(f"Unrecognised pose token: {key}")
        except ValueError as exc:
            raise ActionTransformError(f"Could not parse numeric value for {key}: {value}") from exc
    return {"frame": frame, "x": x, "y": y, "yaw": yaw}


def parse_pose_stamped_text(command: str) -> Dict[str, Any]:
    """Return a geometry_msgs/msg/PoseStamped structure parsed from *command*."""

    tokens = _parse_pose_tokens(command)
    return {
        "header": {
            "frame_id": tokens["frame"],
            "stamp": _current_stamp(),
        },
        "pose": {
            "position": {
                "x": tokens["x"],
                "y": tokens["y"],
                "z": 0.0,
            },
            "orientation": _yaw_to_quaternion(tokens["yaw"]),
        },
    }


def parse_pose_with_covariance_text(command: str) -> Dict[str, Any]:
    """Return a geometry_msgs/msg/PoseWithCovarianceStamped structure parsed from *command*."""

    tokens = _parse_pose_tokens(command)
    return {
        "header": {
            "frame_id": tokens["frame"],
            "stamp": _current_stamp(),
        },
        "pose": {
            "pose": {
                "position": {
                    "x": tokens["x"],
                    "y": tokens["y"],
                    "z": 0.0,
                },
                "orientation": _yaw_to_quaternion(tokens["yaw"]),
            },
        },
    }


TEXT_PARSERS: Mapping[str, Any] = {
    "twist/v1": parse_twist_text,
    "ascii/v1": parse_ascii_text,
    "pose_stamped/v1": parse_pose_stamped_text,
    "pose_with_covariance/v1": parse_pose_with_covariance_text,
}


__all__ = [
    "ActionTransformError",
    "TEXT_PARSERS",
    "parse_twist_text",
    "parse_ascii_text",
    "parse_pose_stamped_text",
    "parse_pose_with_covariance_text",
]
