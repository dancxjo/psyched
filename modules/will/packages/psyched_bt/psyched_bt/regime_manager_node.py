"""Regime manager node responsible for orchestrating Psyched behaviour trees."""
from __future__ import annotations

import json
from typing import Dict, Optional

import py_trees
from py_trees.common import Access, Status
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .trees import converse_on_face
from .trees.common_blackboard import PERSON_INFO_KEY


def parse_face_payload(raw: str) -> Optional[Dict[str, str]]:
    """Parse face-detection payloads into a canonical dictionary."""

    data = raw.strip()
    if not data:
        return None

    try:
        decoded = json.loads(data)
    except json.JSONDecodeError:
        return {"name": data}

    if isinstance(decoded, dict):
        name = decoded.get("name")
        if isinstance(name, str) and name:
            return {"name": name}
        return None
    if isinstance(decoded, str) and decoded:
        return {"name": decoded}
    return None


class RegimeManagerNode(Node):
    """ROS node that selects and ticks the active behavioural regime."""

    def __init__(self) -> None:
        super().__init__("psyched_regime_manager")
        self.declare_parameter("default_regime", "idle")
        default_regime = self.get_parameter("default_regime").get_parameter_value().string_value or "idle"

        self._behaviour_tree: Optional[py_trees.trees.BehaviourTree] = None
        self._current_regime = "idle"
        self._blackboard = py_trees.blackboard.Client(name="regime_manager")
        self._blackboard.register_key(key=PERSON_INFO_KEY, access=Access.WRITE)

        self._face_sub = self.create_subscription(String, "/vision/face_detected", self._on_face_detected, 10)
        self._tick_timer = self.create_timer(0.2, self._tick_current_tree)

        self.get_logger().info(f"Regime manager ready; default regime set to {default_regime}")
        self._set_regime(default_regime)

    def _on_face_detected(self, msg: String) -> None:
        payload = parse_face_payload(msg.data)
        if not payload:
            self.get_logger().debug("Face detected without a name; ignoring")
            return

        setattr(self._blackboard, PERSON_INFO_KEY, payload)
        self.get_logger().info(f"Face detected: {payload['name']}")
        self._set_regime("converse_on_face")

    def _tick_current_tree(self) -> None:
        if self._behaviour_tree is None:
            return

        self._behaviour_tree.tick()
        status = self._behaviour_tree.root.status
        if status in (Status.SUCCESS, Status.FAILURE):
            self.get_logger().info(
                f"Regime {self._current_regime} completed with status {status.name}"
            )
            self._shutdown_tree()
            self._current_regime = "idle"

    def _set_regime(self, name: str) -> None:
        if name == "idle":
            if self._behaviour_tree is not None:
                self.get_logger().debug("Stopping active regime and returning to idle")
            self._shutdown_tree()
            self._current_regime = "idle"
            return

        if name == "converse_on_face":
            self._start_converse_on_face()
            return

        if name == "navigate":
            self.get_logger().info("Navigate regime not implemented yet; remaining idle")
            self._shutdown_tree()
            self._current_regime = "idle"
            return

        if name == "explore":
            self.get_logger().info("Explore regime not implemented yet; remaining idle")
            self._shutdown_tree()
            self._current_regime = "idle"
            return

        self.get_logger().warning(f"Unknown regime '{name}'; staying in {self._current_regime}")

    def _start_converse_on_face(self) -> None:
        self._shutdown_tree()
        try:
            root = converse_on_face.create_tree()
            tree = py_trees.trees.BehaviourTree(root=root)
            tree.setup(timeout=1.0, node=self)
        except Exception as exc:  # pragma: no cover - defensive guard
            self.get_logger().error(f"Failed to start converse_on_face regime: {exc}")
            self._behaviour_tree = None
            self._current_regime = "idle"
            return

        self._behaviour_tree = tree
        self._current_regime = "converse_on_face"
        self.get_logger().info("Switched to converse_on_face regime")

    def _shutdown_tree(self) -> None:
        if self._behaviour_tree is None:
            return
        try:
            self._behaviour_tree.shutdown()
        except Exception:  # pragma: no cover - best effort cleanup
            pass
        self._behaviour_tree = None

    def destroy_node(self) -> bool:
        self._shutdown_tree()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RegimeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:  # pragma: no cover - manual shutdown path
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
