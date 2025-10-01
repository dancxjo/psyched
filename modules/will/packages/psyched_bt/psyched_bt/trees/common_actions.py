"""Common ROS actions used by the Psyched behaviour trees."""

from __future__ import annotations

import time
import types
from typing import Any, Optional

import py_trees
from py_trees.common import Access, Status

from .common_blackboard import CONVERSATION_SEED_KEY, VOICE_TEXT_KEY


class Speak(py_trees.behaviour.Behaviour):
    """Publish a greeting to ``/voice`` and wait for completion.

    The behaviour reads ``VOICE_TEXT_KEY`` from the blackboard, publishes it to
    the configured voice topic, and waits for a matching ``voice_done`` message
    before reporting :class:`~py_trees.common.Status.SUCCESS`.

    Parameters
    ----------
    voice_topic:
        Topic that accepts :class:`std_msgs.msg.String` messages for synthesis.
    voice_done_topic:
        Topic used by the voice node to signal completion of playback.
    accept_any_done:
        When ``True`` the behaviour treats any ``voice_done`` payload as a
        completion signal. When ``False`` it only succeeds when the payload
        matches the published utterance.
    max_wait_sec:
        Maximum seconds to wait for a completion signal before timing out and
        returning :class:`~py_trees.common.Status.SUCCESS`.
    """

    def __init__(
        self,
        *,
        name: str = "Speak Greeting",
        text_key: str = VOICE_TEXT_KEY,
        voice_topic: str = "/voice",
        voice_done_topic: str = "voice_done",
        accept_any_done: bool = False,
        max_wait_sec: float = 10.0,
    ) -> None:
        super().__init__(name=name)
        self.text_key = text_key
        self.voice_topic = voice_topic
        self.voice_done_topic = voice_done_topic
        self._accept_any_done = accept_any_done
        self._max_wait_sec = max_wait_sec
        self._blackboard = py_trees.blackboard.Client(name=f"{name}_bb")
        self._blackboard.register_key(key=self.text_key, access=Access.READ)
        self._publisher: Optional[Any] = None
        self._subscription: Optional[Any] = None
        self._voice_done = False
        self._spoken = False
        self._last_utterance: Optional[str] = None
        self._node: Any = None
        self._string_type: Optional[type] = None
        self._start_time: float = 0.0

    @property
    def last_utterance(self) -> Optional[str]:
        """Expose the last utterance for tests and debugging."""

        return self._last_utterance

    def setup(self, **kwargs: Any) -> None:
        self._node = kwargs.get("node", None)
        if self._node is None:
            raise RuntimeError("Speak requires a ROS node provided via setup(node=...)")

        try:
            from std_msgs.msg import String  # type: ignore
        except ImportError:  # pragma: no cover - ROS environment provides the type
            String = None

        self._string_type = String
        msg_type = String if String is not None else types.SimpleNamespace
        self._publisher = self._node.create_publisher(msg_type, self.voice_topic, 10)
        self._subscription = self._node.create_subscription(msg_type, self.voice_done_topic, self._on_voice_done, 10)

    def initialise(self) -> None:
        self._voice_done = False
        self._spoken = False
        self._last_utterance = None
        self._start_time = time.monotonic()

    def update(self) -> Status:
        if self._publisher is None:
            raise RuntimeError("Speak behaviour has not been setup with a ROS node")

        utterance = getattr(self._blackboard, self.text_key, None)
        if not utterance:
            self.logger.warning("Voice text missing; failing Speak behaviour")
            return Status.FAILURE

        if not self._spoken:
            message = self._build_message(utterance)
            self._publisher.publish(message)
            self._last_utterance = utterance
            self._spoken = True
            self.logger.info(f"Speaking: {utterance}")
            return Status.RUNNING

        elapsed = time.monotonic() - self._start_time
        if self._voice_done:
            self.logger.debug("voice_done received; Speak succeeded")
            return Status.SUCCESS
        if elapsed > self._max_wait_sec:
            self.logger.debug("Timed out waiting for voice_done; Speak succeeded")
            return Status.SUCCESS

        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:  # pragma: no cover - tear down is trivial
        if new_status == Status.INVALID:
            self._spoken = False
            self._voice_done = False

    def _build_message(self, utterance: str) -> Any:
        if self._string_type is not None:
            msg = self._string_type()
            setattr(msg, "data", utterance)
            return msg
        return types.SimpleNamespace(data=utterance)

    def _on_voice_done(self, msg: Any) -> None:
        completed = getattr(msg, "data", None)
        if self._accept_any_done or completed == self._last_utterance:
            self._voice_done = True


class SeedConversation(py_trees.behaviour.Behaviour):
    """Publish a starter :class:`psyched_msgs.msg.Message` to ``/conversation``."""

    def __init__(
        self,
        *,
        name: str = "Seed Conversation",
        seed_key: str = CONVERSATION_SEED_KEY,
        topic: str = "/conversation",
    ) -> None:
        super().__init__(name=name)
        self.seed_key = seed_key
        self.topic = topic
        self._blackboard = py_trees.blackboard.Client(name=f"{name}_bb")
        self._blackboard.register_key(key=self.seed_key, access=Access.READ)
        self._publisher: Optional[Any] = None
        self._message_type: Optional[type] = None
        self._node: Any = None
        self._published = False

    def setup(self, **kwargs: Any) -> None:
        self._node = kwargs.get("node", None)
        if self._node is None:
            raise RuntimeError("SeedConversation requires a ROS node via setup(node=...)")

        try:
            from rclpy.qos import QoSReliabilityPolicy
            from psyched_msgs.msg import Message as ConversationMessage  # type: ignore
            qos = py_trees.common.QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
            )
        except ImportError:  # pragma: no cover - tests stub this module
            ConversationMessage = None
            qos = 10

        self._message_type = ConversationMessage
        msg_type = ConversationMessage if ConversationMessage is not None else types.SimpleNamespace
        self._publisher = self._node.create_publisher(msg_type, self.topic, qos)

    def initialise(self) -> None:
        self._published = False

    def update(self) -> Status:
        if self._publisher is None or self._message_type is None:
            self.logger.warning("Conversation message type unavailable; cannot seed chat yet")
            return Status.FAILURE

        seed = getattr(self._blackboard, self.seed_key, None)
        if not isinstance(seed, dict):
            self.logger.warning("Conversation seed missing; failing behaviour")
            return Status.FAILURE

        if not self._published:
            message = self._message_type()
            message.role = seed.get("role", "user")
            message.content = seed.get("content", "")
            self._publisher.publish(message)
            self._published = True
            self.logger.info("Seeded conversation with starter message")
        return Status.SUCCESS


# TODO: Add GoToGoal and CommentOnScene behaviours as regimes expand.
