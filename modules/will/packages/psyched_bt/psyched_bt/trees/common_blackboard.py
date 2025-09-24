"""Shared blackboard primitives for Psyched behaviour trees."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import py_trees
from py_trees.common import Access, Status


# Blackboard keys used across behaviours. Keep attribute friendly so we can use
# ``getattr`` / ``setattr`` without additional translation helpers.
PERSON_INFO_KEY = "psyched_bt_person"
VOICE_TEXT_KEY = "psyched_bt_voice_text"
CONVERSATION_SEED_KEY = "psyched_bt_conversation_seed"


@dataclass
class PersonInfo:
    """Simple container describing a detected person."""

    name: str


class SetText(py_trees.behaviour.Behaviour):
    """Derive utterances and prompts from face-detection data.

    The behaviour reads a :class:`PersonInfo` dictionary from the blackboard and
    writes two pieces of state:

    ``VOICE_TEXT_KEY``
        A human-friendly greeting that is sent to :class:`~std_msgs.msg.String`
        publishers.

    ``CONVERSATION_SEED_KEY``
        A dictionary compatible with :class:`psyched_msgs.msg.Message`
        containing the starter prompt for the chat system.

    Examples
    --------
    >>> import py_trees
    >>> writer = py_trees.blackboard.Client(name="writer")
    >>> writer.register_key(key=PERSON_INFO_KEY, access=Access.WRITE)
    >>> writer.psyched_bt_person = {"name": "Ada"}
    >>> behaviour = SetText()
    >>> tree = py_trees.trees.BehaviourTree(root=behaviour)
    >>> _ = tree.setup(timeout=1.0)
    >>> _ = tree.tick()
    >>> reader = py_trees.blackboard.Client(name="reader")
    >>> reader.register_key(key=VOICE_TEXT_KEY, access=Access.READ)
    >>> reader.register_key(key=CONVERSATION_SEED_KEY, access=Access.READ)
    >>> reader.psyched_bt_voice_text
    "Hi Ada, I'm Pete. It's great to meet you."
    >>> reader.psyched_bt_conversation_seed["role"]
    'user'
    """

    def __init__(
        self,
        *,
        name: str = "Prepare Greeting",
        person_key: str = PERSON_INFO_KEY,
        voice_key: str = VOICE_TEXT_KEY,
        conversation_key: str = CONVERSATION_SEED_KEY,
    ) -> None:
        super().__init__(name=name)
        self.person_key = person_key
        self.voice_key = voice_key
        self.conversation_key = conversation_key
        self.blackboard = py_trees.blackboard.Client(name=f"{name}_bb")
        self.blackboard.register_key(key=self.person_key, access=Access.READ)
        self.blackboard.register_key(key=self.voice_key, access=Access.WRITE)
        self.blackboard.register_key(key=self.conversation_key, access=Access.WRITE)

    def update(self) -> Status:
        person: Optional[Dict[str, str]] = getattr(self.blackboard, self.person_key, None)
        if not person:
            self.logger.warning("No person info available; cannot prepare greeting")
            return Status.FAILURE

        name = person.get("name") if isinstance(person, dict) else None
        if not name:
            self.logger.info("Detected person is missing a name; skipping greeting")
            return Status.FAILURE

        greeting = f"Hi {name}, I'm Pete. It's great to meet you."
        conversation_seed = {
            "role": "user",
            "content": (
                f"Greet {name} warmly and keep the conversation going. "
                "Offer a follow-up question after the introduction."
            ),
        }

        setattr(self.blackboard, self.voice_key, greeting)
        setattr(self.blackboard, self.conversation_key, conversation_seed)
        self.logger.debug(f"Prepared greeting for {name}")
        return Status.SUCCESS


class SetFlag(py_trees.behaviour.Behaviour):
    """Placeholder for future guard behaviours.

    TODO: implement guard logic when additional regimes are introduced.
    """

    def __init__(self, name: str = "Set Flag") -> None:
        super().__init__(name=name)

    def update(self) -> Status:  # pragma: no cover - stub behaviour
        self.logger.debug("SetFlag is a stub; returning SUCCESS by default")
        return Status.SUCCESS
