"""Behaviour tree that initiates a conversation when a face is detected."""
from __future__ import annotations

from typing import Any, Dict, Optional

import py_trees

from .common_actions import SeedConversation, Speak
from .common_blackboard import SetText


def create_tree(
    name: str = "ConverseOnFace",
    *,
    voice_topic: str = "/voice",
    voice_done_topic: str = "voice_done",
    conversation_topic: str = "/conversation",
    set_text_kwargs: Optional[Dict[str, Any]] = None,
    speak_kwargs: Optional[Dict[str, Any]] = None,
    seed_kwargs: Optional[Dict[str, Any]] = None,
) -> py_trees.behaviour.Behaviour:
    """Create the behaviour tree for the ``converse_on_face`` regime.

    Parameters
    ----------
    voice_topic:
        Topic used for synthesized speech.
    voice_done_topic:
        Topic that signals completion of speech playback.
    conversation_topic:
        Topic that accepts starter :class:`psyched_msgs.msg.Message` prompts.
    set_text_kwargs, speak_kwargs, seed_kwargs:
        Optional keyword arguments forwarded to the respective behaviours.
    """

    set_text_params = dict(set_text_kwargs or {})
    speak_params = {"voice_topic": voice_topic, "voice_done_topic": voice_done_topic}
    speak_params.update(speak_kwargs or {})
    speak_params.setdefault("accept_any_done", True)
    seed_params = {"topic": conversation_topic}
    seed_params.update(seed_kwargs or {})

    root = py_trees.composites.Sequence(name=name, memory=True)
    root.add_children(
        [
            SetText(**set_text_params),
            Speak(**speak_params),
            SeedConversation(**seed_params),
        ]
    )
    return root
