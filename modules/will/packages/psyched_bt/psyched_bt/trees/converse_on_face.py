"""Behaviour tree that initiates a conversation when a face is detected."""
from __future__ import annotations

import py_trees

from .common_actions import SeedConversation, Speak
from .common_blackboard import SetText


def create_tree(name: str = "ConverseOnFace") -> py_trees.behaviour.Behaviour:
    """Create the behaviour tree for the ``converse_on_face`` regime."""

    root = py_trees.composites.Sequence(name=name, memory=True)
    root.add_children(
        [
            SetText(),
            Speak(),
            SeedConversation(),
        ]
    )
    return root
