from __future__ import annotations

import pytest

from pilot.command_script import (
    CommandInvocation,
    CommandScriptError,
    CommandScriptInterpreter,
)


@pytest.fixture()
def interpreter() -> CommandScriptInterpreter:
    return CommandScriptInterpreter()


@pytest.fixture()
def allowed_actions() -> list[str]:
    return [
        "voice.say",
        "voice.resume_speech",
        "nav.move_to",
        "nav.scan",
    ]


def test_execute_script_collects_invocations(interpreter, allowed_actions):
    script = """
voice.resume_speech()
voice.say(text="Hello")
for target in ["alpha", "beta"]:
    nav.move_to(target=target)
"""

    result = interpreter.execute(script, allowed_actions, context={"status": {"nav": "idle"}})

    assert [
        (invocation.module, invocation.action)
        for invocation in result.invocations
    ] == [
        ("voice", "resume_speech"),
        ("voice", "say"),
        ("nav", "move_to"),
        ("nav", "move_to"),
    ]
    assert result.invocations[1].arguments == {"text": "Hello"}
    assert result.invocations[-1].arguments == {"target": "beta"}


def test_execute_script_supports_fallback_action_call(interpreter, allowed_actions):
    script = "action('nav.scan', area='forward')"

    result = interpreter.execute(script, allowed_actions)

    assert result.invocations == [
        CommandInvocation(module="nav", action="scan", arguments={"area": "forward"})
    ]


def test_execute_script_rejects_unknown_action(interpreter, allowed_actions):
    script = "voice.dance()"

    with pytest.raises(CommandScriptError):
        interpreter.execute(script, allowed_actions)


def test_execute_script_rejects_disallowed_nodes(interpreter, allowed_actions):
    script = "import os"

    with pytest.raises(CommandScriptError):
        interpreter.execute(script, allowed_actions)


def test_analyse_identifies_unknown_function(interpreter, allowed_actions):
    script = "unknown_helper()"

    with pytest.raises(CommandScriptError):
        interpreter.analyse(script, allowed_actions)
