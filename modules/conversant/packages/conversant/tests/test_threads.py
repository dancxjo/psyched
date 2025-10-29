from __future__ import annotations

import time

from conversant.threads import (
    ConversationThread,
    ThreadStore,
    build_conversation_export,
    serialise_turn,
)


def test_thread_store_reuses_active_thread() -> None:
    store = ThreadStore(ttl_seconds=10.0, max_turns=3)
    thread_a = store.get("alpha")
    thread_b = store.get("alpha")
    assert thread_a.thread_id == "alpha"
    assert thread_b is thread_a


def test_thread_store_prunes_expired_threads(monkeypatch) -> None:
    store = ThreadStore(ttl_seconds=0.1, max_turns=3)
    store.get("alpha")
    store.get("beta")

    real_monotonic = time.monotonic
    monkeypatch.setattr("conversant.threads.time.monotonic", lambda: real_monotonic() + 1.0)
    removed = store.prune()
    assert sorted(removed) == ["alpha", "beta"]


def test_conversation_thread_trims_turns() -> None:
    thread = ConversationThread(thread_id="t1", created_at=0.0, updated_at=0.0)
    for idx in range(5):
        thread.append(role="user", text=f"msg-{idx}", max_turns=3)
    assert len(thread.turns) == 3
    assert [turn.text for turn in thread.turns] == ["msg-2", "msg-3", "msg-4"]


def test_serialise_turn_contains_expected_keys() -> None:
    thread = ConversationThread(thread_id="t1", created_at=0.0, updated_at=0.0)
    turn = thread.append(role="user", text="hello", intent="", metadata={"origin": "test"})
    payload = serialise_turn(turn)
    assert payload["role"] == "user"
    assert payload["text"] == "hello"
    assert payload["metadata"] == {"origin": "test"}
    assert "timestamp" in payload


def test_build_conversation_export_filters_pending_turns() -> None:
    """Only delivered turns should appear in the default transcript export."""

    thread = ConversationThread(thread_id="thread-1", created_at=0.0, updated_at=0.0)
    thread.append(role="user", text="Hello there", metadata={"origin": "test"})
    thread.append(
        role="conversant",
        text="Queueing response",
        metadata={"pending_delivery": "true"},
    )
    thread.append(role="conversant", text="Response delivered", metadata={"foo": "bar"})

    payload = build_conversation_export(thread, user_id="operator")
    assert payload["thread_id"] == "thread-1"
    assert payload["user_id"] == "operator"
    assert [message["role"] for message in payload["messages"]] == ["user", "assistant"]
    assert [message["content"] for message in payload["messages"]] == [
        "Hello there",
        "Response delivered",
    ]
    assert payload["messages"][1]["metadata"] == {"foo": "bar"}


def test_build_conversation_export_includes_pending_when_requested() -> None:
    """Pending turns are surfaced when requested explicitly."""

    thread = ConversationThread(thread_id="thread-2", created_at=0.0, updated_at=0.0)
    thread.append(role="user", text="Ping")
    thread.append(role="conversant", text="Processing", metadata={"pending_delivery": "true"})

    payload = build_conversation_export(thread, user_id="operator", delivered_only=False)
    assert [message["content"] for message in payload["messages"]] == ["Ping", "Processing"]
    assert payload["messages"][1]["metadata"]["pending_delivery"] == "true"
