from __future__ import annotations

import time

from conversant.threads import ConversationThread, ThreadStore, serialise_turn


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
