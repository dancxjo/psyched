"""Websocket protocol helpers shared by the cockpit bridge."""
from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Dict, Literal, Optional

WsOperation = Literal["pub", "sub", "unsub"]
WsOutboundOp = Literal["msg", "ok", "err"]


class ProtocolError(ValueError):
    """Raised when a websocket payload cannot be parsed."""


@dataclass(slots=True)
class InboundMessage:
    """Normalised representation of a cockpit websocket request."""

    op: WsOperation
    topic: str
    msg: Optional[Dict[str, Any]] = None


@dataclass(slots=True)
class OutboundMessage:
    """Message structure pushed to cockpit websocket clients."""

    op: WsOutboundOp
    topic: Optional[str] = None
    msg: Optional[Dict[str, Any]] = None
    reason: Optional[str] = None
    request_id: Optional[str] = None

    def as_dict(self) -> Dict[str, Any]:
        payload: Dict[str, Any] = {"op": self.op}
        if self.topic is not None:
            payload["topic"] = self.topic
        if self.msg is not None:
            payload["msg"] = self.msg
        if self.reason is not None:
            payload["reason"] = self.reason
        if self.request_id is not None:
            payload["id"] = self.request_id
        return payload

    def to_json(self) -> str:
        return json.dumps(self.as_dict(), separators=(",", ":"))


def parse_inbound(raw: str) -> InboundMessage:
    """Parse a raw websocket payload into :class:`InboundMessage`."""

    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError as exc:  # pragma: no cover - Python guarantees the msg attr
        raise ProtocolError(f"invalid json: {exc.msg}") from exc

    if not isinstance(parsed, dict):
        raise ProtocolError("expected JSON object")

    op = parsed.get("op")
    if op not in {"pub", "sub", "unsub"}:
        raise ProtocolError("unsupported op")

    topic = parsed.get("topic")
    if not isinstance(topic, str) or not topic:
        raise ProtocolError("expected non-empty topic")

    message = parsed.get("msg")
    if op == "pub":
        if not isinstance(message, dict):
            raise ProtocolError("expected msg object")
    else:
        message = None

    return InboundMessage(op=op, topic=topic, msg=message)


def make_ok(request_id: Optional[str] = None) -> OutboundMessage:
    return OutboundMessage(op="ok", request_id=request_id)


def make_error(reason: str, request_id: Optional[str] = None) -> OutboundMessage:
    return OutboundMessage(op="err", reason=reason, request_id=request_id)


def make_message(topic: str, payload: Dict[str, Any]) -> OutboundMessage:
    return OutboundMessage(op="msg", topic=topic, msg=payload)
