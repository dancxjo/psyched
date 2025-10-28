import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from pilot.node import PilotNode


class _ActionServerHandler(BaseHTTPRequestHandler):
    requests_log: list[tuple[str, bytes]] = []

    def do_GET(self):  # noqa: D401 - standard BaseHTTPRequestHandler hook
        if self.path == "/api/actions":
            payload = {
                "modules": {
                    "voice": {
                        "actions": [
                            {
                                "name": "say",
                                "kind": "publish-topic",
                                "parameters": {
                                    "type": "object",
                                    "properties": {
                                        "text": {"type": "string"},
                                    },
                                    "required": ["text"],
                                    "additionalProperties": False,
                                },
                            }
                        ]
                    }
                }
            }
            body = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_response(404)
        self.end_headers()

    def do_POST(self):  # noqa: D401 - standard BaseHTTPRequestHandler hook
        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length)
        self.requests_log.append((self.path, body))
        self.send_response(200)
        response = b"{}"
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(response)))
        self.end_headers()
        self.wfile.write(response)

    def log_message(self, format: str, *args):  # noqa: D401 - silence test server logs
        return None


@pytest.fixture()
def cockpit_action_server():
    handler = _ActionServerHandler
    handler.requests_log = []
    server = HTTPServer(("127.0.0.1", 0), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}", handler.requests_log
    finally:
        server.shutdown()
        thread.join(timeout=2.0)


def test_pilot_expands_positional_arguments(monkeypatch: pytest.MonkeyPatch, cockpit_action_server):
    cockpit_url, requests_log = cockpit_action_server
    monkeypatch.setenv("COCKPIT_URL", cockpit_url)

    node = PilotNode()
    try:
        actions, _ = node._fetch_actions(status=None)
        assert "voice.say" in actions

        node._schedule_script_execution(
            script="voice.say('Hello there')",
            available_actions=actions,
            source="test",
            context={},
        )

        for _ in range(50):
            if requests_log:
                break
            time.sleep(0.05)

        assert requests_log, "pilot did not dispatch the action request"
        path, body = requests_log[0]
        assert path.endswith("/api/actions/voice/say")
        payload = json.loads(body.decode("utf-8"))
        assert payload == {"arguments": {"text": "Hello there"}}
    finally:
        node._script_executor.shutdown(wait=True)
        node.destroy_node()
        monkeypatch.delenv("COCKPIT_URL", raising=False)
