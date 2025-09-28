#!/usr/bin/env python3
"""Quick test script to connect to websocket TTS and send a sample text payload.

Usage: python3 tools/test_tts_ws.py ws://forebrain.local:5002/tts "Hello world"
"""
import sys
import asyncio
import json

try:
    from websockets.asyncio.client import connect as websocket_connect
except Exception as exc:
    print("websockets library not available:", exc)
    raise SystemExit(1)


async def run(uri: str, text: str):
    print(f"Connecting to {uri} ...")
    try:
        async with websocket_connect(uri, open_timeout=5, close_timeout=5, ping_interval=20, ping_timeout=20) as ws:
            payload = {"text": text}
            await ws.send(json.dumps(payload))
            print("Sent payload, waiting for metadata/event frames (timeout 5s)...")
            try:
                msg = await asyncio.wait_for(ws.recv(), timeout=5)
            except asyncio.TimeoutError:
                print("No response received within timeout")
                return
            print("Received:", msg)
    except Exception as exc:
        print("Connection failed:", exc)


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: test_tts_ws.py <ws_uri> <text>")
        raise SystemExit(1)
    asyncio.run(run(sys.argv[1], sys.argv[2]))
