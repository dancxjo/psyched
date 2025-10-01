#!/usr/bin/env bash
set -euo pipefail

# Build and validate the Coqui TTS websocket streamer image by piping streamed PCM
# audio through ffmpeg to produce a demo WAV file.  Run from the repository root.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_TAG="${IMAGE_TAG:-coqui-tts-websocket:local}"
CONTAINER_NAME="${CONTAINER_NAME:-coqui-tts-websocket-demo}"
TEXT_PAYLOAD="${1:-We just proved low-latency streaming from the Coqui websocket.}"
OUTPUT_FILE="${OUTPUT_FILE:-$(pwd)/coqui-tts-demo.wav}"

command -v docker >/dev/null 2>&1 || {
    echo "docker is required to run this demo" >&2
    exit 1
}
command -v ffmpeg >/dev/null 2>&1 || {
    echo "ffmpeg is required to post-process the PCM stream" >&2
    exit 1
}

pushd "${ROOT_DIR}" >/dev/null

echo "[1/4] Building ${IMAGE_TAG}..."
docker build -t "${IMAGE_TAG}" -f tools/tts_websocket/Dockerfile .

echo "[2/4] Launching ${CONTAINER_NAME} on port 5002..."
docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
docker run -d --rm --gpus all -p 5002:5002 --name "${CONTAINER_NAME}" "${IMAGE_TAG}"

cleanup() {
    docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "[3/4] Waiting for websocket readiness..."
python3 - <<'PY'
import asyncio
import sys

import websockets

URI = "ws://forebrain.local.local:5002/tts"

async def wait_for_ready() -> None:
    for attempt in range(30):
        try:
            async with websockets.connect(URI, open_timeout=1, close_timeout=1):
                return
        except Exception:  # pragma: no cover - simple readiness probe
            await asyncio.sleep(1)
    raise SystemExit("Server did not become ready on time")

asyncio.run(wait_for_ready())
PY

TMP_OUTPUT="/tmp/tts-demo.wav"

echo "[4/4] Streaming '${TEXT_PAYLOAD}' and transcoding to ${OUTPUT_FILE}..."
docker exec "${CONTAINER_NAME}" bash -lc "python3 - <<'PY' '${TEXT_PAYLOAD}'
import asyncio
import json
import os
import subprocess
import sys

import websockets

TEXT = sys.argv[1]
URI = "ws://forebrain.local.local:5002/tts"
OUTPUT = "${TMP_OUTPUT}"

async def run_demo() -> None:
    async with websockets.connect(URI, max_size=None, ping_interval=None) as ws:
        await ws.send(json.dumps({"text": TEXT}))
        start = json.loads(await ws.recv())
        if start.get("event") != "start":
            raise RuntimeError(f"Unexpected handshake: {start}")
        ffmpeg = subprocess.Popen(
            [
                "ffmpeg",
                "-loglevel",
                "error",
                "-y",
                "-f",
                "s16le",
                "-ac",
                str(start["channels"]),
                "-ar",
                str(start["sample_rate"]),
                "-i",
                "pipe:0",
                OUTPUT,
            ],
            stdin=subprocess.PIPE,
        )
        try:
            while True:
                message = await ws.recv()
                if isinstance(message, bytes):
                    ffmpeg.stdin.write(message)
                else:
                    payload = json.loads(message)
                    if payload.get("event") == "end":
                        break
                    if payload.get("event") == "error":
                        raise RuntimeError(payload.get("message", "unknown websocket error"))
        finally:
            ffmpeg.stdin.close()
            ffmpeg.wait()

asyncio.run(run_demo())
PY"

docker cp "${CONTAINER_NAME}:${TMP_OUTPUT}" "${OUTPUT_FILE}"
echo "Demo complete. Output saved to ${OUTPUT_FILE}"
