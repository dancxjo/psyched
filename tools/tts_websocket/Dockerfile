# Dockerfile for a low-latency Coqui TTS websocket streamer.
FROM ghcr.io/coqui-ai/tts:latest

# Use Coqui XTTS-v2 by default (requires CPML acceptance)
# Setting COQUI_TOS_AGREED=1 indicates you accept the Coqui Public Model License (CPML)
ENV PYTHONUNBUFFERED=1 \
    # Use Coqui XTTS-v2 by default (requires CPML acceptance)
    # Setting COQUI_TOS_AGREED=1 indicates you accept the Coqui Public Model License (CPML)
    TTS_MODEL="tts_models/multilingual/multi-dataset/xtts_v2" \
    TTS_DEFAULT_SPEAKER="p330" \
    TTS_DEFAULT_LANGUAGE="en" \
    PCM_CHUNK_SAMPLES="4096" \
    TTS_USE_CUDA="true" \
    COQUI_TOS_AGREED=1 \
    NVIDIA_VISIBLE_DEVICES=all

# Install runtime dependencies for websocket streaming and local validation helpers.
RUN apt-get update \
    && apt-get install -y --no-install-recommends ffmpeg \
    && rm -rf /var/lib/apt/lists/* \
    && pip install --no-cache-dir websockets==15.0.1

WORKDIR /app
COPY tools/tts_websocket/websocket_server.py /app/websocket_server.py

EXPOSE 5002

ENTRYPOINT ["python3", "/app/websocket_server.py"]
