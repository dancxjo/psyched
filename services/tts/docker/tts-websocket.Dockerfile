FROM python:3.11-slim
WORKDIR /app

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

COPY services/tts/tts_websocket /app/tts_websocket

RUN set -eux; \
    export DEBIAN_FRONTEND=noninteractive; \
    apt-get update && apt-get install -y --no-install-recommends \
    espeak-ng \
    libsndfile1 \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*; \
    pip install --no-cache-dir --upgrade pip; \
    pip install --no-cache-dir "TTS==0.15.5" "websockets==15.0.1"

EXPOSE 5002
CMD ["python", "/app/tts_websocket/websocket_server.py"]
