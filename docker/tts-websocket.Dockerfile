FROM python:3.11-slim
WORKDIR /app

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

COPY tools/tts_websocket /app/tools/tts_websocket

RUN pip install --no-cache-dir --upgrade pip \
    && pip install --no-cache-dir "TTS==0.15.5" "websockets==15.0.1"

EXPOSE 5002
CMD ["python", "-m", "tools.tts_websocket.websocket_server"]
