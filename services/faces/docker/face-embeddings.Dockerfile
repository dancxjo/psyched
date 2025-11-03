FROM python:3.11-slim
WORKDIR /app

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

RUN set -eux; \
    export DEBIAN_FRONTEND=noninteractive; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        pkg-config \
        libopenblas-dev \
        liblapack-dev \
        libboost-all-dev \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        libgl1 \
    ; \
    rm -rf /var/lib/apt/lists/*

COPY services/faces/embedding_service /app/embedding_service

RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir \
        numpy==1.26.4 \
        opencv-python-headless==4.8.1.78 \
        face-recognition==1.3.0 \
        face-recognition-models==0.3.0 \
        websockets==12.0

ENV LOG_LEVEL="info" \
    WEBSOCKET_HOST="0.0.0.0" \
    WEBSOCKET_PORT="5005" \
    FACES_MODEL="small" \
    FACES_NUM_JITTERS="1" \
    FACES_TIMEOUT_SEC="5.0"

EXPOSE 5005
CMD ["python", "/app/embedding_service/websocket_server.py"]

