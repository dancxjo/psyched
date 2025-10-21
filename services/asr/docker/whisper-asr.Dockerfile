FROM rust:1.81-slim AS builder
WORKDIR /app

# Dependencies required to build whisper.cpp via whisper-rs
RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        build-essential \
        clang \
        cmake \
        pkg-config \
        libssl-dev \
        git; \
    rm -rf /var/lib/apt/lists/*

# Prime dependency cache
WORKDIR /app/asr
COPY services/asr/app/Cargo.toml Cargo.toml
RUN mkdir src && echo "fn main() {}" > src/main.rs
RUN cargo build --release
RUN rm -rf src

# Build the actual application
COPY services/asr/app/src src
RUN cargo build --release

FROM debian:bookworm-slim
WORKDIR /app
RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        libstdc++6 \
        ca-certificates; \
    rm -rf /var/lib/apt/lists/*

COPY --from=builder /app/asr/target/release/whisper-asr /usr/local/bin/whisper-asr

ENV WHISPER_MODEL="/models/ggml-base.en.bin" \
    ASR_SAMPLE_RATE="16000" \
    ASR_STABILITY_HITS="2" \
    ASR_HOP_MS="750" \
    ASR_MIN_DURATION_MS="2000" \
    LOG_LEVEL="info"

EXPOSE 5003
ENTRYPOINT ["whisper-asr"]
