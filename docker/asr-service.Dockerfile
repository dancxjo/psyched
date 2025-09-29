FROM rust:1.82 AS builder
WORKDIR /app

COPY Cargo.toml ./
COPY Cargo.lock ./
# Copy the source tree required for the unified ASR service.
COPY asr-service asr-service
RUN cargo fetch

RUN cargo build --release -p asr-service

FROM debian:bookworm-slim
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates \
    && rm -rf /var/lib/apt/lists/*
COPY --from=builder /app/target/release/asr-service /usr/local/bin/asr-service
RUN mkdir -p /models

EXPOSE 8082
ENV LISTEN=0.0.0.0:8082 \
    ASR_MODEL_PATH=/models/ggml-tiny.en.bin \
    ASR_LANGUAGE=en \
    ASR_QUALITY=fast
CMD ["asr-service"]
