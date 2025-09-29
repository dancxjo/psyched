FROM rust:1.82 AS builder
WORKDIR /app

RUN apt-get update \
    && apt-get install -y --no-install-recommends clang libclang-dev cmake build-essential \
    && rm -rf /var/lib/apt/lists/*

COPY Cargo.toml ./
# Copy package sources (not just manifests) so `cargo fetch` can validate
# workspace members that require targets to be present.
COPY asr-core asr-core
COPY asr-fast asr-fast
COPY asr-mid asr-mid
COPY asr-long asr-long
RUN cargo fetch

RUN cargo build --release -p asr-fast

FROM debian:bookworm-slim
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates \
    && rm -rf /var/lib/apt/lists/*
COPY --from=builder /app/target/release/asr-fast /usr/local/bin/asr-fast
RUN mkdir -p /models

EXPOSE 8082
ENV LISTEN=0.0.0.0:8082 \
    ASR_MODEL_PATH=/models/ggml-tiny.en.bin \
    ASR_LANGUAGE=en
CMD ["asr-fast"]
