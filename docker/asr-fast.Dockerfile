FROM rust:1.76 AS builder
WORKDIR /app

COPY Cargo.toml Cargo.lock ./
COPY asr-core/Cargo.toml asr-core/Cargo.toml
COPY asr-fast/Cargo.toml asr-fast/Cargo.toml
RUN cargo fetch --locked

COPY asr-core asr-core
COPY asr-fast asr-fast
RUN cargo build --release -p asr-fast

FROM debian:bookworm-slim
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates \
    && rm -rf /var/lib/apt/lists/*
COPY --from=builder /app/target/release/asr-fast /usr/local/bin/asr-fast

EXPOSE 8082
ENV LISTEN=0.0.0.0:8082
CMD ["asr-fast"]
