FROM rust:1.82 AS builder
# Build forebrain-llm in isolation to avoid the workspace at repo root.
WORKDIR /app/forebrain-llm
COPY forebrain-llm ./
# Some host-driven Cargo.lock files use lockfile version 4 which older cargo
# in the Rust base image doesn't understand. Remove any upstream lockfile so
# `cargo fetch` can proceed inside the container.
RUN rm -f Cargo.lock && cargo fetch
RUN cargo build --release

FROM debian:bookworm-slim
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates \
    && rm -rf /var/lib/apt/lists/*
COPY --from=builder /app/target/release/forebrain-llm /usr/local/bin/forebrain-llm

EXPOSE 8080
ENV FOREBRAIN_LLM__WEBSOCKET__BIND_ADDR=0.0.0.0:8080
CMD ["forebrain-llm"]
