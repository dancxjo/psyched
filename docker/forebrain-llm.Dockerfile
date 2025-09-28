FROM rust:1.76 AS builder
WORKDIR /app

COPY Cargo.toml Cargo.lock ./
COPY forebrain-llm/Cargo.toml forebrain-llm/Cargo.toml
RUN cargo fetch --locked

COPY forebrain-llm forebrain-llm
RUN cargo build --release -p forebrain-llm

FROM debian:bookworm-slim
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates \
    && rm -rf /var/lib/apt/lists/*
COPY --from=builder /app/target/release/forebrain-llm /usr/local/bin/forebrain-llm

EXPOSE 8080
ENV FOREBRAIN_LLM__WEBSOCKET__BIND_ADDR=0.0.0.0:8080
CMD ["forebrain-llm"]
