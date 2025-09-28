mod pipeline;
mod server;

use std::net::SocketAddr;
use std::sync::Arc;

use anyhow::Context;
use pipeline::{MidPipeline, MidPipelineConfig};
use tokio::net::TcpListener;
use tracing::{info, Level};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    setup_tracing();
    let listen = listen_addr()?;
    let pipeline = Arc::new(MidPipeline::new(MidPipelineConfig::default()));
    let app = server::router(pipeline);
    info!(%listen, "starting asr-mid websocket server");
    let listener = TcpListener::bind(listen)
        .await
        .context("failed to bind tcp listener")?;
    axum::serve(listener, app.into_make_service())
        .await
        .context("websocket server exited")?;
    Ok(())
}

fn setup_tracing() {
    let _ = tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .with_max_level(Level::INFO)
        .try_init();
}

fn listen_addr() -> anyhow::Result<SocketAddr> {
    let default = "0.0.0.0:8082".to_string();
    let mut args = std::env::args().skip(1);
    let mut listen = std::env::var("LISTEN").unwrap_or(default);
    while let Some(arg) = args.next() {
        if arg == "--listen" {
            if let Some(value) = args.next() {
                listen = value;
            }
        }
    }
    listen
        .parse()
        .with_context(|| format!("invalid listen address '{listen}'"))
}
