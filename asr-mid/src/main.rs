mod pipeline;
mod server;

use std::net::SocketAddr;
use std::num::NonZeroUsize;
use std::sync::Arc;

use anyhow::Context;
use asr_core::recognizer::{SpeechRecognizer, WhisperRecognizer};
use pipeline::{MidPipeline, MidPipelineConfig};
use tokio::net::TcpListener;
use tracing::{info, Level};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    setup_tracing();
    let listen = listen_addr()?;
    let recognizer = build_recognizer()?;
    let pipeline = Arc::new(MidPipeline::new(MidPipelineConfig::default(), recognizer));
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

fn build_recognizer() -> anyhow::Result<Arc<dyn SpeechRecognizer>> {
    let model_path =
        std::env::var("ASR_MODEL_PATH").unwrap_or_else(|_| "/models/ggml-tiny.en.bin".to_string());
    let language = std::env::var("ASR_LANGUAGE")
        .ok()
        .map(|value| value.trim().to_string())
        .filter(|value| !value.is_empty());
    let threads = std::env::var("ASR_THREADS")
        .ok()
        .and_then(|value| value.trim().parse::<usize>().ok())
        .and_then(NonZeroUsize::new)
        .or_else(|| std::thread::available_parallelism().ok())
        .unwrap_or_else(|| NonZeroUsize::new(1).expect("non-zero threads"));
    let recognizer = WhisperRecognizer::new(model_path, language, threads)?;
    Ok(Arc::new(recognizer))
}
