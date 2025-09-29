use std::net::SocketAddr;
use std::num::NonZeroUsize;
use std::sync::Arc;

use anyhow::Context;
use tokio::net::TcpListener;
use tracing::{info, Level};

use asr_service::config::{AppConfig, Quality};
use asr_service::core::recognizer::{SpeechRecognizer, WhisperRecognizer};
use asr_service::pipelines::fast::{FastPipeline, FastPipelineConfig};
use asr_service::pipelines::long::{LongPipeline, LongPipelineConfig};
use asr_service::pipelines::medium::{MidPipeline, MidPipelineConfig};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    setup_tracing();
    let config = AppConfig::from_env_and_args();
    let listen = resolve_listen_addr(&config)?;
    let pipeline = build_pipeline(&config).context("failed to construct pipeline")?;
    let app = asr_service::server::router(pipeline);
    info!(%listen, quality=%config.quality, "starting asr-service websocket server");
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

fn resolve_listen_addr(config: &AppConfig) -> anyhow::Result<SocketAddr> {
    config
        .listen
        .parse()
        .with_context(|| format!("invalid listen address '{}'", config.listen))
}

fn build_pipeline(
    config: &AppConfig,
) -> anyhow::Result<Arc<dyn asr_service::core::pipeline::Pipeline>> {
    match config.quality {
        Quality::Fast => {
            let recognizer =
                build_recognizer(&config.model_path, &config.language, config.threads)?;
            let pipeline_config = FastPipelineConfig {
                ring_buffer_seconds: config.fast_ring_buffer_seconds,
                partial_interval: config.fast_partial_interval,
            };
            Ok(Arc::new(FastPipeline::new(pipeline_config, recognizer)))
        }
        Quality::Medium => {
            let recognizer =
                build_recognizer(&config.model_path, &config.language, config.threads)?;
            let pipeline_config = MidPipelineConfig {
                prompt_window: config.mid_prompt_window,
                commit_timeout: config.mid_commit_timeout,
            };
            Ok(Arc::new(MidPipeline::new(pipeline_config, recognizer)))
        }
        Quality::Long => {
            let pipeline_config = LongPipelineConfig {
                window_seconds: config.long_window_seconds,
                fallback_chunk_duration: config.long_fallback_duration,
            };
            Ok(Arc::new(LongPipeline::new(pipeline_config)))
        }
    }
}

fn build_recognizer(
    model_path: &str,
    language: &Option<String>,
    threads: NonZeroUsize,
) -> anyhow::Result<Arc<dyn SpeechRecognizer>> {
    let recognizer = WhisperRecognizer::new(model_path, language.clone(), threads)?;
    Ok(Arc::new(recognizer))
}
