//! Forebrain LLM WebSocket service entrypoint.

use forebrain_llm::{config::AppConfig, model::ModelManager, ws};
use tracing::{error, info, warn};
use tracing_subscriber::EnvFilter;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let env_filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));

    tracing_subscriber::fmt()
        .with_env_filter(env_filter)
        .with_target(false)
        .with_writer(std::io::stdout)
        .init();

    let config = AppConfig::from_env();
    let model = ModelManager::empty();

    match model.load_model(&config.model).await {
        Ok(_) => info!("model loaded at startup"),
        Err(err) => {
            warn!(error = %err, "failed to load model at startup; awaiting load_model command")
        }
    }

    if let Err(err) = ws::serve(config, model).await {
        error!(error = %err, "server encountered error");
        return Err(err);
    }

    Ok(())
}
