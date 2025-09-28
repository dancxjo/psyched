//! Forebrain LLM WebSocket service entrypoint.

use forebrain_llm::{config::AppConfig, model::ModelManager, ws};
use tracing::{error, info, warn};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .with_target(false)
        .init();

    let config = AppConfig::default();
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
