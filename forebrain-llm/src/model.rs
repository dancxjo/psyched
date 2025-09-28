use llm::{
    load_dynamic, InferenceParameters, InferenceRequest, LoadProgress, Model, ModelArchitecture,
    ModelParameters, OutputRequest,
};
use parking_lot::Mutex;
use rand::thread_rng;
use std::{convert::Infallible, sync::Arc};
use thiserror::Error;
use tokio::sync::mpsc;
use tokio_stream::wrappers::ReceiverStream;
use tracing::info;

use crate::{config::ModelConfig, session::Message};

/// Errors returned by the model manager.
#[derive(Debug, Error, Clone)]
pub enum ModelError {
    /// Wraps underlying loader errors.
    #[error("failed to load model: {0}")]
    Load(String),
    /// Raised when inference fails.
    #[error("inference failed: {0}")]
    Inference(String),
    /// Raised when a model has not yet been loaded.
    #[error("model not loaded")]
    NotLoaded,
    /// Transport or runtime errors.
    #[error("transport error: {0}")]
    Transport(String),
}

/// Responsible for loading models and driving inference.
#[derive(Clone)]
pub struct ModelManager {
    inner: Arc<Mutex<Option<Arc<dyn Model>>>>,
}

impl ModelManager {
    /// Creates a manager without a loaded model.
    pub fn empty() -> Self {
        Self {
            inner: Arc::new(Mutex::new(None)),
        }
    }

    /// Loads a GGUF/GGML model into memory.
    pub async fn load_model(&self, cfg: &ModelConfig) -> Result<(), ModelError> {
        let path = cfg.path.clone();
        info!(?path, "loading model");
        let result = tokio::task::spawn_blocking(move || {
            let load_progress = |progress: LoadProgress| match progress {
                LoadProgress::HyperparametersLoaded => {
                    info!("hyperparameters loaded");
                }
                LoadProgress::ContextSize { bytes } => {
                    info!(context_bytes = bytes, "context size reported");
                }
                LoadProgress::TensorLoaded {
                    current_tensor,
                    tensor_count,
                } => {
                    info!(current_tensor, tensor_count, "tensor loaded");
                }
                LoadProgress::Loaded {
                    file_size,
                    tensor_count,
                } => {
                    info!(file_size, tensor_count, "model part loaded");
                }
            };
            load_dynamic(
                ModelArchitecture::Llama,
                &path,
                ModelParameters::default(),
                load_progress,
            )
        })
        .await
        .map_err(|err| ModelError::Transport(format!("load task panicked: {err}")))?;

        let model: Arc<dyn Model> = result
            .map_err(|err| ModelError::Load(err.to_string()))?
            .into();
        *self.inner.lock() = Some(model);
        info!("model loaded");
        Ok(())
    }

    /// Generates tokens for the supplied prompt history.
    pub async fn generate_stream(
        &self,
        cfg: &ModelConfig,
        history: Vec<Message>,
    ) -> Result<ReceiverStream<Result<String, ModelError>>, ModelError> {
        let model = self
            .inner
            .lock()
            .as_ref()
            .cloned()
            .ok_or(ModelError::NotLoaded)?;
        let inference_params = GenerationParams::from(cfg).into_parameters();
        let prompt_history = history;
        let (tx, rx) = mpsc::channel(128);
        tokio::task::spawn_blocking(move || {
            let mut session = model.start_session(Default::default());
            let mut rng = thread_rng();
            let mut output_request = OutputRequest::default();
            let prompt = build_prompt(&prompt_history);
            let request = InferenceRequest {
                prompt: prompt.as_str(),
                parameters: Some(&inference_params),
                play_back_previous_tokens: false,
                maximum_token_count: None,
            };
            let result = session.infer::<Infallible>(
                model.as_ref(),
                &mut rng,
                &request,
                &mut output_request,
                |token| {
                    let _ = tx.blocking_send(Ok(token.to_owned()));
                    Ok(())
                },
            );
            if let Err(err) = result {
                let _ = tx.blocking_send(Err(ModelError::Inference(err.to_string())));
            }
        });
        Ok(ReceiverStream::new(rx))
    }
}

#[derive(Clone)]
struct GenerationParams {
    temperature: f32,
    top_p: f32,
    repeat_penalty: f32,
}

impl From<&ModelConfig> for GenerationParams {
    fn from(cfg: &ModelConfig) -> Self {
        Self {
            temperature: cfg.temperature,
            top_p: cfg.top_p,
            repeat_penalty: cfg.repeat_penalty,
        }
    }
}

impl GenerationParams {
    fn into_parameters(self) -> InferenceParameters {
        InferenceParameters {
            temperature: self.temperature,
            top_p: self.top_p,
            repeat_penalty: self.repeat_penalty,
            ..InferenceParameters::default()
        }
    }
}

fn build_prompt(history: &[Message]) -> String {
    history
        .iter()
        .map(|msg| format!("{}: {}", msg.role, msg.content))
        .collect::<Vec<_>>()
        .join("\n")
}
