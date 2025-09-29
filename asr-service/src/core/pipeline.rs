use std::collections::HashMap;

use async_trait::async_trait;
use parking_lot::Mutex;

use crate::core::audio::{decode_payload, AudioPayloadFormat};
use crate::core::errors::{AsrError, AudioError};
use crate::core::messages::{AudioPayload, ClientMessage, InitPayload, ServerMessage};

/// Derived configuration for a websocket audio stream.
#[derive(Debug, Clone)]
pub struct StreamConfig {
    pub stream_id: String,
    pub lang: Option<String>,
    pub sample_rate: u32,
    pub format: AudioPayloadFormat,
}

impl StreamConfig {
    pub fn from_init(payload: &InitPayload) -> Result<Self, AudioError> {
        let format = AudioPayloadFormat::from_content_type(
            payload.content_type.as_deref(),
            payload.sample_rate,
        )?;
        let sample_rate = match format {
            AudioPayloadFormat::Pcm { sample_rate } => sample_rate,
            AudioPayloadFormat::Wav => payload.sample_rate.unwrap_or(16_000),
        };
        Ok(Self {
            stream_id: payload.stream_id.clone(),
            lang: payload.lang.clone(),
            sample_rate,
            format,
        })
    }
}

/// Maintains the registered stream configurations for a pipeline.
#[derive(Debug, Default)]
pub struct StreamRegistry {
    inner: Mutex<HashMap<String, StreamConfig>>,
}

impl StreamRegistry {
    pub fn insert(&self, config: StreamConfig) {
        self.inner.lock().insert(config.stream_id.clone(), config);
    }

    pub fn remove(&self, stream_id: &str) {
        self.inner.lock().remove(stream_id);
    }

    pub fn get(&self, stream_id: &str) -> Option<StreamConfig> {
        self.inner.lock().get(stream_id).cloned()
    }
}

/// Common helper to decode an audio payload into PCM samples using the
/// configured stream format.
pub fn decode_audio_payload(
    registry: &StreamRegistry,
    payload: &AudioPayload,
) -> Result<Vec<i16>, AsrError> {
    let config = registry
        .get(&payload.stream_id)
        .ok_or_else(|| AsrError::StreamNotInitialised(payload.stream_id.clone()))?;
    let samples = decode_payload(&payload.payload_b64, config.format)?;
    Ok(samples)
}

/// Trait implemented by the per-tier pipelines that accept client messages and
/// emit zero or more server responses.
#[async_trait]
pub trait Pipeline: Send + Sync {
    async fn handle(&self, message: ClientMessage) -> Result<Vec<ServerMessage>, AsrError>;
}
