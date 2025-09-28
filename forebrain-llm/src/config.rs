use std::{path::PathBuf, time::Duration};

use serde::Deserialize;

/// Application configuration derived from environment variables or defaults.
///
/// ```rust
/// use forebrain_llm::config::AppConfig;
/// let cfg = AppConfig::default();
/// assert_eq!(cfg.websocket.bind_addr, "0.0.0.0:8080");
/// ```
#[derive(Debug, Clone, Deserialize)]
pub struct AppConfig {
    /// Model configuration controlling load paths and quantisation hints.
    pub model: ModelConfig,
    /// WebSocket configuration for incoming chat sessions.
    pub websocket: WebSocketConfig,
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            model: ModelConfig::default(),
            websocket: WebSocketConfig::default(),
        }
    }
}

/// Model specific configuration.
#[derive(Debug, Clone, Deserialize)]
pub struct ModelConfig {
    /// Filesystem path to the GGUF/GGML model to load.
    pub path: PathBuf,
    /// Optional prompt template to seed new conversations.
    pub system_prompt: Option<String>,
    /// Token generation temperature.
    pub temperature: f32,
    /// nucleus sampling probability.
    pub top_p: f32,
    /// Repeat penalty discouraging loops.
    pub repeat_penalty: f32,
}

impl Default for ModelConfig {
    fn default() -> Self {
        Self {
            path: PathBuf::from("models/gpt-oss-20b.gguf"),
            system_prompt: Some("You are the forebrain assistant for GPT-OSS.".into()),
            temperature: 0.7,
            top_p: 0.9,
            repeat_penalty: 1.1,
        }
    }
}

/// Configuration for the WebSocket endpoint.
#[derive(Debug, Clone, Deserialize)]
pub struct WebSocketConfig {
    /// Bind address for the chat server.
    pub bind_addr: String,
    /// Interval between heartbeat pings.
    pub heartbeat_interval: Duration,
}

impl Default for WebSocketConfig {
    fn default() -> Self {
        Self {
            bind_addr: "0.0.0.0:8080".into(),
            heartbeat_interval: Duration::from_secs(30),
        }
    }
}
