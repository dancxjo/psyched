use std::{env, path::PathBuf, time::Duration};

use serde::Deserialize;
use tracing::warn;

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

impl AppConfig {
    /// Construct a configuration by layering environment overrides on top of the defaults.
    pub fn from_env() -> Self {
        let mut config = Self::default();

        if let Some(path) = get_env("FOREBRAIN_LLM__MODEL__PATH") {
            config.model.path = PathBuf::from(path);
        }

        if let Some(prompt) = get_env("FOREBRAIN_LLM__MODEL__SYSTEM_PROMPT") {
            let prompt = prompt.trim();
            if prompt.is_empty() {
                config.model.system_prompt = None;
            } else {
                config.model.system_prompt = Some(prompt.to_owned());
            }
        }

        if let Some(temp) = parse_f32("FOREBRAIN_LLM__MODEL__TEMPERATURE") {
            config.model.temperature = temp;
        }
        if let Some(top_p) = parse_f32("FOREBRAIN_LLM__MODEL__TOP_P") {
            config.model.top_p = top_p;
        }
        if let Some(repeat_penalty) = parse_f32("FOREBRAIN_LLM__MODEL__REPEAT_PENALTY") {
            config.model.repeat_penalty = repeat_penalty;
        }

        if let Some(bind_addr) = get_env("FOREBRAIN_LLM__WEBSOCKET__BIND_ADDR") {
            if !bind_addr.trim().is_empty() {
                config.websocket.bind_addr = bind_addr;
            }
        }
        if let Some(interval) = parse_duration_secs("FOREBRAIN_LLM__WEBSOCKET__HEARTBEAT_INTERVAL")
        {
            config.websocket.heartbeat_interval = interval;
        }

        config
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
            path: PathBuf::from("models/gpt-oss-20b-Q5_K_M.gguf"),
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

fn get_env(key: &str) -> Option<String> {
    match env::var(key) {
        Ok(value) => Some(value),
        Err(env::VarError::NotPresent) => None,
        Err(env::VarError::NotUnicode(_)) => {
            warn!(key = key, "ignoring non-unicode environment value");
            None
        }
    }
}

fn parse_f32(key: &str) -> Option<f32> {
    let value = get_env(key)?;
    match value.trim().parse() {
        Ok(parsed) => Some(parsed),
        Err(err) => {
            warn!(key = key, value = %value, error = %err, "ignoring invalid float override");
            None
        }
    }
}

fn parse_duration_secs(key: &str) -> Option<Duration> {
    let value = get_env(key)?;
    match value.trim().parse::<f32>() {
        Ok(parsed) if parsed >= 0.0 => {
            let secs = parsed as f64;
            Some(Duration::from_secs_f64(secs))
        }
        Ok(_) => {
            warn!(key = key, value = %value, "ignoring negative duration override");
            None
        }
        Err(err) => {
            warn!(key = key, value = %value, error = %err, "ignoring invalid duration override");
            None
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn default_model_targets_gpt_oss_20b() {
        let cfg = AppConfig::default();
        assert_eq!(
            cfg.model.path,
            PathBuf::from("models/gpt-oss-20b-Q5_K_M.gguf")
        );
    }

    use super::*;
    use parking_lot::{const_mutex, Mutex};
    use std::{env, path::PathBuf};

    static ENV_LOCK: Mutex<()> = const_mutex(());

    fn with_env(vars: &[(&str, Option<&str>)], test: impl FnOnce()) {
        let _guard = ENV_LOCK.lock();
        let previous: Vec<(String, Option<String>)> = vars
            .iter()
            .map(|(key, _)| ((*key).to_string(), env::var(key).ok()))
            .collect();
        for (key, value) in vars {
            match value {
                Some(val) => env::set_var(key, val),
                None => env::remove_var(key),
            }
        }
        test();
        for (key, value) in previous {
            match value {
                Some(val) => env::set_var(&key, val),
                None => env::remove_var(&key),
            }
        }
    }

    #[test]
    fn env_overrides_defaults() {
        with_env(
            &[
                ("FOREBRAIN_LLM__MODEL__PATH", Some("/tmp/model.gguf")),
                ("FOREBRAIN_LLM__MODEL__SYSTEM_PROMPT", Some("Test prompt")),
                ("FOREBRAIN_LLM__MODEL__TEMPERATURE", Some("0.42")),
                ("FOREBRAIN_LLM__MODEL__TOP_P", Some("0.75")),
                ("FOREBRAIN_LLM__MODEL__REPEAT_PENALTY", Some("1.05")),
                (
                    "FOREBRAIN_LLM__WEBSOCKET__BIND_ADDR",
                    Some("forebrain.local:9000"),
                ),
                ("FOREBRAIN_LLM__WEBSOCKET__HEARTBEAT_INTERVAL", Some("5")),
            ],
            || {
                let cfg = AppConfig::from_env();
                assert_eq!(cfg.model.path, PathBuf::from("/tmp/model.gguf"));
                assert_eq!(cfg.model.system_prompt.as_deref(), Some("Test prompt"));
                assert!((cfg.model.temperature - 0.42).abs() < f32::EPSILON);
                assert!((cfg.model.top_p - 0.75).abs() < f32::EPSILON);
                assert!((cfg.model.repeat_penalty - 1.05).abs() < f32::EPSILON);
                assert_eq!(cfg.websocket.bind_addr, "forebrain.local:9000");
                assert_eq!(cfg.websocket.heartbeat_interval, Duration::from_secs(5));
            },
        );
    }

    #[test]
    fn default_model_targets_gpt_oss_20b() {
        let cfg = AppConfig::default();
        assert_eq!(
            cfg.model.path,
            PathBuf::from("models/gpt-oss-20b-Q5_K_M.gguf")
        );
    }

    #[test]
    fn invalid_overrides_are_ignored() {
        with_env(
            &[
                ("FOREBRAIN_LLM__MODEL__TEMPERATURE", Some("not-a-number")),
                ("FOREBRAIN_LLM__WEBSOCKET__HEARTBEAT_INTERVAL", Some("-1")),
                ("FOREBRAIN_LLM__MODEL__SYSTEM_PROMPT", Some("")),
            ],
            || {
                let cfg = AppConfig::from_env();
                assert_eq!(cfg.model.temperature, ModelConfig::default().temperature);
                assert_eq!(
                    cfg.websocket.heartbeat_interval,
                    WebSocketConfig::default().heartbeat_interval
                );
                assert_eq!(cfg.model.system_prompt, None);
            },
        );
    }
}
