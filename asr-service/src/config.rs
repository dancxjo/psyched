use std::env;
use std::fmt::{Display, Formatter};
use std::num::NonZeroUsize;
use std::time::Duration;

/// Supported recognition quality tiers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Quality {
    Fast,
    Medium,
    Long,
}

impl Display for Quality {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Quality::Fast => write!(f, "fast"),
            Quality::Medium => write!(f, "medium"),
            Quality::Long => write!(f, "long"),
        }
    }
}

impl Quality {
    fn from_str(value: &str) -> Option<Self> {
        match value.to_ascii_lowercase().as_str() {
            "fast" | "short" | "low" => Some(Quality::Fast),
            "medium" | "mid" => Some(Quality::Medium),
            "long" | "high" => Some(Quality::Long),
            _ => None,
        }
    }
}

/// Top-level configuration derived from the environment and CLI arguments.
#[derive(Debug, Clone)]
pub struct AppConfig {
    pub listen: String,
    pub quality: Quality,
    pub model_path: String,
    pub language: Option<String>,
    pub threads: NonZeroUsize,
    pub fast_ring_buffer_seconds: f32,
    pub fast_partial_interval: Duration,
    pub mid_prompt_window: usize,
    pub mid_commit_timeout: Duration,
    pub long_window_seconds: u32,
    pub long_fallback_duration: Duration,
}

impl AppConfig {
    pub fn from_env_and_args() -> Self {
        let mut config = Self::from_env();
        config.apply_args(env::args().skip(1));
        config
    }

    fn from_env() -> Self {
        let listen = env::var("LISTEN").unwrap_or_else(|_| "0.0.0.0:8082".to_string());
        let quality = env::var("ASR_QUALITY")
            .ok()
            .and_then(|raw| Quality::from_str(raw.trim()))
            .unwrap_or(Quality::Fast);
        let model_path =
            env::var("ASR_MODEL_PATH").unwrap_or_else(|_| "/models/ggml-tiny.en.bin".to_string());
        let language = env::var("ASR_LANGUAGE")
            .ok()
            .map(|value| value.trim().to_string())
            .filter(|value| !value.is_empty());
        let threads = env::var("ASR_THREADS")
            .ok()
            .and_then(|value| value.trim().parse::<usize>().ok())
            .and_then(NonZeroUsize::new)
            .or_else(|| std::thread::available_parallelism().ok())
            .unwrap_or_else(|| NonZeroUsize::new(1).expect("non-zero threads"));

        let fast_ring_buffer_seconds = env::var("ASR_FAST_RING_BUFFER_SECONDS")
            .ok()
            .and_then(|value| value.parse::<f32>().ok())
            .filter(|value| *value > 0.1)
            .unwrap_or(12.0);
        let fast_partial_interval = env::var("ASR_FAST_PARTIAL_INTERVAL_MS")
            .ok()
            .and_then(|value| value.parse::<u64>().ok())
            .map(Duration::from_millis)
            .unwrap_or(Duration::from_millis(200));
        let mid_prompt_window = env::var("ASR_MID_PROMPT_WINDOW")
            .ok()
            .and_then(|value| value.parse::<usize>().ok())
            .filter(|value| *value <= 32)
            .unwrap_or(3);
        let mid_commit_timeout = env::var("ASR_MID_COMMIT_TIMEOUT_MS")
            .ok()
            .and_then(|value| value.parse::<u64>().ok())
            .map(Duration::from_millis)
            .unwrap_or(Duration::from_secs(2));
        let long_window_seconds = env::var("ASR_LONG_WINDOW_SECONDS")
            .ok()
            .and_then(|value| value.parse::<u32>().ok())
            .filter(|value| *value > 0)
            .unwrap_or(120);
        let long_fallback_duration = env::var("ASR_LONG_FALLBACK_SECONDS")
            .ok()
            .and_then(|value| value.parse::<u64>().ok())
            .map(Duration::from_secs)
            .unwrap_or(Duration::from_secs(5));

        Self {
            listen,
            quality,
            model_path,
            language,
            threads,
            fast_ring_buffer_seconds,
            fast_partial_interval,
            mid_prompt_window,
            mid_commit_timeout,
            long_window_seconds,
            long_fallback_duration,
        }
    }

    fn apply_args<I>(&mut self, args: I)
    where
        I: IntoIterator,
        I::Item: Into<String>,
    {
        let mut iter = args.into_iter().map(Into::into).peekable();
        while let Some(arg) = iter.next() {
            match arg.as_str() {
                "--listen" => {
                    if let Some(value) = iter.peek() {
                        self.listen = value.clone();
                        iter.next();
                    }
                }
                "--quality" => {
                    if let Some(value) = iter.peek() {
                        if let Some(q) = Quality::from_str(value) {
                            self.quality = q;
                        }
                        iter.next();
                    }
                }
                "--model" | "--model-path" => {
                    if let Some(value) = iter.peek() {
                        self.model_path = value.clone();
                        iter.next();
                    }
                }
                "--language" | "--lang" => {
                    if let Some(value) = iter.peek() {
                        let trimmed = value.trim();
                        self.language = if trimmed.is_empty() {
                            None
                        } else {
                            Some(trimmed.to_string())
                        };
                        iter.next();
                    }
                }
                "--threads" => {
                    if let Some(value) = iter.peek() {
                        if let Ok(parsed) = value.parse::<usize>() {
                            if let Some(non_zero) = NonZeroUsize::new(parsed) {
                                self.threads = non_zero;
                            }
                        }
                        iter.next();
                    }
                }
                _ => {}
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quality_parsing_accepts_synonyms() {
        assert_eq!(Quality::from_str("fast"), Some(Quality::Fast));
        assert_eq!(Quality::from_str("low"), Some(Quality::Fast));
        assert_eq!(Quality::from_str("medium"), Some(Quality::Medium));
        assert_eq!(Quality::from_str("mid"), Some(Quality::Medium));
        assert_eq!(Quality::from_str("long"), Some(Quality::Long));
        assert_eq!(Quality::from_str("high"), Some(Quality::Long));
        assert_eq!(Quality::from_str("unknown"), None);
    }

    #[test]
    fn arguments_override_environment_defaults() {
        let mut config = AppConfig::from_env();
        config.apply_args([
            "--listen",
            "127.0.0.1:9000",
            "--quality",
            "medium",
            "--model",
            "/tmp/model.bin",
            "--language",
            "fr",
            "--threads",
            "4",
        ]);
        assert_eq!(config.listen, "127.0.0.1:9000");
        assert_eq!(config.quality, Quality::Medium);
        assert_eq!(config.model_path, "/tmp/model.bin");
        assert_eq!(config.language.as_deref(), Some("fr"));
        assert_eq!(config.threads.get(), 4);
    }

    #[test]
    fn blank_language_argument_clears_language() {
        let mut config = AppConfig::from_env();
        config.language = Some("en".into());
        config.apply_args(["--language", " "]);
        assert!(config.language.is_none());
    }
}
