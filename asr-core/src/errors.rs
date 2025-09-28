use thiserror::Error;

/// High-level protocol errors shared across services.
#[derive(Debug, Error)]
pub enum AsrError {
    #[error("unknown stream '{0}'")]
    UnknownStream(String),
    #[error("stream '{0}' has not been initialised")]
    StreamNotInitialised(String),
    #[error("decoder error: {0}")]
    Decoder(String),
    #[error(transparent)]
    Audio(#[from] AudioError),
    #[error(transparent)]
    Internal(#[from] anyhow::Error),
}

/// Errors surfaced while handling inbound audio payloads.
#[derive(Debug, Error)]
pub enum AudioError {
    #[error("base64 decode failed: {0}")]
    Base64(#[from] base64::DecodeError),
    #[error("unsupported content type '{0}'")]
    UnsupportedContentType(String),
    #[error("unsupported WAV spec: channels={channels}, bits={bits}")]
    UnsupportedWavSpec { channels: u16, bits: u16 },
    #[error("PCM payload must contain an even number of bytes (got {0})")]
    OddPcmByteLength(usize),
    #[error("WAV decode failed: {0}")]
    Wav(#[from] hound::Error),
}

impl From<hound::WavSpec> for AudioError {
    fn from(spec: hound::WavSpec) -> Self {
        AudioError::UnsupportedWavSpec {
            channels: spec.channels,
            bits: spec.bits_per_sample,
        }
    }
}
