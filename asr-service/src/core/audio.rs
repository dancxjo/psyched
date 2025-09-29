use std::io::Cursor;

use base64::{engine::general_purpose, Engine as _};

use crate::errors::AudioError;

/// Supported encodings for inbound audio payloads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AudioPayloadFormat {
    /// Base64-encoded WAV blobs (16-bit, mono).
    Wav,
    /// Base64-encoded raw PCM frames (16-bit, mono).
    Pcm { sample_rate: u32 },
}

impl AudioPayloadFormat {
    /// Derive the audio payload format from a `content_type` header value.
    pub fn from_content_type(
        value: Option<&str>,
        sample_rate: Option<u32>,
    ) -> Result<Self, AudioError> {
        match value.unwrap_or("audio/wav") {
            "audio/wav" => Ok(Self::Wav),
            other if other.starts_with("audio/pcm") => {
                let rate = sample_rate
                    .or_else(|| parse_rate_from_content_type(other))
                    .unwrap_or(16_000);
                Ok(Self::Pcm { sample_rate: rate })
            }
            other => Err(AudioError::UnsupportedContentType(other.to_string())),
        }
    }
}

fn parse_rate_from_content_type(value: &str) -> Option<u32> {
    value
        .split(';')
        .map(|part| part.trim())
        .find_map(|part| part.strip_prefix("rate="))
        .and_then(|rate| rate.parse().ok())
}

/// Decode a base64 payload into raw samples.
///
/// The resulting samples are always mono 16-bit PCM.
pub fn decode_payload(
    payload_b64: &str,
    format: AudioPayloadFormat,
) -> Result<Vec<i16>, AudioError> {
    let bytes = general_purpose::STANDARD
        .decode(payload_b64)
        .map_err(AudioError::from)?;
    match format {
        AudioPayloadFormat::Wav => decode_wav(&bytes),
        AudioPayloadFormat::Pcm { .. } => decode_pcm(&bytes),
    }
}

fn decode_wav(bytes: &[u8]) -> Result<Vec<i16>, AudioError> {
    let cursor = Cursor::new(bytes);
    let mut reader = hound::WavReader::new(cursor).map_err(AudioError::from)?;
    let spec = reader.spec();
    if spec.channels != 1
        || spec.sample_format != hound::SampleFormat::Int
        || spec.bits_per_sample != 16
    {
        return Err(AudioError::from(spec));
    }
    reader
        .samples::<i16>()
        .collect::<Result<Vec<_>, _>>()
        .map_err(AudioError::from)
}

fn decode_pcm(bytes: &[u8]) -> Result<Vec<i16>, AudioError> {
    if bytes.len() % 2 != 0 {
        return Err(AudioError::OddPcmByteLength(bytes.len()));
    }
    Ok(bytes
        .chunks_exact(2)
        .map(|chunk| i16::from_le_bytes([chunk[0], chunk[1]]))
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_content_type_rates() {
        assert_eq!(
            AudioPayloadFormat::from_content_type(Some("audio/pcm; rate=8000"), None).unwrap(),
            AudioPayloadFormat::Pcm { sample_rate: 8_000 }
        );
    }
}
