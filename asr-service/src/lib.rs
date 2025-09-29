pub mod config;

pub mod core {
    pub mod audio;
    pub mod errors;
    pub mod messages;
    pub mod pipeline;
    pub mod recognizer;
}

pub mod pipelines {
    pub mod fast;
    pub mod long;
    pub mod medium;
}

pub mod server;

pub use core::errors;
pub use core::messages;
pub use core::pipeline;
pub use core::recognizer;

#[cfg(test)]
mod tests {
    use crate::core::audio::{decode_payload, AudioPayloadFormat};
    use crate::core::messages::{ClientMessage, InitPayload, ServerMessage, TranscriptSegment};

    fn sample_wav_base64() -> String {
        use base64::{engine::general_purpose, Engine as _};
        // Construct a tiny 16-bit PCM mono WAV header with two samples. The
        // exact signal does not matter for the test; it simply exercises the
        // payload decoding logic.
        let mut bytes = Vec::new();
        // RIFF header for PCM with 2 samples at 16 kHz.
        bytes.extend_from_slice(b"RIFF");
        bytes.extend_from_slice(&(36u32).to_le_bytes());
        bytes.extend_from_slice(b"WAVE");
        bytes.extend_from_slice(b"fmt ");
        bytes.extend_from_slice(&(16u32).to_le_bytes()); // PCM fmt chunk size
        bytes.extend_from_slice(&(1u16).to_le_bytes()); // PCM format
        bytes.extend_from_slice(&(1u16).to_le_bytes()); // mono
        bytes.extend_from_slice(&(16_000u32).to_le_bytes());
        bytes.extend_from_slice(&(32_000u32).to_le_bytes()); // byte rate
        bytes.extend_from_slice(&(2u16).to_le_bytes()); // block align
        bytes.extend_from_slice(&(16u16).to_le_bytes()); // bits per sample
        bytes.extend_from_slice(b"data");
        bytes.extend_from_slice(&(4u32).to_le_bytes()); // 2 samples * 2 bytes
        bytes.extend_from_slice(&0i16.to_le_bytes());
        bytes.extend_from_slice(&1i16.to_le_bytes());
        general_purpose::STANDARD.encode(bytes)
    }

    #[test]
    fn client_init_roundtrip() {
        let init = ClientMessage::Init(InitPayload {
            stream_id: "demo".into(),
            lang: Some("en".into()),
            content_type: Some("audio/wav".into()),
            sample_rate: Some(16_000),
            extras: Default::default(),
        });
        let json = serde_json::to_string(&init).expect("serialize init");
        assert!(json.contains("\"type\":\"init\""));
        let decoded: ClientMessage = serde_json::from_str(&json).expect("deserialize init");
        assert_eq!(decoded.stream_id(), "demo");
    }

    #[test]
    fn wav_payload_decodes_to_samples() {
        let payload = sample_wav_base64();
        let samples = decode_payload(&payload, AudioPayloadFormat::Wav).expect("decode wav");
        assert_eq!(samples.len(), 2);
        assert_eq!(samples[0], 0);
        assert_eq!(samples[1], 1);
    }

    #[test]
    fn server_partial_serializes() {
        let message = ServerMessage::partial(
            "demo",
            1,
            "testing",
            -0.12,
            vec![TranscriptSegment {
                start: 0.0,
                end: 0.5,
                text: "testing".into(),
                speaker: None,
            }],
        );
        let json = serde_json::to_string(&message).expect("serialize partial");
        assert!(json.contains("\"type\":\"partial\""));
    }
}
