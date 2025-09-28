use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

use crate::errors::AsrError;

/// Inbound messages sent by clients over the websocket.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum ClientMessage {
    Init(InitPayload),
    Audio(AudioPayload),
    Commit(CommitPayload),
    Cancel(CancelPayload),
    #[serde(rename = "final_text")]
    FinalText(FinalTextPayload),
}

impl ClientMessage {
    /// Convenience accessor for the message's stream identifier.
    pub fn stream_id(&self) -> &str {
        match self {
            ClientMessage::Init(payload) => &payload.stream_id,
            ClientMessage::Audio(payload) => &payload.stream_id,
            ClientMessage::Commit(payload) => &payload.stream_id,
            ClientMessage::Cancel(payload) => &payload.stream_id,
            ClientMessage::FinalText(payload) => &payload.stream_id,
        }
    }
}

/// Details for a new or resumed audio stream.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct InitPayload {
    pub stream_id: String,
    pub lang: Option<String>,
    #[serde(rename = "content_type")]
    pub content_type: Option<String>,
    pub sample_rate: Option<u32>,
    #[serde(default)]
    pub extras: Map<String, Value>,
}

/// Audio append event containing base64-encoded PCM/WAV data.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct AudioPayload {
    pub stream_id: String,
    pub seq: u64,
    pub payload_b64: String,
}

/// Marks the upstream boundary for a chunk.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CommitPayload {
    pub stream_id: String,
    pub chunk_id: String,
}

/// Aborts the active stream.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct CancelPayload {
    pub stream_id: String,
}

/// Finalised transcript text forwarded to the refinement tier.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct FinalTextPayload {
    pub stream_id: String,
    pub chunk_id: String,
    pub text: String,
    #[serde(default)]
    pub segments: Vec<TranscriptSegment>,
    #[serde(default)]
    pub confidence: Option<f32>,
}

/// Outbound server messages.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum ServerMessage {
    Partial(PartialHypothesis),
    Final(FinalHypothesis),
    Refine(RefinementMessage),
    Stats(StatsMessage),
    #[serde(rename = "error")]
    Error(ErrorMessage),
}

impl ServerMessage {
    /// Helper to construct a partial hypothesis response.
    pub fn partial<S, T>(
        stream_id: S,
        seq: u64,
        text: T,
        avg_logprob: f32,
        segments: Vec<TranscriptSegment>,
    ) -> Self
    where
        S: Into<String>,
        T: Into<String>,
    {
        Self::Partial(PartialHypothesis {
            stream_id: stream_id.into(),
            seq,
            text: text.into(),
            avg_logprob,
            segments,
        })
    }

    /// Helper to construct an error response for the websocket client.
    pub fn error<E: Into<String>>(stream_id: Option<String>, message: E) -> Self {
        Self::Error(ErrorMessage {
            stream_id,
            message: message.into(),
        })
    }
}

/// Partial recognition result emitted by the fast tier.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct PartialHypothesis {
    pub stream_id: String,
    pub seq: u64,
    pub text: String,
    pub avg_logprob: f32,
    pub segments: Vec<TranscriptSegment>,
}

/// Final recognition result for a chunk.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct FinalHypothesis {
    pub stream_id: String,
    pub chunk_id: String,
    pub text: String,
    pub segments: Vec<TranscriptSegment>,
    #[serde(default)]
    pub confidence: Option<f32>,
    #[serde(default)]
    pub tier: Option<String>,
}

/// Rolling refinement over the recent transcript window.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct RefinementMessage {
    pub window_sec: u32,
    pub text: String,
    pub segments: Vec<TranscriptSegment>,
    #[serde(default)]
    pub notes: RefinementNotes,
}

/// Metadata about the refinement adjustments.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Default)]
pub struct RefinementNotes {
    #[serde(default)]
    pub disfluencies_removed: bool,
    #[serde(default)]
    pub punctuation_enhanced: bool,
}

/// Server statistics suitable for monitoring.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct StatsMessage {
    pub rtf: f32,
    pub queue_len: usize,
    pub gpu: bool,
}

/// Error payload forwarded to websocket clients.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ErrorMessage {
    pub stream_id: Option<String>,
    pub message: String,
}

/// Transcript segment with start/end times and optional speaker tag.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct TranscriptSegment {
    #[serde(rename = "t0")]
    pub start: f32,
    #[serde(rename = "t1")]
    pub end: f32,
    pub text: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    #[serde(default)]
    pub speaker: Option<String>,
}

impl From<AsrError> for ServerMessage {
    fn from(err: AsrError) -> Self {
        ServerMessage::error(None, err.to_string())
    }
}
