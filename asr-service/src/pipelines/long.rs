use std::collections::{HashMap, VecDeque};
use std::time::Duration;

use async_trait::async_trait;
use parking_lot::Mutex;

use crate::core::errors::AsrError;
use crate::core::messages::{
    ClientMessage, FinalTextPayload, RefinementMessage, RefinementNotes, ServerMessage,
    TranscriptSegment,
};
use crate::core::pipeline::{Pipeline as PipelineTrait, StreamConfig, StreamRegistry};

/// Configuration for the long-context refinement pipeline.
#[derive(Debug, Clone)]
pub struct LongPipelineConfig {
    pub window_seconds: u32,
    pub fallback_chunk_duration: Duration,
}

impl Default for LongPipelineConfig {
    fn default() -> Self {
        Self {
            window_seconds: 120,
            fallback_chunk_duration: Duration::from_secs(5),
        }
    }
}

pub struct LongPipeline {
    config: LongPipelineConfig,
    registry: StreamRegistry,
    sessions: Mutex<HashMap<String, Session>>,
}

impl LongPipeline {
    pub fn new(config: LongPipelineConfig) -> Self {
        Self {
            config,
            registry: StreamRegistry::default(),
            sessions: Mutex::new(HashMap::new()),
        }
    }
}

#[async_trait]
impl PipelineTrait for LongPipeline {
    async fn handle(&self, message: ClientMessage) -> Result<Vec<ServerMessage>, AsrError> {
        match message {
            ClientMessage::Init(payload) => {
                let config = StreamConfig::from_init(&payload)?;
                self.registry.insert(config.clone());
                self.sessions
                    .lock()
                    .insert(config.stream_id.clone(), Session::new(config));
                Ok(Vec::new())
            }
            ClientMessage::FinalText(payload) => {
                let mut sessions = self.sessions.lock();
                let session = sessions
                    .get_mut(&payload.stream_id)
                    .ok_or_else(|| AsrError::StreamNotInitialised(payload.stream_id.clone()))?;
                session.integrate_final_text(payload, &self.config)
            }
            ClientMessage::Cancel(payload) => {
                self.sessions.lock().remove(&payload.stream_id);
                self.registry.remove(&payload.stream_id);
                Ok(Vec::new())
            }
            // Audio-bearing messages are ignored in the refinement tier.
            ClientMessage::Audio(_) | ClientMessage::Commit(_) => Ok(Vec::new()),
        }
    }
}

struct Session {
    utterances: VecDeque<Utterance>,
}

impl Session {
    fn new(_config: StreamConfig) -> Self {
        Self {
            utterances: VecDeque::new(),
        }
    }

    fn integrate_final_text(
        &mut self,
        payload: FinalTextPayload,
        config: &LongPipelineConfig,
    ) -> Result<Vec<ServerMessage>, AsrError> {
        let segments = if payload.segments.is_empty() {
            let default_duration = config.fallback_chunk_duration.as_secs_f32();
            let end_time = self
                .utterances
                .back()
                .map(|utterance| utterance.end_time + default_duration)
                .unwrap_or(default_duration);
            let start_time = (end_time - default_duration).max(0.0);
            vec![TranscriptSegment {
                start: start_time,
                end: end_time,
                text: payload.text.clone(),
                speaker: None,
            }]
        } else {
            payload.segments.clone()
        };
        let end_time = segments.last().map(|segment| segment.end).unwrap_or(0.0);
        let utterance = Utterance {
            segments: segments.clone(),
            end_time,
        };
        self.utterances.push_back(utterance);
        self.prune_old_segments(config.window_seconds, end_time);
        if let Some((segments, notes, text)) = self.build_refinement() {
            Ok(vec![ServerMessage::Refine(RefinementMessage {
                window_sec: config.window_seconds,
                text,
                segments,
                notes,
            })])
        } else {
            Ok(Vec::new())
        }
    }

    fn prune_old_segments(&mut self, window_seconds: u32, latest_end: f32) {
        let threshold = latest_end - window_seconds as f32;
        while let Some(front) = self.utterances.front() {
            if front.end_time < threshold {
                self.utterances.pop_front();
            } else {
                break;
            }
        }
    }

    fn build_refinement(&self) -> Option<(Vec<TranscriptSegment>, RefinementNotes, String)> {
        if self.utterances.is_empty() {
            return None;
        }
        let mut notes = RefinementNotes::default();
        let mut refined_segments = Vec::new();
        let mut first_segment = true;
        for utterance in &self.utterances {
            for segment in &utterance.segments {
                let cleaned = clean_phrase(&segment.text, &mut notes, first_segment);
                if cleaned.is_empty() {
                    continue;
                }
                let mut refined = segment.clone();
                refined.text = cleaned;
                refined_segments.push(refined);
                first_segment = false;
            }
        }
        if refined_segments.is_empty() {
            return None;
        }
        if let Some(last) = refined_segments.last_mut() {
            if !last.text.ends_with(['.', '!', '?']) {
                last.text.push('.');
                notes.punctuation_enhanced = true;
            }
        }
        let text = refined_segments
            .iter()
            .map(|segment| segment.text.clone())
            .collect::<Vec<_>>()
            .join(" ");
        Some((refined_segments, notes, text))
    }
}

struct Utterance {
    segments: Vec<TranscriptSegment>,
    end_time: f32,
}

fn clean_phrase(phrase: &str, notes: &mut RefinementNotes, capitalise: bool) -> String {
    let mut words = phrase
        .split_whitespace()
        .map(|word| word.trim_matches(|c: char| c == ',' || c == '.'))
        .collect::<Vec<_>>();
    let original_len = words.len();
    words.retain(|word| {
        let w = word.to_ascii_lowercase();
        !(w == "uh" || w == "um" || w == "erm")
    });
    if words.len() < original_len {
        notes.disfluencies_removed = true;
    }
    if words.is_empty() {
        return String::new();
    }
    let mut text = words.join(" ");
    if capitalise {
        let mut chars = text.chars();
        if let Some(first) = chars.next() {
            let rest = chars.as_str();
            text = format!("{}{}", first.to_uppercase(), rest);
        }
    }
    text
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::messages::{FinalTextPayload, InitPayload};

    fn init_message() -> ClientMessage {
        ClientMessage::Init(InitPayload {
            stream_id: "demo".into(),
            lang: Some("en".into()),
            content_type: Some("audio/pcm; rate=16000".into()),
            sample_rate: Some(16_000),
            extras: Default::default(),
        })
    }

    #[tokio::test]
    async fn refines_text_within_window() {
        let pipeline = LongPipeline::new(LongPipelineConfig {
            window_seconds: 60,
            fallback_chunk_duration: Duration::from_secs(4),
        });
        pipeline.handle(init_message()).await.unwrap();
        let response = pipeline
            .handle(ClientMessage::FinalText(FinalTextPayload {
                stream_id: "demo".into(),
                chunk_id: "chunk-1".into(),
                text: "turn left uh".into(),
                segments: vec![TranscriptSegment {
                    start: 0.0,
                    end: 3.5,
                    text: "turn left uh".into(),
                    speaker: None,
                }],
                confidence: Some(0.8),
            }))
            .await
            .unwrap();
        assert_eq!(response.len(), 1);
        match &response[0] {
            ServerMessage::Refine(refine) => {
                assert_eq!(refine.text, "Turn left.");
                assert!(refine.notes.disfluencies_removed);
                assert!(refine.notes.punctuation_enhanced);
            }
            other => panic!("expected refine message, got {other:?}"),
        }
    }
}
