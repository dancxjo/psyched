use std::collections::{HashMap, VecDeque};
use std::time::{Duration, Instant};

use async_trait::async_trait;
use parking_lot::Mutex;

use asr_core::errors::AsrError;
use asr_core::messages::{ClientMessage, FinalHypothesis, ServerMessage, TranscriptSegment};
use asr_core::pipeline::{
    decode_audio_payload, Pipeline as PipelineTrait, StreamConfig, StreamRegistry,
};

/// Configuration for the medium-latency pipeline.
#[derive(Debug, Clone)]
pub struct MidPipelineConfig {
    pub prompt_window: usize,
    pub commit_timeout: Duration,
}

impl Default for MidPipelineConfig {
    fn default() -> Self {
        Self {
            prompt_window: 3,
            commit_timeout: Duration::from_secs(2),
        }
    }
}

pub struct MidPipeline {
    config: MidPipelineConfig,
    registry: StreamRegistry,
    sessions: Mutex<HashMap<String, Session>>,
}

impl MidPipeline {
    pub fn new(config: MidPipelineConfig) -> Self {
        Self {
            config,
            registry: StreamRegistry::default(),
            sessions: Mutex::new(HashMap::new()),
        }
    }
}

#[async_trait]
impl PipelineTrait for MidPipeline {
    async fn handle(&self, message: ClientMessage) -> Result<Vec<ServerMessage>, AsrError> {
        match message {
            ClientMessage::Init(payload) => {
                let config = StreamConfig::from_init(&payload)?;
                self.registry.insert(config.clone());
                self.sessions
                    .lock()
                    .insert(config.stream_id.clone(), Session::new(config, &self.config));
                Ok(Vec::new())
            }
            ClientMessage::Audio(payload) => {
                let samples = decode_audio_payload(&self.registry, &payload)?;
                let mut sessions = self.sessions.lock();
                let session = sessions
                    .get_mut(&payload.stream_id)
                    .ok_or_else(|| AsrError::StreamNotInitialised(payload.stream_id.clone()))?;
                session.append_audio(payload.seq, samples);
                Ok(Vec::new())
            }
            ClientMessage::Commit(payload) => {
                let mut sessions = self.sessions.lock();
                let session = sessions
                    .get_mut(&payload.stream_id)
                    .ok_or_else(|| AsrError::StreamNotInitialised(payload.stream_id.clone()))?;
                session.commit(payload.chunk_id)
            }
            ClientMessage::Cancel(payload) => {
                self.sessions.lock().remove(&payload.stream_id);
                self.registry.remove(&payload.stream_id);
                Ok(Vec::new())
            }
            ClientMessage::FinalText(_) => Ok(Vec::new()),
        }
    }
}

struct Session {
    config: StreamConfig,
    current_chunk: Vec<i16>,
    last_seq: Option<u64>,
    chunk_start_sample: usize,
    total_samples: usize,
    prompt_window: usize,
    prompt_history: VecDeque<String>,
    commit_timeout: Duration,
    last_audio_at: Option<Instant>,
}

impl Session {
    fn new(config: StreamConfig, pipeline_config: &MidPipelineConfig) -> Self {
        let prompt_window = pipeline_config.prompt_window;
        Self {
            config,
            current_chunk: Vec::new(),
            last_seq: None,
            chunk_start_sample: 0,
            total_samples: 0,
            prompt_window,
            prompt_history: VecDeque::with_capacity(prompt_window.max(1)),
            commit_timeout: pipeline_config.commit_timeout,
            last_audio_at: None,
        }
    }

    fn append_audio(&mut self, seq: u64, samples: Vec<i16>) {
        if let Some(last) = self.last_seq {
            if seq <= last {
                return;
            }
        }
        self.last_seq = Some(seq);
        self.total_samples += samples.len();
        self.current_chunk.extend_from_slice(&samples);
        self.last_audio_at = Some(Instant::now());
    }

    fn commit(&mut self, chunk_id: String) -> Result<Vec<ServerMessage>, AsrError> {
        if self.current_chunk.is_empty() {
            return Ok(Vec::new());
        }
        let start_time = self.chunk_start_sample as f32 / self.config.sample_rate as f32;
        let duration = self.current_chunk.len() as f32 / self.config.sample_rate as f32;
        let end_time = start_time + duration;
        let summary = summarise_samples(&self.current_chunk);
        self.prompt_history.push_back(summary.clone());
        if self.prompt_window > 0 {
            while self.prompt_history.len() > self.prompt_window {
                self.prompt_history.pop_front();
            }
        }
        let segment = TranscriptSegment {
            start: start_time,
            end: end_time,
            text: summary.clone(),
            speaker: None,
        };
        let confidence = self.estimate_confidence();
        let message = ServerMessage::Final(FinalHypothesis {
            stream_id: self.config.stream_id.clone(),
            chunk_id,
            text: summary,
            segments: vec![segment],
            confidence: Some(confidence),
            tier: Some("mid".into()),
        });
        self.chunk_start_sample = self.total_samples;
        self.current_chunk.clear();
        self.last_audio_at = None;
        Ok(vec![message])
    }

    fn estimate_confidence(&self) -> f32 {
        let timeout = self
            .commit_timeout
            .max(Duration::from_millis(1))
            .as_secs_f32();
        let time_factor = self
            .last_audio_at
            .map(|instant| {
                let elapsed = instant.elapsed().as_secs_f32();
                1.0 - (elapsed / timeout).clamp(0.0, 1.0)
            })
            .unwrap_or(1.0);
        let history_factor = if self.prompt_window == 0 {
            0.0
        } else {
            (self.prompt_history.len().min(self.prompt_window) as f32 / self.prompt_window as f32)
                .clamp(0.0, 1.0)
        };
        (0.6 + 0.25 * time_factor + 0.1 * history_factor).min(0.95)
    }
}

fn summarise_samples(samples: &[i16]) -> String {
    let sum: i64 = samples.iter().map(|&sample| sample as i64).sum();
    format!("samples={} sum={}", samples.len(), sum)
}

#[cfg(test)]
mod tests {
    use super::*;
    use asr_core::messages::{AudioPayload, CommitPayload, InitPayload};

    fn init_message() -> ClientMessage {
        ClientMessage::Init(InitPayload {
            stream_id: "demo".into(),
            lang: Some("en".into()),
            content_type: Some("audio/pcm; rate=16000".into()),
            sample_rate: Some(16_000),
            extras: Default::default(),
        })
    }

    fn audio_payload(seq: u64, samples: &[i16]) -> AudioPayload {
        use base64::{engine::general_purpose, Engine as _};
        let mut bytes = Vec::with_capacity(samples.len() * 2);
        for sample in samples {
            bytes.extend_from_slice(&sample.to_le_bytes());
        }
        AudioPayload {
            stream_id: "demo".into(),
            seq,
            payload_b64: general_purpose::STANDARD.encode(bytes),
        }
    }

    #[tokio::test]
    async fn emits_final_when_committed() {
        let pipeline = MidPipeline::new(MidPipelineConfig::default());
        pipeline.handle(init_message()).await.unwrap();
        pipeline
            .handle(ClientMessage::Audio(audio_payload(1, &[2, 2, 2, 2])))
            .await
            .unwrap();
        let responses = pipeline
            .handle(ClientMessage::Commit(CommitPayload {
                stream_id: "demo".into(),
                chunk_id: "chunk-2".into(),
            }))
            .await
            .unwrap();
        assert_eq!(responses.len(), 1);
        match &responses[0] {
            ServerMessage::Final(final_msg) => {
                assert_eq!(final_msg.text, "samples=4 sum=8");
                assert_eq!(final_msg.tier.as_deref(), Some("mid"));
            }
            other => panic!("expected final, got {other:?}"),
        }
    }
}
