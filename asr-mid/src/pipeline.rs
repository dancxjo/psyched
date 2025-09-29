use std::collections::{HashMap, VecDeque};
use std::sync::Arc;
use std::time::{Duration, Instant};

use async_trait::async_trait;
use parking_lot::Mutex;

use asr_core::errors::AsrError;
use asr_core::messages::{ClientMessage, FinalHypothesis, ServerMessage, TranscriptSegment};
use asr_core::pipeline::{
    decode_audio_payload, Pipeline as PipelineTrait, StreamConfig, StreamRegistry,
};
use asr_core::recognizer::SpeechRecognizer;

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
    recognizer: Arc<dyn SpeechRecognizer>,
}

impl MidPipeline {
    pub fn new(config: MidPipelineConfig, recognizer: Arc<dyn SpeechRecognizer>) -> Self {
        Self {
            config,
            registry: StreamRegistry::default(),
            sessions: Mutex::new(HashMap::new()),
            recognizer,
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
                self.sessions.lock().insert(
                    config.stream_id.clone(),
                    Session::new(config, &self.config, Arc::clone(&self.recognizer)),
                );
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
    recognizer: Arc<dyn SpeechRecognizer>,
}

impl Session {
    fn new(
        config: StreamConfig,
        pipeline_config: &MidPipelineConfig,
        recognizer: Arc<dyn SpeechRecognizer>,
    ) -> Self {
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
            recognizer,
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
        let prompt = self.prompt_text();
        let recognition = self.recognizer.transcribe(
            &self.current_chunk,
            self.config.sample_rate,
            prompt.as_deref(),
        )?;
        let chunk_start = self.chunk_start_sample as f32 / self.config.sample_rate as f32;
        let mut segments = Vec::new();
        for segment in &recognition.segments {
            segments.push(TranscriptSegment {
                start: chunk_start + segment.start,
                end: chunk_start + segment.end,
                text: segment.text.clone(),
                speaker: None,
            });
        }
        if segments.is_empty() {
            let duration = self.current_chunk.len() as f32 / self.config.sample_rate as f32;
            segments.push(TranscriptSegment {
                start: chunk_start,
                end: chunk_start + duration,
                text: recognition.text.clone(),
                speaker: None,
            });
        }
        self.remember_prompt(recognition.text.clone());
        let confidence = self.combine_confidence(recognition.confidence());
        let message = ServerMessage::Final(FinalHypothesis {
            stream_id: self.config.stream_id.clone(),
            chunk_id,
            text: recognition.text,
            segments,
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

    fn prompt_text(&self) -> Option<String> {
        if self.prompt_window == 0 || self.prompt_history.is_empty() {
            return None;
        }
        let combined = self
            .prompt_history
            .iter()
            .cloned()
            .collect::<Vec<_>>()
            .join(" ");
        if combined.trim().is_empty() {
            None
        } else {
            Some(combined)
        }
    }

    fn remember_prompt(&mut self, utterance: String) {
        if self.prompt_window == 0 {
            return;
        }
        if utterance.trim().is_empty() {
            return;
        }
        self.prompt_history.push_back(utterance);
        while self.prompt_history.len() > self.prompt_window {
            self.prompt_history.pop_front();
        }
    }

    fn combine_confidence(&self, recognition_conf: f32) -> f32 {
        let timing_conf = self.estimate_confidence();
        ((recognition_conf + timing_conf) / 2.0).clamp(0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use asr_core::messages::{AudioPayload, CommitPayload, InitPayload};
    use asr_core::recognizer::{RecognizedSegment, RecognizedTranscript, RecognizerError};
    use std::sync::Arc;

    #[derive(Default)]
    struct RecordingRecognizer {
        prompts: Mutex<Vec<Option<String>>>,
    }

    impl RecordingRecognizer {
        fn prompts(&self) -> Vec<Option<String>> {
            self.prompts.lock().clone()
        }
    }

    impl SpeechRecognizer for RecordingRecognizer {
        fn transcribe(
            &self,
            pcm_samples: &[i16],
            sample_rate: u32,
            prompt: Option<&str>,
        ) -> Result<RecognizedTranscript, RecognizerError> {
            assert_eq!(sample_rate, 16_000);
            self.prompts
                .lock()
                .push(prompt.map(|value| value.to_string()));
            let text = format!("mid decoded {}", pcm_samples.len());
            let duration = pcm_samples.len() as f32 / sample_rate as f32;
            Ok(RecognizedTranscript {
                text: text.clone(),
                segments: vec![RecognizedSegment {
                    start: 0.0,
                    end: duration,
                    text,
                    avg_logprob: -0.05,
                }],
                avg_logprob: -0.05,
            })
        }
    }

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
        let recognizer = Arc::new(RecordingRecognizer::default());
        let pipeline = MidPipeline::new(
            MidPipelineConfig::default(),
            recognizer.clone() as Arc<dyn SpeechRecognizer>,
        );
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
                assert_eq!(final_msg.text, "mid decoded 4");
                assert_eq!(final_msg.tier.as_deref(), Some("mid"));
                assert!(final_msg.confidence.unwrap() > 0.0);
            }
            other => panic!("expected final, got {other:?}"),
        }
        assert_eq!(recognizer.prompts(), vec![None]);
    }

    #[tokio::test]
    async fn reuses_prompt_history_for_follow_up_chunks() {
        let recognizer = Arc::new(RecordingRecognizer::default());
        let pipeline = MidPipeline::new(
            MidPipelineConfig {
                prompt_window: 2,
                commit_timeout: Duration::from_secs(2),
            },
            recognizer.clone() as Arc<dyn SpeechRecognizer>,
        );
        pipeline.handle(init_message()).await.unwrap();
        pipeline
            .handle(ClientMessage::Audio(audio_payload(1, &[1, 1, 1, 1])))
            .await
            .unwrap();
        pipeline
            .handle(ClientMessage::Commit(CommitPayload {
                stream_id: "demo".into(),
                chunk_id: "chunk-1".into(),
            }))
            .await
            .unwrap();

        pipeline
            .handle(ClientMessage::Audio(audio_payload(2, &[3, 3, 3, 3])))
            .await
            .unwrap();
        pipeline
            .handle(ClientMessage::Commit(CommitPayload {
                stream_id: "demo".into(),
                chunk_id: "chunk-2".into(),
            }))
            .await
            .unwrap();

        let prompts = recognizer.prompts();
        assert_eq!(prompts.len(), 2);
        assert_eq!(prompts[0], None);
        assert_eq!(prompts[1].as_deref(), Some("mid decoded 4"));
    }
}
