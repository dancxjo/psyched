use std::collections::HashMap;
use std::time::Duration;

use async_trait::async_trait;
use parking_lot::Mutex;

use asr_core::errors::AsrError;
use asr_core::messages::{ClientMessage, FinalHypothesis, ServerMessage, TranscriptSegment};
use asr_core::pipeline::{
    decode_audio_payload, Pipeline as PipelineTrait, StreamConfig, StreamRegistry,
};

/// Configuration knobs for the fast tier pipeline.
#[derive(Debug, Clone)]
pub struct FastPipelineConfig {
    pub ring_buffer_seconds: f32,
    pub partial_interval: Duration,
}

impl Default for FastPipelineConfig {
    fn default() -> Self {
        Self {
            ring_buffer_seconds: 12.0,
            partial_interval: Duration::from_millis(200),
        }
    }
}

/// Fast tier pipeline implementation.
pub struct FastPipeline {
    config: FastPipelineConfig,
    registry: StreamRegistry,
    sessions: Mutex<HashMap<String, Session>>, // keyed by stream id
}

impl FastPipeline {
    pub fn new(config: FastPipelineConfig) -> Self {
        Self {
            config,
            registry: StreamRegistry::default(),
            sessions: Mutex::new(HashMap::new()),
        }
    }
}

#[async_trait]
impl PipelineTrait for FastPipeline {
    async fn handle(&self, message: ClientMessage) -> Result<Vec<ServerMessage>, AsrError> {
        match message {
            ClientMessage::Init(payload) => {
                let config = StreamConfig::from_init(&payload)?;
                let session = Session::new(config.clone(), &self.config);
                self.registry.insert(config.clone());
                self.sessions
                    .lock()
                    .insert(config.stream_id.clone(), session);
                Ok(Vec::new())
            }
            ClientMessage::Audio(payload) => {
                let samples = decode_audio_payload(&self.registry, &payload)?;
                let mut sessions = self.sessions.lock();
                let session = sessions
                    .get_mut(&payload.stream_id)
                    .ok_or_else(|| AsrError::StreamNotInitialised(payload.stream_id.clone()))?;
                session.ingest(payload.seq, samples)
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
    ring_buffer: Vec<i16>,
    ring_capacity: usize,
    current_chunk: Vec<i16>,
    chunk_start_sample: usize,
    total_samples: usize,
    samples_since_partial: usize,
    partial_interval_samples: usize,
    last_seq: Option<u64>,
}

impl Session {
    fn new(config: StreamConfig, pipeline_config: &FastPipelineConfig) -> Self {
        let ring_capacity =
            (config.sample_rate as f32 * pipeline_config.ring_buffer_seconds).ceil() as usize;
        let partial_interval_samples = ((config.sample_rate as f32)
            * (pipeline_config.partial_interval.as_secs_f32().max(0.001)))
        .ceil() as usize;
        Self {
            config,
            ring_buffer: Vec::with_capacity(ring_capacity),
            ring_capacity,
            current_chunk: Vec::new(),
            chunk_start_sample: 0,
            total_samples: 0,
            samples_since_partial: partial_interval_samples, // emit first partial immediately
            partial_interval_samples: partial_interval_samples.max(1),
            last_seq: None,
        }
    }

    fn ingest(&mut self, seq: u64, samples: Vec<i16>) -> Result<Vec<ServerMessage>, AsrError> {
        if let Some(last) = self.last_seq {
            if seq <= last {
                return Err(AsrError::Decoder(format!(
                    "out-of-order audio sequence: {} <= {}",
                    seq, last
                )));
            }
        }
        self.last_seq = Some(seq);
        self.total_samples += samples.len();
        self.samples_since_partial += samples.len();
        self.current_chunk.extend_from_slice(&samples);
        self.extend_ring(&samples);

        if self.samples_since_partial < self.partial_interval_samples
            && self.current_chunk.len() > samples.len()
        {
            return Ok(Vec::new());
        }
        self.samples_since_partial = 0;
        let (text, segment) = self.describe_current_chunk();
        Ok(vec![ServerMessage::partial(
            &self.config.stream_id,
            seq,
            text,
            -0.12,
            vec![segment],
        )])
    }

    fn commit(&mut self, chunk_id: String) -> Result<Vec<ServerMessage>, AsrError> {
        if self.current_chunk.is_empty() {
            return Ok(Vec::new());
        }
        let (text, segment) = self.describe_current_chunk();
        let message = ServerMessage::Final(FinalHypothesis {
            stream_id: self.config.stream_id.clone(),
            chunk_id,
            text,
            segments: vec![segment],
            confidence: Some(0.5),
            tier: Some("fast".into()),
        });
        self.chunk_start_sample = self.total_samples;
        self.current_chunk.clear();
        self.samples_since_partial = self.partial_interval_samples;
        Ok(vec![message])
    }

    fn extend_ring(&mut self, samples: &[i16]) {
        self.ring_buffer.extend_from_slice(samples);
        if self.ring_buffer.len() > self.ring_capacity {
            let overflow = self.ring_buffer.len() - self.ring_capacity;
            self.ring_buffer.drain(0..overflow);
        }
    }

    fn describe_current_chunk(&self) -> (String, TranscriptSegment) {
        let duration = self.current_chunk.len() as f32 / self.config.sample_rate as f32;
        let start_time = self.chunk_start_sample as f32 / self.config.sample_rate as f32;
        let end_time = start_time + duration;
        let summary = summarise_samples(&self.current_chunk);
        let segment = TranscriptSegment {
            start: start_time,
            end: end_time,
            text: summary.clone(),
            speaker: None,
        };
        (summary, segment)
    }
}

fn summarise_samples(samples: &[i16]) -> String {
    let sum: i64 = samples.iter().map(|&sample| sample as i64).sum();
    format!("samples={} sum={}", samples.len(), sum)
}

#[cfg(test)]
mod tests {
    use super::*;
    use asr_core::messages::{AudioPayload, InitPayload};

    fn make_payload(seq: u64, samples: &[i16]) -> AudioPayload {
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
    async fn emits_partial_after_audio() {
        let pipeline = FastPipeline::new(FastPipelineConfig {
            ring_buffer_seconds: 4.0,
            partial_interval: Duration::from_millis(1),
        });
        pipeline.handle(init_message()).await.unwrap();
        let responses = pipeline
            .handle(ClientMessage::Audio(make_payload(1, &[0, 1, 2, 3])))
            .await
            .unwrap();
        assert_eq!(responses.len(), 1);
        match &responses[0] {
            ServerMessage::Partial(partial) => {
                assert_eq!(partial.stream_id, "demo");
                assert_eq!(partial.text, "samples=4 sum=6");
            }
            other => panic!("expected partial, got {other:?}"),
        }
    }

    #[tokio::test]
    async fn emits_final_on_commit() {
        let pipeline = FastPipeline::new(FastPipelineConfig::default());
        pipeline.handle(init_message()).await.unwrap();
        pipeline
            .handle(ClientMessage::Audio(make_payload(1, &[1, 1, 1, 1])))
            .await
            .unwrap();
        let responses = pipeline
            .handle(ClientMessage::Commit(asr_core::messages::CommitPayload {
                stream_id: "demo".into(),
                chunk_id: "chunk-1".into(),
            }))
            .await
            .unwrap();
        assert_eq!(responses.len(), 1);
        match &responses[0] {
            ServerMessage::Final(final_msg) => {
                assert_eq!(final_msg.chunk_id, "chunk-1");
                assert_eq!(final_msg.text, "samples=4 sum=4");
            }
            other => panic!("expected final, got {other:?}"),
        }
    }
}
