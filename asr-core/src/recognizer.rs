use std::num::NonZeroUsize;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use thiserror::Error;
use whisper_rs::{
    convert_integer_to_float_audio, FullParams, SamplingStrategy, WhisperContext,
    WhisperContextParameters, WhisperError,
};

use crate::errors::AsrError;

/// Description of an individual segment returned by a recognizer.
#[derive(Debug, Clone, PartialEq)]
pub struct RecognizedSegment {
    /// Start timestamp relative to the analysed audio chunk, in seconds.
    pub start: f32,
    /// End timestamp relative to the analysed audio chunk, in seconds.
    pub end: f32,
    /// Recognised text for the segment.
    pub text: String,
    /// Average log probability reported by the decoder for the segment.
    pub avg_logprob: f32,
}

/// Consolidated transcript emitted by a speech recognizer.
#[derive(Debug, Clone, PartialEq)]
pub struct RecognizedTranscript {
    /// Full concatenated text for the analysed audio chunk.
    pub text: String,
    /// Segment-level alignments for the recognition.
    pub segments: Vec<RecognizedSegment>,
    /// Average log probability across all recognised segments.
    pub avg_logprob: f32,
}

impl RecognizedTranscript {
    /// Convert the average log probability into a ``0.0..=1.0`` confidence score.
    pub fn confidence(&self) -> f32 {
        let probability = self.avg_logprob.exp();
        if probability.is_nan() || !probability.is_finite() {
            return 0.0;
        }
        probability.clamp(0.0, 1.0)
    }
}

/// Errors produced while initialising or running a speech recognizer.
#[derive(Debug, Error)]
pub enum RecognizerError {
    #[error("whisper initialisation failed: {0}")]
    Init(WhisperError),
    #[error("whisper inference failed: {0}")]
    Inference(WhisperError),
    #[error("model file '{0}' does not exist")]
    MissingModel(PathBuf),
    #[error("unsupported sample rate {sample_rate} Hz (expected 16000 Hz)")]
    UnsupportedSampleRate { sample_rate: u32 },
    #[error("recognition failed: {0}")]
    Recognition(String),
}

impl From<RecognizerError> for AsrError {
    fn from(value: RecognizerError) -> Self {
        AsrError::Decoder(value.to_string())
    }
}

/// Object-safe speech recognizer interface shared by the ASR tiers.
pub trait SpeechRecognizer: Send + Sync {
    /// Transcribe a chunk of PCM audio.
    fn transcribe(
        &self,
        pcm_samples: &[i16],
        sample_rate: u32,
        prompt: Option<&str>,
    ) -> Result<RecognizedTranscript, RecognizerError>;
}

/// Whisper.cpp-backed recognizer used by the fast and mid tiers.
pub struct WhisperRecognizer {
    context: Arc<WhisperContext>,
    language: Option<String>,
    threads: NonZeroUsize,
}

impl WhisperRecognizer {
    /// Load a whisper.cpp model from ``model_path``.
    pub fn new<P: AsRef<Path>>(
        model_path: P,
        language: Option<String>,
        threads: NonZeroUsize,
    ) -> Result<Self, RecognizerError> {
        let model_path = model_path.as_ref();
        if !model_path.exists() {
            return Err(RecognizerError::MissingModel(model_path.to_path_buf()));
        }
        let context = WhisperContext::new_with_params(
            model_path.to_str().ok_or_else(|| {
                RecognizerError::Recognition("model path contains invalid unicode".into())
            })?,
            WhisperContextParameters::default(),
        )
        .map_err(RecognizerError::Init)?;
        Ok(Self {
            context: Arc::new(context),
            language,
            threads,
        })
    }
}

impl SpeechRecognizer for WhisperRecognizer {
    fn transcribe(
        &self,
        pcm_samples: &[i16],
        sample_rate: u32,
        prompt: Option<&str>,
    ) -> Result<RecognizedTranscript, RecognizerError> {
        if pcm_samples.is_empty() {
            return Ok(RecognizedTranscript {
                text: String::new(),
                segments: Vec::new(),
                avg_logprob: -10.0,
            });
        }
        if sample_rate != 16_000 {
            return Err(RecognizerError::UnsupportedSampleRate { sample_rate });
        }

        let mut state = self
            .context
            .create_state()
            .map_err(RecognizerError::Inference)?;

        let mut params = FullParams::new(SamplingStrategy::Greedy { best_of: 1 });
        params.set_n_threads(self.threads.get() as i32);
        params.set_translate(false);
        params.set_print_progress(false);
        params.set_print_realtime(false);
        params.set_print_timestamps(false);
        params.set_no_timestamps(false);
        params.set_single_segment(false);
        params.set_suppress_blank(true);
        params.set_suppress_nst(true);
        params.set_no_context(prompt.is_none());
        if let Some(lang) = self.language.as_deref() {
            params.set_language(Some(lang));
        }
        if let Some(prompt_text) = prompt {
            if !prompt_text.trim().is_empty() {
                params.set_initial_prompt(prompt_text);
                params.set_no_context(false);
            }
        }

        let mut audio = vec![0.0f32; pcm_samples.len()];
        convert_integer_to_float_audio(pcm_samples, &mut audio)
            .map_err(RecognizerError::Inference)?;
        state
            .full(params, &audio)
            .map_err(RecognizerError::Inference)?;

        let mut segments = Vec::new();
        let mut concatenated = String::new();
        let mut logprob_sum = 0.0f32;
        let mut count = 0f32;

        for segment in state.as_iter() {
            let owned_text = segment
                .to_str_lossy()
                .map_err(RecognizerError::Inference)?
                .into_owned();
            let trimmed = owned_text.trim();
            let confidence = (1.0 - segment.no_speech_probability()).clamp(0.0, 1.0);
            let logprob = if confidence <= f32::EPSILON {
                -10.0
            } else {
                confidence.ln()
            };
            if !trimmed.is_empty() {
                if !concatenated.is_empty() {
                    concatenated.push(' ');
                }
                concatenated.push_str(trimmed);
            }
            let start = segment.start_timestamp() as f32 * 0.01;
            let end = segment.end_timestamp() as f32 * 0.01;
            logprob_sum += logprob;
            count += 1.0;
            segments.push(RecognizedSegment {
                start,
                end,
                text: trimmed.to_string(),
                avg_logprob: logprob,
            });
        }

        let avg_logprob = if count > 0.0 {
            logprob_sum / count
        } else {
            -10.0
        };

        if segments.is_empty() && !concatenated.is_empty() {
            segments.push(RecognizedSegment {
                start: 0.0,
                end: pcm_samples.len() as f32 / sample_rate as f32,
                text: concatenated.clone(),
                avg_logprob,
            });
        }

        Ok(RecognizedTranscript {
            text: concatenated,
            segments,
            avg_logprob,
        })
    }
}
