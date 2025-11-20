use std::collections::VecDeque;
use std::env;
use std::io::Cursor;
use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{anyhow, Context, Result};
use axum::extract::ws::{Message, WebSocket};
use axum::extract::State;
use axum::response::{Html, IntoResponse};
use axum::routing::get;
use axum::{Json, Router};
use base64::engine::general_purpose::STANDARD as BASE64_STANDARD;
use base64::Engine;
use futures::{SinkExt, StreamExt};
use hound::{SampleFormat, WavSpec, WavWriter};
use serde::{Deserialize, Serialize};
use tokio::sync::mpsc;
use tokio::time::{interval, MissedTickBehavior};
use tracing::{error, info, warn};
use whisper_rs::{FullParams, SamplingStrategy, WhisperContext, WhisperContextParameters};

const HARNESS_HTML: &str = include_str!("../static/harness.html");

#[derive(Clone)]
struct WhisperService {
    context: Arc<Mutex<WhisperContext>>,
    sample_rate: u32,
    stability_hits: usize,
    hop: Duration,
    min_duration: Duration,
    finality_lag: Duration,
    silence_threshold: f32,
    silence_duration: Duration,
}

#[derive(Clone, Debug, Serialize)]
struct WordTiming {
    text: String,
    start_ms: u32,
    end_ms: u32,
}

#[derive(Clone, Debug, Serialize)]
struct SegmentMessage {
    text: String,
    start_ms: u32,
    end_ms: u32,
    words: Vec<WordTiming>,
}

#[derive(Debug, Deserialize)]
struct ChunkPayload {
    #[serde(rename = "type")]
    kind: String,
    sample_rate: Option<u32>,
    channels: Option<u32>,
    pcm: Option<String>,
}

#[derive(Debug, Clone)]
struct SegmentInternal {
    text: String,
    start_s: f32,
    end_s: f32,
    words: Vec<WordTiming>,
}

#[derive(Debug, Clone)]
struct TrackedSegment {
    text: String,
    start_s: f32,
    end_s: f32,
    stability: usize,
}

#[derive(Serialize)]
#[serde(tag = "event", rename_all = "snake_case")]
enum OutgoingMessage {
    Ready {
        sample_rate: u32,
    },
    Partial {
        segments: Vec<SegmentMessage>,
    },
    Final {
        text: String,
        start_ms: u32,
        end_ms: u32,
        audio_base64: String,
        segments: Vec<SegmentMessage>,
    },
    Error {
        message: String,
    },
}

#[tokio::main]
async fn main() -> Result<()> {
    init_logging();

    let model_path = env::var("WHISPER_MODEL").context("WHISPER_MODEL env var not set")?;
    let sample_rate: u32 = env::var("ASR_SAMPLE_RATE")
        .unwrap_or_else(|_| "16000".to_string())
        .parse()
        .context("invalid ASR_SAMPLE_RATE")?;
    let stability_hits: usize = env::var("ASR_STABILITY_HITS")
        .unwrap_or_else(|_| "2".to_string())
        .parse()
        .context("invalid ASR_STABILITY_HITS")?;
    let hop_ms: u64 = env::var("ASR_HOP_MS")
        .unwrap_or_else(|_| "750".to_string())
        .parse()
        .context("invalid ASR_HOP_MS")?;
    let min_duration_ms: u64 = env::var("ASR_MIN_DURATION_MS")
        .unwrap_or_else(|_| "2000".to_string())
        .parse()
        .context("invalid ASR_MIN_DURATION_MS")?;
    let finality_lag_ms: u64 = env::var("ASR_FINALITY_LAG_MS")
        .unwrap_or_else(|_| "900".to_string())
        .parse()
        .context("invalid ASR_FINALITY_LAG_MS")?;
    let silence_threshold: f32 = env::var("ASR_SILENCE_THRESHOLD")
        .unwrap_or_else(|_| "0.015".to_string())
        .parse()
        .context("invalid ASR_SILENCE_THRESHOLD")?;
    let silence_duration_ms: u64 = env::var("ASR_SILENCE_DURATION_MS")
        .unwrap_or_else(|_| "1200".to_string())
        .parse()
        .context("invalid ASR_SILENCE_DURATION_MS")?;

    let listen_host = env::var("WEBSOCKET_HOST").unwrap_or_else(|_| "0.0.0.0".to_string());
    let listen_port: u16 = env::var("WEBSOCKET_PORT")
        .unwrap_or_else(|_| "5003".to_string())
        .parse()
        .context("invalid WEBSOCKET_PORT")?;

    info!(model_path = %model_path, "loading whisper model");
    let context = WhisperContext::new_with_params(&model_path, WhisperContextParameters::default())
        .with_context(|| format!("failed to load whisper model from {model_path}"))?;

    let service = Arc::new(WhisperService {
        context: Arc::new(Mutex::new(context)),
        sample_rate,
        stability_hits: stability_hits.max(1),
        hop: Duration::from_millis(hop_ms.max(100)),
        min_duration: Duration::from_millis(min_duration_ms.max(100)),
        finality_lag: Duration::from_millis(finality_lag_ms.min(10_000).max(100)),
        silence_threshold: silence_threshold.clamp(0.0, 1.0),
        silence_duration: Duration::from_millis(silence_duration_ms.min(10_000).max(100)),
    });

    let app = Router::new()
        .route("/", get(harness_page))
        .route("/harness", get(harness_page))
        .route("/asr", get(ws_handler))
        .with_state(service.clone())
        .route(
            "/health",
            get(|| async { Json(serde_json::json!({"status": "ok"})) }),
        );

    let addr: SocketAddr = format!("{}:{}", listen_host, listen_port)
        .parse()
        .context("failed to parse listen address")?;

    let listener = tokio::net::TcpListener::bind(addr)
        .await
        .context("failed to bind TCP listener")?;
    let actual_addr = listener
        .local_addr()
        .context("failed to read local listener address")?;

    info!(address = %actual_addr, sample_rate = service.sample_rate, "starting websocket server");
    axum::serve(listener, app.into_make_service())
        .await
        .context("axum server error")?;

    Ok(())
}

async fn harness_page() -> Html<&'static str> {
    Html(HARNESS_HTML)
}

#[cfg(test)]
mod tests {
    use super::{
        reconcile_segments, SegmentInternal, SilenceTracker, TrackedSegment, WordTiming, HARNESS_HTML,
    };
    use std::time::Duration;

    #[test]
    fn harness_contains_basic_controls() {
        assert!(HARNESS_HTML.contains("ASR Service Harness"));
        assert!(HARNESS_HTML.contains("start-record"));
        assert!(HARNESS_HTML.contains("event-log"));
    }

    fn make_segment(text: &str, start: f32, end: f32) -> SegmentInternal {
        SegmentInternal {
            text: text.to_string(),
            start_s: start,
            end_s: end,
            words: Vec::<WordTiming>::new(),
        }
    }

    #[test]
    fn reconcile_segments_promotes_stable_prefix() {
        let latest = vec![
            make_segment("hello", 0.0, 1.0),
            make_segment("world", 1.0, 2.0),
        ];
        let tracked = vec![
            TrackedSegment {
                text: "hello".to_string(),
                start_s: 0.0,
                end_s: 1.0,
                stability: 2,
            },
            TrackedSegment {
                text: "world".to_string(),
                start_s: 1.0,
                end_s: 2.0,
                stability: 1,
            },
        ];

        let (finalised, remaining) =
            reconcile_segments(&latest, &tracked, 2, 2.5, 0.9, false);

        assert_eq!(finalised.len(), 1);
        assert_eq!(finalised[0].text, "hello");
        assert_eq!(remaining.len(), 1);
        assert_eq!(remaining[0].text, "world");
        assert_eq!(remaining[0].stability, 1);
    }

    #[test]
    fn reconcile_segments_emits_when_audio_has_aged_out() {
        let latest = vec![make_segment("hello there", 0.0, 1.2)];
        let tracked = vec![TrackedSegment {
            text: "hello".to_string(),
            start_s: 0.0,
            end_s: 1.2,
            stability: 1,
        }];

        let (finalised, remaining) =
            reconcile_segments(&latest, &tracked, 4, 5.0, 0.5, false);

        assert_eq!(finalised.len(), 1, "segment should finalize once it ages out");
        assert!(remaining.is_empty(), "no tracked segments expected after finalization");
    }

    #[test]
    fn reconcile_segments_can_force_finalization() {
        let latest = vec![make_segment("forced", 0.0, 0.8)];
        let tracked = vec![TrackedSegment {
            text: "forced".to_string(),
            start_s: 0.0,
            end_s: 0.8,
            stability: 1,
        }];

        let (finalised, remaining) =
            reconcile_segments(&latest, &tracked, 10, 2.0, 5.0, true);

        assert_eq!(finalised.len(), 1);
        assert!(remaining.is_empty());
    }

    #[test]
    fn silence_tracker_detects_required_duration() {
        let mut tracker = SilenceTracker::new(16_000, 0.01, Duration::from_millis(500));
        assert!(!tracker.has_boundary());

        tracker.ingest(0.005, 8_000);
        assert!(tracker.has_boundary());

        tracker.reset();
        tracker.ingest(0.02, 8_000);
        assert!(!tracker.has_boundary());
    }
}

fn init_logging() {
    let env_filter = env::var("LOG_LEVEL").unwrap_or_else(|_| "info".to_string());
    let _ = tracing_subscriber::fmt()
        .with_env_filter(env_filter)
        .with_target(false)
        .try_init();
}

async fn ws_handler(
    State(service): State<Arc<WhisperService>>,
    upgrade: axum::extract::ws::WebSocketUpgrade,
) -> impl IntoResponse {
    upgrade.on_upgrade(move |ws| async move {
        if let Err(err) = handle_socket(service, ws).await {
            error!(error = %err, "websocket handler failed");
        }
    })
}

async fn handle_socket(service: Arc<WhisperService>, socket: WebSocket) -> Result<()> {
    info!("ASR websocket client connected");
    let (mut sender, mut receiver) = socket.split();

    let (pcm_tx, pcm_rx) = mpsc::channel::<Vec<u8>>(64);
    let (out_tx, mut out_rx) = mpsc::channel::<OutgoingMessage>(64);

    out_tx
        .send(OutgoingMessage::Ready {
            sample_rate: service.sample_rate,
        })
        .await
        .ok();

    let send_task = tokio::spawn(async move {
        while let Some(msg) = out_rx.recv().await {
            let payload = match serde_json::to_string(&msg) {
                Ok(payload) => payload,
                Err(err) => {
                    error!(error = %err, "failed to serialize websocket payload");
                    continue;
                }
            };
            if sender.send(Message::Text(payload)).await.is_err() {
                break;
            }
        }
    });

    let processor = tokio::spawn(run_connection(service.clone(), pcm_rx, out_tx));
    let mut warned_sample_rate = false;
    let mut warned_channels = false;

    while let Some(Ok(message)) = receiver.next().await {
        match message {
            Message::Binary(data) => {
                if pcm_tx.send(data).await.is_err() {
                    break;
                }
            }
            Message::Close(_) => {
                break;
            }
            Message::Text(text) => {
                let trimmed = text.trim();
                if trimmed.is_empty() {
                    continue;
                }
                match serde_json::from_str::<ChunkPayload>(trimmed) {
                    Ok(chunk) if chunk.kind.eq_ignore_ascii_case("chunk") => {
                        if let Some(rate) = chunk.sample_rate {
                            if rate != service.sample_rate && !warned_sample_rate {
                                warn!(
                                    expected = service.sample_rate,
                                    received = rate,
                                    "client sample rate mismatches service configuration"
                                );
                                warned_sample_rate = true;
                            }
                        }
                        if let Some(channels) = chunk.channels {
                            if channels != 1 && !warned_channels {
                                warn!(
                                    channels = channels,
                                    "client reported unsupported channel count; assuming mono audio"
                                );
                                warned_channels = true;
                            }
                        }
                        let Some(pcm_b64) = chunk.pcm.as_deref() else {
                            warn!("chunk payload missing pcm field");
                            continue;
                        };
                        let pcm_bytes = match BASE64_STANDARD.decode(pcm_b64.trim().as_bytes()) {
                            Ok(bytes) => bytes,
                            Err(error) => {
                                warn!(%error, "failed to decode base64 pcm payload");
                                continue;
                            }
                        };
                        if pcm_bytes.is_empty() {
                            continue;
                        }
                        if pcm_tx.send(pcm_bytes).await.is_err() {
                            break;
                        }
                    }
                    Ok(chunk) => {
                        warn!(kind = %chunk.kind, "unsupported text frame received from client");
                    }
                    Err(error) => {
                        warn!(%error, payload = %trimmed, "failed to parse text frame from client");
                    }
                }
            }
            Message::Ping(_) | Message::Pong(_) => {}
        }
    }

    drop(pcm_tx);
    if let Err(err) = processor.await {
        error!(error = %err, "processor task join failed");
    }
    send_task.abort();
    info!("ASR websocket client disconnected");
    Ok(())
}

async fn run_connection(
    service: Arc<WhisperService>,
    mut pcm_rx: mpsc::Receiver<Vec<u8>>,
    out_tx: mpsc::Sender<OutgoingMessage>,
) -> Result<()> {
    let mut buffer = VecDeque::<f32>::new();
    let mut tracked: Vec<TrackedSegment> = Vec::new();
    let mut total_consumed_samples: usize = 0;
    let sample_rate = service.sample_rate as f32;
    let mut silence_tracker =
        SilenceTracker::new(service.sample_rate, service.silence_threshold, service.silence_duration);
    let min_samples = (service.min_duration.as_secs_f32() * sample_rate) as usize;
    let mut pcm_open = true;
    let mut ticker = interval(service.hop);
    ticker.set_missed_tick_behavior(MissedTickBehavior::Skip);

    while pcm_open || !buffer.is_empty() {
        tokio::select! {
            chunk = pcm_rx.recv(), if pcm_open => {
                match chunk {
                    Some(bytes) => {
                        let (samples, rms) = extend_buffer(&mut buffer, &bytes);
                        silence_tracker.ingest(rms, samples);
                    }
                    None => {
                        pcm_open = false;
                    }
                }
            }
            _ = ticker.tick() => {
                if buffer.len() < min_samples && pcm_open {
                    continue;
                }
                if buffer.is_empty() {
                    if !pcm_open {
                        break;
                    }
                    continue;
                }
                let audio: Vec<f32> = buffer.iter().copied().collect();

                // Trim leading/trailing silence before sending to the model.
                let (trimmed_audio, leading_trim) = trim_silence(
                    &audio,
                    service.sample_rate,
                    service.silence_threshold,
                    service.silence_duration,
                );

                // If the trimmed audio is empty, it's silence — optionally drop
                // the whole buffer if we've detected a silence boundary, otherwise
                // continue waiting for more audio.
                if trimmed_audio.is_empty() {
                    if silence_tracker.has_boundary() && !buffer.is_empty() {
                        let dropped = buffer.len();
                        buffer.clear();
                        total_consumed_samples += dropped;
                        silence_tracker.reset();
                        tracked.clear();
                    }
                    continue;
                }

                match service.transcribe(trimmed_audio).await {
                    Ok(segments) => {
                        let silence_boundary = silence_tracker.has_boundary();
                        if segments.is_empty() {
                            if silence_boundary && !buffer.is_empty() {
                                let dropped = buffer.len();
                                buffer.clear();
                                total_consumed_samples += dropped;
                                silence_tracker.reset();
                                tracked.clear();
                            }
                            continue;
                        }
                        // offset_seconds and buffer_duration must reflect the amount of
                        // leading silence removed so that token timings returned from
                        // the model (which are relative to the trimmed audio) map
                        // correctly into the original buffer/timebase.
                        let offset_seconds = (total_consumed_samples + leading_trim) as f32 / sample_rate;
                        let buffer_duration = (buffer.len().saturating_sub(leading_trim)) as f32 / sample_rate;
                        let (finalised, mut new_tracked) = reconcile_segments(
                            &segments,
                            &tracked,
                            service.stability_hits,
                            buffer_duration,
                            service.finality_lag.as_secs_f32(),
                            silence_boundary,
                        );

                        let partial_payload = segments
                            .iter()
                            .map(|seg| segment_to_message(seg, offset_seconds))
                            .collect::<Vec<_>>();

                        if !partial_payload.is_empty() {
                            let _ = out_tx
                                .send(OutgoingMessage::Partial {
                                    segments: partial_payload,
                                })
                                .await;
                        }

                        if !finalised.is_empty() {
                            let final_end = finalised
                                .iter()
                                .map(|seg| seg.end_s)
                                .fold(0.0f32, f32::max);
                            let final_samples = (final_end * sample_rate).round() as usize;
                            // final_samples is relative to the trimmed_audio; map that
                            // back into the original buffer by accounting for the
                            // leading_trim we removed before transcription.
                            if final_samples > 0 && final_samples <= (buffer.len().saturating_sub(leading_trim)) {
                                let consumed_before = total_consumed_samples;
                                let global_offset = (consumed_before + leading_trim) as f32 / sample_rate;

                                let wav_audio = buffer
                                    .iter()
                                    .skip(leading_trim)
                                    .take(final_samples)
                                    .copied()
                                    .collect::<Vec<_>>();
                                let wav_bytes = encode_wav(&wav_audio, service.sample_rate)
                                    .context("failed to encode wav")?;

                                let forced_by_silence = silence_boundary;
                                if forced_by_silence {
                                    let dropped = buffer.len();
                                    buffer.clear();
                                    total_consumed_samples += dropped;
                                    silence_tracker.reset();
                                    new_tracked.clear();
                                } else {
                                    // drain leading_trim + final_samples from the original buffer
                                    let drain_amount = leading_trim + final_samples;
                                    buffer.drain(..drain_amount);
                                    total_consumed_samples += drain_amount;
                                    for seg in new_tracked.iter_mut() {
                                        seg.start_s = (seg.start_s - final_end).max(0.0);
                                        seg.end_s = (seg.end_s - final_end).max(0.0);
                                    }
                                }

                                let final_segments = finalised
                                    .iter()
                                    .map(|seg| segment_to_message(seg, global_offset))
                                    .collect::<Vec<_>>();

                                let text = finalised
                                    .iter()
                                    .map(|s| s.text.trim())
                                    .filter(|s| !s.is_empty())
                                    .collect::<Vec<_>>()
                                    .join(" ");

                                if let (Some(first), Some(last)) = (finalised.first(), finalised.last()) {
                                    let start_ms = ((global_offset + first.start_s) * 1000.0) as u32;
                                    let end_ms = ((global_offset + last.end_s) * 1000.0) as u32;
                                    let payload = OutgoingMessage::Final {
                                        text,
                                        start_ms,
                                        end_ms,
                                        audio_base64: BASE64_STANDARD.encode(&wav_bytes),
                                        segments: final_segments,
                                    };
                                    let _ = out_tx.send(payload).await;
                                }
                            }
                        } else if silence_boundary && !buffer.is_empty() {
                            let dropped = buffer.len();
                            buffer.clear();
                            total_consumed_samples += dropped;
                            silence_tracker.reset();
                            new_tracked.clear();
                        }

                        tracked = new_tracked;
                    }
                    Err(err) => {
                        error!(error = %err, "transcription error");
                        let _ = out_tx
                            .send(OutgoingMessage::Error {
                                message: err.to_string(),
                            })
                            .await;
                        tokio::time::sleep(Duration::from_millis(250)).await;
                    }
                }
            }
        }
    }

    Ok(())
}

impl WhisperService {
    async fn transcribe(&self, audio: Vec<f32>) -> Result<Vec<SegmentInternal>> {
        let ctx = self.context.clone();
        Ok(tokio::task::spawn_blocking(move || {
            let guard = ctx
                .lock()
                .map_err(|_| anyhow!("failed to lock whisper context"))?;
            let mut state = guard
                .create_state()
                .context("failed to create whisper state")?;

            let mut params = FullParams::new(SamplingStrategy::Greedy { best_of: 1 });
            params.set_print_progress(false);
            params.set_print_realtime(false);
            params.set_print_timestamps(false);
            params.set_token_timestamps(true);
            params.set_no_context(true);
            params.set_single_segment(false);
            params.set_speed_up(false);
            params.set_n_threads(std::cmp::max(1, num_cpus::get() as i32 - 1));

            state
                .full(params, &audio)
                .context("whisper full() failed")?;

            let mut segments = Vec::new();
            let count = state.full_n_segments()?;
            for s in 0..count {
                let text = state.full_get_segment_text(s)?;
                let start_s = state.full_get_segment_t0(s)? as f32 / 100.0;
                let end_s = state.full_get_segment_t1(s)? as f32 / 100.0;
                let mut words = Vec::new();
                let token_count = state.full_n_tokens(s)?;
                let mut current = String::new();
                let mut word_start = None;
                for t in 0..token_count {
                    let token = state.full_get_token_data(s, t)?;
                    let piece = guard
                        .token_to_str(token.id)
                        .context("failed to convert token to string")?
                        .to_string();
                    let clean = piece.replace('▁', " ");
                    if piece.contains('▁') {
                        if let Some(start) = word_start.take() {
                            let word = current.trim().to_string();
                            if !word.is_empty() {
                                let end = token.t0 as f32 / 100.0;
                                words.push(WordTiming {
                                    text: word,
                                    start_ms: (start * 1000.0) as u32,
                                    end_ms: (end * 1000.0) as u32,
                                });
                            }
                            current.clear();
                        }
                        word_start = Some(token.t0 as f32 / 100.0);
                    } else if word_start.is_none() {
                        word_start = Some(token.t0 as f32 / 100.0);
                    }
                    current.push_str(&clean);
                    let end = token.t1 as f32 / 100.0;
                    if end > start_s
                        && current
                            .trim()
                            .ends_with(|c: char| c == ' ' || c == ',' || c == '.')
                    {
                        if let Some(start) = word_start.take() {
                            let word = current.trim().to_string();
                            if !word.is_empty() {
                                words.push(WordTiming {
                                    text: word,
                                    start_ms: (start * 1000.0) as u32,
                                    end_ms: (end * 1000.0) as u32,
                                });
                            }
                            current.clear();
                        }
                    }
                }
                if let Some(start) = word_start {
                    let end = end_s;
                    let word = current.trim().to_string();
                    if !word.is_empty() {
                        words.push(WordTiming {
                            text: word,
                            start_ms: (start * 1000.0) as u32,
                            end_ms: (end * 1000.0) as u32,
                        });
                    }
                }

                segments.push(SegmentInternal {
                    text: text.trim().to_string(),
                    start_s,
                    end_s,
                    words,
                });
            }

            Ok::<_, anyhow::Error>(segments)
        })
        .await??)
    }
}

struct SilenceTracker {
    threshold: f32,
    required_samples: usize,
    tail_silence_samples: usize,
}

impl SilenceTracker {
    fn new(sample_rate: u32, threshold: f32, duration: Duration) -> Self {
        let mut required_samples =
            (sample_rate as f32 * duration.as_secs_f32()).round() as usize;
        if required_samples == 0 {
            required_samples = ((sample_rate as f32) * 0.2).round() as usize; // default 200ms
        }
        SilenceTracker {
            threshold,
            required_samples,
            tail_silence_samples: 0,
        }
    }

    fn ingest(&mut self, rms: f32, samples: usize) {
        if samples == 0 {
            return;
        }
        if rms <= self.threshold {
            self.tail_silence_samples = self.tail_silence_samples.saturating_add(samples);
        } else {
            self.tail_silence_samples = 0;
        }
    }

    fn has_boundary(&self) -> bool {
        self.required_samples > 0 && self.tail_silence_samples >= self.required_samples
    }

    fn reset(&mut self) {
        self.tail_silence_samples = 0;
    }

}

fn extend_buffer(buffer: &mut VecDeque<f32>, bytes: &[u8]) -> (usize, f32) {
    let mut sum_sq = 0.0f64;
    let mut count = 0usize;
    for chunk in bytes.chunks_exact(2) {
        let sample = i16::from_le_bytes([chunk[0], chunk[1]]);
        let normalized = sample as f32 / i16::MAX as f32;
        let value = f64::from(normalized);
        sum_sq += value * value;
        buffer.push_back(normalized);
        count += 1;
    }
    let rms = if count > 0 {
        (sum_sq / count as f64).sqrt() as f32
    } else {
        0.0
    };
    (count, rms)
}

fn trim_silence(
    samples: &[f32],
    sample_rate: u32,
    threshold: f32,
    silence_duration: Duration,
) -> (Vec<f32>, usize) {
    let len = samples.len();
    if len == 0 {
        return (Vec::new(), 0);
    }

    // Window to measure short-term RMS for silence detection. Use the configured
    // silence duration as a guideline but clamp to reasonable bounds.
    let window_ms = silence_duration.as_millis().clamp(20, 500) as usize;
    let window = ((sample_rate as usize) * window_ms) / 1000;
    let window = window.max(1);
    let step = (window / 2).max(1);

    // find leading boundary
    let mut start = 0usize;
    'outer_start: while start + window <= len {
        let mut sum_sq = 0.0f64;
        for s in &samples[start..start + window] {
            let v = *s as f64;
            sum_sq += v * v;
        }
        let rms = (sum_sq / window as f64).sqrt() as f32;
        if rms > threshold {
            break 'outer_start;
        }
        start = (start + step).min(len);
    }

    if start >= len {
        return (Vec::new(), len);
    }

    // find trailing boundary
    let mut end = len;
    'outer_end: while end >= window && end > start {
        let from = end.saturating_sub(window);
        let mut sum_sq = 0.0f64;
        for s in &samples[from..end] {
            let v = *s as f64;
            sum_sq += v * v;
        }
        let rms = (sum_sq / window as f64).sqrt() as f32;
        if rms > threshold {
            break 'outer_end;
        }
        end = end.saturating_sub(step);
    }

    if end <= start {
        return (Vec::new(), len);
    }

    let trimmed = samples[start..end].to_vec();
    (trimmed, start)
}

fn segment_to_message(segment: &SegmentInternal, offset_seconds: f32) -> SegmentMessage {
    let start_ms = ((offset_seconds + segment.start_s) * 1000.0) as u32;
    let end_ms = ((offset_seconds + segment.end_s) * 1000.0) as u32;
    let words = segment
        .words
        .iter()
        .map(|word| WordTiming {
            text: word.text.clone(),
            start_ms: word.start_ms + (offset_seconds * 1000.0) as u32,
            end_ms: word.end_ms + (offset_seconds * 1000.0) as u32,
        })
        .collect();

    SegmentMessage {
        text: segment.text.clone(),
        start_ms,
        end_ms,
        words,
    }
}

fn reconcile_segments(
    latest: &[SegmentInternal],
    tracked: &[TrackedSegment],
    stability_threshold: usize,
    buffer_duration: f32,
    finality_lag_s: f32,
    force_finalize: bool,
) -> (Vec<SegmentInternal>, Vec<TrackedSegment>) {
    if force_finalize {
        return (latest.to_vec(), Vec::new());
    }

    let mut prefix = Vec::new();
    let mut new_tracked = Vec::new();

    for (idx, seg) in latest.iter().enumerate() {
        let mut stability = 1;
        if let Some(prev) = tracked.get(idx) {
            if prev.text == seg.text {
                stability = prev.stability + 1;
            }
        }
        let segment_age = (buffer_duration - seg.end_s).max(0.0);
        let aged_out = finality_lag_s <= 0.0 || segment_age >= finality_lag_s;
        if idx == prefix.len() && (stability >= stability_threshold || aged_out) {
            prefix.push(seg.clone());
            continue;
        }
        new_tracked.push(TrackedSegment {
            text: seg.text.clone(),
            start_s: seg.start_s,
            end_s: seg.end_s,
            stability,
        });
    }

    (prefix, new_tracked)
}

fn encode_wav(samples: &[f32], sample_rate: u32) -> Result<Vec<u8>> {
    let mut cursor = Cursor::new(Vec::new());
    {
        let spec = WavSpec {
            channels: 1,
            sample_rate,
            bits_per_sample: 16,
            sample_format: SampleFormat::Int,
        };
        let mut writer = WavWriter::new(&mut cursor, spec)?;
        for &sample in samples {
            let clamped = sample.clamp(-1.0, 1.0);
            let value = (clamped * i16::MAX as f32) as i16;
            writer.write_sample(value)?;
        }
        writer.finalize()?;
    }
    Ok(cursor.into_inner())
}
