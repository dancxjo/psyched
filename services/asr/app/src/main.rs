use std::collections::VecDeque;
use std::env;
use std::io::Cursor;
use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{anyhow, Context, Result};
use axum::extract::ws::{Message, WebSocket};
use axum::extract::State;
use axum::response::IntoResponse;
use axum::routing::get;
use axum::{Json, Router};
use base64::engine::general_purpose::STANDARD as BASE64_STANDARD;
use base64::Engine;
use futures::{SinkExt, StreamExt};
use hound::{SampleFormat, WavSpec, WavWriter};
use serde::Serialize;
use tokio::sync::mpsc;
use tokio::time::{interval, MissedTickBehavior};
use tracing::{error, info, warn};
use whisper_rs::{FullParams, SamplingStrategy, WhisperContext, WhisperContextParameters};

#[derive(Clone)]
struct WhisperService {
    context: Arc<Mutex<WhisperContext>>,
    sample_rate: u32,
    stability_hits: usize,
    hop: Duration,
    min_duration: Duration,
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
    });

    let app = Router::new()
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
                warn!(payload = %text, "unexpected text frame received");
            }
            Message::Ping(_) | Message::Pong(_) => {}
        }
    }

    drop(pcm_tx);
    if let Err(err) = processor.await {
        error!(error = %err, "processor task join failed");
    }
    send_task.abort();
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
    let min_samples = (service.min_duration.as_secs_f32() * sample_rate) as usize;
    let mut pcm_open = true;
    let mut ticker = interval(service.hop);
    ticker.set_missed_tick_behavior(MissedTickBehavior::Skip);

    while pcm_open || !buffer.is_empty() {
        tokio::select! {
            chunk = pcm_rx.recv(), if pcm_open => {
                match chunk {
                    Some(bytes) => {
                        extend_buffer(&mut buffer, &bytes);
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
                match service.transcribe(audio).await {
                    Ok(segments) => {
                        if segments.is_empty() {
                            continue;
                        }
                        let offset_seconds = total_consumed_samples as f32 / sample_rate;
                        let (finalised, mut new_tracked) = reconcile_segments(
                            &segments,
                            &tracked,
                            service.stability_hits,
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
                            if final_samples > 0 && final_samples <= buffer.len() {
                                let consumed_before = total_consumed_samples;
                                let global_offset = consumed_before as f32 / sample_rate;

                                let wav_audio = buffer
                                    .iter()
                                    .take(final_samples)
                                    .copied()
                                    .collect::<Vec<_>>();
                                let wav_bytes = encode_wav(&wav_audio, service.sample_rate)
                                    .context("failed to encode wav")?;

                                buffer.drain(..final_samples);
                                total_consumed_samples += final_samples;

                                for seg in new_tracked.iter_mut() {
                                    seg.start_s = (seg.start_s - final_end).max(0.0);
                                    seg.end_s = (seg.end_s - final_end).max(0.0);
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

fn extend_buffer(buffer: &mut VecDeque<f32>, bytes: &[u8]) {
    for chunk in bytes.chunks_exact(2) {
        let sample = i16::from_le_bytes([chunk[0], chunk[1]]);
        buffer.push_back(sample as f32 / i16::MAX as f32);
    }
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
) -> (Vec<SegmentInternal>, Vec<TrackedSegment>) {
    let mut prefix = Vec::new();
    let mut new_tracked = Vec::new();

    for (idx, seg) in latest.iter().enumerate() {
        if let Some(prev) = tracked.get(idx) {
            if prev.text == seg.text {
                let stability = prev.stability + 1;
                if idx == prefix.len() && stability >= stability_threshold {
                    prefix.push(seg.clone());
                    continue;
                } else {
                    new_tracked.push(TrackedSegment {
                        text: seg.text.clone(),
                        start_s: seg.start_s,
                        end_s: seg.end_s,
                        stability,
                    });
                }
            } else {
                new_tracked.push(TrackedSegment {
                    text: seg.text.clone(),
                    start_s: seg.start_s,
                    end_s: seg.end_s,
                    stability: 1,
                });
            }
        } else {
            new_tracked.push(TrackedSegment {
                text: seg.text.clone(),
                start_s: seg.start_s,
                end_s: seg.end_s,
                stability: 1,
            });
        }
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
