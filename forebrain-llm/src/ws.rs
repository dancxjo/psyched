use std::sync::Arc;

use anyhow::{anyhow, Error};
use axum::{
    extract::{
        ws::{Message as WsMessage, WebSocket, WebSocketUpgrade},
        State,
    },
    response::IntoResponse,
    routing::get,
    Router,
};
use futures::{pin_mut, StreamExt};
use serde::{Deserialize, Serialize};
use serde_json::json;
use tracing::{error, info, warn};

use crate::{config::AppConfig, model::ModelManager, session::ConversationSession};

/// Shared server state passed to websocket handlers.
#[derive(Clone)]
pub struct ServerState {
    config: AppConfig,
    model: ModelManager,
}

impl ServerState {
    /// Creates a new server state container.
    pub fn new(config: AppConfig, model: ModelManager) -> Self {
        Self { config, model }
    }
}

/// Builds the application router with websocket endpoint attached.
pub fn build_router(state: ServerState) -> Router {
    Router::new()
        .route("/chat", get(chat_handler))
        .with_state(Arc::new(state))
}

async fn chat_handler(
    ws: WebSocketUpgrade,
    State(state): State<Arc<ServerState>>,
) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_socket(socket, state))
}

#[derive(Debug, Deserialize)]
struct ClientMessage {
    role: Option<String>,
    content: Option<String>,
    command: Option<String>,
    #[serde(default)]
    temperature: Option<f32>,
    #[serde(default)]
    top_p: Option<f32>,
    #[serde(default)]
    repeat_penalty: Option<f32>,
}

#[derive(Debug, Serialize)]
struct ServerStats {
    generated_tokens: usize,
    uptime_ms: u128,
}

async fn handle_socket(mut socket: WebSocket, state: Arc<ServerState>) {
    let session = ConversationSession::new(state.config.model.system_prompt.as_deref());

    while let Some(Ok(msg)) = socket.recv().await {
        match msg {
            WsMessage::Text(text) => match serde_json::from_str::<ClientMessage>(&text) {
                Ok(client) => {
                    if let Err(err) =
                        process_client_message(&state, &session, client, &mut socket).await
                    {
                        error!(error = %err, "processing client message failed");
                        break;
                    }
                }
                Err(err) => {
                    warn!(error = %err, "invalid message");
                    if socket
                        .send(WsMessage::Text(
                            json!({
                                "role": "system",
                                "content": format!("invalid message: {err}")
                            })
                            .to_string(),
                        ))
                        .await
                        .is_err()
                    {
                        break;
                    }
                }
            },
            WsMessage::Close(_) => break,
            _ => {}
        }
    }
}

async fn process_client_message(
    state: &ServerState,
    session: &ConversationSession,
    mut client: ClientMessage,
    socket: &mut WebSocket,
) -> Result<(), Error> {
    if let Some(command) = client.command.take() {
        match command.as_str() {
            "reset" => {
                session.reset(state.config.model.system_prompt.as_deref());
                socket
                    .send(WsMessage::Text(
                        json!({
                            "role": "system",
                            "content": "session reset"
                        })
                        .to_string(),
                    ))
                    .await
                    .ok();
            }
            "stats" => {
                let stats = session.stats();
                let payload = json!({
                    "role": "system",
                    "content": ServerStats {
                        generated_tokens: stats.generated_tokens,
                        uptime_ms: stats.uptime().as_millis(),
                    }
                });
                socket.send(WsMessage::Text(payload.to_string())).await.ok();
            }
            "load_model" => match state.model.load_model(&state.config.model).await {
                Ok(_) => {
                    socket
                        .send(WsMessage::Text(
                            json!({
                                "role": "system",
                                "content": "model reloaded"
                            })
                            .to_string(),
                        ))
                        .await
                        .ok();
                }
                Err(err) => {
                    warn!(error = %err, "model reload command failed");
                    socket
                        .send(WsMessage::Text(
                            json!({
                                "role": "system",
                                "content": format!("model reload failed: {err}")
                            })
                            .to_string(),
                        ))
                        .await
                        .ok();
                }
            },
            other => {
                warn!(command = other, "unknown command");
            }
        }
        return Ok(());
    }

    let role = client.role.unwrap_or_else(|| "user".into());
    let content = client.content.unwrap_or_default();
    session.push(role.clone(), content.clone());

    if role != "user" {
        return Ok(());
    }

    let mut model_cfg = state.config.model.clone();
    if let Some(temp) = client.temperature {
        model_cfg.temperature = temp;
    }
    if let Some(top_p) = client.top_p {
        model_cfg.top_p = top_p;
    }
    if let Some(repeat) = client.repeat_penalty {
        model_cfg.repeat_penalty = repeat;
    }

    let history = session.history();
    let stream = state
        .model
        .generate_stream(&model_cfg, history)
        .await
        .map_err(|err| anyhow!(err.to_string()))?;
    pin_mut!(stream);
    let mut assistant_response = String::new();

    while let Some(chunk) = stream.next().await {
        match chunk {
            Ok(token) => {
                assistant_response.push_str(&token);
                session.add_generated_tokens(1);
                socket
                    .send(WsMessage::Text(
                        json!({
                            "role": "assistant",
                            "content": token
                        })
                        .to_string(),
                    ))
                    .await
                    .map_err(|err| {
                        warn!(error = %err, "failed to send token");
                        anyhow!(format!("transport error: {err}"))
                    })?;
            }
            Err(err) => {
                warn!(error = %err, "generation error");
                socket
                    .send(WsMessage::Text(
                        json!({
                            "role": "system",
                            "content": format!("generation error: {err}")
                        })
                        .to_string(),
                    ))
                    .await
                    .ok();
                return Err(anyhow!(err.to_string()));
            }
        }
    }

    session.push("assistant", assistant_response);
    Ok(())
}

/// Starts the websocket server and awaits shutdown.
pub async fn serve(config: AppConfig, model: ModelManager) -> anyhow::Result<()> {
    let state = ServerState::new(config.clone(), model.clone());
    let router = build_router(state);
    info!(addr = %config.websocket.bind_addr, "websocket server listening");
    axum::Server::bind(&config.websocket.bind_addr.parse()?)
        .serve(router.into_make_service())
        .await?;
    Ok(())
}
