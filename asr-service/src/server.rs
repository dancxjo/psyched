use std::sync::Arc;

use crate::core::errors::AsrError;
use crate::core::messages::{ClientMessage, ServerMessage};
use crate::core::pipeline::Pipeline;
pub use crate::core::pipeline::Pipeline as PipelineTrait;
use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::State;
use axum::response::IntoResponse;
use axum::routing::get;
use axum::Router;
use futures::stream::StreamExt;
use tracing::{error, info, warn};

#[derive(Clone)]
struct AppState {
    pipeline: Arc<dyn Pipeline>,
}

pub fn router(pipeline: Arc<dyn Pipeline>) -> Router {
    let state = AppState { pipeline };
    Router::new()
        .route("/ws", get(ws_handler))
        .with_state(state)
}

async fn ws_handler(ws: WebSocketUpgrade, State(state): State<AppState>) -> impl IntoResponse {
    ws.on_upgrade(|socket| websocket_loop(socket, state.pipeline))
}

async fn websocket_loop(mut socket: WebSocket, pipeline: Arc<dyn Pipeline>) {
    info!("websocket connected");
    while let Some(result) = socket.next().await {
        match result {
            Ok(Message::Text(text)) => match serde_json::from_str::<ClientMessage>(&text) {
                Ok(message) => match pipeline.handle(message).await {
                    Ok(responses) => {
                        for response in responses {
                            if let Err(err) = send_message(&mut socket, response).await {
                                warn!(error = ?err, "failed to send response");
                                break;
                            }
                        }
                    }
                    Err(err) => {
                        warn!(error = ?err, "pipeline error");
                        let message: ServerMessage = err.into();
                        if let Err(send_err) = send_message(&mut socket, message).await {
                            warn!(error = ?send_err, "failed to send error response");
                            break;
                        }
                    }
                },
                Err(err) => {
                    warn!(error = %err, "failed to parse client message");
                    let payload = ServerMessage::error(None, format!("invalid message: {err}"));
                    if let Err(send_err) = send_message(&mut socket, payload).await {
                        warn!(error = ?send_err, "failed to send parse error");
                        break;
                    }
                }
            },
            Ok(Message::Binary(_)) => {
                warn!("binary websocket frames are not supported");
                if let Err(err) = send_message(
                    &mut socket,
                    ServerMessage::error(None, "binary frames not supported"),
                )
                .await
                {
                    warn!(error = ?err, "failed to send binary warning");
                    break;
                }
            }
            Ok(Message::Close(frame)) => {
                info!(?frame, "websocket closed by client");
                break;
            }
            Ok(Message::Ping(payload)) => {
                if let Err(err) = socket.send(Message::Pong(payload)).await {
                    warn!(error = ?err, "failed to reply to ping");
                    break;
                }
            }
            Ok(Message::Pong(_)) => {}
            Err(err) => {
                error!(error = ?err, "websocket error");
                break;
            }
        }
    }
    info!("websocket disconnected");
}

async fn send_message(socket: &mut WebSocket, message: ServerMessage) -> Result<(), AsrError> {
    let json = serde_json::to_string(&message).map_err(|err| AsrError::Internal(err.into()))?;
    socket
        .send(Message::Text(json))
        .await
        .map_err(|err| AsrError::Internal(err.into()))
}
