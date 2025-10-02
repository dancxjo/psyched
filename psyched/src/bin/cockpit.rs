use anyhow::Context as _;
use axum::{
    extract::ws::{Message, WebSocket, WebSocketUpgrade},
    extract::State,
    response::IntoResponse,
    routing::get,
    serve, Router,
};
use futures::{stream::SplitSink, SinkExt, StreamExt};
use rclrs::{
    vendor::example_interfaces::msg::String as RosString, vendor::sensor_msgs::msg::Imu as RosImu,
    Context, CreateBasicExecutor, QoSProfile, RclReturnCode, RclrsError, SpinOptions,
};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;
use tokio::net::TcpListener;
use tokio::sync::mpsc::error::TryRecvError;
use tokio::sync::oneshot;
use tokio::sync::{broadcast, mpsc, Mutex};
use tokio::task;
use tracing::{error, info, warn};

#[derive(Clone)]
struct AppState {
    tx_broadcast: broadcast::Sender<WsOutbound>,
    tx_ros: mpsc::UnboundedSender<RosCommand>,
}

#[derive(Debug, Deserialize)]
#[serde(tag = "op")]
enum WsInbound {
    #[serde(rename = "pub")]
    Pub {
        topic: String,
        msg: serde_json::Value,
    },
    #[serde(rename = "sub")]
    Sub { topic: String },
    #[serde(rename = "unsub")]
    Unsub { topic: String },
}

#[derive(Debug, Serialize, Clone)]
#[serde(tag = "op")]
enum WsOutbound {
    #[serde(rename = "msg")]
    Msg {
        topic: String,
        msg: serde_json::Value,
    },
    #[serde(rename = "ok")]
    Ok { id: Option<String> },
    #[serde(rename = "err")]
    Err { reason: String },
}

#[derive(Debug, thiserror::Error)]
enum CockpitError {
    #[error("unsupported topic '{0}'")]
    UnsupportedTopic(String),
    #[error("expected string field '{0}'")]
    MissingString(&'static str),
    #[error("ROS bridge offline")]
    BridgeOffline,
    #[error("failed to subscribe to topic '{topic}': {source}")]
    SubscribeFailed {
        topic: String,
        #[source]
        source: RclrsError,
    },
}

#[derive(Debug)]
enum RosCommand {
    PublishConversation(String),
    SubscribeTopic {
        topic: String,
        respond_to: oneshot::Sender<Result<(), CockpitError>>,
    },
    UnsubscribeTopic {
        topic: String,
        respond_to: oneshot::Sender<Result<(), CockpitError>>,
    },
}

type SharedSender = Arc<Mutex<SplitSink<WebSocket, Message>>>;

struct SubscriptionEntry {
    ref_count: usize,
    handle: TopicSubscription,
}

enum TopicSubscription {
    Transcript(rclrs::Subscription<RosString>),
    Imu(rclrs::Subscription<RosImu>),
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(tracing_subscriber::EnvFilter::from_default_env())
        .with_target(false)
        .init();

    let (tx_ros, rx_ros) = mpsc::unbounded_channel::<RosCommand>();
    let (tx_broadcast, _) = broadcast::channel::<WsOutbound>(256);
    let ros_broadcast = tx_broadcast.clone();

    let ros_thread = std::thread::Builder::new()
        .name("ros-spin".into())
        .spawn(move || {
            if let Err(err) = ros_spin(rx_ros, ros_broadcast) {
                error!(?err, "ROS spin loop exited with error");
            }
        })
        .context("failed to spawn ROS spin thread")?;

    let state = AppState {
        tx_broadcast: tx_broadcast.clone(),
        tx_ros: tx_ros.clone(),
    };

    let app = Router::new()
        .route("/ws", get(ws_handler))
        .with_state(state);

    let addr: SocketAddr = "0.0.0.0:8088".parse()?;
    let listener = TcpListener::bind(addr).await?;
    let local_addr = listener.local_addr()?;
    info!("listening on ws://{local_addr}/ws");

    let shutdown_signal = async {
        if let Err(err) = tokio::signal::ctrl_c().await {
            warn!(?err, "failed to listen for shutdown signal");
        }
        info!("shutdown signal received");
    };

    serve(listener, app.into_make_service())
        .with_graceful_shutdown(shutdown_signal)
        .await
        .context("web server error")?;

    drop(tx_ros);
    let _ = ros_thread.join();

    Ok(())
}

async fn ws_handler(ws: WebSocketUpgrade, State(state): State<AppState>) -> impl IntoResponse {
    ws.on_upgrade(move |socket| client_loop(socket, state))
}

async fn client_loop(socket: WebSocket, state: AppState) {
    let (sender, mut receiver) = socket.split();
    let shared_sender: SharedSender = Arc::new(Mutex::new(sender));

    let mut rx_broadcast = state.tx_broadcast.subscribe();
    let forward_sender = shared_sender.clone();

    let forward = task::spawn(async move {
        while let Ok(outbound) = rx_broadcast.recv().await {
            if let Err(err) = send_json(&forward_sender, &outbound).await {
                warn!(?err, "failed to push broadcast message to client");
                break;
            }
        }
    });

    while let Some(msg_result) = receiver.next().await {
        match msg_result {
            Ok(Message::Text(payload)) => match serde_json::from_str::<WsInbound>(&payload) {
                Ok(WsInbound::Sub { topic }) => {
                    let (respond_to, response) = oneshot::channel();
                    if state
                        .tx_ros
                        .send(RosCommand::SubscribeTopic {
                            topic: topic.clone(),
                            respond_to,
                        })
                        .is_err()
                    {
                        if let Err(err) = send_json(
                            &shared_sender,
                            &WsOutbound::Err {
                                reason: CockpitError::BridgeOffline.to_string(),
                            },
                        )
                        .await
                        {
                            warn!(?err, "client disconnected while sending error");
                        }
                        break;
                    }

                    match response.await {
                        Ok(Ok(())) => {
                            if let Err(err) =
                                send_json(&shared_sender, &WsOutbound::Ok { id: None }).await
                            {
                                warn!(?err, "client disconnected while sending ack");
                                break;
                            }
                        }
                        Ok(Err(err)) => {
                            if let Err(send_err) = send_json(
                                &shared_sender,
                                &WsOutbound::Err {
                                    reason: err.to_string(),
                                },
                            )
                            .await
                            {
                                warn!(?send_err, "client disconnected while sending error");
                                break;
                            }
                        }
                        Err(_canceled) => {
                            if let Err(send_err) = send_json(
                                &shared_sender,
                                &WsOutbound::Err {
                                    reason: CockpitError::BridgeOffline.to_string(),
                                },
                            )
                            .await
                            {
                                warn!(?send_err, "client disconnected while sending error");
                            }
                            break;
                        }
                    }
                }
                Ok(WsInbound::Unsub { topic }) => {
                    let (respond_to, response) = oneshot::channel();
                    if state
                        .tx_ros
                        .send(RosCommand::UnsubscribeTopic {
                            topic: topic.clone(),
                            respond_to,
                        })
                        .is_err()
                    {
                        if let Err(err) = send_json(
                            &shared_sender,
                            &WsOutbound::Err {
                                reason: CockpitError::BridgeOffline.to_string(),
                            },
                        )
                        .await
                        {
                            warn!(?err, "client disconnected while sending error");
                        }
                        break;
                    }

                    match response.await {
                        Ok(Ok(())) => {
                            if let Err(err) =
                                send_json(&shared_sender, &WsOutbound::Ok { id: None }).await
                            {
                                warn!(?err, "client disconnected while sending ack");
                                break;
                            }
                        }
                        Ok(Err(err)) => {
                            if let Err(send_err) = send_json(
                                &shared_sender,
                                &WsOutbound::Err {
                                    reason: err.to_string(),
                                },
                            )
                            .await
                            {
                                warn!(?send_err, "client disconnected while sending error");
                                break;
                            }
                        }
                        Err(_canceled) => {
                            if let Err(send_err) = send_json(
                                &shared_sender,
                                &WsOutbound::Err {
                                    reason: CockpitError::BridgeOffline.to_string(),
                                },
                            )
                            .await
                            {
                                warn!(?send_err, "client disconnected while sending error");
                            }
                            break;
                        }
                    }
                }
                Ok(WsInbound::Pub { topic, msg }) => match convert_pub(&topic, msg) {
                    Ok(command) => {
                        if let Err(_err) = state.tx_ros.send(command) {
                            let _ = send_json(
                                &shared_sender,
                                &WsOutbound::Err {
                                    reason: "ROS bridge unavailable".into(),
                                },
                            )
                            .await;
                            break;
                        } else if let Err(err) =
                            send_json(&shared_sender, &WsOutbound::Ok { id: None }).await
                        {
                            warn!(?err, "client disconnected while sending ack");
                            break;
                        }
                    }
                    Err(err) => {
                        if let Err(send_err) = send_json(
                            &shared_sender,
                            &WsOutbound::Err {
                                reason: err.to_string(),
                            },
                        )
                        .await
                        {
                            warn!(?send_err, "client disconnected while sending error");
                        }
                    }
                },
                Err(err) => {
                    if let Err(send_err) = send_json(
                        &shared_sender,
                        &WsOutbound::Err {
                            reason: format!("invalid json: {err}"),
                        },
                    )
                    .await
                    {
                        warn!(?send_err, "client disconnected while sending parse error");
                    }
                }
            },
            Ok(Message::Ping(payload)) => {
                if let Err(err) = send_control(&shared_sender, Message::Pong(payload)).await {
                    warn!(?err, "failed replying to ping");
                    break;
                }
            }
            Ok(Message::Pong(_)) => {}
            Ok(Message::Binary(_)) => {}
            Ok(Message::Close(_)) => {
                break;
            }
            Err(err) => {
                warn!(?err, "websocket receive error");
                break;
            }
        }
    }

    forward.abort();
    let _ = forward.await;
}

fn convert_pub(topic: &str, msg: serde_json::Value) -> Result<RosCommand, CockpitError> {
    match topic {
        "/conversation" => {
            let data = msg
                .get("data")
                .and_then(|value| value.as_str())
                .ok_or(CockpitError::MissingString("msg.data"))?;
            Ok(RosCommand::PublishConversation(data.to_owned()))
        }
        other => Err(CockpitError::UnsupportedTopic(other.to_owned())),
    }
}

fn ros_spin(
    mut rx_ros: mpsc::UnboundedReceiver<RosCommand>,
    ws_outbound: broadcast::Sender<WsOutbound>,
) -> anyhow::Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("cockpit")?;

    let conversation_publisher = node.create_publisher::<RosString>("/conversation")?;
    let mut subscriptions: HashMap<String, SubscriptionEntry> = HashMap::new();

    loop {
        let mut disconnected = false;
        loop {
            match rx_ros.try_recv() {
                Ok(command) => match command {
                    RosCommand::PublishConversation(data) => {
                        let ros_msg = RosString { data: data.clone() };
                        if let Err(err) = conversation_publisher.publish(ros_msg) {
                            error!(?err, "failed to publish /conversation");
                            let _ = ws_outbound.send(WsOutbound::Err {
                                reason: format!("failed to publish to /conversation: {err}"),
                            });
                        } else {
                            let _ = ws_outbound.send(WsOutbound::Msg {
                                topic: "/conversation".into(),
                                msg: serde_json::json!({ "data": data }),
                            });
                        }
                    }
                    RosCommand::SubscribeTopic { topic, respond_to } => {
                        let result =
                            subscribe_topic(&node, &ws_outbound, &mut subscriptions, &topic);
                        let _ = respond_to.send(result);
                    }
                    RosCommand::UnsubscribeTopic { topic, respond_to } => {
                        let result = unsubscribe_topic(&mut subscriptions, &topic);
                        let _ = respond_to.send(result);
                    }
                },
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    disconnected = true;
                    break;
                }
            }
        }

        let spin_errors =
            executor.spin(SpinOptions::spin_once().timeout(Duration::from_millis(20)));
        for err in spin_errors {
            if let rclrs::RclrsError::RclError {
                code: RclReturnCode::Timeout,
                ..
            } = err
            {
                continue;
            }
            warn!(?err, "executor reported error while spinning");
        }

        if disconnected {
            break;
        }
    }

    drop(node);

    Ok(())
}

fn subscribe_topic(
    node: &rclrs::Node,
    ws_outbound: &broadcast::Sender<WsOutbound>,
    subscriptions: &mut HashMap<String, SubscriptionEntry>,
    topic: &str,
) -> Result<(), CockpitError> {
    if let Some(entry) = subscriptions.get_mut(topic) {
        entry.ref_count += 1;
        return Ok(());
    }

    let handle = create_topic_subscription(node, topic, ws_outbound.clone())?;
    subscriptions.insert(
        topic.to_owned(),
        SubscriptionEntry {
            ref_count: 1,
            handle,
        },
    );
    Ok(())
}

fn unsubscribe_topic(
    subscriptions: &mut HashMap<String, SubscriptionEntry>,
    topic: &str,
) -> Result<(), CockpitError> {
    if !is_supported_topic(topic) {
        return Err(CockpitError::UnsupportedTopic(topic.to_owned()));
    }

    if let Some(entry) = subscriptions.get_mut(topic) {
        if entry.ref_count > 1 {
            entry.ref_count -= 1;
        } else {
            subscriptions.remove(topic);
        }
    }

    Ok(())
}

fn is_supported_topic(topic: &str) -> bool {
    matches!(topic, "/audio/transcript/final" | "/imu/data")
}

fn create_topic_subscription(
    node: &rclrs::Node,
    topic: &str,
    ws_outbound: broadcast::Sender<WsOutbound>,
) -> Result<TopicSubscription, CockpitError> {
    match topic {
        "/audio/transcript/final" => {
            let ws_for_topic = ws_outbound;
            let topic_for_message = topic.to_owned();
            let subscription = node
                .create_subscription((topic, QoSProfile::default()), move |msg: RosString| {
                        let outbound = WsOutbound::Msg {
                            topic: topic_for_message.clone(),
                            msg: serde_json::json!({ "data": msg.data }),
                        };
                        if let Err(err) = ws_for_topic.send(outbound) {
                            warn!(?err, "no websocket listeners for transcript broadcast");
                        }
                })
                .map_err(|source| CockpitError::SubscribeFailed {
                    topic: topic.to_owned(),
                    source,
                })?;
            Ok(TopicSubscription::Transcript(subscription))
        }
        "/imu/data" => {
            let ws_for_topic = ws_outbound;
            let topic_for_message = topic.to_owned();
            let subscription = node
                .create_subscription((topic, QoSProfile::sensor_data()), move |msg: RosImu| {
                    let RosImu {
                        header,
                        orientation,
                        angular_velocity,
                        linear_acceleration,
                        ..
                    } = msg;

                    let frame_id = header.frame_id;
                    let stamp = header.stamp;

                    let payload = serde_json::json!({
                        "header": {
                            "frame_id": frame_id,
                            "stamp": {
                                "sec": stamp.sec,
                                "nanosec": stamp.nanosec,
                            },
                        },
                        "orientation": {
                            "x": orientation.x,
                            "y": orientation.y,
                            "z": orientation.z,
                            "w": orientation.w,
                        },
                        "angular_velocity": {
                            "x": angular_velocity.x,
                            "y": angular_velocity.y,
                            "z": angular_velocity.z,
                        },
                        "linear_acceleration": {
                            "x": linear_acceleration.x,
                            "y": linear_acceleration.y,
                            "z": linear_acceleration.z,
                        },
                    });

                    let outbound = WsOutbound::Msg {
                        topic: topic_for_message.clone(),
                        msg: payload,
                    };

                    if let Err(err) = ws_for_topic.send(outbound) {
                        warn!(?err, "no websocket listeners for imu telemetry broadcast");
                    }
                })
                .map_err(|source| CockpitError::SubscribeFailed {
                    topic: topic.to_owned(),
                    source,
                })?;
            Ok(TopicSubscription::Imu(subscription))
        }
        _ => Err(CockpitError::UnsupportedTopic(topic.to_owned())),
    }
}

async fn send_json(sender: &SharedSender, payload: &WsOutbound) -> anyhow::Result<()> {
    let text = serde_json::to_string(payload)?;
    send_control(sender, Message::Text(text)).await
}

async fn send_control(sender: &SharedSender, message: Message) -> anyhow::Result<()> {
    let mut guard = sender.lock().await;
    guard
        .send(message)
        .await
        .context("failed sending websocket message")
}
