use anyhow::Context as _;
use axum::{
    extract::ws::{Message, WebSocket, WebSocketUpgrade},
    extract::State,
    response::IntoResponse,
    routing::get,
    serve, Router,
};
use futures::{stream::SplitSink, SinkExt, StreamExt};
use geometry_msgs::msg::{Twist as RosTwist, Vector3 as RosVector3};
use rclrs::IntoPrimitiveOptions as _;
use rclrs::{
    sensor_msgs::msg::Imu as RosImu, vendor::example_interfaces::msg::String as RosString, Context,
    CreateBasicExecutor, RclReturnCode, RclrsError, SpinOptions,
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
use tracing::{error, info, trace, warn};

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
    #[error("expected field '{0}'")]
    MissingField(&'static str),
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
    PublishCmdVel(RosTwist),
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
        "/cmd_vel" => {
            let twist = parse_twist(&msg)?;
            Ok(RosCommand::PublishCmdVel(twist))
        }
        other => Err(CockpitError::UnsupportedTopic(other.to_owned())),
    }
}

fn parse_twist(value: &serde_json::Value) -> Result<RosTwist, CockpitError> {
    let linear = value
        .get("linear")
        .ok_or(CockpitError::MissingField("msg.linear"))?;
    let angular = value
        .get("angular")
        .ok_or(CockpitError::MissingField("msg.angular"))?;

    let linear_x = extract_number(linear, "x", "msg.linear.x")?;
    let linear_y = extract_number(linear, "y", "msg.linear.y")?;
    let linear_z = extract_number(linear, "z", "msg.linear.z")?;
    let angular_x = extract_number(angular, "x", "msg.angular.x")?;
    let angular_y = extract_number(angular, "y", "msg.angular.y")?;
    let angular_z = extract_number(angular, "z", "msg.angular.z")?;

    Ok(RosTwist {
        linear: RosVector3 {
            x: linear_x,
            y: linear_y,
            z: linear_z,
        },
        angular: RosVector3 {
            x: angular_x,
            y: angular_y,
            z: angular_z,
        },
    })
}

fn extract_number(
    parent: &serde_json::Value,
    field: &str,
    label: &'static str,
) -> Result<f64, CockpitError> {
    parent
        .get(field)
        .and_then(|value| value.as_f64())
        .ok_or(CockpitError::MissingField(label))
}

fn ros_spin(
    mut rx_ros: mpsc::UnboundedReceiver<RosCommand>,
    ws_outbound: broadcast::Sender<WsOutbound>,
) -> anyhow::Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("cockpit")?;

    let conversation_publisher = node.create_publisher::<RosString>("/conversation")?;
    let cmd_vel_publisher = node.create_publisher::<RosTwist>("/cmd_vel")?;
    let foot_bridge = foot::FootTelemetryBridge::new(&node, ws_outbound.clone())?;
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
                    RosCommand::PublishCmdVel(twist) => {
                        if let Err(err) = cmd_vel_publisher.publish(twist.clone()) {
                            error!(?err, "failed to publish /cmd_vel");
                            let _ = ws_outbound.send(WsOutbound::Err {
                                reason: format!("failed to publish to /cmd_vel: {err}"),
                            });
                        } else {
                            foot_bridge.record_command(&twist);
                        }
                    }
                    RosCommand::SubscribeTopic { topic, respond_to } => {
                        let result = subscribe_topic(
                            &node,
                            &ws_outbound,
                            &mut subscriptions,
                            &topic,
                            &foot_bridge,
                        );
                        let _ = respond_to.send(result);
                    }
                    RosCommand::UnsubscribeTopic { topic, respond_to } => {
                        let result = unsubscribe_topic(&mut subscriptions, &topic, &foot_bridge);
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
    foot_bridge: &foot::FootTelemetryBridge,
) -> Result<(), CockpitError> {
    if foot::FootTelemetryBridge::handles_topic(topic) {
        foot_bridge.handle_subscribe(topic);
        return Ok(());
    }

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
    foot_bridge: &foot::FootTelemetryBridge,
) -> Result<(), CockpitError> {
    if !is_supported_topic(topic) {
        return Err(CockpitError::UnsupportedTopic(topic.to_owned()));
    }

    if foot::FootTelemetryBridge::handles_topic(topic) {
        foot_bridge.handle_unsubscribe(topic);
        return Ok(());
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
        || foot::FootTelemetryBridge::handles_topic(topic)
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
                .create_subscription(topic, move |msg: RosString| {
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
                .create_subscription(topic.sensor_data_qos(), move |msg: RosImu| {
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
                    } else {
                        trace!("forwarded imu telemetry frame");
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

mod foot {
    use super::{CockpitError, WsOutbound};
    use create_msgs::msg::{
        Bumper as RosBumper, ChargingState as RosChargingState, Cliff as RosCliff, Mode as RosMode,
    };
    use geometry_msgs::msg::Twist as RosTwist;
    use nav_msgs::msg::Odometry as RosOdometry;
    use rclrs;
    use serde::Serialize;
    use serde_json::json;
    use std::sync::{Arc, Mutex};
    use std::time::{SystemTime, UNIX_EPOCH};
    use std_msgs::msg::{Float32 as RosFloat32, Int16 as RosInt16};
    use tokio::sync::broadcast;
    use tracing::{error, warn};

    const TELEMETRY_TOPIC: &str = "/foot/telemetry";
    const CMD_VEL_TOPIC: &str = "/cmd_vel";

    #[derive(Debug, Clone, Default, Serialize)]
    pub struct FootTelemetrySnapshot {
        #[serde(skip_serializing_if = "Option::is_none")]
        pub battery: Option<BatterySnapshot>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub status: Option<StatusSnapshot>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub motion: Option<MotionSnapshot>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub hazards: Option<HazardSnapshot>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub last_update_ms: Option<u64>,
    }

    #[derive(Debug, Clone, Default, Serialize)]
    pub struct BatterySnapshot {
        #[serde(skip_serializing_if = "Option::is_none")]
        pub percentage: Option<f32>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub charge_ratio: Option<f32>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub charge_ah: Option<f32>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub capacity_ah: Option<f32>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub current_a: Option<f32>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub voltage_v: Option<f32>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub temperature_c: Option<f32>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub charging_state: Option<String>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub is_charging: Option<bool>,
    }

    #[derive(Debug, Clone, Default, Serialize)]
    pub struct StatusSnapshot {
        #[serde(skip_serializing_if = "Option::is_none")]
        pub mode: Option<String>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub mode_code: Option<u8>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub charging_state: Option<String>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub charging_state_code: Option<u8>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub last_command_ms: Option<u64>,
    }

    #[derive(Debug, Clone, Default, Serialize)]
    pub struct MotionSnapshot {
        #[serde(skip_serializing_if = "Option::is_none")]
        pub command: Option<TwistSummary>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub odometry: Option<TwistSummary>,
    }

    #[derive(Debug, Clone, Default, Serialize)]
    pub struct HazardSnapshot {
        #[serde(skip_serializing_if = "Option::is_none")]
        pub bumper_left: Option<bool>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub bumper_right: Option<bool>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub bumper_light_front: Option<bool>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub bumper_light_center: Option<bool>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub cliff_left: Option<bool>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub cliff_front_left: Option<bool>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub cliff_right: Option<bool>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub cliff_front_right: Option<bool>,
    }

    #[derive(Debug, Clone, Default, Serialize)]
    pub struct TwistSummary {
        pub linear_x: f64,
        pub linear_y: f64,
        pub linear_z: f64,
        pub angular_x: f64,
        pub angular_y: f64,
        pub angular_z: f64,
    }

    pub struct FootTelemetryBridge {
        state: Arc<Mutex<FootTelemetrySnapshot>>,
        #[allow(dead_code)]
        subscriptions: Vec<FootSubscription>,
        ws_outbound: broadcast::Sender<WsOutbound>,
    }

    enum FootSubscription {
        Float32(rclrs::Subscription<RosFloat32>),
        Int16(rclrs::Subscription<RosInt16>),
        ChargingState(rclrs::Subscription<RosChargingState>),
        Mode(rclrs::Subscription<RosMode>),
        Bumper(rclrs::Subscription<RosBumper>),
        Cliff(rclrs::Subscription<RosCliff>),
        CmdVel(rclrs::Subscription<RosTwist>),
        Odom(rclrs::Subscription<RosOdometry>),
    }

    impl FootTelemetryBridge {
        pub fn new(
            node: &rclrs::Node,
            ws_outbound: broadcast::Sender<WsOutbound>,
        ) -> Result<Self, CockpitError> {
            let state = Arc::new(Mutex::new(FootTelemetrySnapshot::default()));
            let mut subscriptions = Vec::new();

            subscriptions.push(FootSubscription::Float32(
                node.create_subscription("/battery/charge_ratio", {
                    let state = Arc::clone(&state);
                    let ws = ws_outbound.clone();
                    move |msg: RosFloat32| handle_charge_ratio(&state, &ws, msg.data)
                })
                .map_err(|source| CockpitError::SubscribeFailed {
                    topic: "/battery/charge_ratio".into(),
                    source,
                })?,
            ));

            subscriptions.push(FootSubscription::Float32(
                node.create_subscription("/battery/capacity", {
                    let state = Arc::clone(&state);
                    let ws = ws_outbound.clone();
                    move |msg: RosFloat32| handle_capacity(&state, &ws, msg.data)
                })
                .map_err(|source| CockpitError::SubscribeFailed {
                    topic: "/battery/capacity".into(),
                    source,
                })?,
            ));

            subscriptions.push(FootSubscription::Float32(
                node.create_subscription("/battery/charge", {
                    let state = Arc::clone(&state);
                    let ws = ws_outbound.clone();
                    move |msg: RosFloat32| handle_charge(&state, &ws, msg.data)
                })
                .map_err(|source| CockpitError::SubscribeFailed {
                    topic: "/battery/charge".into(),
                    source,
                })?,
            ));

            subscriptions.push(FootSubscription::Float32(
                node.create_subscription("/battery/current", {
                    let state = Arc::clone(&state);
                    let ws = ws_outbound.clone();
                    move |msg: RosFloat32| handle_current(&state, &ws, msg.data)
                })
                .map_err(|source| CockpitError::SubscribeFailed {
                    topic: "/battery/current".into(),
                    source,
                })?,
            ));

            subscriptions.push(FootSubscription::Float32(
                node.create_subscription("/battery/voltage", {
                    let state = Arc::clone(&state);
                    let ws = ws_outbound.clone();
                    move |msg: RosFloat32| handle_voltage(&state, &ws, msg.data)
                })
                .map_err(|source| CockpitError::SubscribeFailed {
                    topic: "/battery/voltage".into(),
                    source,
                })?,
            ));

            subscriptions.push(FootSubscription::Int16(
                node.create_subscription("/battery/temperature", {
                    let state = Arc::clone(&state);
                    let ws = ws_outbound.clone();
                    move |msg: RosInt16| handle_temperature(&state, &ws, msg.data)
                })
                .map_err(|source| CockpitError::SubscribeFailed {
                    topic: "/battery/temperature".into(),
                    source,
                })?,
            ));

            subscriptions.push(FootSubscription::ChargingState(
                node.create_subscription("/battery/charging_state", {
                    let state = Arc::clone(&state);
                    let ws = ws_outbound.clone();
                    move |msg: RosChargingState| handle_charging_state(&state, &ws, msg)
                })
                .map_err(|source| CockpitError::SubscribeFailed {
                    topic: "/battery/charging_state".into(),
                    source,
                })?,
            ));

            subscriptions.push(FootSubscription::Mode(
                node.create_subscription("/mode", {
                    let state = Arc::clone(&state);
                    let ws = ws_outbound.clone();
                    move |msg: RosMode| handle_mode(&state, &ws, msg)
                })
                .map_err(|source| CockpitError::SubscribeFailed {
                    topic: "/mode".into(),
                    source,
                })?,
            ));

            subscriptions.push(FootSubscription::Bumper(
                node.create_subscription("/bumper", {
                    let state = Arc::clone(&state);
                    let ws = ws_outbound.clone();
                    move |msg: RosBumper| handle_bumper(&state, &ws, msg)
                })
                .map_err(|source| CockpitError::SubscribeFailed {
                    topic: "/bumper".into(),
                    source,
                })?,
            ));

            subscriptions.push(FootSubscription::Cliff(
                node.create_subscription("/cliff", {
                    let state = Arc::clone(&state);
                    let ws = ws_outbound.clone();
                    move |msg: RosCliff| handle_cliff(&state, &ws, msg)
                })
                .map_err(|source| CockpitError::SubscribeFailed {
                    topic: "/cliff".into(),
                    source,
                })?,
            ));

            subscriptions.push(FootSubscription::CmdVel(
                node.create_subscription("/cmd_vel", {
                    let state = Arc::clone(&state);
                    let ws = ws_outbound.clone();
                    move |msg: RosTwist| handle_cmd_vel(&state, &ws, &msg)
                })
                .map_err(|source| CockpitError::SubscribeFailed {
                    topic: "/cmd_vel".into(),
                    source,
                })?,
            ));

            subscriptions.push(FootSubscription::Odom(
                node.create_subscription("/odom", {
                    let state = Arc::clone(&state);
                    let ws = ws_outbound.clone();
                    move |msg: RosOdometry| handle_odom(&state, &ws, &msg)
                })
                .map_err(|source| CockpitError::SubscribeFailed {
                    topic: "/odom".into(),
                    source,
                })?,
            ));

            Ok(Self {
                state,
                subscriptions,
                ws_outbound,
            })
        }

        pub fn handles_topic(topic: &str) -> bool {
            matches!(topic, TELEMETRY_TOPIC | CMD_VEL_TOPIC)
        }

        pub fn handle_subscribe(&self, topic: &str) {
            match topic {
                TELEMETRY_TOPIC => {
                    if let Some(snapshot) = self.snapshot() {
                        broadcast_snapshot(&self.ws_outbound, snapshot);
                    }
                }
                CMD_VEL_TOPIC => {
                    if let Some(snapshot) = self.snapshot() {
                        if let Some(summary) = snapshot.motion.and_then(|motion| motion.command) {
                            self.send_cmd_vel_summary(&summary);
                        }
                    }
                }
                _ => {}
            }
        }

        pub fn handle_unsubscribe(&self, _topic: &str) {}

        pub fn record_command(&self, twist: &RosTwist) {
            handle_cmd_vel(&self.state, &self.ws_outbound, twist);
        }

        fn snapshot(&self) -> Option<FootTelemetrySnapshot> {
            match self.state.lock() {
                Ok(guard) => Some(guard.clone()),
                Err(poisoned) => {
                    error!("foot telemetry state mutex poisoned");
                    Some(poisoned.into_inner().clone())
                }
            }
        }

        fn send_cmd_vel_summary(&self, summary: &TwistSummary) {
            let payload = json!({
                "linear": {
                    "x": summary.linear_x,
                    "y": summary.linear_y,
                    "z": summary.linear_z,
                },
                "angular": {
                    "x": summary.angular_x,
                    "y": summary.angular_y,
                    "z": summary.angular_z,
                },
            });
            if let Err(err) = self.ws_outbound.send(WsOutbound::Msg {
                topic: CMD_VEL_TOPIC.into(),
                msg: payload,
            }) {
                warn!(?err, "no websocket listeners for /cmd_vel broadcast");
            }
        }
    }

    fn handle_charge_ratio(
        state: &Arc<Mutex<FootTelemetrySnapshot>>,
        ws: &broadcast::Sender<WsOutbound>,
        ratio: f32,
    ) {
        if let Some(snapshot) = update_state(state, |state, _| {
            let ratio = ratio.clamp(0.0, 1.0);
            let battery = battery_section(state);
            battery.charge_ratio = Some(ratio);
            battery.percentage = Some(ratio * 100.0);
        }) {
            broadcast_snapshot(ws, snapshot);
        }
    }

    fn handle_capacity(
        state: &Arc<Mutex<FootTelemetrySnapshot>>,
        ws: &broadcast::Sender<WsOutbound>,
        capacity: f32,
    ) {
        if let Some(snapshot) = update_state(state, |state, _| {
            let battery = battery_section(state);
            battery.capacity_ah = Some(capacity);
        }) {
            broadcast_snapshot(ws, snapshot);
        }
    }

    fn handle_charge(
        state: &Arc<Mutex<FootTelemetrySnapshot>>,
        ws: &broadcast::Sender<WsOutbound>,
        charge: f32,
    ) {
        if let Some(snapshot) = update_state(state, |state, _| {
            let battery = battery_section(state);
            battery.charge_ah = Some(charge.max(0.0));
        }) {
            broadcast_snapshot(ws, snapshot);
        }
    }

    fn handle_current(
        state: &Arc<Mutex<FootTelemetrySnapshot>>,
        ws: &broadcast::Sender<WsOutbound>,
        current: f32,
    ) {
        if let Some(snapshot) = update_state(state, |state, _| {
            let battery = battery_section(state);
            battery.current_a = Some(current);
            battery.is_charging = Some(current > 0.0);
        }) {
            broadcast_snapshot(ws, snapshot);
        }
    }

    fn handle_voltage(
        state: &Arc<Mutex<FootTelemetrySnapshot>>,
        ws: &broadcast::Sender<WsOutbound>,
        voltage: f32,
    ) {
        if let Some(snapshot) = update_state(state, |state, _| {
            let battery = battery_section(state);
            battery.voltage_v = Some(voltage);
        }) {
            broadcast_snapshot(ws, snapshot);
        }
    }

    fn handle_temperature(
        state: &Arc<Mutex<FootTelemetrySnapshot>>,
        ws: &broadcast::Sender<WsOutbound>,
        temperature: i16,
    ) {
        if let Some(snapshot) = update_state(state, |state, _| {
            let battery = battery_section(state);
            battery.temperature_c = Some(temperature as f32);
        }) {
            broadcast_snapshot(ws, snapshot);
        }
    }

    fn handle_charging_state(
        state: &Arc<Mutex<FootTelemetrySnapshot>>,
        ws: &broadcast::Sender<WsOutbound>,
        msg: RosChargingState,
    ) {
        if let Some(snapshot) = update_state(state, |state, _| {
            let label = charging_state_label(msg.state).to_owned();
            let battery = battery_section(state);
            battery.charging_state = Some(label.clone());
            battery.is_charging = Some(matches!(
                msg.state,
                RosChargingState::CHARGE_RECONDITION
                    | RosChargingState::CHARGE_FULL
                    | RosChargingState::CHARGE_TRICKLE
                    | RosChargingState::CHARGE_WAITING
            ));
            let status = status_section(state);
            status.charging_state = Some(label);
            status.charging_state_code = Some(msg.state);
        }) {
            broadcast_snapshot(ws, snapshot);
        }
    }

    fn handle_mode(
        state: &Arc<Mutex<FootTelemetrySnapshot>>,
        ws: &broadcast::Sender<WsOutbound>,
        msg: RosMode,
    ) {
        if let Some(snapshot) = update_state(state, |state, _| {
            let status = status_section(state);
            status.mode_code = Some(msg.mode);
            status.mode = Some(mode_label(msg.mode).to_owned());
        }) {
            broadcast_snapshot(ws, snapshot);
        }
    }

    fn handle_bumper(
        state: &Arc<Mutex<FootTelemetrySnapshot>>,
        ws: &broadcast::Sender<WsOutbound>,
        msg: RosBumper,
    ) {
        if let Some(snapshot) = update_state(state, |state, _| {
            let hazards = hazard_section(state);
            hazards.bumper_left = Some(msg.is_left_pressed);
            hazards.bumper_right = Some(msg.is_right_pressed);
            hazards.bumper_light_front = Some(msg.is_light_front_left || msg.is_light_front_right);
            hazards.bumper_light_center =
                Some(msg.is_light_center_left || msg.is_light_center_right);
        }) {
            broadcast_snapshot(ws, snapshot);
        }
    }

    fn handle_cliff(
        state: &Arc<Mutex<FootTelemetrySnapshot>>,
        ws: &broadcast::Sender<WsOutbound>,
        msg: RosCliff,
    ) {
        if let Some(snapshot) = update_state(state, |state, _| {
            let hazards = hazard_section(state);
            hazards.cliff_left = Some(msg.is_cliff_left);
            hazards.cliff_front_left = Some(msg.is_cliff_front_left);
            hazards.cliff_right = Some(msg.is_cliff_right);
            hazards.cliff_front_right = Some(msg.is_cliff_front_right);
        }) {
            broadcast_snapshot(ws, snapshot);
        }
    }

    fn handle_cmd_vel(
        state: &Arc<Mutex<FootTelemetrySnapshot>>,
        ws: &broadcast::Sender<WsOutbound>,
        twist: &RosTwist,
    ) {
        if let Some(snapshot) = update_state(state, |state, now| {
            let motion = motion_section(state);
            motion.command = Some(summary_from_twist(twist));
            let status = status_section(state);
            status.last_command_ms = Some(now);
        }) {
            broadcast_cmd_vel_from_twist(ws, twist);
            broadcast_snapshot(ws, snapshot);
        }
    }

    fn handle_odom(
        state: &Arc<Mutex<FootTelemetrySnapshot>>,
        ws: &broadcast::Sender<WsOutbound>,
        msg: &RosOdometry,
    ) {
        if let Some(snapshot) = update_state(state, |state, _| {
            let motion = motion_section(state);
            motion.odometry = Some(summary_from_odometry(msg));
        }) {
            broadcast_snapshot(ws, snapshot);
        }
    }

    fn update_state<F>(
        state: &Arc<Mutex<FootTelemetrySnapshot>>,
        mut update: F,
    ) -> Option<FootTelemetrySnapshot>
    where
        F: FnMut(&mut FootTelemetrySnapshot, u64),
    {
        let mut guard = match state.lock() {
            Ok(guard) => guard,
            Err(poisoned) => {
                error!("foot telemetry state mutex poisoned");
                poisoned.into_inner()
            }
        };
        let timestamp = now_millis();
        update(&mut guard, timestamp);
        guard.last_update_ms = Some(timestamp);
        Some(guard.clone())
    }

    fn broadcast_snapshot(ws: &broadcast::Sender<WsOutbound>, snapshot: FootTelemetrySnapshot) {
        match serde_json::to_value(snapshot) {
            Ok(msg) => {
                if let Err(err) = ws.send(WsOutbound::Msg {
                    topic: TELEMETRY_TOPIC.into(),
                    msg,
                }) {
                    warn!(?err, "no websocket listeners for foot telemetry broadcast");
                }
            }
            Err(err) => {
                error!(?err, "failed to serialize foot telemetry snapshot");
            }
        }
    }

    fn broadcast_cmd_vel_from_twist(ws: &broadcast::Sender<WsOutbound>, twist: &RosTwist) {
        broadcast_cmd_vel_summary(ws, &summary_from_twist(twist));
    }

    fn broadcast_cmd_vel_summary(ws: &broadcast::Sender<WsOutbound>, summary: &TwistSummary) {
        let payload = json!({
            "linear": {
                "x": summary.linear_x,
                "y": summary.linear_y,
                "z": summary.linear_z,
            },
            "angular": {
                "x": summary.angular_x,
                "y": summary.angular_y,
                "z": summary.angular_z,
            },
        });
        if let Err(err) = ws.send(WsOutbound::Msg {
            topic: CMD_VEL_TOPIC.into(),
            msg: payload,
        }) {
            warn!(?err, "no websocket listeners for /cmd_vel broadcast");
        }
    }

    fn battery_section(state: &mut FootTelemetrySnapshot) -> &mut BatterySnapshot {
        state.battery.get_or_insert_with(BatterySnapshot::default)
    }

    fn status_section(state: &mut FootTelemetrySnapshot) -> &mut StatusSnapshot {
        state.status.get_or_insert_with(StatusSnapshot::default)
    }

    fn motion_section(state: &mut FootTelemetrySnapshot) -> &mut MotionSnapshot {
        state.motion.get_or_insert_with(MotionSnapshot::default)
    }

    fn hazard_section(state: &mut FootTelemetrySnapshot) -> &mut HazardSnapshot {
        state.hazards.get_or_insert_with(HazardSnapshot::default)
    }

    fn summary_from_twist(twist: &RosTwist) -> TwistSummary {
        TwistSummary {
            linear_x: twist.linear.x,
            linear_y: twist.linear.y,
            linear_z: twist.linear.z,
            angular_x: twist.angular.x,
            angular_y: twist.angular.y,
            angular_z: twist.angular.z,
        }
    }

    fn summary_from_odometry(msg: &RosOdometry) -> TwistSummary {
        let twist = &msg.twist.twist;
        TwistSummary {
            linear_x: twist.linear.x,
            linear_y: twist.linear.y,
            linear_z: twist.linear.z,
            angular_x: twist.angular.x,
            angular_y: twist.angular.y,
            angular_z: twist.angular.z,
        }
    }

    fn charging_state_label(state: u8) -> &'static str {
        match state {
            RosChargingState::CHARGE_NONE => "Idle",
            RosChargingState::CHARGE_RECONDITION => "Recondition",
            RosChargingState::CHARGE_FULL => "Full",
            RosChargingState::CHARGE_TRICKLE => "Trickle",
            RosChargingState::CHARGE_WAITING => "Waiting",
            RosChargingState::CHARGE_FAULT => "Fault",
            _ => "Unknown",
        }
    }

    fn mode_label(mode: u8) -> &'static str {
        match mode {
            RosMode::MODE_OFF => "Off",
            RosMode::MODE_PASSIVE => "Passive",
            RosMode::MODE_SAFE => "Safe",
            RosMode::MODE_FULL => "Full",
            _ => "Unknown",
        }
    }

    fn now_millis() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|duration| duration.as_millis() as u64)
            .unwrap_or(0)
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        use geometry_msgs::msg::{Twist as RosTwist, Vector3 as RosVector3};

        #[test]
        fn charging_state_labels_cover_known_values() {
            assert_eq!(charging_state_label(RosChargingState::CHARGE_NONE), "Idle");
            assert_eq!(
                charging_state_label(RosChargingState::CHARGE_FAULT),
                "Fault"
            );
            assert_eq!(charging_state_label(99), "Unknown");
        }

        #[test]
        fn mode_label_covers_known_values() {
            assert_eq!(mode_label(RosMode::MODE_SAFE), "Safe");
            assert_eq!(mode_label(42), "Unknown");
        }

        #[test]
        fn summary_from_twist_maps_all_axes() {
            let twist = RosTwist {
                linear: RosVector3 {
                    x: 0.4,
                    y: -0.1,
                    z: 0.2,
                },
                angular: RosVector3 {
                    x: 0.5,
                    y: 0.0,
                    z: -0.75,
                },
            };
            let summary = summary_from_twist(&twist);
            assert_eq!(summary.linear_x, 0.4);
            assert_eq!(summary.linear_y, -0.1);
            assert_eq!(summary.linear_z, 0.2);
            assert_eq!(summary.angular_z, -0.75);
        }
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
