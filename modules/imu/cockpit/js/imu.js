import { createTopicSocket } from '/js/cockpit.js';

export function imuDashboard() {
  return {
    status: 'Connecting…',
    orientation: { x: 0, y: 0, z: 0, w: 0 },
    angularVelocity: { x: 0, y: 0, z: 0 },
    linearAcceleration: { x: 0, y: 0, z: 0 },
    init() {
      this.connectImu();
    },
    /**
     * Subscribe to the IMU topic and keep the dashboard's kinematic telemetry fresh.
     *
     * Temperature readings are intentionally omitted because the current IMU lacks
     * a thermistor channel.
     */
    connectImu() {
      const socket = createTopicSocket({
        topic: '/imu/data',
        type: 'sensor_msgs/msg/Imu',
        role: 'subscribe',
      });
      socket.addEventListener('message', (event) => {
        const payload = JSON.parse(event.data);
        if (payload.event === 'message' && payload.data) {
          const data = payload.data;
          this.orientation = data.orientation ?? this.orientation;
          this.angularVelocity = data.angular_velocity ?? this.angularVelocity;
          this.linearAcceleration = data.linear_acceleration ?? this.linearAcceleration;
          this.status = 'Live';
        }
      });
      socket.addEventListener('close', () => {
        this.status = 'Disconnected';
      });
      socket.addEventListener('error', () => {
        this.status = 'Error';
      });
    },
    format(value, fractionDigits = 2) {
      return Number(value).toFixed(fractionDigits);
    },
  };
}

window.Imu = { imuDashboard };
