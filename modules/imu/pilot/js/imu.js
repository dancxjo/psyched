import { createTopicSocket } from '/js/pilot.js';

export function imuDashboard() {
  return {
    status: 'Connecting…',
    orientation: { x: 0, y: 0, z: 0, w: 0 },
    angularVelocity: { x: 0, y: 0, z: 0 },
    linearAcceleration: { x: 0, y: 0, z: 0 },
    temperatureC: null,
    temperatureF: null,
    init() {
      this.connectImu();
      this.connectTemperature();
    },
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
    connectTemperature() {
      const socket = createTopicSocket({
        topic: '/imu/temperature',
        type: 'std_msgs/msg/Float32',
        role: 'subscribe',
      });
      socket.addEventListener('message', (event) => {
        const payload = JSON.parse(event.data);
        if (payload.event === 'message' && payload.data && 'data' in payload.data) {
          const celsius = Number(payload.data.data);
          if (!Number.isNaN(celsius)) {
            this.temperatureC = celsius;
            this.temperatureF = celsius * (9 / 5) + 32;
          }
        }
      });
    },
    format(value, fractionDigits = 2) {
      return Number(value).toFixed(fractionDigits);
    },
  };
}

window.Imu = { imuDashboard };
