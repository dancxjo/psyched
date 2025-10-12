import { createTopicSocket } from '/js/pilot.js';
import '/components/joystick-control.js';

const FOOT_TOPICS = [
  {
    name: 'battery/capacity',
    type: 'std_msgs/msg/Float32',
    direction: 'publisher',
    description: 'Estimated charge capacity of the robot battery (Ah).',
  },
  {
    name: 'battery/charge',
    type: 'std_msgs/msg/Float32',
    direction: 'publisher',
    description: 'Current charge level of the battery (Ah).',
  },
  {
    name: 'battery/charge_ratio',
    type: 'std_msgs/msg/Float32',
    direction: 'publisher',
    description: 'Ratio of charge to capacity.',
  },
  {
    name: 'battery/charging_state',
    type: 'create_msgs/msg/ChargingState',
    direction: 'publisher',
    description: 'Charging state of the Create base.',
  },
  {
    name: 'battery/current',
    type: 'std_msgs/msg/Float32',
    direction: 'publisher',
    description: 'Current flowing through the battery (A). Positive when charging.',
  },
  {
    name: 'battery/temperature',
    type: 'std_msgs/msg/Int16',
    direction: 'publisher',
    description: 'Battery temperature in Celsius.',
  },
  {
    name: 'battery/voltage',
    type: 'std_msgs/msg/Float32',
    direction: 'publisher',
    description: 'Battery voltage (V).',
  },
  {
    name: 'bumper',
    type: 'create_msgs/msg/Bumper',
    direction: 'publisher',
    description: 'Bumper state including light sensors.',
  },
  {
    name: 'cliff',
    type: 'create_msgs/msg/Cliff',
    direction: 'publisher',
    description: 'Cliff sensor state.',
  },
  {
    name: 'clean_button',
    type: 'std_msgs/msg/Empty',
    direction: 'publisher',
    description: "Reports when the 'clean' button is pressed.",
  },
  {
    name: 'day_button',
    type: 'std_msgs/msg/Empty',
    direction: 'publisher',
    description: "Reports when the 'day' button is pressed.",
  },
  {
    name: 'hour_button',
    type: 'std_msgs/msg/Empty',
    direction: 'publisher',
    description: "Reports when the 'hour' button is pressed.",
  },
  {
    name: 'minute_button',
    type: 'std_msgs/msg/Empty',
    direction: 'publisher',
    description: "Reports when the 'minute' button is pressed.",
  },
  {
    name: 'dock_button',
    type: 'std_msgs/msg/Empty',
    direction: 'publisher',
    description: "Reports when the 'dock' button is pressed.",
  },
  {
    name: 'spot_button',
    type: 'std_msgs/msg/Empty',
    direction: 'publisher',
    description: "Reports when the 'spot' button is pressed.",
  },
  {
    name: 'ir_omni',
    type: 'std_msgs/msg/UInt16',
    direction: 'publisher',
    description: 'IR character read by the omnidirectional receiver.',
  },
  {
    name: 'joint_states',
    type: 'sensor_msgs/msg/JointState',
    direction: 'publisher',
    description: 'Wheel joint positions and velocities.',
  },
  {
    name: 'mode',
    type: 'create_msgs/msg/Mode',
    direction: 'publisher',
    description: 'Current Create operating mode.',
  },
  {
    name: 'odom',
    type: 'nav_msgs/msg/Odometry',
    direction: 'publisher',
    description: 'Odometry from wheel encoders.',
  },
  {
    name: 'wheeldrop',
    type: 'std_msgs/msg/Empty',
    direction: 'publisher',
    description: 'Indicates at least one wheel has dropped.',
  },
  {
    name: '/tf',
    type: 'tf2_msgs/msg/TFMessage',
    direction: 'publisher',
    description: 'Transform from odom to base_footprint when publish_tf is true.',
  },
  {
    name: 'diagnostics',
    type: 'diagnostic_msgs/msg/DiagnosticArray',
    direction: 'publisher',
    description: 'Diagnostics covering battery, wheeldrop/cliff, mode, and serial status.',
  },
  {
    name: 'cmd_vel',
    type: 'geometry_msgs/msg/Twist',
    direction: 'subscriber',
    description: 'Command robot velocity.',
    subscribe: true,
    example: {
      linear: { x: 0.1, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: 0.0 },
    },
  },
  {
    name: 'debris_led',
    type: 'std_msgs/msg/Bool',
    direction: 'subscriber',
    description: "Toggle the 'debris' LED.",
    example: { data: true },
  },
  {
    name: 'spot_led',
    type: 'std_msgs/msg/Bool',
    direction: 'subscriber',
    description: "Toggle the 'spot' LED.",
    example: { data: true },
  },
  {
    name: 'dock_led',
    type: 'std_msgs/msg/Bool',
    direction: 'subscriber',
    description: "Toggle the 'dock' LED.",
    example: { data: true },
  },
  {
    name: 'check_led',
    type: 'std_msgs/msg/Bool',
    direction: 'subscriber',
    description: "Toggle the 'check robot' LED.",
    example: { data: false },
  },
  {
    name: 'power_led',
    type: 'std_msgs/msg/UInt8MultiArray',
    direction: 'subscriber',
    description: "Set the 'power' LED color and intensity.",
    example: { data: [0, 255] },
  },
  {
    name: 'set_ascii',
    type: 'std_msgs/msg/UInt8MultiArray',
    direction: 'subscriber',
    description: 'Set the 4-digit ASCII display.',
    example: { data: [80, 69, 84, 69] },
  },
  {
    name: 'dock',
    type: 'std_msgs/msg/Empty',
    direction: 'subscriber',
    description: 'Activate docking behaviour (Passive mode).',
    example: {},
  },
  {
    name: 'undock',
    type: 'std_msgs/msg/Empty',
    direction: 'subscriber',
    description: 'Return control to Full mode.',
    example: {},
  },
  {
    name: 'define_song',
    type: 'create_msgs/msg/DefineSong',
    direction: 'subscriber',
    description: 'Define a song with up to 16 notes.',
    example: {
      song: { number: 0 },
      notes: [{ note: 64, duration: 0.5 }],
    },
  },
  {
    name: 'play_song',
    type: 'create_msgs/msg/PlaySong',
    direction: 'subscriber',
    description: 'Play a predefined song.',
    example: { song: 0 },
  },
];

export function footDashboard() {
  return {
    topics: FOOT_TOPICS,
    sockets: {},
    latest: {},
    errors: {},
    commands: {},
    cmdVelPublisher: null,
    velocityLimits: {
      linear: { min: -0.5, max: 0.5 },
      angular: { min: -4.25, max: 4.25 },
    },
    init() {
      this.topics
        .filter((topic) => topic.direction !== 'subscriber' || topic.subscribe)
        .forEach((topic) => this.ensureSocket(topic, 'subscribe'));
      this.$nextTick(() => {
        this.setupJoystick();
      });
    },
    ensureSocket(topic, role) {
      const key = `${topic.name}:${role}`;
      if (this.sockets[key]) {
        return this.sockets[key];
      }
      const socket = createTopicSocket({
        topic: topic.name,
        type: topic.type,
        role,
      });
      if (role === 'subscribe') {
        socket.addEventListener('message', (event) => {
          const payload = JSON.parse(event.data);
          if (payload.event === 'message') {
            this.latest[topic.name] = payload.data;
          }
        });
      }
      socket.addEventListener('open', () => {
        if (role === 'publish') {
          this.errors[topic.name] = '';
        }
      });
      socket.addEventListener('error', () => {
        this.errors[topic.name] = 'Connection error';
      });
      if (topic.name === 'cmd_vel' && role === 'publish') {
        this.cmdVelPublisher = socket;
      }
      this.sockets[key] = socket;
      return socket;
    },
    formatted(topic) {
      const data = this.latest[topic.name];
      if (!data) {
        return '—';
      }
      return JSON.stringify(data, null, 2);
    },
    example(topic) {
      if (!topic.example) {
        return '';
      }
      return JSON.stringify(topic.example, null, 2);
    },
    setupJoystick() {
      const joystick = this.$refs?.cmdVelJoystick;
      const topic = this.topics.find((entry) => entry.name === 'cmd_vel');
      if (!joystick || !topic) {
        return;
      }
      const record = {
        send: (message) => this.sendJoystickCommand(topic, message),
      };
      joystick.record = record;
    },
    sendJoystickCommand(topic, message) {
      try {
        const socket = this.cmdVelPublisher || this.ensureSocket(topic, 'publish');
        socket.send(JSON.stringify(message));
        this.latest[topic.name] = message;
        this.errors[topic.name] = 'Command sent';
      } catch (error) {
        this.errors[topic.name] = error instanceof Error ? error.message : String(error);
      }
    },
    sendCommand(topic) {
      const socket = this.ensureSocket(topic, 'publish');
      const body = this.commands[topic.name] || this.example(topic);
      if (!body.trim()) {
        this.errors[topic.name] = 'Message body required';
        return;
      }
      try {
        const parsed = JSON.parse(body);
        socket.send(JSON.stringify(parsed));
        this.errors[topic.name] = 'Command sent';
        this.latest[topic.name] = parsed;
      } catch (error) {
        this.errors[topic.name] = error instanceof Error ? error.message : String(error);
      }
    },
  };
}

window.Foot = { footDashboard };
