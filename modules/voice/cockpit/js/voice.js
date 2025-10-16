import { createTopicSocket } from '/js/cockpit.js';

function generateId() {
  if (typeof crypto !== 'undefined' && crypto.randomUUID) {
    return crypto.randomUUID();
  }
  return `event-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

export function voiceDashboard() {
  return {
    status: 'Connecting…',
    voiceFeedback: '',
    commandFeedback: '',
    volumeFeedback: '',
    voiceMessage: '',
    volume: 255,
    lastVoice: '',
    eventLog: [],
    voicePublisher: null,
    interruptPublisher: null,
    resumePublisher: null,
    clearPublisher: null,
    volumePublisher: null,
    init() {
      this.connectVoice();
      this.connectEventStreams();
    },
    connectVoice() {
      const socket = createTopicSocket({
        topic: '/voice',
        type: 'std_msgs/msg/String',
        role: 'subscribe',
      });
      socket.addEventListener('message', (event) => {
        const payload = JSON.parse(event.data);
        if (payload.event === 'message' && payload.data && typeof payload.data.data !== 'undefined') {
          this.lastVoice = String(payload.data.data ?? '');
          this.status = 'Live';
        }
      });
      socket.addEventListener('close', () => {
        this.status = 'Disconnected';
      });
      socket.addEventListener('error', () => {
        this.status = 'Error';
      });
      this.ensureVoicePublisher();
    },
    connectEventStreams() {
      this.createEventListener('/voice_done', 'Playback complete');
      this.createEventListener('/voice_interrupt', 'Playback interrupted');
    },
    createEventListener(topic, label) {
      const socket = createTopicSocket({
        topic,
        type: 'std_msgs/msg/Empty',
        role: 'subscribe',
      });
      socket.addEventListener('message', () => {
        const event = {
          id: generateId(),
          label,
          topic,
          timestamp: new Date().toLocaleTimeString(),
        };
        this.eventLog = [event, ...this.eventLog].slice(0, 30);
      });
    },
    ensureVoicePublisher() {
      if (this.voicePublisher) {
        return this.voicePublisher;
      }
      const publisher = createTopicSocket({
        topic: '/voice',
        type: 'std_msgs/msg/String',
        role: 'publish',
      });
      publisher.addEventListener('open', () => {
        this.voiceFeedback = '';
      });
      publisher.addEventListener('error', () => {
        this.voiceFeedback = 'Unable to publish to /voice';
      });
      this.voicePublisher = publisher;
      return publisher;
    },
    ensurePublisher(property, topic, type, onError) {
      if (this[property]) {
        return this[property];
      }
      const publisher = createTopicSocket({
        topic,
        type,
        role: 'publish',
      });
      publisher.addEventListener('error', () => {
        const message = `Unable to publish to ${topic}`;
        if (typeof onError === 'function') {
          onError(message);
        } else {
          this.commandFeedback = message;
        }
      });
      this[property] = publisher;
      return publisher;
    },
    sendVoice() {
      const text = this.voiceMessage.trim();
      if (!text) {
        this.voiceFeedback = 'Message text required for /voice';
        return;
      }
      try {
        const publisher = this.ensureVoicePublisher();
        publisher.send(JSON.stringify({ data: text }));
        this.voiceFeedback = 'Voice message sent';
        this.voiceMessage = '';
        this.lastVoice = text;
      } catch (error) {
        this.voiceFeedback = error instanceof Error ? error.message : String(error);
      }
    },
    sendInterrupt() {
      this.sendEmptyCommand('interruptPublisher', '/voice/interrupt', 'std_msgs/msg/Empty', 'Interrupt request sent');
    },
    sendResume() {
      this.sendEmptyCommand('resumePublisher', '/voice/resume', 'std_msgs/msg/Empty', 'Resume request sent');
    },
    sendClear() {
      this.sendEmptyCommand('clearPublisher', '/voice/clear', 'std_msgs/msg/Empty', 'Clear request sent');
    },
    sendEmptyCommand(property, topic, type, successMessage) {
      try {
        const publisher = this.ensurePublisher(property, topic, type);
        publisher.send(JSON.stringify({}));
        this.commandFeedback = successMessage;
      } catch (error) {
        this.commandFeedback = error instanceof Error ? error.message : String(error);
      }
    },
    applyVolume() {
      const value = Math.max(0, Math.min(255, Number(this.volume)));
      this.volume = value;
      try {
        const publisher = this.ensurePublisher(
          'volumePublisher',
          '/voice/volume',
          'std_msgs/msg/UInt8',
          (message) => {
            this.volumeFeedback = message;
          },
        );
        publisher.send(JSON.stringify({ data: value }));
        this.volumeFeedback = `Volume command sent (${value})`;
      } catch (error) {
        this.volumeFeedback = error instanceof Error ? error.message : String(error);
      }
    },
  };
}

window.Voice = { voiceDashboard };
