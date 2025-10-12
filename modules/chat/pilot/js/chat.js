import { createTopicSocket } from '/js/pilot.js';

function generateId() {
  if (typeof crypto !== 'undefined' && crypto.randomUUID) {
    return crypto.randomUUID();
  }
  return `msg-${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

export function chatDashboard() {
  return {
    status: 'Connecting…',
    voiceStatus: 'Connecting…',
    formFeedback: '',
    voiceFeedback: '',
    conversationPublisher: null,
    voicePublisher: null,
    messages: [],
    voiceLast: '',
    roles: ['user', 'assistant', 'system', 'pilot'],
    composer: {
      role: 'user',
      speaker: 'pilot',
      confidence: 1.0,
      content: '',
    },
    voiceCommand: '',
    init() {
      this.connectConversation();
      this.connectVoice();
    },
    connectConversation() {
      const socket = createTopicSocket({
        topic: '/conversation',
        type: 'psyched_msgs/msg/Message',
        role: 'subscribe',
      });
      socket.addEventListener('message', (event) => {
        const payload = JSON.parse(event.data);
        if (payload.event === 'message' && payload.data) {
          this.addMessage(payload.data);
          this.status = 'Live';
        }
      });
      socket.addEventListener('close', () => {
        this.status = 'Disconnected';
      });
      socket.addEventListener('error', () => {
        this.status = 'Error';
      });
      this.conversationPublisher = createTopicSocket({
        topic: '/conversation',
        type: 'psyched_msgs/msg/Message',
        role: 'publish',
      });
      this.conversationPublisher.addEventListener('open', () => {
        this.formFeedback = '';
      });
      this.conversationPublisher.addEventListener('error', () => {
        this.formFeedback = 'Unable to publish to /conversation';
      });
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
          this.voiceLast = String(payload.data.data ?? '');
          this.voiceStatus = 'Live';
        }
      });
      socket.addEventListener('close', () => {
        this.voiceStatus = 'Disconnected';
      });
      socket.addEventListener('error', () => {
        this.voiceStatus = 'Error';
      });
      this.voicePublisher = createTopicSocket({
        topic: '/voice',
        type: 'std_msgs/msg/String',
        role: 'publish',
      });
      this.voicePublisher.addEventListener('open', () => {
        this.voiceFeedback = '';
      });
      this.voicePublisher.addEventListener('error', () => {
        this.voiceFeedback = 'Unable to publish to /voice';
      });
    },
    addMessage(data) {
      const message = {
        id: generateId(),
        role: data.role || 'unknown',
        speaker: data.speaker || '',
        confidence: typeof data.confidence === 'number' ? data.confidence : null,
        content: data.content || '',
        timestamp: new Date().toLocaleTimeString(),
      };
      this.messages = [message, ...this.messages].slice(0, 50);
    },
    formatConfidence(value) {
      if (typeof value !== 'number' || Number.isNaN(value)) {
        return '—';
      }
      return `${Math.round(value * 100)}%`;
    },
    sendMessage() {
      const text = this.composer.content.trim();
      if (!text) {
        this.formFeedback = 'Message text required';
        return;
      }
      const payload = {
        role: this.composer.role || 'user',
        content: text,
        speaker: this.composer.speaker || 'pilot',
        confidence: Number.isFinite(Number(this.composer.confidence))
          ? Number(this.composer.confidence)
          : 1.0,
        segments: [],
        words: [],
      };
      try {
        const publisher = this.conversationPublisher || createTopicSocket({
          topic: '/conversation',
          type: 'psyched_msgs/msg/Message',
          role: 'publish',
        });
        this.conversationPublisher = publisher;
        publisher.send(JSON.stringify(payload));
        this.formFeedback = 'Message sent to /conversation';
        this.composer.content = '';
      } catch (error) {
        this.formFeedback = error instanceof Error ? error.message : String(error);
      }
    },
    sendVoice() {
      const text = this.voiceCommand.trim();
      if (!text) {
        this.voiceFeedback = 'Text required to publish to /voice';
        return;
      }
      const payload = { data: text };
      try {
        const publisher = this.voicePublisher || createTopicSocket({
          topic: '/voice',
          type: 'std_msgs/msg/String',
          role: 'publish',
        });
        this.voicePublisher = publisher;
        publisher.send(JSON.stringify(payload));
        this.voiceFeedback = 'Voice command sent';
        this.voiceCommand = '';
        this.voiceLast = text;
      } catch (error) {
        this.voiceFeedback = error instanceof Error ? error.message : String(error);
      }
    },
  };
}

window.Chat = { chatDashboard };
