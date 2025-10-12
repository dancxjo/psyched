import { LitElement, html } from 'https://unpkg.com/lit@3.1.4/index.js?module';

import { updateVoiceAction } from '../utils/voice.js';

function actionFromTopic(topic) {
  if (typeof topic !== 'string') {
    return '';
  }
  const segments = topic.split('/').filter(Boolean);
  return segments.length ? segments[segments.length - 1] : topic;
}

/**
 * Registers a voice transport command with the shared control store.
 */
class PilotVoiceControlBridge extends LitElement {
  static properties = {
    record: { type: Object },
    topic: { type: Object },
    _action: { state: true },
  };

  constructor() {
    super();
    this.record = null;
    this.topic = null;
    this._action = '';
    this._cleanup = null;
  }

  createRenderRoot() {
    return this;
  }

  connectedCallback() {
    super.connectedCallback();
    this._register();
  }

  disconnectedCallback() {
    super.disconnectedCallback();
    if (typeof this._cleanup === 'function') {
      this._cleanup();
    }
    this._cleanup = null;
  }

  updated(changed) {
    if (changed.has('topic')) {
      this._action = actionFromTopic(this.topic?.topic ?? this.topic?.name ?? '');
    }
    if (changed.has('record') || changed.has('topic')) {
      this._register();
    }
  }

  _register() {
    const action = this._action;
    if (!action) {
      return;
    }
    if (typeof this._cleanup === 'function') {
      this._cleanup();
      this._cleanup = null;
    }
    if (this.record?.send) {
      const descriptor = {
        send: (message = {}) => {
          try {
            this.record.send(message);
          } catch (error) {
            console.warn('Failed to publish voice action', action, error);
          }
        },
        state: this.record?.state ?? 'idle',
      };
      this._cleanup = updateVoiceAction(action, descriptor);
    } else {
      updateVoiceAction(action, null);
    }
  }

  render() {
    const label = this._action ? this._action.replace(/_/g, ' ') : 'voice control';
    const state = this.record?.state ?? 'idle';
    return html`
      <div class="voice-control-bridge" data-state=${state}>
        <p>${label} channel is ${state}</p>
      </div>
    `;
  }
}

customElements.define('pilot-voice-control-bridge', PilotVoiceControlBridge);
