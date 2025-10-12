const DEFAULT_BRIDGE = {
  mode: 'rosbridge',
  rosbridge_uri: 'ws://127.0.0.1:9090',
  video_base: null,
  video_port: 8089,
};

function mergeBridgeConfig(values = {}) {
  return {
    ...DEFAULT_BRIDGE,
    ...(typeof values === 'object' && values ? values : {}),
  };
}

function resolveBridge() {
  if (typeof window !== 'undefined' && window.Pilot && window.Pilot.bridge) {
    return mergeBridgeConfig(window.Pilot.bridge);
  }
  return mergeBridgeConfig();
}

function resolveRosbridgeUrl() {
  const bridge = resolveBridge();
  const raw = bridge.rosbridge_uri || DEFAULT_BRIDGE.rosbridge_uri;
  try {
    const url = new URL(raw, window.location.href);
    if (url.protocol === 'http:') {
      url.protocol = 'ws:';
    } else if (url.protocol === 'https:') {
      url.protocol = 'wss:';
    }
    return url.toString();
  } catch (error) {
    console.warn('Invalid rosbridge URI; falling back to current host with port 9090', error);
    // Use the current host from window.location instead of hardcoding 127.0.0.1
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    return `${protocol}//${window.location.hostname}:9090`;
  }
}

class TopicSocket {
  constructor(options) {
    this.topic = options.topic;
    this.type = options.type;
    this.role = options.role || 'subscribe';
    this.queueLength = options.queueLength || 10;
    this._listeners = new Map();
    this._pendingMessages = [];
    this._advertised = false;
    this._subscribed = false;
    this._closed = false;

    this._ws = new WebSocket(resolveRosbridgeUrl());
    this._ws.addEventListener('open', () => this._onOpen());
    this._ws.addEventListener('message', (event) => this._onMessage(event));
    this._ws.addEventListener('close', (event) => this._emit('close', event));
    this._ws.addEventListener('error', (event) => this._emit('error', event));
  }

  get readyState() {
    return this._ws.readyState;
  }

  addEventListener(type, handler) {
    if (!this._listeners.has(type)) {
      this._listeners.set(type, new Set());
    }
    this._listeners.get(type).add(handler);
  }

  removeEventListener(type, handler) {
    const bucket = this._listeners.get(type);
    if (bucket) {
      bucket.delete(handler);
    }
  }

  send(rawMessage) {
    if (this._closed) {
      throw new Error('Socket has been closed');
    }
    let parsed;
    try {
      parsed = typeof rawMessage === 'string' ? JSON.parse(rawMessage) : rawMessage;
    } catch (error) {
      throw new Error('Messages must be JSON-serializable');
    }
    const payload = {
      op: 'publish',
      topic: this.topic,
      msg: parsed,
    };
    this._dispatch(JSON.stringify(payload));
  }

  close() {
    this._closed = true;
    if (this._ws.readyState === WebSocket.OPEN) {
      if (this._subscribed) {
        this._ws.send(JSON.stringify({ op: 'unsubscribe', topic: this.topic }));
      }
      if (this._advertised) {
        this._ws.send(JSON.stringify({ op: 'unadvertise', topic: this.topic }));
      }
    }
    this._ws.close();
  }

  // Internal ----------------------------------------------------------
  _dispatch(payload) {
    if (this._ws.readyState === WebSocket.CONNECTING) {
      this._pendingMessages.push(payload);
      return;
    }
    if (this._ws.readyState !== WebSocket.OPEN) {
      throw new Error('Socket is not ready');
    }
    this._ws.send(payload);
  }

  _emit(type, event) {
    const listeners = this._listeners.get(type);
    if (!listeners) {
      return;
    }
    for (const handler of listeners) {
      try {
        handler(event);
      } catch (error) {
        console.error(`Listener for ${type} threw`, error);
      }
    }
  }

  _onOpen() {
    if (this.role === 'publish' || this.role === 'both') {
      const message = {
        op: 'advertise',
        topic: this.topic,
        type: this.type,
        queue_size: this.queueLength,
      };
      this._ws.send(JSON.stringify(message));
      this._advertised = true;
    }

    if (this.role === 'subscribe' || this.role === 'both') {
      const message = {
        op: 'subscribe',
        topic: this.topic,
        type: this.type,
        queue_length: this.queueLength,
      };
      this._ws.send(JSON.stringify(message));
      this._subscribed = true;
    }

    while (this._pendingMessages.length) {
      const payload = this._pendingMessages.shift();
      this._ws.send(payload);
    }

    this._emit('open', new Event('open'));
  }

  _onMessage(event) {
    let payload;
    try {
      payload = JSON.parse(event.data);
    } catch (error) {
      return;
    }
    if (payload.op === 'publish' && payload.topic === this.topic) {
      const adapted = {
        data: JSON.stringify({
          event: 'message',
          data: payload.msg,
        }),
      };
      this._emit('message', adapted);
    } else if (payload.op === 'status') {
      const adapted = {
        data: JSON.stringify({
          event: 'status',
          data: payload,
        }),
      };
      this._emit('message', adapted);
    }
  }
}

export function createTopicSocket(options) {
  if (!options || !options.topic || !options.type) {
    throw new Error('Both topic and type are required');
  }
  return new TopicSocket(options);
}

export function pilotDashboard() {
  return {
    modules: [],
    loading: true,
    error: '',
    async init() {
      try {
        const response = await fetch('/api/modules');
        if (!response.ok) {
          throw new Error(`Request failed with status ${response.status}`);
        }
        const payload = await response.json();
        this.modules = Array.isArray(payload.modules) ? payload.modules : [];
        if (typeof window !== 'undefined') {
          const pilotGlobals = window.Pilot ? { ...window.Pilot } : {};
          pilotGlobals.bridge = mergeBridgeConfig(payload.bridge);
          window.Pilot = pilotGlobals;
        }
      } catch (error) {
        console.error('Failed to load modules', error);
        this.error = error instanceof Error ? error.message : String(error);
      } finally {
        this.loading = false;
      }
    },
    moduleUrl(module) {
      if (module.dashboard_url) {
        return module.dashboard_url;
      }
      if (module.slug) {
        return `/modules/${module.slug}/`;
      }
      return `/modules/${module.name}/`;
    },
  };
}

if (typeof window !== 'undefined') {
  const pilotGlobals = window.Pilot ? { ...window.Pilot } : {};
  pilotGlobals.createTopicSocket = createTopicSocket;
  pilotGlobals.dashboard = pilotDashboard;
  pilotGlobals.bridge = mergeBridgeConfig(pilotGlobals.bridge);
  window.Pilot = pilotGlobals;
  window.pilotDashboard = pilotDashboard;
  if (typeof window.__startAlpineIfReady === 'function') {
    window.__startAlpineIfReady();
  }
}
