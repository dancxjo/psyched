const DEFAULT_INTERFACE = {
  mode: 'actions',
  actions: {
    list: '/api/actions',
    invoke: '/api/actions/{module}/{action}',
    stream: '/api/streams/{stream}',
  },
  video_base: null,
  video_port: 8089,
};

const LOOPBACK_HOSTNAMES = new Set(['127.0.0.1', 'localhost', '::1', '', '0.0.0.0']);

function mergeBridgeConfig(values = {}) {
  const actions = normaliseActions(values && typeof values === 'object' ? values.actions : undefined);
  return {
    ...DEFAULT_INTERFACE,
    ...(typeof values === 'object' && values ? values : {}),
    actions,
  };
}

function normaliseActions(actionsRaw) {
  const source = typeof actionsRaw === 'object' && actionsRaw !== null ? actionsRaw : {};
  const list = normaliseActionPath(source.list, DEFAULT_INTERFACE.actions.list);
  const invoke = normaliseActionPath(source.invoke, DEFAULT_INTERFACE.actions.invoke);
  const stream = normaliseActionPath(source.stream, DEFAULT_INTERFACE.actions.stream);
  return { list, invoke, stream };
}

function normaliseActionPath(raw, fallback) {
  if (typeof raw === 'string' && raw.trim()) {
    return raw.trim();
  }
  return fallback;
}

function resolveInterface() {
  if (typeof window !== 'undefined' && window.Cockpit && window.Cockpit.bridge) {
    return mergeBridgeConfig(window.Cockpit.bridge);
  }
  return mergeBridgeConfig();
}

function fillTemplate(template, replacements) {
  if (!template || typeof template !== 'string') {
    return '';
  }
  return template.replace(/\{([a-zA-Z0-9_]+)\}/g, (match, key) => {
    if (Object.prototype.hasOwnProperty.call(replacements, key)) {
      return replacements[key];
    }
    return '';
  });
}

function resolveActionUrl(moduleName, actionName) {
  const interfaceConfig = resolveInterface();
  const template = interfaceConfig.actions.invoke || DEFAULT_INTERFACE.actions.invoke;
  const encodedModule = encodeURIComponent(moduleName);
  const encodedAction = encodeURIComponent(actionName);
  return fillTemplate(template, { module: encodedModule, action: encodedAction }) || template;
}

function resolveStreamUrl(streamId) {
  const interfaceConfig = resolveInterface();
  const template = interfaceConfig.actions.stream || DEFAULT_INTERFACE.actions.stream;
  const encodedId = encodeURIComponent(streamId);
  const path = fillTemplate(template, { stream: encodedId }) || template;
  if (/^wss?:\/\//i.test(path)) {
    return path;
  }
  if (/^https?:\/\//i.test(path)) {
    try {
      const url = new URL(path);
      url.protocol = url.protocol === 'https:' ? 'wss:' : 'ws:';
      return url.toString();
    } catch (_error) {
      // Fall through to relative handling below if URL construction fails.
    }
  }
  const location = typeof window !== 'undefined' ? window.location : undefined;
  const protocol = location && location.protocol === 'https:' ? 'wss:' : 'ws:';
  const host = location && location.host ? location.host : '127.0.0.1';
  if (!path.startsWith('/')) {
    return `${protocol}//${host}/${path}`;
  }
  return `${protocol}//${host}${path}`;
}

function resolveActionsListUrl() {
  const interfaceConfig = resolveInterface();
  return interfaceConfig.actions.list || DEFAULT_INTERFACE.actions.list;
}

function retrieveLocation() {
  return typeof window !== 'undefined' ? window.location : undefined;
}

function createDeferred() {
  let resolve;
  let reject;
  const promise = new Promise((res, rej) => {
    resolve = res;
    reject = rej;
  });
  return { promise, resolve, reject };
}

class TopicSocket {
  constructor(options) {
    if (!options || typeof options !== 'object') {
      throw new Error('Options are required for topic sockets');
    }
    const moduleName = typeof options.module === 'string' && options.module.trim()
      ? options.module.trim()
      : '';
    if (!moduleName) {
      throw new Error('module must be provided when creating a topic socket');
    }
    this.module = moduleName;
    this.actionName = typeof options.action === 'string' && options.action.trim()
      ? options.action.trim()
      : '';
    this.actionArgs = typeof options.arguments === 'object' && options.arguments !== null
      ? options.arguments
      : {};

    if (!this.actionName) {
      if (!options.topic || !options.type) {
        throw new Error('Both topic and type are required when no action override is provided');
      }
      this.topic = options.topic;
      this.messageType = options.type;
      this.role = options.role || 'subscribe';
      this.queueLength = typeof options.queueLength === 'number' && options.queueLength > 0
        ? Math.floor(options.queueLength)
        : 10;
      this.qos = typeof options.qos === 'object' && options.qos !== null ? options.qos : undefined;
    } else {
      this.topic = options.topic || '';
      this.messageType = options.type || '';
      this.role = options.role || 'subscribe';
      this.queueLength = typeof options.queueLength === 'number' && options.queueLength > 0
        ? Math.floor(options.queueLength)
        : 10;
      this.qos = typeof options.qos === 'object' && options.qos !== null ? options.qos : undefined;
    }

    this._listeners = new Map();
    this._pendingMessages = [];
    this._ws = null;
    this._streamId = null;
    this._publishEnabled = false;
    this._closed = false;
    this._initialisationError = null;
    this._readyDeferred = createDeferred();
    this.ready = this._readyDeferred.promise;

    void this._initialise();
  }

  get readyState() {
    if (!this._ws) {
      return WebSocket.CLOSED;
    }
    return this._ws.readyState;
  }

  addEventListener(type, handler) {
    if (typeof handler !== 'function') {
      throw new Error('Event listener must be a function');
    }
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

  async send(rawMessage) {
    if (!this._publishEnabled) {
      throw new Error('Socket is not configured for publishing');
    }
    if (this._closed) {
      throw new Error('Socket has been closed');
    }
    if (this._initialisationError) {
      throw this._initialisationError;
    }
    let parsed;
    try {
      parsed = typeof rawMessage === 'string' ? JSON.parse(rawMessage) : rawMessage;
    } catch (_error) {
      throw new Error('Messages must be JSON-serialisable');
    }
    const envelope = JSON.stringify({ event: 'publish', data: parsed });
    if (!this._ws || this._ws.readyState === WebSocket.CONNECTING) {
      this._pendingMessages.push(envelope);
      return;
    }
    if (this._ws.readyState !== WebSocket.OPEN) {
      throw new Error('Socket is not ready');
    }
    this._ws.send(envelope);
  }

  close() {
    if (this._closed) {
      return;
    }
    this._closed = true;
    if (this._ws) {
      try {
        this._ws.close();
      } catch (_error) {
        // Ignore socket shutdown errors.
      }
    }
  }

  // Internal helpers ------------------------------------------------
  async _initialise() {
    try {
      const response = this.actionName
        ? await invokeAction(this.module, this.actionName, this.actionArgs)
        : await invokeAction(this.module, 'stream_topic', {
          topic: this.topic,
          message_type: this.messageType,
          role: this.role,
          queue_length: this.queueLength,
          qos: this.qos,
        });
      const stream = response && response.stream ? response.stream : null;
      if (!stream || !stream.id) {
        throw new Error('Backend returned an invalid stream descriptor');
      }
      this._streamId = stream.id;
      this._publishEnabled = stream.role === 'publish' || stream.role === 'both';
      this._ws = new WebSocket(resolveStreamUrl(stream.id));
      this._ws.addEventListener('open', () => this._onOpen());
      this._ws.addEventListener('message', (event) => this._onMessage(event));
      this._ws.addEventListener('close', (event) => this._emit('close', event));
      this._ws.addEventListener('error', (event) => this._emit('error', event));
      this._readyDeferred.resolve(stream);
    } catch (error) {
      this._initialisationError = error instanceof Error ? error : new Error(String(error));
      this._readyDeferred.reject(this._initialisationError);
      this._emit('error', new CustomEvent('error', { detail: { message: this._initialisationError.message } }));
    }
  }

  _onOpen() {
    this._emit('open', new Event('open'));
    while (this._pendingMessages.length) {
      const payload = this._pendingMessages.shift();
      if (this._ws && this._ws.readyState === WebSocket.OPEN) {
        this._ws.send(payload);
      }
    }
  }

  _onMessage(event) {
    if (typeof event.data !== 'string') {
      return;
    }
    let payload;
    try {
      payload = JSON.parse(event.data);
    } catch (_error) {
      return;
    }
    const envelope = new MessageEvent('message', {
      data: JSON.stringify(payload),
    });
    this._emit('message', envelope);
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
}

async function invokeAction(moduleName, actionName, args = {}) {
  const url = resolveActionUrl(moduleName, actionName);
  const response = await fetch(url, {
    method: 'POST',
    headers: {
      'content-type': 'application/json',
    },
    body: JSON.stringify({ arguments: args }),
  });
  if (!response.ok) {
    const message = await safeReadError(response);
    throw new Error(`Action ${moduleName}.${actionName} failed: ${message}`);
  }
  return response.json();
}

async function safeReadError(response) {
  try {
    const text = await response.text();
    return text || `${response.status} ${response.statusText}`;
  } catch (_error) {
    return `${response.status} ${response.statusText}`;
  }
}

export function createTopicSocket(options) {
  return new TopicSocket(options);
}

export async function callModuleAction(moduleName, actionName, args = {}) {
  if (!moduleName || typeof moduleName !== 'string') {
    throw new Error('moduleName must be a non-empty string');
  }
  if (!actionName || typeof actionName !== 'string') {
    throw new Error('actionName must be a non-empty string');
  }
  const response = await invokeAction(moduleName, actionName, args);
  return response.result || {};
}

/**
 * Call a ROS service through the cockpit backend and return the response payload.
 *
 * @param {{ module: string, service: string, type?: string, args?: object, timeoutMs?: number }} options
 * @returns {Promise<object>}
 */
export async function callRosService(options) {
  const moduleName = typeof options.module === 'string' && options.module.trim()
    ? options.module.trim()
    : '';
  if (!moduleName) {
    return Promise.reject(new Error('module must be provided when calling a ROS service'));
  }
  const service = typeof options.service === 'string' && options.service.trim()
    ? options.service.trim()
    : '';
  if (!service) {
    return Promise.reject(new Error('service name is required'));
  }
  const serviceType = typeof options.type === 'string' && options.type.trim() ? options.type.trim() : undefined;
  const args = typeof options.args === 'object' && options.args !== null ? options.args : {};
  const timeoutMs = Number.isFinite(options.timeoutMs) ? Number(options.timeoutMs) : 8000;
  const payload = await callModuleAction(moduleName, 'call_service', {
    service,
    service_type: serviceType,
    arguments: args,
    timeout_ms: timeoutMs,
  });
  return payload.result || {};
}

export function cockpitDashboard() {
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
          const cockpitGlobals = window.Cockpit ? { ...window.Cockpit } : {};
          cockpitGlobals.bridge = mergeBridgeConfig(payload.bridge);
          if (payload.host && typeof payload.host === 'object') {
            cockpitGlobals.host = {
              ...(cockpitGlobals.host || {}),
              ...payload.host,
            };
          }
          window.Cockpit = cockpitGlobals;
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

export async function fetchActionCatalogue() {
  const response = await fetch(resolveActionsListUrl());
  if (!response.ok) {
    const message = await safeReadError(response);
    throw new Error(`Failed to load action catalogue: ${message}`);
  }
  return response.json();
}

if (typeof window !== 'undefined') {
  const cockpitGlobals = window.Cockpit ? { ...window.Cockpit } : {};
  cockpitGlobals.createTopicSocket = createTopicSocket;
  cockpitGlobals.callModuleAction = callModuleAction;
  cockpitGlobals.callRosService = callRosService;
  cockpitGlobals.fetchActionCatalogue = fetchActionCatalogue;
  cockpitGlobals.dashboard = cockpitDashboard;
  cockpitGlobals.bridge = mergeBridgeConfig(cockpitGlobals.bridge);
  window.Cockpit = cockpitGlobals;
  window.cockpitDashboard = cockpitDashboard;
  if (typeof window.__startAlpineIfReady === 'function') {
    window.__startAlpineIfReady();
  }
}
