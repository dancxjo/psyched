const subscribers = new Set();
const actions = new Map();
let volumeDescriptor = null;

function snapshot() {
  const actionState = {};
  actions.forEach((descriptor, action) => {
    actionState[action] = {
      available: descriptor?.state === 'connected',
      state: descriptor?.state ?? 'idle',
    };
  });
  return {
    actions: actionState,
    volume: volumeDescriptor,
  };
}

function notify() {
  const state = snapshot();
  for (const listener of subscribers) {
    try {
      listener(state);
    } catch (error) {
      console.warn('Voice control subscriber failed', error);
    }
  }
}

function normaliseAction(action) {
  return typeof action === 'string' ? action.trim().toLowerCase() : '';
}

export function updateVoiceAction(action, descriptor) {
  const key = normaliseAction(action);
  if (!key) {
    return () => { };
  }
  if (descriptor == null) {
    actions.delete(key);
    notify();
    return () => { };
  }
  actions.set(key, descriptor);
  notify();
  return () => {
    const existing = actions.get(key);
    if (existing === descriptor) {
      actions.delete(key);
      notify();
    }
  };
}

export function publishVoiceAction(action, payload = {}) {
  const key = normaliseAction(action);
  const descriptor = actions.get(key);
  if (descriptor && typeof descriptor.send === 'function') {
    descriptor.send(payload);
    return true;
  }
  return false;
}

export function subscribeVoiceControls(listener) {
  if (typeof listener !== 'function') {
    return () => { };
  }
  subscribers.add(listener);
  try {
    listener(snapshot());
  } catch (error) {
    console.warn('Voice control subscriber failed during initial delivery', error);
  }
  return () => {
    subscribers.delete(listener);
  };
}

export function updateVoiceVolume(descriptor) {
  volumeDescriptor = descriptor;
  notify();
}

export function setVoiceVolume(value) {
  if (!volumeDescriptor || typeof volumeDescriptor.send !== 'function') {
    return false;
  }
  const message = { data: Number.isFinite(value) ? Math.max(0, Math.min(100, Math.round(value))) : 0 };
  volumeDescriptor.send(message);
  return true;
}

export default {
  updateVoiceAction,
  publishVoiceAction,
  subscribeVoiceControls,
  updateVoiceVolume,
  setVoiceVolume,
};

// Optionally register this helper in the runtime registry so other components
// can access it dynamically via `/utils/registry.js`.
try {
  // Use a dynamic import to avoid circular static import problems in older
  // bundlers. If the registry is not present, this will fail harmlessly.
  import('./registry.js').then((r) => {
    try {
      r.exportsify('voice', exports.default || module?.exports);
    } catch (_e) {
      // swallow
    }
  }).catch(() => { });
} catch (_) {
  // ignore; registration is optional
}
