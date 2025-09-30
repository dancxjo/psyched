import { extractNumeric } from './metrics.js';

const subscribers = new Set();
const metrics = new Map();

const FRIENDLY_NAMES = {
  '/battery/charge_ratio': 'Charge',
  '/battery/capacity': 'Capacity',
  '/battery/charge': 'Charge',
  '/battery/charging_state': 'State',
  '/battery/current': 'Current',
  '/battery/temperature': 'Temperature',
  '/battery/voltage': 'Voltage',
};

const CHARGING_STATE = {
  0: 'Not charging',
  1: 'Reconditioning',
  2: 'Full',
  3: 'Charging',
  4: 'Trickle',
};

function snapshot() {
  const data = {};
  metrics.forEach((value, key) => {
    data[key] = value;
  });
  return data;
}

function notify() {
  const state = snapshot();
  for (const listener of subscribers) {
    try {
      listener(state);
    } catch (error) {
      console.warn('Battery subscriber failed', error);
    }
  }
}

function normalise(topic) {
  if (typeof topic !== 'string') {
    return '';
  }
  return topic.trim();
}

export function updateBatteryMetric(topic, payload) {
  const key = normalise(topic);
  if (!key) {
    return;
  }
  let value = payload;
  if (key === '/battery/charging_state') {
    const numeric = extractNumeric(payload, Number.NaN);
    value = {
      numeric,
      label: Number.isFinite(numeric) ? CHARGING_STATE[numeric] ?? `State ${numeric}` : 'Unknown',
    };
  } else {
    value = extractNumeric(payload, Number.NaN);
  }
  metrics.set(key, value);
  notify();
}

export function subscribeBattery(listener) {
  if (typeof listener !== 'function') {
    return () => {};
  }
  subscribers.add(listener);
  try {
    listener(snapshot());
  } catch (error) {
    console.warn('Battery subscriber failed during initial delivery', error);
  }
  return () => {
    subscribers.delete(listener);
  };
}

export function batteryLabel(topic) {
  const key = normalise(topic);
  return FRIENDLY_NAMES[key] ?? key.split('/').pop() ?? key;
}

export default {
  updateBatteryMetric,
  subscribeBattery,
  batteryLabel,
};
