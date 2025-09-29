import { extractNumeric } from './metrics.js';

const subscribers = new Set();
const metrics = new Map();

const FRIENDLY_DETAILS = {
  '/battery/charge_ratio': { label: 'Charge Level', unit: '%' },
  '/battery/capacity': { label: 'Capacity', unit: 'mAh' },
  '/battery/charge': { label: 'Charge (mAh)', unit: 'mAh' },
  '/battery/charging_state': { label: 'State', unit: '' },
  '/battery/current': { label: 'Current', unit: 'A' },
  '/battery/temperature': { label: 'Temperature', unit: '°C' },
  '/battery/voltage': { label: 'Voltage', unit: 'V' },
};

const FRIENDLY_NAMES = Object.fromEntries(
  Object.entries(FRIENDLY_DETAILS).map(([topic, detail]) => [topic, detail.label]),
);

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

export function batteryUnit(topic) {
  const key = normalise(topic);
  return FRIENDLY_DETAILS[key]?.unit ?? '';
}

export function chargingStateLabel(value) {
  const source = value && typeof value === 'object' && 'numeric' in value ? value.numeric : value;
  const numeric = extractNumeric(source, Number.NaN);
  return Number.isFinite(numeric) ? CHARGING_STATE[numeric] ?? `State ${numeric}` : 'Unknown';
}

export default {
  updateBatteryMetric,
  subscribeBattery,
  batteryLabel,
  batteryUnit,
  chargingStateLabel,
};
