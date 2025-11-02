import { extractNumeric } from './metrics.js';

function normalize(topic) {
  if (typeof topic !== 'string') {
    return '';
  }
  return topic.trim();
}

function titleize(text) {
  return text
    .split(/[_\s/]+/)
    .filter(Boolean)
    .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
    .join(' ');
}

const CHARGING_STATE = {
  0: 'Not charging',
  1: 'Reconditioning',
  2: 'Full',
  3: 'Charging',
  4: 'Trickle',
};

const BATTERY_METADATA = {
  '/battery/charge_ratio': {
    label: 'Charge Level',
    unit: '%',
    digits: 1,
    scale: 100,
  },
  '/battery/capacity': {
    label: 'Capacity',
    unit: 'mAh',
    digits: 0,
  },
  '/battery/charge': {
    label: 'Charge Remaining',
    unit: 'mAh',
    digits: 0,
  },
  '/battery/charging_state': {
    label: 'Charging State',
    format: 'state',
  },
  '/battery/current': {
    label: 'Current Draw',
    unit: 'A',
    digits: 2,
  },
  '/battery/temperature': {
    label: 'Temperature',
    unit: '°C',
    digits: 1,
  },
  '/battery/voltage': {
    label: 'Voltage',
    unit: 'V',
    digits: 2,
  },
};

const DEFAULT_METADATA = {
  label: 'Battery',
  unit: '',
  digits: 2,
  scale: 1,
  format: 'number',
};

export function batteryMetadata(topic) {
  const key = normalize(topic);
  const source = BATTERY_METADATA[key];
  if (source) {
    return { ...DEFAULT_METADATA, ...source };
  }
  if (!key) {
    return { ...DEFAULT_METADATA };
  }
  const fallbackLabel = titleize(key.split('/').pop() || key);
  return { ...DEFAULT_METADATA, label: fallbackLabel };
}

export function batteryLabel(topic) {
  return batteryMetadata(topic).label;
}

export function batteryUnit(topic) {
  return batteryMetadata(topic).unit;
}

export function formatBatteryValue(topic, payload) {
  const metadata = batteryMetadata(topic);
  if (metadata.format === 'state') {
    const numeric = extractNumeric(payload, Number.NaN);
    return Number.isFinite(numeric) ? CHARGING_STATE[numeric] ?? `State ${numeric}` : 'Unknown';
  }
  const numeric = extractNumeric(payload, Number.NaN);
  if (!Number.isFinite(numeric)) {
    return '—';
  }
  const scale = metadata.scale ?? 1;
  const scaled = scale === 100 && numeric > 1 ? numeric : numeric * scale;
  const digits = typeof metadata.digits === 'number' ? metadata.digits : DEFAULT_METADATA.digits;
  const fixed = scaled.toFixed(digits);
  return metadata.unit ? `${fixed} ${metadata.unit}` : fixed;
}

export default {
  batteryMetadata,
  batteryLabel,
  batteryUnit,
  formatBatteryValue,
};
