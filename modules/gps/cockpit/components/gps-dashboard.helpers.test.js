import {
  buildGpsResetPayload,
  describeNavSatFix,
  normalizeResetMode,
  rosTimeToDate,
} from './gps-dashboard.helpers.js';

Deno.test('normalizeResetMode falls back to hot', () => {
  if (normalizeResetMode('warm') !== 'warm') throw new Error('expected warm');
  if (normalizeResetMode('Cold') !== 'cold') throw new Error('expected cold');
  if (normalizeResetMode('invalid') !== 'hot') throw new Error('fallback should be hot');
});

Deno.test('buildGpsResetPayload enforces note length', () => {
  const ok = buildGpsResetPayload({ mode: 'cold', note: 'Reset after antenna swap' });
  if (!ok.ok) throw new Error(`unexpected error: ${ok.error}`);
  const bad = buildGpsResetPayload({ mode: 'hot', note: 'x'.repeat(300) });
  if (bad.ok) throw new Error('should reject overly long notes');
});

Deno.test('rosTimeToDate converts ROS stamps to Date instances', () => {
  const stamp = { sec: 1, nanosec: 500_000_000 };
  const date = rosTimeToDate(stamp);
  if (!(date instanceof Date)) throw new Error('expected a Date');
  if (date.getTime() !== 1500) throw new Error(`unexpected timestamp: ${date.getTime()}`);
  if (rosTimeToDate(undefined) !== null) throw new Error('missing stamp should return null');
});

Deno.test('describeNavSatFix summarises NavSatFix telemetry', () => {
  const fix = describeNavSatFix({
    header: { stamp: { sec: 1_697_040_000, nanosec: 123_000_000 } },
    status: { status: 0 },
    latitude: 49.2827,
    longitude: -123.1207,
    altitude: 70.04,
  });
  if (!fix || fix.hasFix !== true) throw new Error('expected fix telemetry');
  if (fix.statusText !== '3D fix') throw new Error(`unexpected status: ${fix.statusText}`);
  if (fix.tone !== 'success') throw new Error(`unexpected tone: ${fix.tone}`);
  if (fix.latitude !== '49.2827°N') throw new Error(`bad latitude: ${fix.latitude}`);
  if (fix.longitude !== '123.1207°W') throw new Error(`bad longitude: ${fix.longitude}`);
  if (fix.altitude !== '70.0 m') throw new Error(`bad altitude: ${fix.altitude}`);
  if (!(fix.timestamp instanceof Date)) throw new Error('timestamp should be a Date');
  if (
    fix.eventSummary !== 'Fix update: 49.2827°N 123.1207°W alt=70.0 m (3D fix).'
  ) {
    throw new Error(`unexpected summary: ${fix.eventSummary}`);
  }

  const pending = describeNavSatFix({ status: { status: -1 } });
  if (!pending || pending.hasFix !== false) throw new Error('pending fix should be falsy');
  if (pending.statusText !== 'Awaiting satellite lock') {
    throw new Error(`bad pending status: ${pending.statusText}`);
  }
  if (pending.eventSummary !== 'Fix lost; awaiting satellite lock.') {
    throw new Error(`bad pending summary: ${pending.eventSummary}`);
  }
});
