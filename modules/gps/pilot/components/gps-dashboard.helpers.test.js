import {
  buildGpsResetPayload,
  normalizeResetMode,
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
