import {
  buildFeltIntentPayload,
  clampFloat,
} from './felt-dashboard.helpers.js';

Deno.test('clampFloat normalises decimal ranges', () => {
  if (clampFloat(0.25, { min: 0, max: 1, defaultValue: 0.5 }) !== 0.25) throw new Error('pass through');
  if (clampFloat(-5, { min: -1, max: 1, defaultValue: 0 }) !== -1) throw new Error('min bound');
  if (clampFloat(5, { min: -1, max: 1, defaultValue: 0 }) !== 1) throw new Error('max bound');
  if (clampFloat('oops', { min: 0, max: 1, defaultValue: 0.4 }) !== 0.4) throw new Error('default fallback');
});

Deno.test('buildFeltIntentPayload validates context requirement', () => {
  const ok = buildFeltIntentPayload({
    valence: 0.2,
    arousal: 0.5,
    stance: 0.7,
    context: 'Greeting visitors at the lab entrance',
  });
  if (!ok.ok) {
    throw new Error(`Expected success but received ${ok.error}`);
  }
  if (ok.value.context.length === 0) {
    throw new Error('Context should be preserved');
  }

  const bad = buildFeltIntentPayload({ valence: 0, arousal: 0.3, stance: 0.4, context: '   ' });
  if (bad.ok) {
    throw new Error('Blank context should be rejected');
  }
});
