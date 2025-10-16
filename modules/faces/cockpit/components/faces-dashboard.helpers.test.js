import {
  buildFacesSettingsPayload,
  clampNumber,
} from './faces-dashboard.helpers.js';

Deno.test('clampNumber respects numeric bounds', () => {
  if (clampNumber(0.8, { min: 0, max: 1, defaultValue: 0.6 }) !== 0.8) {
    throw new Error('should pass through valid values');
  }
  if (clampNumber(-1, { min: 0, max: 1, defaultValue: 0.6 }) !== 0) {
    throw new Error('should clamp to min');
  }
  if (clampNumber(5, { min: 0, max: 1, defaultValue: 0.6 }) !== 1) {
    throw new Error('should clamp to max');
  }
  if (clampNumber('nan', { min: 0, max: 1, defaultValue: 0.6 }) !== 0.6) {
    throw new Error('should fallback to default');
  }
});

Deno.test('buildFacesSettingsPayload validates threshold', () => {
  const ok = buildFacesSettingsPayload({
    threshold: 0.7,
    window: 12,
    publishCrops: true,
    publishEmbeddings: false,
  });
  if (!ok.ok) {
    throw new Error(`Expected success but received ${ok.error}`);
  }
  if (ok.value.smoothing_window !== 12) {
    throw new Error('Smoothing window should be preserved');
  }

  const bad = buildFacesSettingsPayload({ threshold: 0 });
  if (bad.ok) {
    throw new Error('Zero threshold should be rejected');
  }
});
