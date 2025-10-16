import {
  buildEarConfigPayload,
  clampInteger,
  normalizeBackend,
} from './ear-dashboard.helpers.js';

Deno.test('normalizeBackend enforces known slugs', () => {
  const cases = [
    ['Console', 'console'],
    ['FASTER_WHISPER', 'faster_whisper'],
    ['service', 'service'],
    ['unknown', 'console'],
    [null, 'console'],
  ];
  for (const [input, expected] of cases) {
    const actual = normalizeBackend(input);
    if (actual !== expected) {
      throw new Error(`Expected ${expected} but received ${actual}`);
    }
  }
});

Deno.test('clampInteger bounds values safely', () => {
  const options = { min: 1, max: 10, defaultValue: 5 };
  if (clampInteger('7', options) !== 7) throw new Error('should pass through');
  if (clampInteger('-5', options) !== 1) throw new Error('should clamp to min');
  if (clampInteger('25', options) !== 10) throw new Error('should clamp to max');
  if (clampInteger('nan', options) !== 5) throw new Error('should fall back to default');
});

Deno.test('buildEarConfigPayload validates inputs', () => {
  const success = buildEarConfigPayload({
    backend: 'faster_whisper',
    beamSize: '4',
    sampleRate: 16000,
    channels: 2,
    language: 'en',
  });
  if (!success.ok) {
    throw new Error(`Expected success but received ${success.error}`);
  }
  if (success.value.backend_options.beam_size !== 4) {
    throw new Error('Beam size not normalised');
  }
  if (success.value.backend_options.language !== 'en') {
    throw new Error('Language should persist');
  }

  const missingUri = buildEarConfigPayload({
    backend: 'service',
    beamSize: 5,
    sampleRate: 16000,
    channels: 1,
  });
  if (missingUri.ok) {
    throw new Error('Service backend must require a URI');
  }

  const badRate = buildEarConfigPayload({
    backend: 'console',
    beamSize: 5,
    sampleRate: 1000,
    channels: 1,
  });
  if (badRate.ok) {
    throw new Error('Sample rate bounds should be enforced');
  }
});
