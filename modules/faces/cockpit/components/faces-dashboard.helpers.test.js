import {
  buildFacesSettingsPayload,
  clampNumber,
  parseFaceTriggerPayload,
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

Deno.test('parseFaceTriggerPayload handles std_msgs/String envelopes', () => {
  const result = parseFaceTriggerPayload({
    data: JSON.stringify({
      name: 'Stranger',
      memory_id: 'mem-123',
      vector_id: 'vec-456',
      collection: 'faces',
    }),
  });
  if (!result.ok) {
    throw new Error(`Expected success but received error: ${result.error}`);
  }
  if (result.value.name !== 'Stranger') {
    throw new Error('Name should be preserved');
  }
  if (result.value.memoryId !== 'mem-123' || result.value.vectorId !== 'vec-456') {
    throw new Error('Identifiers should be normalised');
  }
  if (!result.value.raw.includes('"memory_id"')) {
    throw new Error('Raw payload should echo the JSON string');
  }
});

Deno.test('parseFaceTriggerPayload handles face sensations', () => {
  const result = parseFaceTriggerPayload({
    kind: 'face',
    collection_hint: 'faces',
    json_payload: JSON.stringify({
      memory_id: 'mem-900',
      vector_id: 'vec-321',
      confidence: 0.82,
      embedding_dim: 512,
    }),
    vector: [0.1, 0.2],
  });
  if (!result.ok) {
    throw new Error(`Expected success but received error: ${result.error}`);
  }
  if (result.value.memoryId !== 'mem-900' || result.value.vectorId !== 'vec-321') {
    throw new Error('Identifiers should be normalised from sensations');
  }
  if (result.value.collection !== 'faces') {
    throw new Error('Collection should fall back to collection_hint when missing');
  }
  if (!result.value.note || !result.value.note.includes('confidence')) {
    throw new Error('Sensation notes should include confidence details when available');
  }
});

Deno.test('parseFaceTriggerPayload ignores non-face sensations', () => {
  const result = parseFaceTriggerPayload({
    kind: 'audio',
    collection_hint: 'audio',
    json_payload: JSON.stringify({ memory_id: 'mem-x' }),
  });
  if (result.ok) {
    throw new Error('Non-face sensations should be ignored');
  }
  if (result.reason !== 'ignored') {
    throw new Error('Non-face sensations should be flagged as ignored');
  }
});

Deno.test('parseFaceTriggerPayload rejects malformed payloads', () => {
  const empty = parseFaceTriggerPayload({ data: ' ' });
  if (empty.ok) {
    throw new Error('Empty payload should be rejected');
  }
  if (empty.reason !== 'empty') {
    throw new Error('Empty payloads should be flagged with the empty reason');
  }
  const invalid = parseFaceTriggerPayload({ data: '{' });
  if (invalid.ok) {
    throw new Error('Malformed JSON should be rejected');
  }
  if (invalid.reason !== 'invalid-json') {
    throw new Error('Malformed payloads should surface the invalid-json reason');
  }
});
