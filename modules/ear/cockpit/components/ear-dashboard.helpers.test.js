import {
  buildEarConfigPayload,
  clampInteger,
  coerceTranscriptInt,
  createTranscriptDeduplicator,
  normalizeBackend,
  normalizeTranscriptEntry,
  transcriptSignature,
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

Deno.test('coerceTranscriptInt safely parses values', () => {
  const cases = [
    [42, 42],
    [' 17 ', 17],
    ['nan', null],
    [null, null],
    [undefined, null],
    [25.8, 25],
    ['-12', -12],
  ];
  for (const [input, expected] of cases) {
    const actual = coerceTranscriptInt(input);
    if (actual !== expected) {
      throw new Error(`Expected ${expected} but received ${actual}`);
    }
  }
});

Deno.test('normalizeTranscriptEntry trims text and segments', () => {
  const entry = normalizeTranscriptEntry({
    text: '  hello world  ',
    timestamp: '12:34:56',
    startMs: ' 1000 ',
    end_ms: 2500.9,
    source: ' asr ',
    segments: [
      {
        text: ' first ',
        start_ms: '500',
        end_ms: '1500',
        words: [
          { text: ' hi ', start_ms: '500', end_ms: '650' },
          { text: '', start_ms: '900', end_ms: '1200' },
        ],
      },
      null,
      {
        text: ' ',
        start_ms: 'bad',
        end_ms: 'data',
      },
    ],
  });

  if (entry.text !== 'hello world') {
    throw new Error('Text should be trimmed');
  }
  if (entry.startMs !== 1000 || entry.endMs !== 2500) {
    throw new Error('Start/end should be numeric');
  }
  if (entry.source !== 'asr') {
    throw new Error('Source should be trimmed');
  }
  if (entry.segments.length !== 1) {
    throw new Error('Empty segments should be discarded');
  }
  const [segment] = entry.segments;
  if (segment.text !== 'first') {
    throw new Error('Segment text trimmed');
  }
  if (segment.words.length !== 1 || segment.words[0].text !== 'hi') {
    throw new Error('Words should be normalised and empty words dropped');
  }
});

Deno.test('transcriptSignature is stable for equivalent entries', () => {
  const base = {
    text: 'Test case',
    startMs: 1200,
    endMs: 3500,
    segments: [
      { text: 'Test', startMs: 1200, endMs: 2000 },
      { text: 'case', startMs: 2000, endMs: 3500 },
    ],
  };
  const signatureA = transcriptSignature(base);
  const signatureB = transcriptSignature({
    text: '  Test case  ',
    start_ms: '1200',
    end_ms: '3500',
    segments: [
      { text: ' Test ', start_ms: '1200', end_ms: '2000' },
      { text: 'case', start_ms: 2000, end_ms: 3500 },
    ],
  });
  if (signatureA !== signatureB) {
    throw new Error('Equivalent entries should share a signature');
  }
});

Deno.test('createTranscriptDeduplicator enforces capacity', () => {
  const dedupe = createTranscriptDeduplicator(3);
  const tokens = ['a', 'b', 'c', 'd'];
  for (const token of tokens) {
    dedupe.remember(token);
  }
  if (dedupe.has('a')) {
    throw new Error('Oldest token should have been evicted');
  }
  if (!dedupe.has('d')) {
    throw new Error('Newest token should be retained');
  }
});
