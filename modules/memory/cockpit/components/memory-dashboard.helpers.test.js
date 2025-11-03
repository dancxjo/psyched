import {
  buildMemoryQueryPayload,
  buildMemoryStorePayload,
  normaliseRecallResults,
} from './memory-dashboard.helpers.js';

Deno.test('buildMemoryQueryPayload requires query text', () => {
  const ok = buildMemoryQueryPayload({ query: 'Where did we dock?', topK: 3 });
  if (!ok.ok) throw new Error(`unexpected error: ${ok.error}`);
  const bad = buildMemoryQueryPayload({ query: '   ' });
  if (bad.ok) throw new Error('blank query should be rejected');
});

Deno.test('buildMemoryStorePayload normalises tags', () => {
  const ok = buildMemoryStorePayload({
    title: 'Recharged batteries',
    body: 'Completed recharge cycle at 22:00.',
    tags: 'maintenance, power , nightly',
  });
  if (!ok.ok) throw new Error(`unexpected error: ${ok.error}`);
  if (ok.value.tags.length !== 3) throw new Error('expected three tags');
  const bad = buildMemoryStorePayload({ title: '', body: 'Missing title' });
  if (bad.ok) throw new Error('missing title should be rejected');
});

Deno.test('normaliseRecallResults derives display metadata from recall payload', () => {
  const raw = [{
    memory_id: 'abc123',
    score: 0.87,
    json_metadata: JSON.stringify({
      title: 'Docking complete',
      summary: 'Docked at port 3 at 0900Z.',
      tags: ['mission', 'docking'],
      labels: ['episodic'],
      source_topics: ['harbour'],
      memory_tag: 'docking',
      memory_collection_text: 'episodic',
      timestamp: '2024-05-10T12:00:00Z',
    }),
  }];
  const results = normaliseRecallResults(raw);
  if (results.length !== 1) throw new Error('expected a single result');
  const [entry] = results;
  if (entry.memoryId !== 'abc123') throw new Error('memoryId should be preserved');
  if (entry.score !== 0.87) throw new Error('score should be normalised to a number');
  if (entry.title !== 'Docking complete') throw new Error('title should prioritise metadata.title');
  if (entry.body !== 'Docked at port 3 at 0900Z.') throw new Error('body should reuse summary');
  if (entry.timestamp !== '2024-05-10T12:00:00Z') throw new Error('timestamp should use metadata timestamp');
  const tagSet = new Set(entry.tags);
  if (!tagSet.has('mission') || !tagSet.has('docking') || !tagSet.has('episodic')) {
    throw new Error('tags should include metadata derived values');
  }
});

Deno.test('normaliseRecallResults prefers identity names when present', () => {
  const raw = [{
    memory_id: 'face-1',
    score: 0.92,
    json_metadata: JSON.stringify({
      identity: { id: 'person:alice', name: 'Alice Example' },
      name: 'Alice Example',
      summary: 'Recognised Alice near the atrium.',
    }),
  }];
  const results = normaliseRecallResults(raw);
  if (results.length !== 1) throw new Error('expected a single result');
  const [entry] = results;
  if (entry.title !== 'Alice Example') throw new Error('identity name should take precedence in titles');
  if (!entry.tags.includes('Alice Example')) throw new Error('identity names should be included in tags');
});

Deno.test('normaliseRecallResults tolerates malformed metadata payloads', () => {
  const raw = [{ memory_id: 'xyz', score: 'not-a-number', json_metadata: '{' }];
  const results = normaliseRecallResults(raw);
  if (results.length !== 1) throw new Error('malformed metadata should still yield a fallback entry');
  const [entry] = results;
  if (entry.title !== 'Memory entry') throw new Error('fallback title should be applied');
  if (entry.body !== '') throw new Error('fallback body should be empty');
  if (entry.score !== 0) throw new Error('invalid scores should normalize to zero');
});
