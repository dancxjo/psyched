import {
  buildMemoryQueryPayload,
  buildMemoryStorePayload,
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
