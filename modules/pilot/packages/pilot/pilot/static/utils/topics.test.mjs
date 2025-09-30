import test from 'node:test';
import assert from 'node:assert/strict';

const moduleUrl = new URL('./topics.js', import.meta.url);
const topicsModule = await import(moduleUrl.href);

const { topicUpdateProfile } = topicsModule;

test('image presentation defaults to throttled latest-frame profile', () => {
  const profile = topicUpdateProfile({ presentation: 'image' });
  assert.equal(profile.mode, 'throttle');
  assert.equal(profile.collapse, true);
});

test('waveform presentation batches updates but preserves samples', () => {
  const profile = topicUpdateProfile({ presentation: 'waveform' });
  assert.equal(profile.mode, 'batch');
  assert.equal(profile.collapse, false);
  assert(profile.interval >= 50);
});

test('unknown topics fall back to immediate updates', () => {
  const profile = topicUpdateProfile({ presentation: 'other' });
  assert.equal(profile.mode, 'immediate');
});
