import test from 'node:test';
import assert from 'node:assert/strict';

import { streamActionForTopic } from './streams.js';

test('streamActionForTopic returns defined actions for known topics', () => {
  assert.equal(streamActionForTopic('battery/charge'), 'battery_charge_stream');
  assert.equal(streamActionForTopic('/battery/charge'), 'battery_charge_stream');
  assert.equal(streamActionForTopic('tf_static'), 'tf_static_stream');
});

test('streamActionForTopic returns null for unknown topics', () => {
  assert.equal(streamActionForTopic('unknown/topic'), null);
});
