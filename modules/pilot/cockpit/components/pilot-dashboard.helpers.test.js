import { strict as assert } from 'node:assert';
import test from 'node:test';

import {
  buildPilotIntentPayload,
  clampFloat,
} from './pilot-dashboard.helpers.js';

test('clampFloat normalises decimal ranges', () => {
  assert.equal(clampFloat(0.25, { min: 0, max: 1, defaultValue: 0.5 }), 0.25);
  assert.equal(clampFloat(-5, { min: -1, max: 1, defaultValue: 0 }), -1);
  assert.equal(clampFloat(5, { min: -1, max: 1, defaultValue: 0 }), 1);
  assert.equal(clampFloat('oops', { min: 0, max: 1, defaultValue: 0.4 }), 0.4);
});

test('buildPilotIntentPayload validates context requirement', () => {
  const ok = buildPilotIntentPayload({
    valence: 0.2,
    arousal: 0.5,
    stance: 0.7,
    context: 'Greeting visitors at the lab entrance',
  });
  assert.equal(ok.ok, true);
  assert.equal(ok.value.context.length > 0, true);

  const bad = buildPilotIntentPayload({ valence: 0, arousal: 0.3, stance: 0.4, context: '   ' });
  assert.equal(bad.ok, false);
});
