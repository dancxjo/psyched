import test from 'node:test';
import assert from 'node:assert/strict';

import {
  PARAMETER_TYPES,
  buildParameterRequest,
  prepareParameterValue,
  formatJointState,
  formatOdometry,
  formatParameterEvent,
  parseSongSheet,
  toAsciiPayload,
  buildPowerLedPayload,
} from './foot-dashboard.helpers.js';

test('prepareParameterValue infers booleans, integers, doubles, and strings', () => {
  assert.deepEqual(prepareParameterValue('true'), { type: PARAMETER_TYPES.BOOL, bool_value: true });
  assert.deepEqual(prepareParameterValue('false'), { type: PARAMETER_TYPES.BOOL, bool_value: false });
  assert.deepEqual(prepareParameterValue('42'), { type: PARAMETER_TYPES.INTEGER, integer_value: 42 });
  assert.deepEqual(prepareParameterValue('-12'), { type: PARAMETER_TYPES.INTEGER, integer_value: -12 });
  assert.deepEqual(prepareParameterValue('3.14'), { type: PARAMETER_TYPES.DOUBLE, double_value: 3.14 });
  assert.deepEqual(prepareParameterValue('-0.5'), { type: PARAMETER_TYPES.DOUBLE, double_value: -0.5 });
  assert.deepEqual(prepareParameterValue('create_2'), {
    type: PARAMETER_TYPES.STRING,
    string_value: 'create_2',
  });
});

test('buildParameterRequest skips blank entries and trims keys', () => {
  const values = {
    ' publish_tf ': 'true',
    'loop_hz': '10.0',
    empty: '',
  };
  const parameters = buildParameterRequest(values);
  assert.equal(parameters.length, 2);
  assert.deepEqual(parameters[0], {
    name: 'publish_tf',
    value: { type: PARAMETER_TYPES.BOOL, bool_value: true },
  });
});

test('formatJointState summarises wheel positions', () => {
  const summary = formatJointState({ position: [0.1234, -0.4567] });
  assert.equal(summary, 'wheel 1: 0.123 rad · wheel 2: -0.457 rad');
  assert.equal(formatJointState(null), 'joint state unavailable');
});

test('formatOdometry formats pose and twist vectors', () => {
  const summary = formatOdometry({
    pose: { pose: { position: { x: 1.23, y: -4.56 } } },
    twist: { twist: { linear: { x: 0.5 }, angular: { z: -0.7 } } },
  });
  assert.equal(summary, 'x 1.230 m · y -4.560 m · v 0.500 m/s · ω -0.700 rad/s');
});

test('formatParameterEvent highlights parameter changes', () => {
  const summary = formatParameterEvent({
    new_parameters: [
      { name: 'publish_tf', value: { bool_value: true } },
    ],
    changed_parameters: [],
    deleted_parameters: [
      { name: 'old_param', value: {} },
    ],
  });
  assert.match(summary, /publish_tf/);
  assert.match(summary, /old_param/);
});

test('parseSongSheet converts note-duration pairs', () => {
  const notes = parseSongSheet('60,0.5\n64,0.25; 67,1.0');
  assert.deepEqual(notes, [
    { note: 60, duration: 0.5 },
    { note: 64, duration: 0.25 },
    { note: 67, duration: 1 },
  ]);
  assert.deepEqual(parseSongSheet(''), []);
});

test('toAsciiPayload limits output to four characters', () => {
  assert.deepEqual(toAsciiPayload('ABCD'), { data: [65, 66, 67, 68] });
  assert.deepEqual(toAsciiPayload('HELLO'), { data: [72, 69, 76, 76] });
});

test('buildPowerLedPayload clamps values to byte range', () => {
  assert.deepEqual(buildPowerLedPayload(128, 255), { data: [128, 255] });
  assert.deepEqual(buildPowerLedPayload(-5, 600), { data: [0, 255] });
});
