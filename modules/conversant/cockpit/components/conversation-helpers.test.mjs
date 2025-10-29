import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import {
  extractThreadIdFromStream,
  parseConversationSnapshot,
  normaliseMessages,
  parseLlmLog,
} from './conversation-helpers.mjs';

const SAMPLE_MESSAGES = [
  { role: 'assistant', content: 'Hello there', timestamp: '2024-06-01T12:00:00Z' },
  { role: 'user', content: 'Hi', timestamp: '2024-06-01T12:00:05Z' },
];

describe('extractThreadIdFromStream', () => {
  it('returns the last segment of a ROS topic path', () => {
    assert.equal(extractThreadIdFromStream('/conversation/default'), 'default');
    assert.equal(extractThreadIdFromStream('/conversation/alpha-beta'), 'alpha-beta');
    assert.equal(extractThreadIdFromStream('/conversation/demo/thread-42'), 'thread-42');
  });

  it('falls back to the provided default when no segment is found', () => {
    assert.equal(extractThreadIdFromStream('', 'fallback'), 'fallback');
    assert.equal(extractThreadIdFromStream('/', 'fallback'), 'fallback');
    assert.equal(extractThreadIdFromStream('   ', 'fallback'), 'fallback');
  });
});

describe('parseLlmLog', () => {
  it('parses a valid log payload', () => {
    const payload = JSON.stringify({
      thread_id: 'alpha',
      timestamp: '2024-07-01T12:00:00Z',
      system_message: 'Context',
      hint: 'Keep it upbeat',
      source: 'take_turn',
      response: 'Sure, one sentence.',
      response_intent: '<intend/>',
      response_escalate: false,
      chat_messages: [
        { role: 'system', content: 'Context' },
        { role: 'user', content: 'Hello' },
      ],
    });
    const log = parseLlmLog(payload);
    assert.deepEqual(log, {
      threadId: 'alpha',
      timestamp: '2024-07-01T12:00:00Z',
      systemMessage: 'Context',
      hint: 'Keep it upbeat',
      source: 'take_turn',
      response: 'Sure, one sentence.',
      responseIntent: '<intend/>',
      responseEscalate: false,
      chatMessages: [
        { role: 'system', content: 'Context' },
        { role: 'user', content: 'Hello' },
      ],
    });
  });

  it('returns null when required fields are missing', () => {
    const payload = JSON.stringify({ system_message: '', thread_id: 'alpha' });
    assert.equal(parseLlmLog(payload), null);
  });
});

describe('parseConversationSnapshot', () => {
  it('parses a valid snapshot payload', () => {
    const payload = JSON.stringify({
      thread_id: 'alpha',
      user_id: 'operator',
      messages: SAMPLE_MESSAGES,
    });
    const snapshot = parseConversationSnapshot(payload);
    assert.deepEqual(snapshot, {
      threadId: 'alpha',
      userId: 'operator',
      messages: normaliseMessages(SAMPLE_MESSAGES),
    });
  });

  it('rejects payloads without a messages array', () => {
    const payload = JSON.stringify({ thread_id: 'missing', user_id: 'test', messages: {} });
    assert.equal(parseConversationSnapshot(payload), null);
  });

  it('rejects invalid JSON payloads', () => {
    assert.equal(parseConversationSnapshot('{'), null);
    assert.equal(parseConversationSnapshot(''), null);
  });

  it('omits messages that are missing required fields', () => {
    const payload = JSON.stringify({
      thread_id: 'alpha',
      user_id: 'operator',
      messages: [
        { role: 'assistant', content: 'complete', timestamp: '2024-06-01T12:00:00Z' },
        { role: 'assistant', content: 'no timestamp' },
        { content: 'no role', timestamp: '2024-06-01T12:00:05Z' },
      ],
    });
    const snapshot = parseConversationSnapshot(payload);
    assert.deepEqual(snapshot, {
      threadId: 'alpha',
      userId: 'operator',
      messages: normaliseMessages([
        { role: 'assistant', content: 'complete', timestamp: '2024-06-01T12:00:00Z' },
      ]),
    });
  });

  it('keeps the thread when the snapshot contains no valid messages yet', () => {
    const payload = JSON.stringify({
      thread_id: 'beta',
      user_id: 'observer',
      messages: [],
    });
    const snapshot = parseConversationSnapshot(payload);
    assert.deepEqual(snapshot, {
      threadId: 'beta',
      userId: 'observer',
      messages: [],
    });
  });
});
