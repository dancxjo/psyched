import {
  copyTextToClipboard,
  formatCommandLogForCopy,
  formatConversationHistoryForCopy,
  formatModuleLogForCopy,
  formatVoiceEventLogForCopy,
} from './log-copy.js';

Deno.test('formatModuleLogForCopy annotates truncated logs', () => {
  const text = formatModuleLogForCopy(['ready', 123], {
    truncated: true,
    updatedAt: '2024-05-20T12:00:00Z',
  });
  if (!text.includes('Older entries truncated.') || !text.includes('Updated at: 2024-05-20T12:00:00Z')) {
    throw new Error(`metadata missing in copy text: ${text}`);
  }
  if (!text.endsWith('ready\n123')) {
    throw new Error(`expected joined lines but received ${text}`);
  }
});

Deno.test('formatCommandLogForCopy renders detail payloads', () => {
  const text = formatCommandLogForCopy([
    { time: '10:00', type: 'pose', message: 'Sent goal', detail: '{"x": 1}' },
    { time: '10:05', type: 'cancel', message: 'Cancel requested' },
  ]);
  if (!text.includes('[10:00] pose — Sent goal')) {
    throw new Error(`expected timestamp and type in copy text: ${text}`);
  }
  if (!text.includes('{"x": 1')) {
    throw new Error(`expected detail payload to appear in copy text: ${text}`);
  }
  if (!text.includes('[10:05] cancel — Cancel requested')) {
    throw new Error(`missing second entry in copy output: ${text}`);
  }
});

Deno.test('formatVoiceEventLogForCopy applies sensible defaults', () => {
  const text = formatVoiceEventLogForCopy([
    { label: 'Resume', topic: '/voice/resume', timestamp: '2024-05-20T12:10:00Z' },
    {},
  ]);
  if (!text.includes('[2024-05-20T12:10:00Z] Resume (/voice/resume)')) {
    throw new Error(`expected explicit event copy text but received ${text}`);
  }
  if (!text.includes('Event 2')) {
    throw new Error(`expected fallback label for missing fields: ${text}`);
  }
});

Deno.test('formatConversationHistoryForCopy summarises speaker context', () => {
  const text = formatConversationHistoryForCopy([
    { role: 'assistant', speaker: 'pete', content: 'Hello there', confidence: 0.91 },
    { role: 'user', content: { text: 'Hi' } },
  ]);
  if (!text.includes('role:assistant speaker:pete confidence:91%')) {
    throw new Error(`expected confidence and speaker context in copy text: ${text}`);
  }
  if (!text.includes('role:user')) {
    throw new Error(`expected user entry to be present: ${text}`);
  }
  if (!text.includes('"text":"Hi"')) {
    throw new Error(`expected structured content to be stringified: ${text}`);
  }
});

Deno.test('copyTextToClipboard prefers async clipboard API', async () => {
  let captured = '';
  const stubClipboard = {
    async writeText(value) {
      captured = value;
    },
  };
  const success = await copyTextToClipboard('nav ready', { clipboard: stubClipboard });
  if (!success) {
    throw new Error('copy should succeed when clipboard API is available');
  }
  if (captured !== 'nav ready') {
    throw new Error(`expected clipboard stub to receive text but saw ${captured}`);
  }
});

Deno.test('copyTextToClipboard falls back to execCommand when provided', async () => {
  let appended = false;
  let removed = false;
  let execCommandInvoked = false;
  let selectionRange = null;
  const textarea = {
    value: '',
    style: {},
    setAttribute() {},
    select() {
      execCommandInvoked = true;
    },
    setSelectionRange(start, end) {
      selectionRange = [start, end];
    },
  };
  const documentStub = {
    body: {
      appendChild(element) {
        appended = element === textarea;
      },
      removeChild(element) {
        removed = element === textarea;
      },
    },
    createElement(tag) {
      if (tag !== 'textarea') {
        throw new Error(`unexpected element requested: ${tag}`);
      }
      return textarea;
    },
    execCommand(command) {
      if (command !== 'copy') {
        throw new Error(`unexpected command: ${command}`);
      }
      return true;
    },
  };
  const success = await copyTextToClipboard('voice ready', { document: documentStub });
  if (!success) {
    throw new Error('copy should succeed when execCommand reports success');
  }
  if (!appended || !removed) {
    throw new Error('textarea should be appended and removed during fallback copy');
  }
  if (!selectionRange || selectionRange[0] !== 0 || selectionRange[1] !== 'voice ready'.length) {
    throw new Error(`expected selection range to cover full text but received ${selectionRange}`);
  }
});

Deno.test('copyTextToClipboard returns false when clipboard unavailable', async () => {
  const success = await copyTextToClipboard('noop', { clipboard: { writeText: null }, document: null });
  if (success) {
    throw new Error('copy should fail when no clipboard or document fallback is provided');
  }
});
