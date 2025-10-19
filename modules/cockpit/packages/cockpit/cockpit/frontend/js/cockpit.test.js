import assert from 'node:assert/strict';
import test from 'node:test';

const originalFetch = globalThis.fetch;
const originalWebSocket = globalThis.WebSocket;

test('TopicSocket allows publishing before the backend confirms stream capabilities', async (t) => {
  const streamDescriptor = {
    id: 'stream-123',
    module: 'foot',
    topic: 'foo',
    message_type: 'std_msgs/msg/Bool',
    role: 'publish',
  };

  globalThis.fetch = async () => {
    return {
      ok: true,
      async json() {
        return { stream: streamDescriptor };
      },
    };
  };

  const sockets = [];
  class WebSocketStub {
    static CONNECTING = 0;
    static OPEN = 1;
    static CLOSING = 2;
    static CLOSED = 3;

    constructor(url) {
      this.url = url;
      this.readyState = WebSocketStub.CONNECTING;
      this.sent = [];
      this._listeners = new Map();
      sockets.push(this);
      queueMicrotask(() => {
        this.readyState = WebSocketStub.OPEN;
        const handlers = this._listeners.get('open') || [];
        for (const handler of handlers) {
          handler();
        }
      });
    }

    addEventListener(type, handler) {
      const handlers = this._listeners.get(type) || [];
      handlers.push(handler);
      this._listeners.set(type, handlers);
    }

    send(payload) {
      this.sent.push(payload);
    }

    close() {}
  }

  globalThis.WebSocket = WebSocketStub;
  t.after(() => {
    globalThis.fetch = originalFetch;
    globalThis.WebSocket = originalWebSocket;
  });

  const { createTopicSocket } = await import('./cockpit.js');
  const socket = createTopicSocket({
    module: 'foot',
    topic: 'foo',
    type: 'std_msgs/msg/Bool',
    role: 'publish',
  });

  assert.equal(socket.readyState, WebSocketStub.CLOSED, 'socket should start closed before backend responds');

  assert.doesNotThrow(() => socket.send(JSON.stringify({ data: true })), 'publish attempts should be queued before ready');

  const stream = await socket.ready;
  assert.equal(stream.id, streamDescriptor.id);

  await new Promise((resolve) => setImmediate(resolve));

  assert.equal(sockets.length, 1, 'websocket should be created');
  assert.equal(sockets[0].sent.length, 1, 'queued payload should flush once socket opens');

  socket.close();
});
