import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
import test from 'node:test';
import assert from 'node:assert/strict';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const cockpitIndexHtml = readFileSync(path.join(__dirname, 'index.html'), 'utf8');

test('anchors the cockpit navigation inside the Alpine component', () => {
  assert.ok(
    /<nav[^>]*\s+x-data="cockpitNav\(\)"[^>]*class="lcars-nav"/i.test(cockpitIndexHtml),
    'navigation should be a nav element that participates in the Alpine program',
  );
  assert.ok(
    !/<body[^>]*\s+x-data="cockpitNav\(\)"/.test(cockpitIndexHtml),
    'the body should not host the Alpine data scope so the nav can receive updates',
  );
});

test('keeps the navigation adjacent to the main content column', () => {
  const navIndex = cockpitIndexHtml.indexOf('<nav');
  const mainIndex = cockpitIndexHtml.indexOf('<main');
  assert.ok(navIndex !== -1, 'nav element should be present');
  assert.ok(mainIndex !== -1, 'main element should be present');
  assert.ok(navIndex < mainIndex, 'nav should render before the main content to maintain layout');
});
