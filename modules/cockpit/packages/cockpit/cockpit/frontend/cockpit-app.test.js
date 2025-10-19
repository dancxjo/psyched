import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import path from 'node:path';
import test from 'node:test';
import assert from 'node:assert/strict';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const cockpitAppSource = readFileSync(
  path.join(__dirname, '..', '..', '..', '..', 'cockpit', 'components', 'cockpit-app.js'),
  'utf8',
);

test('module surfaces render dashboards and logs inside the shared surface grid', () => {
  assert.match(
    cockpitAppSource,
    /<div\s+class="module-surface\s+surface-grid\s+surface-grid--stack"/m,
    'module surfaces should wrap dashboards and module logs in the shared surface grid layout',
  );
});

test('cockpit app exposes the shared surface styles to its template', () => {
  assert.match(
    cockpitAppSource,
    /import\s+\{\s*surfaceStyles\s*\}\s+from\s+'\.\/cockpit-style\.js';/,
    'cockpit app should import the shared surface style helpers',
  );
  assert.match(
    cockpitAppSource,
    /static\s+styles\s*=\s*\[\s*surfaceStyles(?:\s*,|\s*\])/m,
    'cockpit app should apply the shared surface styles to its light DOM',
  );
});
