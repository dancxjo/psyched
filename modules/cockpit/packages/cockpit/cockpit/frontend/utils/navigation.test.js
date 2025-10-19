import test from 'node:test';
import assert from 'node:assert/strict';

import { buildNavigationSections, normaliseModuleSlug } from './navigation.js';

test('includes the configuration section first', () => {
  const sections = buildNavigationSections([]);
  assert.equal(sections.length, 1);
  assert.deepEqual(sections[0], {
    id: 'cockpit-config',
    label: 'Module Configuration',
    index: 0,
    url: '/config/',
  });
});

test('appends dashboard sections for known modules in order', () => {
  const sections = buildNavigationSections([
    { name: 'imu', display_name: 'IMU', slug: 'imu', has_cockpit: true },
    { name: 'voice', display_name: 'Voice', slug: 'voice', dashboard_url: '/modules/voice/' },
  ]);

  assert.equal(sections.length, 3);
  assert.deepEqual(sections[1], {
    id: 'module-imu',
    label: 'IMU',
    index: 1,
    url: '/modules/imu/',
  });
  assert.deepEqual(sections[2], {
    id: 'module-voice',
    label: 'Voice',
    index: 2,
    url: '/modules/voice/',
  });
});

test('skips entries without slugs or names and preserves numbering', () => {
  const sections = buildNavigationSections([
    { slug: '', display_name: 'Missing Name' },
    { name: 'gps', display_name: 'GPS Dashboard', has_cockpit: true },
  ]);

  assert.equal(sections.length, 2);
  assert.deepEqual(sections[1], {
    id: 'module-gps',
    label: 'GPS Dashboard',
    index: 1,
    url: '/modules/gps/',
  });
});

test('normaliseModuleSlug collapses unsafe characters', () => {
  const slug = normaliseModuleSlug({
    name: 'pilot',
    slug: ' Pilot Control! ',
  });
  assert.equal(slug, 'pilot-control');
});

test('buildNavigationSections slugifies ids for anchors', () => {
  const sections = buildNavigationSections([
    { name: 'pilot', display_name: 'Pilot', slug: 'Pilot Control!', has_cockpit: true },
  ]);

  assert.equal(sections.length, 2);
  assert.deepEqual(sections[1], {
    id: 'module-pilot-control',
    label: 'Pilot',
    index: 1,
    url: '/modules/pilot/',
  });
});
