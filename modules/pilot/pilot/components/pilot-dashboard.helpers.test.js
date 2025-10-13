import {
  normaliseHostMetadata,
  summariseModules,
  normaliseBridgeSettings,
  __test__,
} from './pilot-dashboard.helpers.js';

Deno.test('normaliseHostMetadata falls back to derived shortname', () => {
  const host = normaliseHostMetadata({ name: 'pete.local' });
  if (host.shortname !== 'pete') {
    throw new Error(`expected shortname to be pete but received ${host.shortname}`);
  }
  const fallback = normaliseHostMetadata(null);
  if (fallback.name !== 'host' || fallback.shortname !== 'host') {
    throw new Error('missing metadata should fall back to host');
  }
});

Deno.test('summariseModules deduplicates payloads and counts dashboards', () => {
  const summary = summariseModules([
    { name: 'imu', slug: 'imu', display_name: 'IMU', has_pilot: true },
    { name: 'imu', slug: 'imu', has_pilot: true },
    { name: 'pilot', display_name: 'Pilot', has_pilot: false },
    { name: '', slug: '', has_pilot: true },
  ]);
  if (summary.total !== 2) {
    throw new Error(`expected 2 modules but received ${summary.total}`);
  }
  if (summary.withPilot !== 1 || summary.withoutPilot !== 1) {
    throw new Error(`unexpected pilot counts: ${summary.withPilot}/${summary.withoutPilot}`);
  }
  if (summary.modules[0].displayName !== 'IMU') {
    throw new Error('modules should be sorted by display name');
  }
  if (!summary.modules[0].dashboardUrl.endsWith('/modules/imu/')) {
    throw new Error('dashboard URL should default to module route');
  }
});

Deno.test('normaliseBridgeSettings resolves relative rosbridge URLs', () => {
  const location = { protocol: 'https:', hostname: 'pilot.example', href: 'https://pilot.example/dashboard' };
  const bridge = normaliseBridgeSettings({ rosbridge_uri: 'http://localhost:9090', video_port: 8089 }, location);
  if (bridge.effectiveRosbridgeUri !== 'wss://pilot.example:9090/') {
    throw new Error(`unexpected rosbridge URI: ${bridge.effectiveRosbridgeUri}`);
  }
  if (bridge.videoPort !== 8089) {
    throw new Error(`expected explicit video port to be retained but received ${bridge.videoPort}`);
  }
});

Deno.test('resolveRosbridgeUri falls back on invalid payload', () => {
  const resolved = __test__.resolveRosbridgeUri('not a url', { protocol: 'ws:', hostname: 'control.local' });
  if (resolved !== 'ws://control.local:9090') {
    throw new Error(`invalid URI should fallback to ws://control.local:9090 but received ${resolved}`);
  }
});
