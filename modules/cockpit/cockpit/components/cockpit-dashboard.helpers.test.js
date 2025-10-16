import {
  normaliseHostMetadata,
  summariseModules,
  normaliseBridgeSettings,
  __test__,
} from './cockpit-dashboard.helpers.js';

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
    { name: 'imu', slug: 'imu', display_name: 'IMU', has_cockpit: true, systemd: { supported: true, exists: true, active: true, enabled: true } },
    { name: 'imu', slug: 'imu', has_cockpit: true },
    { name: 'cockpit', display_name: 'Cockpit', has_cockpit: false },
    { name: '', slug: '', has_cockpit: true },
  ]);
  if (summary.total !== 2) {
    throw new Error(`expected 2 modules but received ${summary.total}`);
  }
  if (summary.withCockpit !== 1 || summary.withoutCockpit !== 1) {
    throw new Error(`unexpected cockpit counts: ${summary.withCockpit}/${summary.withoutCockpit}`);
  }
  if (summary.modules[0].displayName !== 'IMU') {
    throw new Error('modules should be sorted by display name');
  }
  if (!summary.modules[0].dashboardUrl.endsWith('/modules/imu/')) {
    throw new Error('dashboard URL should default to module route');
  }
  if (!summary.modules[0].systemd.supported || !summary.modules[0].systemd.exists) {
    throw new Error('systemd status should be normalised');
  }
});

Deno.test('normaliseBridgeSettings resolves relative rosbridge URLs', () => {
  const location = { protocol: 'https:', hostname: 'cockpit.example', href: 'https://cockpit.example/dashboard' };
  const bridge = normaliseBridgeSettings({ rosbridge_uri: 'http://localhost:9090', video_port: 8089 }, location);
  if (bridge.effectiveRosbridgeUri !== 'wss://cockpit.example:9090/') {
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

Deno.test('normaliseSystemdStatus applies defaults for missing data', () => {
  const status = __test__.normaliseSystemdStatus(null);
  if (status.supported !== false || status.exists !== false) {
    throw new Error('missing status should report unsupported');
  }
  const populated = __test__.normaliseSystemdStatus({
    supported: true,
    exists: true,
    active: true,
    enabled: false,
    unit: 'psh-module-imu.service',
    load_state: 'loaded',
    active_state: 'active',
    sub_state: 'running',
    unit_file_state: 'disabled',
    message: 'ok',
  });
  if (!populated.supported || !populated.exists || !populated.active) {
    throw new Error('populated status should retain truthy flags');
  }
  if (populated.unit !== 'psh-module-imu.service') {
    throw new Error('unit should be preserved');
  }
});
