import {
  buildEyeSettingsPayload,
  clampSetting,
  normalizeDepthMode,
} from './eye-dashboard.helpers.js';

Deno.test('normalizeDepthMode filters invalid values', () => {
  if (normalizeDepthMode('depth') !== 'depth') throw new Error('expected depth');
  if (normalizeDepthMode('aligned_depth') !== 'aligned_depth') throw new Error('expected aligned');
  if (normalizeDepthMode('unknown') !== 'disabled') throw new Error('fallback to disabled');
});

Deno.test('clampSetting bounds values', () => {
  const options = { min: 1, max: 5, defaultValue: 3 };
  if (clampSetting(2, options) !== 2) throw new Error('pass through');
  if (clampSetting(-10, options) !== 1) throw new Error('min bound');
  if (clampSetting(10, options) !== 5) throw new Error('max bound');
  if (clampSetting('NaN', options) !== 3) throw new Error('default fallback');
});

Deno.test('buildEyeSettingsPayload enforces resolution bounds', () => {
  const result = buildEyeSettingsPayload({
    width: 2560,
    height: 1440,
    frameRate: 30,
    depthMode: 'depth',
    alignDepth: true,
    autoExposure: true,
  });
  if (result.ok) {
    throw new Error('Payload should reject overly large resolution');
  }

  const ok = buildEyeSettingsPayload({
    width: 960,
    height: 540,
    frameRate: 30,
    depthMode: 'aligned_depth',
    alignDepth: true,
    autoExposure: false,
    exposure: 4000,
    gain: 12,
  });
  if (!ok.ok) {
    throw new Error(`Expected success but received ${ok.error}`);
  }
  if (!ok.value.manual) {
    throw new Error('Manual overrides should be present when auto exposure disabled');
  }
  if (ok.value.manual.exposure !== 4000) {
    throw new Error('Exposure should be preserved');
  }
});
