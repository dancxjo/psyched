import { assertStringIncludes, assert } from "@std/assert";
import { buildUnitContent } from "./systemd.ts";

Deno.test("buildUnitContent no longer injects config-derived environment variables", () => {
  const spec = {
    launch_command: "/bin/true",
  };
  const unit = buildUnitContent(
    "voice",
    spec,
    "/repo",
    "test-host",
    "/repo/hosts/test-host/config/voice.toml",
    {
      engine: "piper",
      voices_dir: "/voices",
      model: "en_US-john-medium",
    },
  );

  // Should still include HOST and PSH_MODULE_NAME
  assertStringIncludes(unit, "Environment=HOST=test-host");
  assertStringIncludes(unit, "Environment=PSH_MODULE_NAME=voice");

  // But should NOT include config-derived environment variables or PSH_MODULE_CONFIG
  assert(unit.indexOf("Environment=VOICE_ENGINE=") === -1);
  assert(unit.indexOf("Environment=PIPER_VOICES_DIR=") === -1);
  assert(unit.indexOf("Environment=VOICE_MODEL=") === -1);
  assert(unit.indexOf("Environment=PSH_MODULE_CONFIG=") === -1);
});
