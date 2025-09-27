import { assertEquals, assertStringIncludes } from "@std/assert";
import { buildUnitContent, deriveModuleConfigEnv } from "./systemd.ts";

Deno.test("deriveModuleConfigEnv flattens primitives and applies module prefix", () => {
  const env = deriveModuleConfigEnv("voice", {
    engine: "piper",
    enable_tts: true,
    voices_dir: "/voices",
    channels: 1,
    nested: { unsupported: true },
    list: ["a", "b"],
    empty: null,
  });

  assertEquals(env.VOICE_ENGINE, "piper");
  assertEquals(env.VOICE_ENABLE_TTS, "true");
  assertEquals(env.VOICE_CHANNELS, "1");
  assertEquals(env.PIPER_VOICES_DIR, "/voices");
  // Non-primitive entries should be omitted
  assertEquals(Object.hasOwn(env, "VOICE_NESTED"), false);
  assertEquals(Object.hasOwn(env, "VOICE_LIST"), false);
});

Deno.test("buildUnitContent injects config-derived environment variables", () => {
  const spec = {
    launch_command: "/bin/true",
  };
  const unit = buildUnitContent(
    "voice",
    spec,
    "/repo",
    "test-host",
    "/repo/hosts/test-host/config/voice.yaml",
    {
      engine: "piper",
      voices_dir: "/voices",
      model: "en_US-john-medium",
    },
  );

  assertStringIncludes(unit, "Environment=VOICE_ENGINE=piper");
  assertStringIncludes(unit, "Environment=PIPER_VOICES_DIR=/voices");
  assertStringIncludes(unit, "Environment=VOICE_MODEL=en_US-john-medium");
});
