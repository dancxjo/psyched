import { assertEquals } from "$std/testing/asserts.ts";
import {
  __internals__,
  defaultModuleTargets,
  defaultServiceTargets,
  resetHostTargetCache,
  resolveModuleTargets,
  resolveServiceTargets,
} from "./host_targets.ts";

function denoTest(
  name: string,
  fn: () => void | Promise<void>,
) {
  Deno.test(name, fn);
}

function withEnv<T>(key: string, value: string, fn: () => T): T {
  const previous = Deno.env.get(key);
  try {
    Deno.env.set(key, value);
    return fn();
  } finally {
    if (previous === undefined) {
      Deno.env.delete(key);
    } else {
      Deno.env.set(key, previous);
    }
  }
}

denoTest("defaultModuleTargets respects host launch directives", () => {
  withEnv("PSH_HOST", "motherbrain", () => {
    resetHostTargetCache();
    __internals__.reset();
    const modules = defaultModuleTargets("launch");
    assertEquals(modules, ["imu", "foot", "eye"]);
  });
  resetHostTargetCache();
  __internals__.reset();
});

denoTest("defaultServiceTargets respects host service directives", () => {
  withEnv("PSH_HOST", "forebrain", () => {
    resetHostTargetCache();
    __internals__.reset();
    const services = defaultServiceTargets("up");
    assertEquals(services, ["tts", "llm", "graphs", "vectors", "asr"]);
  });
  resetHostTargetCache();
  __internals__.reset();
});

denoTest("falls back to discovered targets when host is unknown", () => {
  __internals__.setModuleLister(() => ["alpha", "beta"]);
  __internals__.setServiceLister(() => ["gamma"]);
  __internals__.setCandidateProvider(() => []);
  resetHostTargetCache();
  const modules = defaultModuleTargets("setup");
  const services = defaultServiceTargets("up");
  assertEquals(modules, ["alpha", "beta"]);
  assertEquals(services, ["gamma"]);
  __internals__.reset();
  resetHostTargetCache();
});

denoTest("resolve helpers honour explicit selections", () => {
  __internals__.setModuleLister(() => ["alpha", "beta"]);
  __internals__.setServiceLister(() => ["gamma"]);
  __internals__.setCandidateProvider(() => []);
  resetHostTargetCache();
  const modules = resolveModuleTargets("setup", ["beta"]);
  const services = resolveServiceTargets("up", ["gamma"]);
  assertEquals(modules, ["beta"]);
  assertEquals(services, ["gamma"]);
  __internals__.reset();
  resetHostTargetCache();
});
