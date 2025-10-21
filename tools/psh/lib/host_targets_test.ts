import { assertEquals, assertThrows } from "$std/testing/asserts.ts";
import { join } from "$std/path/mod.ts";
import {
  __internals__,
  defaultModuleTargets,
  defaultServiceTargets,
  resetHostTargetCache,
  resolveModuleTargets,
  resolveServiceTargets,
} from "./host_targets.ts";
import { HostConfigFormatError } from "./host.ts";
import { hostsRoot } from "./paths.ts";

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

denoTest("defaultModuleTargets ignore undeclared modules", () => {
  withEnv("PSH_HOST", "motherbrain", () => {
    resetHostTargetCache();
    __internals__.reset();
    const setupModules = defaultModuleTargets("setup");
    assertEquals(setupModules, [
      "cockpit",
      "foot",
      "viscera",
      "voice",
      "pilot",
    ]);
    const launchModules = defaultModuleTargets("launch");
    assertEquals(launchModules, [
      "cockpit",
      "foot",
      "viscera",
      "voice",
      "pilot",
    ]);
  });
  resetHostTargetCache();
  __internals__.reset();
});

denoTest("defaultServiceTargets respects host service directives", () => {
  withEnv("PSH_HOST", "forebrain", () => {
    resetHostTargetCache();
    __internals__.reset();
    const servicesForSetup = defaultServiceTargets("setup");
    assertEquals(servicesForSetup, ["asr", "tts", "llm", "graphs", "vectors"]);
    const servicesForUp = defaultServiceTargets("up");
    assertEquals(servicesForUp, ["asr", "tts", "llm", "graphs", "vectors"]);
  });
  resetHostTargetCache();
  __internals__.reset();
});

denoTest("defaultServiceTargets honour disabled services", () => {
  const hostName = "__services_disable__";
  const path = join(hostsRoot(), `${hostName}.json`);
  const config = {
    host: {
      name: hostName,
      services: ["alpha"],
    },
    services: {
      alpha: {
        up: false,
      },
    },
  };
  Deno.writeTextFileSync(path, `${JSON.stringify(config, null, 2)}\n`);
  try {
    withEnv("PSH_HOST", hostName, () => {
      resetHostTargetCache();
      __internals__.reset();
      __internals__.setServiceLister(() => ["alpha"]);
      const setupServices = defaultServiceTargets("setup");
      assertEquals(setupServices, ["alpha"]);
      const upServices = defaultServiceTargets("up");
      assertEquals(upServices, []);
    });
  } finally {
    resetHostTargetCache();
    __internals__.reset();
    try {
      Deno.removeSync(path);
    } catch (_error) {
      // ignore
    }
  }
});

denoTest("defaultServiceTargets ignore undeclared services", () => {
  const hostName = "__services_ignore__";
  const path = join(hostsRoot(), `${hostName}.json`);
  const config = {
    host: {
      name: hostName,
      services: ["alpha"],
    },
    services: {
      alpha: {
        intent: "test",
        runtime: "container",
        up: true,
      },
      beta: {
        intent: "test",
        runtime: "container",
        up: true,
      },
    },
  };
  Deno.writeTextFileSync(path, `${JSON.stringify(config, null, 2)}\n`);
  try {
    withEnv("PSH_HOST", hostName, () => {
      resetHostTargetCache();
      __internals__.reset();
      __internals__.setServiceLister(() => ["alpha"]);
      const setupServices = defaultServiceTargets("setup");
      assertEquals(setupServices, ["alpha"]);
      const upServices = defaultServiceTargets("up");
      assertEquals(upServices, ["alpha"]);
    });
  } finally {
    resetHostTargetCache();
    __internals__.reset();
    try {
      Deno.removeSync(path);
    } catch (_error) {
      // ignore
    }
  }
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

denoTest("invalid host configuration surfaces a helpful error", () => {
  const hostName = "__invalid__";
  const path = join(hostsRoot(), `${hostName}.json`);
  const config = {
    host: {
      name: hostName,
      modules: ["oops"],
    },
    modules: {
      oops: "bad shape",
    },
  };
  Deno.writeTextFileSync(path, `${JSON.stringify(config, null, 2)}\n`);
  try {
    withEnv("PSH_HOST", hostName, () => {
      resetHostTargetCache();
      __internals__.reset();
      assertThrows(
        () => defaultModuleTargets("setup"),
        HostConfigFormatError,
        "modules.oops",
      );
    });
  } finally {
    resetHostTargetCache();
    __internals__.reset();
    try {
      Deno.removeSync(path);
    } catch (_error) {
      // ignore removal errors
    }
  }
});
