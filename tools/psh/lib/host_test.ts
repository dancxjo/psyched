import {
  assertEquals,
  assertInstanceOf,
  assertThrows,
} from "$std/testing/asserts.ts";
import { join } from "$std/path/mod.ts";
import * as host from "./host.ts";
import { hostsRoot } from "./paths.ts";

const { createTaskRegistry, registerTask, resolveDependencyAliases } =
  (host as unknown as {
    __test__: {
      createTaskRegistry: () => unknown;
      registerTask: (
        registry: unknown,
        task: {
          id: string;
          label: string;
          dependencies: string[];
          run: () => Promise<void>;
        },
        aliases?: string[],
      ) => void;
      resolveDependencyAliases: (
        registry: unknown,
        task: {
          id: string;
          label: string;
          dependencies: string[];
          run: () => Promise<void>;
        },
      ) => void;
    };
  }).__test__;

type TestTask = {
  id: string;
  label: string;
  dependencies: string[];
  run: () => Promise<void>;
};

async function withTempRepo(
  hostName: string,
  hostToml: string,
  fn: () => Promise<void>,
): Promise<void> {
  const hostsDir = hostsRoot();
  const hostPath = join(hostsDir, `${hostName}.toml`);
  try {
    await Deno.lstat(hostPath);
    throw new Error(`host config already exists at ${hostPath}`);
  } catch (error) {
    if (!(error instanceof Deno.errors.NotFound)) {
      throw error;
    }
  }

  await Deno.writeTextFile(hostPath, hostToml);
  try {
    await fn();
  } finally {
    await Deno.remove(hostPath);
  }
}

type ModuleOps = {
  setup: (module: string) => Promise<void>;
  launch: (module: string) => Promise<void>;
};

type ServiceOps = {
  setup: (service: string) => Promise<void>;
  start: (service: string) => Promise<void>;
};

const internals = (host as unknown as {
  __internals__: { moduleOps: ModuleOps; serviceOps: ServiceOps };
}).__internals__;

function patchOps<T extends Record<string, unknown>>(
  target: T,
  updates: Partial<T>,
): () => void {
  const originals = new Map<keyof T, T[keyof T]>();
  for (const key of Object.keys(updates) as (keyof T)[]) {
    const value = updates[key];
    if (value === undefined) continue;
    originals.set(key, target[key]);
    target[key] = value;
  }
  return () => {
    for (const [key, value] of originals.entries()) {
      target[key] = value;
    }
  };
}

Deno.test("resolves service alias to start task id", () => {
  const registry = createTaskRegistry();
  registerTask(registry, {
    id: "service:docker:start",
    label: "Docker service",
    dependencies: [],
    run: async () => {},
  } as TestTask, ["service:docker", "docker"]);

  const moduleTask: TestTask = {
    id: "module:ros-client:launch",
    label: "ROS client",
    dependencies: ["service:docker"],
    run: async () => {},
  };
  registerTask(registry, moduleTask);
  resolveDependencyAliases(registry, moduleTask);

  assertEquals(moduleTask.dependencies, ["service:docker:start"]);
});

Deno.test("falls back to setup task when launch is unavailable", () => {
  const registry = createTaskRegistry();
  registerTask(registry, {
    id: "module:logger:setup",
    label: "Logger setup",
    dependencies: [],
    run: async () => {},
  } as TestTask, ["module:logger", "logger"]);

  const dependent: TestTask = {
    id: "module:diagnostics:launch",
    label: "Diagnostics launch",
    dependencies: ["module:logger"],
    run: async () => {},
  };
  registerTask(registry, dependent);
  resolveDependencyAliases(registry, dependent);

  assertEquals(dependent.dependencies, ["module:logger:setup"]);
});

Deno.test("registerTask rejects conflicting aliases", () => {
  const registry = createTaskRegistry();
  registerTask(registry, {
    id: "module:pilot:launch",
    label: "Pilot launch",
    dependencies: [],
    run: async () => {},
  } as TestTask, ["pilot"]);

  assertThrows(
    () => {
      registerTask(registry, {
        id: "service:pilot:start",
        label: "Pilot service",
        dependencies: [],
        run: async () => {},
      } as TestTask, ["pilot"]);
    },
    Error,
    "Alias 'pilot' already registered for task 'module:pilot:launch'",
  );
});

Deno.test("locateHostConfig throws HostConfigNotFoundError for unknown host", () => {
  const error = assertThrows(
    () => {
      host.locateHostConfig("definitely_missing_host");
    },
    Error,
    "No host config found",
  );
  assertInstanceOf(error, host.HostConfigNotFoundError);
  assertEquals(error.hostname, "definitely_missing_host");
});

Deno.test(
  "provisionHost skips module provisioning unless explicitly requested",
  async () => {
    const hostToml = `
[host]
name = "demo"

modules = ["alpha"]
`;
    await withTempRepo("demo", hostToml, async () => {
      let setupCalls = 0;
      let launchCalls = 0;
      const restore = patchOps(internals.moduleOps, {
        setup: (_module: string) => {
          setupCalls += 1;
          return Promise.resolve();
        },
        launch: (_module: string) => {
          launchCalls += 1;
          return Promise.resolve();
        },
      });
      try {
        await host.provisionHost("demo");
        assertEquals(setupCalls, 0);
        assertEquals(launchCalls, 0);
      } finally {
        restore();
      }
    });
  },
);

Deno.test(
  "provisionHost runs module provisioning when includeModules is true",
  async () => {
    const hostToml = `
[host]
name = "demo"

modules = ["alpha"]
`;
    await withTempRepo("demo", hostToml, async () => {
      let setupCalls = 0;
      const restore = patchOps(internals.moduleOps, {
        setup: (_module: string) => {
          setupCalls += 1;
          return Promise.resolve();
        },
      });
      try {
        await host.provisionHost("demo", { includeModules: true });
        assertEquals(setupCalls, 1);
      } finally {
        restore();
      }
    });
  },
);

Deno.test(
  "provisionHost skips service provisioning unless includeServices is true",
  async () => {
    const hostToml = `
[host]
name = "demo"

services = ["telemetry"]
`;
    await withTempRepo("demo", hostToml, async () => {
      let setupCalls = 0;
      let startCalls = 0;
      const restore = patchOps(internals.serviceOps, {
        setup: (_service: string) => {
          setupCalls += 1;
          return Promise.resolve();
        },
        start: (_service: string) => {
          startCalls += 1;
          return Promise.resolve();
        },
      });
      try {
        await host.provisionHost("demo");
        assertEquals(setupCalls, 0);
        assertEquals(startCalls, 0);
      } finally {
        restore();
      }
    });
  },
);

Deno.test(
  "provisionHost runs service provisioning when includeServices is true",
  async () => {
    const hostToml = `
[host]
name = "demo"

services = ["telemetry"]
`;
    await withTempRepo("demo", hostToml, async () => {
      let setupCalls = 0;
      const restore = patchOps(internals.serviceOps, {
        setup: (_service: string) => {
          setupCalls += 1;
          return Promise.resolve();
        },
      });
      try {
        await host.provisionHost("demo", { includeServices: true });
        assertEquals(setupCalls, 1);
      } finally {
        restore();
      }
    });
  },
);
