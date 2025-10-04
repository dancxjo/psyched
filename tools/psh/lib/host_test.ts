import { assertEquals, assertThrows } from "$std/testing/asserts.ts";
import * as host from "./host.ts";

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

  assertThrows(() => {
    registerTask(registry, {
      id: "service:pilot:start",
      label: "Pilot service",
      dependencies: [],
      run: async () => {},
    } as TestTask, ["pilot"]);
  }, Error, "Alias 'pilot' already registered for task 'module:pilot:launch'");
});
