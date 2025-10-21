import {
  assert,
  assertArrayIncludes,
  assertEquals,
  assertRejects,
  assertStringIncludes,
} from "$std/testing/asserts.ts";
import { delay } from "$std/async/delay.ts";
import { colors } from "$cliffy/ansi/colors.ts";
import { join } from "$std/path/mod.ts";
import {
  AptPackagePlanner,
  awaitModuleStability,
  bringModulesUp,
  composeLaunchCommand,
  formatExitSummary,
  formatLaunchDiagnostics,
  listModules,
  moduleStatuses,
  PipPackagePlanner,
  RosBuildPlanner,
  RosBuildPlannerRunner,
  __internals__,
} from "./module.ts";
import { repoRoot } from "./paths.ts";

Deno.test("listModules discovers known modules", () => {
  const modules = listModules();
  assert(modules.length > 0, "expected at least one module");
  assertArrayIncludes(modules, ["cockpit"], "cockpit module should be present");
});

Deno.test("moduleStatuses reports state for each module", () => {
  const statuses = moduleStatuses();
  const names = statuses.map((status) => status.name);
  assertArrayIncludes(names, ["cockpit"], "cockpit module should be listed");
  for (const status of statuses) {
    assert(["running", "stopped"].includes(status.status));
    if (status.status === "running") {
      assert(typeof status.pid === "number");
    }
  }
});

Deno.test("composeLaunchCommand prepares a direct module launch", () => {
  const command = composeLaunchCommand({
    envCommands: ["source /tmp/env"],
    launchScript: "/tmp/launch.sh",
    logFile: "/var/log/demo module.log",
    module: "demo module",
  });
  const prefix = join(
    repoRoot(),
    "tools",
    "psh",
    "scripts",
    "prefix_logs.sh",
  );
  assertStringIncludes(
    command,
    "source /tmp/env && exec '/tmp/launch.sh' > >(",
  );
  assertStringIncludes(
    command,
    `'${prefix}' 'demo module' stdout | tee -a '/var/log/demo module.log'`,
  );
  assertStringIncludes(
    command,
    `'${prefix}' 'demo module' stderr | tee -a '/var/log/demo module.log'`,
  );
});

Deno.test("composeLaunchCommand escapes single quotes", () => {
  const command = composeLaunchCommand({
    envCommands: [],
    launchScript: "/tmp/launch.sh",
    logFile: "/var/log/demo.log",
    module: "rocky's rig",
  });
  assertStringIncludes(command, `'rocky'"'"'s rig'`);
});

Deno.test("formatLaunchDiagnostics summarises the launch plan", () => {
  const lines = formatLaunchDiagnostics({
    module: "imu",
    launchScript: "/tmp/launch.sh",
    envCommands: ["source /env", "psyched::activate --quiet"],
    logFile: "/logs/imu.log",
    command: "exec launch --test",
  });
  assertEquals(lines, [
    "module: imu",
    "launch script: /tmp/launch.sh",
    "log file: /logs/imu.log",
    "environment bootstrap: source /env && psyched::activate --quiet",
    "module env overrides: (none)",
    "spawn command: exec launch --test",
  ]);
});

Deno.test("formatExitSummary highlights success and failure", () => {
  const success = colors.stripColor(
    formatExitSummary("imu", { success: true, code: 0, signal: null }),
  );
  assertStringIncludes(success, "exited cleanly (code 0)");

  const failure = colors.stripColor(
    formatExitSummary("imu", {
      success: false,
      code: 1,
      signal: "SIGTERM",
    }),
  );
  assertStringIncludes(failure, "exited with code 1 (signal SIGTERM)");
});

Deno.test(
  "awaitModuleStability reports early exit when the process stops immediately",
  async () => {
    const status: Deno.CommandStatus = {
      success: false,
      code: 1,
      signal: null,
    };
    const result = await awaitModuleStability(Promise.resolve(status), 5);
    assertEquals(result, status);
    await delay(10);
  },
);

Deno.test(
  "awaitModuleStability returns null when the process keeps running",
  async () => {
    const pending = new Promise<Deno.CommandStatus>(() => {});
    const result = await awaitModuleStability(pending, 5);
    assertEquals(result, null);
    await delay(10);
  },
);

Deno.test(
  "moduleEnvOverrides reads host-specific environment overrides",
  async () => {
    const tempRoot = Deno.makeTempDirSync();
    try {
      Deno.mkdirSync(join(tempRoot, "modules"), { recursive: true });
      Deno.mkdirSync(join(tempRoot, "tools", "psh"), { recursive: true });
      Deno.mkdirSync(join(tempRoot, "hosts"), { recursive: true });
      Deno.writeTextFileSync(join(tempRoot, "tools", "psh", "deno.json"), "{}\n");
      Deno.writeTextFileSync(
        join(tempRoot, "hosts", "testbed.toml"),
        `
[host]
name = "testbed"
modules = ["pilot"]

[config.mod.pilot.env]
OLLAMA_HOST = "http://forebrain.local:11434"
        `.trimStart(),
      );

      const originalRepoRoot = Deno.env.get("PSYCHED_REPO_ROOT");
      const originalHost = Deno.env.get("PSH_HOST");
      try {
        Deno.env.set("PSYCHED_REPO_ROOT", tempRoot);
        Deno.env.set("PSH_HOST", "testbed");
        __internals__.resetModuleEnvCache();
        const env = await __internals__.moduleEnvOverrides("pilot");
        assertEquals(env, { OLLAMA_HOST: "http://forebrain.local:11434" });
        const cached = await __internals__.moduleEnvOverrides("pilot");
        assertEquals(cached, env);
      } finally {
        if (originalRepoRoot === null || originalRepoRoot === undefined) {
          Deno.env.delete("PSYCHED_REPO_ROOT");
        } else {
          Deno.env.set("PSYCHED_REPO_ROOT", originalRepoRoot);
        }
        if (originalHost === null || originalHost === undefined) {
          Deno.env.delete("PSH_HOST");
        } else {
          Deno.env.set("PSH_HOST", originalHost);
        }
        __internals__.resetModuleEnvCache();
      }
    } finally {
      await Deno.remove(tempRoot, { recursive: true });
    }
  },
);

Deno.test(
  "bringModulesUp continues launching remaining modules when one fails",
  async () => {
    const modules = ["faces", "nav", "gps"];
    const launched: string[] = [];
    const launcher = async (module: string) => {
      launched.push(module);
      if (module === "nav") {
        throw new Error("nav is unavailable");
      }
    };

    const error = await assertRejects(async () => {
      await bringModulesUp(modules, { launcher });
    });

    assert(error instanceof AggregateError);
    assertEquals(launched, modules);
    assertEquals(error.errors.length, 1);
    assertStringIncludes(error.message, "nav");
  },
);

Deno.test(
  "RosBuildPlanner aggregates rosdep invocation and defers workspace build",
  async () => {
    const tempWorkspace = Deno.makeTempDirSync();
    try {
      const invocations: Array<{
        type: "rosdep";
        payload: unknown;
      }> = [];
      const runner: RosBuildPlannerRunner = {
        async rosdep(invocation) {
          invocations.push({ type: "rosdep", payload: invocation });
        },
      };
      const planner = new RosBuildPlanner(runner);
      planner.add("alpha", {
        workspace: tempWorkspace,
        packages: ["alpha_pkg"],
        skip_rosdep_keys: ["foo"],
      });
      planner.add("beta", {
        workspace: tempWorkspace,
        packages: ["beta_pkg"],
        build_args: ["--cmake-args", "-DCMAKE_BUILD_TYPE=Release"],
        skip_rosdep: true,
      });
      const messages: string[] = [];
      const originalLog = console.log;
      console.log = (...args: unknown[]) => {
        const rendered = args.map((arg) =>
          typeof arg === "string" ? colors.stripColor(arg) : String(arg)
        ).join(" ");
        messages.push(rendered);
      };
      try {
        await planner.execute();
      } finally {
        console.log = originalLog;
      }

      assertEquals(invocations.length, 1);

      const rosdepCall = invocations[0];
      assertEquals(rosdepCall.type, "rosdep");
      const rosdepPayload = rosdepCall.payload as {
        workspace: string;
        skipKeys: string[];
        modules: string[];
      };
      assertEquals(rosdepPayload.workspace, tempWorkspace);
      assertEquals(rosdepPayload.skipKeys, ["foo"]);
      assertEquals(rosdepPayload.modules, ["alpha", "beta"]);

      const stripped = messages.map((line) => colors.stripColor(line));
      const buildNotice = stripped.find((line) =>
        line.includes("colcon build deferred") && line.includes("psh build")
      );
      assert(buildNotice, "expected deferred build notice to be logged");
      const packageLog = stripped.find((line) =>
        line.includes("packages queued for workspace build")
      );
      assert(
        packageLog?.includes("alpha_pkg, beta_pkg"),
        "expected package list in deferred build log",
      );
    } finally {
      Deno.removeSync(tempWorkspace, { recursive: true });
    }
  },
);

Deno.test(
  "RosBuildPlanner skips rosdep when all modules opt out",
  async () => {
    const tempWorkspace = Deno.makeTempDirSync();
    try {
      const calls: string[] = [];
      const runner: RosBuildPlannerRunner = {
        async rosdep() {
          calls.push("rosdep");
        },
      };
      const planner = new RosBuildPlanner(runner);
      planner.add("gamma", {
        workspace: tempWorkspace,
        skip_rosdep: true,
        skip_colcon: true,
      });
      await planner.execute();
      assertEquals(calls, []);
    } finally {
      Deno.removeSync(tempWorkspace, { recursive: true });
    }
  },
);

Deno.test("AptPackagePlanner batches modules and packages once", async () => {
  const invocations: Array<{ modules: string[]; packages: string[] }> = [];
  const planner = new AptPackagePlanner(async (invocation) => {
    invocations.push(invocation);
  });
  planner.add("alpha", ["libfoo-dev", "libbar-dev", "libfoo-dev"]);
  planner.add("beta", ["libbar-dev", "libbaz-dev"]);
  await planner.execute();
  assertEquals(invocations.length, 1);
  const invocation = invocations[0];
  assertEquals(invocation.modules, ["alpha", "beta"]);
  assertEquals(invocation.packages, ["libfoo-dev", "libbar-dev", "libbaz-dev"]);
});

Deno.test("AptPackagePlanner skips execution when nothing queued", async () => {
  let invoked = false;
  const planner = new AptPackagePlanner(async () => {
    invoked = true;
  });
  planner.add("alpha", []);
  await planner.execute();
  assertEquals(invoked, false);
});

Deno.test("PipPackagePlanner batches modules and packages once", async () => {
  const invocations: Array<{ modules: string[]; packages: string[] }> = [];
  const planner = new PipPackagePlanner(async (invocation) => {
    invocations.push(invocation);
  });
  planner.add("alpha", ["numpy", "scipy", "numpy"]);
  planner.add("beta", ["scipy", "pydantic"]);
  await planner.execute();
  assertEquals(invocations.length, 1);
  const invocation = invocations[0];
  assertEquals(invocation.modules, ["alpha", "beta"]);
  assertEquals(invocation.packages, ["numpy", "scipy", "pydantic"]);
});

Deno.test("PipPackagePlanner skips execution when nothing queued", async () => {
  let invoked = false;
  const planner = new PipPackagePlanner(async () => {
    invoked = true;
  });
  planner.add("alpha", []);
  await planner.execute();
  assertEquals(invoked, false);
});
