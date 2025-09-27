import { assertEquals } from "@std/assert";
import { join } from "@std/path";
import {
  resetModCommandRunner,
  runModuleScript,
  setModCommandRunner,
} from "./mod.ts";
import {
  repoDirFromModules,
  resetAptInstallHandler,
  resetPipInstallHandler,
  setAptInstallHandler,
  setPipInstallHandler,
} from "./modules.ts";
import { createDaxStub, type StubInvocation } from "./test_utils.ts";

async function withTempModule(
  name: string,
  setup: (moduleDir: string) => Promise<void>,
  testFn: (moduleDir: string) => Promise<void>,
): Promise<void> {
  const repoDir = repoDirFromModules();
  const moduleDir = join(repoDir, "modules", name);
  await Deno.mkdir(moduleDir, { recursive: true });
  try {
    await setup(moduleDir);
    await testFn(moduleDir);
  } finally {
    await Deno.remove(moduleDir, { recursive: true }).catch(() => undefined);
  }
}

Deno.test("runModuleScript executes launch script via dax", async () => {
  let captured: StubInvocation | null = null;
  const stub = createDaxStub((invocation) => {
    captured = invocation;
    return { code: 0 };
  });
  setModCommandRunner(stub);
  try {
    await withTempModule(
      "dax_launch_script",
      async (moduleDir) => {
        const scriptPath = join(moduleDir, "launch.sh");
        await Deno.writeTextFile(scriptPath, "#!/usr/bin/env bash\nexit 0\n");
        await Deno.chmod(scriptPath, 0o755);
      },
      async (moduleDir) => {
        await runModuleScript("dax_launch_script", "launch");
        if (!captured) {
          throw new Error("Expected launch.sh to be invoked");
        }
        assertEquals(captured.parts[0], "bash ");
        assertEquals(captured.values[0], [join(moduleDir, "launch.sh")]);
      },
    );
  } finally {
    resetModCommandRunner();
  }
});

Deno.test("runModuleScript falls back to systemd launch command", async () => {
  let captured: StubInvocation | null = null;
  const stub = createDaxStub((invocation) => {
    captured = invocation;
    return { code: 0 };
  });
  setModCommandRunner(stub);
  try {
    await withTempModule(
      "dax_systemd_launch",
      async (moduleDir) => {
        await Deno.writeTextFile(
          join(moduleDir, "module.toml"),
          `name = "dax_systemd_launch"\n[systemd]\nlaunch_command = "echo hi"\n`,
        );
      },
      async () => {
        await runModuleScript("dax_systemd_launch", "launch");
        if (!captured) {
          throw new Error("Expected launch_command to be invoked");
        }
        assertEquals(captured.parts[0], "bash -lc ");
        assertEquals(captured.values[0], "echo hi");
      },
    );
  } finally {
    resetModCommandRunner();
  }
});

Deno.test(
  "runModuleScript aggregates dependency installs across modules",
  async () => {
    const repoDir = repoDirFromModules();
    const moduleRoot = join(repoDir, "modules");
    const modA = join(moduleRoot, "aggregate_mod_a");
    const modB = join(moduleRoot, "aggregate_mod_b");
    await Deno.mkdir(modA, { recursive: true });
    await Deno.mkdir(modB, { recursive: true });

    await Deno.writeTextFile(
      join(modA, "module.toml"),
      `[[actions]]\n` +
        `type = "apt_install"\n` +
        `packages = ["curl", "htop"]\n` +
        `update = true\n\n` +
        `[[actions]]\n` +
        `type = "pip_install"\n` +
        `packages = ["fastapi", "psutil"]\n` +
        `import_check = ["fastapi"]\n` +
        `break_system = true\n`,
    );

    await Deno.writeTextFile(
      join(modB, "module.toml"),
      `[[actions]]\n` +
        `type = "apt_install"\n` +
        `packages = ["curl", "vim"]\n\n` +
        `[[actions]]\n` +
        `type = "pip_install"\n` +
        `packages = ["psutil", "uvicorn"]\n` +
        `import_check = ["uvicorn"]\n` +
        `break_system = true\n`,
    );

    const aptCalls: Array<{ packages: string[]; update: boolean; module: string }> = [];
    const pipCalls: Array<{
      packages: string[];
      import_check?: string[];
      python?: string;
      break_system?: boolean;
      module: string;
    }> = [];

    setAptInstallHandler(async (action, ctx) => {
      aptCalls.push({
        packages: [...action.packages],
        update: Boolean(action.update),
        module: ctx.module,
      });
    });

    setPipInstallHandler(async (action, ctx) => {
      pipCalls.push({
        packages: [...action.packages],
        import_check: action.import_check ? [...action.import_check] : undefined,
        python: action.python,
        break_system: action.break_system,
        module: ctx.module,
      });
    });

    try {
      await runModuleScript(["aggregate_mod_a", "aggregate_mod_b"], "setup");

      assertEquals(aptCalls.length, 1);
      assertEquals(aptCalls[0].packages, ["curl", "htop", "vim"]);
      assertEquals(aptCalls[0].update, true);
      assertEquals(aptCalls[0].module, "multi:aggregate_mod_a+aggregate_mod_b");

      assertEquals(pipCalls.length, 1);
      assertEquals(pipCalls[0].packages, ["fastapi", "psutil", "uvicorn"]);
      assertEquals(pipCalls[0].import_check, ["fastapi", "uvicorn"]);
      assertEquals(pipCalls[0].break_system, true);
      assertEquals(
        pipCalls[0].module,
        "multi:aggregate_mod_a+aggregate_mod_b",
      );
    } finally {
      resetAptInstallHandler();
      resetPipInstallHandler();
      await Deno.remove(modA, { recursive: true }).catch(() => undefined);
      await Deno.remove(modB, { recursive: true }).catch(() => undefined);
    }
  },
);
