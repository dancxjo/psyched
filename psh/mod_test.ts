import { assertEquals } from "@std/assert";
import { join } from "@std/path";
import {
  resetModCommandRunner,
  runModuleScript,
  setModCommandRunner,
} from "./mod.ts";
import { repoDirFromModules } from "./modules.ts";
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
        const repoDir = repoDirFromModules();
        assertEquals(
          captured.values[0],
          join(repoDir, "tools", "systemd_entrypoint.sh"),
        );
        assertEquals(captured.values[1], ["bash", "-lc", "echo hi"]);
      },
    );
  } finally {
    resetModCommandRunner();
  }
});
