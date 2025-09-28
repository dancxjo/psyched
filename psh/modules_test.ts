import { assertEquals, assertStringIncludes } from "@std/assert";
import { dirname, join } from "@std/path";
import {
  applyModuleActions,
  resetModuleCommandRunner,
  setModuleCommandRunner,
} from "./modules.ts";
import { createDaxStub } from "./test_utils.ts";

function buildTempContext(moduleName: string) {
  const repoDir = Deno.makeTempDirSync({ prefix: "psh_modules_test_repo_" });
  const moduleDir = join(repoDir, "modules", moduleName);
  const srcDir = join(repoDir, "src");
  Deno.mkdirSync(moduleDir, { recursive: true });
  Deno.mkdirSync(srcDir, { recursive: true });
  return {
    module: moduleName,
    repoDir,
    moduleDir,
    srcDir,
    moduleConfig: null,
    moduleConfigPath: null,
    moduleConfigOwned: false,
  };
}

Deno.test("run action resolves relative script path from module directory", async () => {
  const ctx = buildTempContext("sample");
  const scriptPath = join(ctx.moduleDir, "scripts", "demo.sh");
  Deno.mkdirSync(dirname(scriptPath), { recursive: true });
  Deno.writeTextFileSync(scriptPath, "#!/bin/bash\necho demo\n");
  Deno.chmodSync(scriptPath, 0o755);

  const executedCommands: string[] = [];
  const stub = createDaxStub(({ values }) => {
    executedCommands.push(String(values[1]));
    return { code: 0 };
  });
  setModuleCommandRunner(stub);
  try {
    await applyModuleActions([
      {
        type: "run",
        script: "scripts/demo.sh",
      },
    ], ctx);
  } finally {
    resetModuleCommandRunner();
    await Deno.remove(ctx.repoDir, { recursive: true });
  }

  assertEquals(executedCommands.length, 1);
  assertStringIncludes(executedCommands[0], scriptPath);
});
