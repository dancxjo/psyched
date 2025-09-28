import { assertEquals } from "@std/assert";
import { type CliDeps, createCli } from "./cli.ts";

function createStubDeps() {
  const calls: Record<string, unknown[]> = {};
  const record = (key: keyof CliDeps, args: unknown[] = []) => {
    calls[key as string] = args;
  };

  const deps: CliDeps = {
    printSummaryTable() {
      record("printSummaryTable");
    },
    installPsh() {
      record("installPsh");
    },
    uninstallPsh() {
      record("uninstallPsh");
    },
    setupHosts(hosts) {
      record("setupHosts", [hosts]);
    },
    runInstallRos2() {
      record("runInstallRos2");
    },
    runInstallDocker() {
      record("runInstallDocker");
    },
    systemdGenerate() {
      record("systemdGenerate");
    },
    systemdInstall() {
      record("systemdInstall");
    },
    systemdUninstall() {
      record("systemdUninstall");
    },
    printEnvSource() {
      record("printEnvSource");
    },
    runModuleScript(modules, action) {
      record("runModuleScript", [modules, action]);
    },
    cleanWorkspace() {
      record("cleanWorkspace");
    },
    colconBuild() {
      record("colconBuild");
    },
    colconInstall() {
      record("colconInstall");
    },
    downloadSpeechModels() {
      record("downloadSpeechModels");
    },
    runSetupSpeech() {
      record("runSetupSpeech");
    },
    launchSpeechStack(options) {
      record("launchSpeechStack", [options]);
    },
    stopSpeechStack(options) {
      record("stopSpeechStack", [options]);
    },
    testSpeechStack(options) {
      record("testSpeechStack", [options]);
    },
  };

  return { calls, deps };
}

async function captureOutput(fn: () => Promise<unknown>): Promise<string> {
  const originalLog = console.log;
  const originalError = console.error;
  const originalWarn = console.warn;
  const buffer: string[] = [];
  console.log = (...args: unknown[]) => {
    buffer.push(args.map((arg) => String(arg)).join(" "));
  };
  console.error = (...args: unknown[]) => {
    buffer.push(args.map((arg) => String(arg)).join(" "));
  };
  console.warn = (...args: unknown[]) => {
    buffer.push(args.map((arg) => String(arg)).join(" "));
  };
  let thrown: unknown;
  try {
    await fn();
  } catch (err) {
    thrown = err;
  } finally {
    console.log = originalLog;
    console.error = originalError;
    console.warn = originalWarn;
  }
  if (thrown) {
    const message = thrown instanceof Error ? thrown.message : String(thrown);
    if (!message.includes("Test case attempted to exit with exit code: 0")) {
      throw thrown;
    }
  }
  return buffer.join("\n");
}

Deno.test("default command prints summary", async () => {
  const { calls, deps } = createStubDeps();
  const cli = createCli(deps);
  await cli.parse([]);
  assertEquals(Object.keys(calls), ["printSummaryTable"]);
});

Deno.test("install command installs shim", async () => {
  const { calls, deps } = createStubDeps();
  const cli = createCli(deps);
  await cli.parse(["install"]);
  assertEquals(Object.keys(calls), ["installPsh"]);
});

Deno.test("setup passes host list", async () => {
  const { calls, deps } = createStubDeps();
  const cli = createCli(deps);
  await cli.parse(["setup", "alpha", "beta"]);
  assertEquals(calls.setupHosts, [["alpha", "beta"]]);
});

Deno.test("dependency subcommand routes ros2", async () => {
  const { calls, deps } = createStubDeps();
  const cli = createCli(deps);
  await cli.parse(["dep", "ros2"]);
  assertEquals(Object.keys(calls), ["runInstallRos2"]);
});

Deno.test("dependency subcommand routes docker", async () => {
  const { calls, deps } = createStubDeps();
  const cli = createCli(deps);
  await cli.parse(["dep", "docker"]);
  assertEquals(Object.keys(calls), ["runInstallDocker"]);
});

Deno.test("dependency subcommand routes speech setup", async () => {
  const { calls, deps } = createStubDeps();
  const cli = createCli(deps);
  await cli.parse(["dep", "speech"]);
  assertEquals(Object.keys(calls), ["runSetupSpeech"]);
});

Deno.test("systemd generate is default", async () => {
  const { calls, deps } = createStubDeps();
  const cli = createCli(deps);
  await cli.parse(["systemd"]);
  assertEquals(Object.keys(calls), ["systemdGenerate"]);
});

Deno.test("systemd install subcommand", async () => {
  const { calls, deps } = createStubDeps();
  const cli = createCli(deps);
  await cli.parse(["systemd", "install"]);
  assertEquals(Object.keys(calls), ["systemdInstall"]);
});

Deno.test("systemd uninstall subcommand", async () => {
  const { calls, deps } = createStubDeps();
  const cli = createCli(deps);
  await cli.parse(["systemd", "uninstall"]);
  assertEquals(Object.keys(calls), ["systemdUninstall"]);
});

Deno.test("env command prints shell snippet", async () => {
  const { calls, deps } = createStubDeps();
  const cli = createCli(deps);
  await cli.parse(["env"]);
  assertEquals(Object.keys(calls), ["printEnvSource"]);
});

Deno.test("mod command forwards module and action", async () => {
  const { calls, deps } = createStubDeps();
  const cli = createCli(deps);
  await cli.parse(["mod", "pilot", "launch"]);
  assertEquals(calls.runModuleScript, [["pilot"], "launch"]);
});

Deno.test("clean command defaults to workspace", async () => {
  const { calls, deps } = createStubDeps();
  const cli = createCli(deps);
  await cli.parse(["clean"]);
  assertEquals(Object.keys(calls), ["cleanWorkspace"]);
});

Deno.test("clean all triggers every cleaner", async () => {
  const { calls, deps } = createStubDeps();
  const cli = createCli(deps);
  await cli.parse(["clean", "all"]);
  assertEquals(calls.cleanWorkspace, []);
  assertEquals(Object.keys(calls), [
    "cleanWorkspace",
    "uninstallPsh",
    "systemdUninstall",
  ]);
});

Deno.test("clean systemd route", async () => {
  const { calls, deps } = createStubDeps();
  const cli = createCli(deps);
  await cli.parse(["clean", "systemd"]);
  assertEquals(Object.keys(calls), ["systemdUninstall"]);
});

Deno.test("clean install route", async () => {
  const { calls, deps } = createStubDeps();
  const cli = createCli(deps);
  await cli.parse(["clean", "install"]);
  assertEquals(Object.keys(calls), ["uninstallPsh"]);
});

Deno.test("speech launch forwards build flag", async () => {
  const { calls, deps } = createStubDeps();
  const cli = createCli(deps);
  await cli.parse(["speech", "launch", "--build=true"]);
  assertEquals(Object.keys(calls), ["launchSpeechStack"]);
  assertEquals(calls.launchSpeechStack, [{ build: true }]);
});

Deno.test("speech down forwards volumes flag", async () => {
  const { calls, deps } = createStubDeps();
  const cli = createCli(deps);
  await cli.parse(["speech", "down", "--volumes=true"]);
  assertEquals(Object.keys(calls), ["stopSpeechStack"]);
  assertEquals(calls.stopSpeechStack, [{ volumes: true }]);
});

Deno.test("speech test forwards build flag", async () => {
  const { calls, deps } = createStubDeps();
  const cli = createCli(deps);
  await cli.parse(["speech", "test", "--build=true"]);
  assertEquals(Object.keys(calls), ["testSpeechStack"]);
  assertEquals(calls.testSpeechStack, [{ build: true }]);
});

Deno.test("help command output matches --help without invoking actions", async () => {
  const { calls, deps } = createStubDeps();

  const cliHelp = createCli(deps);
  const helpOutput = await captureOutput(() => cliHelp.parse(["help"]));

  const cliFlag = createCli(deps);
  const flagOutput = await captureOutput(() => cliFlag.parse(["--help"]));

  assertEquals(helpOutput, flagOutput);
  assertEquals(Object.keys(calls).length, 0);
});
