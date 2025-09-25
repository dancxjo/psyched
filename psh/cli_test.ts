import { assertEquals } from "@std/assert";
import { type CliDeps, createCli } from "./cli.ts";

function createStubDeps() {
  const calls: Record<string, unknown[]> = {};
  const record = (key: keyof CliDeps, args: unknown[] = []) => {
    calls[key as string] = args;
  };

  const deps: CliDeps = {
    async printSummaryTable() {
      record("printSummaryTable");
    },
    async installPsh() {
      record("installPsh");
    },
    async uninstallPsh() {
      record("uninstallPsh");
    },
    async setupHosts(hosts) {
      record("setupHosts", [hosts]);
    },
    async runInstallRos2() {
      record("runInstallRos2");
    },
    async runInstallDocker() {
      record("runInstallDocker");
    },
    async systemdGenerate() {
      record("systemdGenerate");
    },
    async systemdInstall() {
      record("systemdInstall");
    },
    async systemdUninstall() {
      record("systemdUninstall");
    },
    async printEnvSource() {
      record("printEnvSource");
    },
    async runModuleScript(module, action) {
      record("runModuleScript", [module, action]);
    },
    async cleanWorkspace() {
      record("cleanWorkspace");
    },
    async colconBuild() {
      record("colconBuild");
    },
    async colconInstall() {
      record("colconInstall");
    },
  };

  return { calls, deps };
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
  assertEquals(calls.runModuleScript, ["pilot", "launch"]);
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
