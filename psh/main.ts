import { Command } from "$cliffy/command/mod.ts";
import { colors } from "$cliffy/ansi/colors.ts";
import { runWizard } from "./lib/wizard.ts";
import { provisionHost } from "./lib/host.ts";
import {
  bringModulesDown,
  bringModulesUp,
  listModules,
  moduleStatuses,
  setupModules,
  teardownModules,
} from "./lib/module.ts";
import {
  bringServiceDown,
  bringServiceUp,
  listServices,
  serviceStatuses,
  setupServices,
  teardownServices,
} from "./lib/service.ts";
import {
  disableSystemd,
  enableSystemd,
  setupSystemd,
  startSystemd,
  stopSystemd,
  teardownSystemd,
} from "./lib/systemd.ts";

const version = "0.2.0";

async function main() {
  const root = new Command()
    .name("psh")
    .version(version)
    .description("psyched shell – Deno edition")
    .action(async () => {
      await runWizard();
    });

  const hostCommand = new Command()
    .description("Host provisioning commands");

  hostCommand
    .command("setup [hosts...:string]")
    .description("Provision the local or specified host(s)")
    .action(async (_, ...hosts: string[]) => {
      if (!hosts.length) {
        await provisionHost();
      } else {
        for (const host of hosts) {
          await provisionHost(host);
        }
      }
    });

  root.command("host", hostCommand);

  const moduleCommand = new Command()
    .description("Module lifecycle commands");

  moduleCommand
    .command("list")
    .description("List available modules and status")
    .action(() => {
      console.log(colors.bold("Modules:"));
      for (const status of moduleStatuses()) {
        const state = status.status === "running"
          ? colors.green(`running${status.pid ? ` (pid ${status.pid})` : ""}`)
          : colors.yellow("stopped");
        console.log(`- ${status.name}: ${state}`);
      }
    });

  moduleCommand
    .command("setup [modules...:string]")
    .description("Run setup lifecycle for modules")
    .action(async (_, ...modules: string[]) => {
      const targets = modules.length ? modules : listModules();
      await setupModules(targets);
    });

  moduleCommand
    .command("teardown [modules...:string]")
    .description("Run teardown lifecycle for modules")
    .action(async (_, ...modules: string[]) => {
      const targets = modules.length ? modules : listModules();
      await teardownModules(targets);
    });

  moduleCommand
    .command("up [modules...:string]")
    .description("Launch module processes")
    .action(async (_, ...modules: string[]) => {
      const targets = modules.length ? modules : listModules();
      await bringModulesUp(targets);
    });

  moduleCommand
    .command("down [modules...:string]")
    .description("Stop module processes")
    .action(async (_, ...modules: string[]) => {
      const targets = modules.length ? modules : listModules();
      await bringModulesDown(targets);
    });

  root.command("mod", moduleCommand).alias("module");

  const serviceCommand = new Command()
    .description("Service lifecycle commands");

  serviceCommand
    .command("list")
    .description("List services and status")
    .action(async () => {
      console.log(colors.bold("Services:"));
      for (const status of await serviceStatuses()) {
        const state = status.status === "running"
          ? colors.green("running")
          : status.status === "stopped"
            ? colors.yellow("stopped")
            : colors.red("error");
        console.log(
          `- ${status.name}: ${state}${status.description ? ` – ${status.description}` : ""
          }`,
        );
      }
    });

  serviceCommand
    .command("setup [services...:string]")
    .description("Run setup for services")
    .action(async (_, ...services: string[]) => {
      const targets = services.length ? services : listServices();
      await setupServices(targets);
    });

  serviceCommand
    .command("teardown [services...:string]")
    .description("Run teardown for services")
    .action(async (_, ...services: string[]) => {
      const targets = services.length ? services : listServices();
      await teardownServices(targets);
    });

  serviceCommand
    .command("up [services...:string]")
    .description("Start services")
    .action(async (_, ...services: string[]) => {
      const targets = services.length ? services : listServices();
      for (const svc of targets) await bringServiceUp(svc);
    });

  serviceCommand
    .command("down [services...:string]")
    .description("Stop services")
    .action(async (_, ...services: string[]) => {
      const targets = services.length ? services : listServices();
      for (const svc of targets) await bringServiceDown(svc);
    });

  root.command("srv", serviceCommand).alias("service");

  const systemCommand = new Command()
    .description("Systemd integration for modules");

  systemCommand
    .command("setup <module:string>")
    .description("Generate a systemd unit for a module")
    .action(async (_, module: string) => {
      await setupSystemd(module);
    });

  systemCommand
    .command("teardown <module:string>")
    .description("Remove the module's systemd unit")
    .action((_, module: string) => {
      teardownSystemd(module);
    });

  systemCommand
    .command("enable <module:string>")
    .description("Enable the module's systemd unit")
    .action(async (_, module: string) => {
      await enableSystemd(module);
    });

  systemCommand
    .command("disable <module:string>")
    .description("Disable the module's systemd unit")
    .action(async (_, module: string) => {
      await disableSystemd(module);
    });

  systemCommand
    .command("up <module:string>")
    .description("Start the module via systemd")
    .action(async (_, module: string) => {
      await startSystemd(module);
    });

  systemCommand
    .command("down <module:string>")
    .description("Stop the module via systemd")
    .action(async (_, module: string) => {
      await stopSystemd(module);
    });

  root.command("sys", systemCommand);

  await root.parse(Deno.args);
}

if (import.meta.main) {
  await main();
}
