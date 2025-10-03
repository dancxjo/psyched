import { colors } from "$cliffy/ansi/colors.ts";
import { Confirm, Select } from "$cliffy/prompt/mod.ts";
import { availableHosts, provisionHost, readHostConfig } from "./host.ts";

export async function runWizard(): Promise<void> {
  console.log(colors.bold(colors.magenta("Psyched Provisioning Wizard")));
  console.log(
    "Let's prepare this host with the correct modules and services.\n",
  );

  const hosts = availableHosts();
  if (!hosts.length) {
    console.error(colors.red("No host configurations found under hosts/."));
    return;
  }

  const defaultHost = hosts.includes(Deno.hostname())
    ? Deno.hostname()
    : hosts[0];
  const hostname = await Select.prompt({
    message: "Select the host profile to apply",
    options: hosts.map((name) => ({ name, value: name })),
    default: defaultHost,
  });

  const config = readHostConfig(hostname);
  console.log(colors.cyan(`\nHost '${hostname}' provisions:`));
  if (config.modules?.length) {
    console.log(colors.green("  Modules:"));
    for (const module of config.modules) {
      const actions = [module.setup !== false ? "setup" : "skip setup"];
      if (module.launch) actions.push("launch");
      console.log(`    • ${module.name} (${actions.join(", ")})`);
    }
  } else {
    console.log(colors.yellow("  No modules configured."));
  }

  if (config.services?.length) {
    console.log(colors.green("  Services:"));
    for (const service of config.services) {
      const actions = [service.setup !== false ? "setup" : "skip setup"];
      if (service.up) actions.push("start");
      console.log(`    • ${service.name} (${actions.join(", ")})`);
    }
  } else {
    console.log(colors.yellow("  No services configured."));
  }

  if (config.provision?.scripts?.length) {
    console.log(colors.green("  Provisioning scripts:"));
    for (const script of config.provision.scripts) {
      console.log(`    • ${script}`);
    }
  }

  const verbose = await Confirm.prompt({
    message: "Show detailed logs during provisioning?",
    default: false,
  });

  const confirmed = await Confirm.prompt({
    message: `Proceed with provisioning for '${hostname}'?`,
    default: true,
  });

  if (!confirmed) {
    console.log(colors.yellow("Provisioning cancelled."));
    return;
  }

  await provisionHost(hostname, { verbose });
  console.log(colors.bold(colors.green("\nAll done!")));
  console.log(
    "Next steps: bring up your modules and/or modules with `psh mod up` and `psh srv up`.",
  );
}
