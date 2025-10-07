import { assertEquals } from "$std/testing/asserts.ts";
import type { ProvisionHostOptions } from "./host.ts";
import {
  __test__ as workflowInternals,
  runSetupWorkflow,
  runTeardownWorkflow,
} from "./workflow.ts";

Deno.test("runSetupWorkflow provisions host then modules and services", async () => {
  const calls: string[] = [];
  let receivedHost: string | undefined;
  let receivedOptions: ProvisionHostOptions | undefined;

  workflowInternals.replaceHostProvisioner((host, options) => {
    receivedHost = host;
    receivedOptions = options;
    calls.push("host");
    return Promise.resolve();
  });

  workflowInternals.replacePshInvoker((args: string[]) => {
    calls.push(`psh:${args.join(" ")}`);
    return Promise.resolve();
  });

  try {
    await runSetupWorkflow({ host: "pete", verbose: true });
  } finally {
    workflowInternals.reset();
  }

  assertEquals(receivedHost, "pete");
  assertEquals(receivedOptions, {
    verbose: true,
    includeModules: false,
    includeServices: false,
  });
  assertEquals(calls, ["host", "psh:mod setup", "psh:srv setup"]);
});

Deno.test("runSetupWorkflow respects skip flags", async () => {
  const commands: string[] = [];
  workflowInternals.replaceHostProvisioner(() => Promise.resolve());
  workflowInternals.replacePshInvoker((args: string[]) => {
    commands.push(args.join(" "));
    return Promise.resolve();
  });

  try {
    await runSetupWorkflow({ skipModules: true });
  } finally {
    workflowInternals.reset();
  }

  assertEquals(commands, ["srv setup"]);
});

Deno.test("runTeardownWorkflow tears down services, modules, and cleans", async () => {
  const calls: string[] = [];
  workflowInternals.replacePshInvoker((args: string[]) => {
    calls.push(`psh:${args.join(" ")}`);
    return Promise.resolve();
  });
  workflowInternals.replaceWorkspaceCleaner(() => {
    calls.push("clean");
    return Promise.resolve();
  });

  try {
    await runTeardownWorkflow();
  } finally {
    workflowInternals.reset();
  }

  assertEquals(calls, [
    "psh:srv teardown",
    "psh:mod teardown",
    "clean",
  ]);
});

Deno.test("runTeardownWorkflow respects skip options", async () => {
  const calls: string[] = [];
  workflowInternals.replacePshInvoker((args: string[]) => {
    calls.push(`psh:${args.join(" ")}`);
    return Promise.resolve();
  });
  workflowInternals.replaceWorkspaceCleaner(() => {
    calls.push("clean");
    return Promise.resolve();
  });

  try {
    await runTeardownWorkflow({
      skipServices: true,
      skipClean: true,
    });
  } finally {
    workflowInternals.reset();
  }

  assertEquals(calls, ["psh:mod teardown"]);
});
