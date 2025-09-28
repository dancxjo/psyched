import { $, hasFlag, repoPath } from "./util.ts";
import { colors } from "@cliffy/ansi/colors";
import { dirname, join, relative } from "@std/path";

export async function printSummaryTable() {
  const { Table } = await import("@cliffy/table");
  let ros2Installed = false;
  try {
    const ros2Dir = "/opt/ros/";
    for await (const ent of Deno.readDir(ros2Dir)) {
      if (ent.isDirectory) {
        ros2Installed = true;
        break;
      }
    }
  } catch {
    ros2Installed = false;
  }

  let dockerInstalled = false;
  try {
    const r = await $`docker --version`.stdout("null").stderr("null");
    dockerInstalled = r.code === 0;
  } catch {
    dockerInstalled = false;
  }

  const hn = await $`hostname -s`.stdout("piped").stderr("piped");
  const host = (hn.stdout || hn.stderr || "").toString().trim() ||
    Deno.env.get("HOST") || "unknown";
  const hostLabel = host === "unknown" ? colors.red("unknown") : colors.yellow(host);

  const hostsDir = repoPath("hosts");
  const repoRoot = dirname(hostsDir);
  const hostSpecPath = join(hostsDir, `${host}.toml`);
  const hostSpecRel = relative(repoRoot, hostSpecPath);

  const availableHosts: string[] = [];
  try {
    for await (const ent of Deno.readDir(hostsDir)) {
      if (ent.isFile && ent.name.endsWith(".toml")) {
        availableHosts.push(ent.name.replace(/\.toml$/, ""));
      }
    }
  } catch {
    // ignore errors scanning hosts directory
  }
  availableHosts.sort();

  type HostSpecState = "ok" | "missing" | "error";
  let specState: HostSpecState = "missing";
  let specError: string | null = null;
  let spec: Record<string, unknown> | null = null;
  let modules: string[] | null = null;
  try {
    const tomlText = await Deno.readTextFile(hostSpecPath);
    const { parse: parseToml } = await import("@std/toml");
    const parsed = parseToml(tomlText) as Record<string, unknown>;
    spec = parsed;
    specState = "ok";
    if (Array.isArray(parsed.modules)) {
      modules = parsed.modules.map((entry) => String(entry));
    } else {
      modules = [];
    }
  } catch (err) {
    if (err instanceof Deno.errors.NotFound) {
      specState = "missing";
    } else {
      specState = "error";
      specError = String(err);
    }
  }

  const moduleStatus: Array<[string, boolean, boolean]> = [];
  const running: string[] = [];
  if (Array.isArray(modules) && modules.length) {
    for (const mod of modules) {
      let isRunning = false;
      try {
        const r = await $`systemctl is-active psyched-${mod}.service`.stdout(
          "null",
        ).stderr("null");
        isRunning = (r.stdout || "").toString().trim() === "active";
      } catch {
        // ignore systemctl errors
      }
      moduleStatus.push([mod, true, isRunning]);
      if (isRunning) running.push(mod);
    }
  }

  const units: Array<{ name: string; active: boolean }> = [];
  if (specState === "ok") {
    const unitsDir = join(hostsDir, host, "systemd");
    try {
      for await (const ent of Deno.readDir(unitsDir)) {
        if (!ent.name.endsWith(".service")) continue;
        let active = false;
        try {
          const r = await $`systemctl is-active ${ent.name}`.stdout("null")
            .stderr("null");
          active = (r.stdout || "").toString().trim() === "active";
        } catch {
          // ignore systemctl errors
        }
        units.push({ name: ent.name, active });
      }
    } catch {
      // ignore missing systemd dir
    }
  }

  const flagSummary = specState === "ok" && spec
    ? [
      hasFlag(spec, "setup_ros2", "setup-ros2") ? "ros2:on" : "ros2:off",
      hasFlag(spec, "setup_docker", "setup-docker") ? "docker:on" : "docker:off",
      hasFlag(spec, "setup_speech", "setup-speech") ? "speech:on" : "speech:off",
    ].join(", ")
    : "unknown";

  const modulesRow = (() => {
    if (modules === null) {
      return colors.gray("unknown (host spec unavailable)");
    }
    if (modules.length === 0) {
      return colors.gray("none");
    }
    return colors.yellow(modules.join(", "));
  })();

  const runningRow = (() => {
    if (modules === null) {
      return colors.gray("unknown");
    }
    if (!running.length) {
      return colors.gray("none");
    }
    return colors.green(running.join(", "));
  })();

  const unitsRow = (() => {
    if (specState !== "ok") {
      return colors.gray("unknown");
    }
    if (!units.length) {
      return colors.gray("none");
    }
    return colors.magenta(units.map((u) => u.name).join(", "));
  })();

  const hostSpecRow = (() => {
    if (specState === "ok") {
      return colors.green(hostSpecRel);
    }
    if (specState === "missing") {
      return colors.red(`missing (${hostSpecRel})`);
    }
    return colors.red(`error (${hostSpecRel}): ${specError ?? "unknown"}`);
  })();

  const summaryRows: Array<[string, string]> = [
    [colors.cyan("Host"), hostLabel],
    [colors.cyan("Host Spec"), hostSpecRow],
    [colors.cyan("Available Hosts"),
      availableHosts.length
        ? colors.gray(availableHosts.join(", "))
        : colors.gray("none")],
    [
      colors.cyan("Provision Flags"),
      specState === "ok" ? colors.yellow(flagSummary) : colors.gray(flagSummary),
    ],
    [
      colors.cyan("ROS2 Installed"),
      ros2Installed ? colors.green("Yes") : colors.red("No"),
    ],
    [
      colors.cyan("Docker Installed"),
      dockerInstalled ? colors.green("Yes") : colors.red("No"),
    ],
    [colors.cyan("Enabled Modules"), modulesRow],
    [colors.cyan("Running Modules"), runningRow],
    [colors.cyan("Systemd Units"), unitsRow],
  ];

  new Table()
    .header([
      colors.bold("Info"),
      colors.bold("Details"),
    ])
    .body(summaryRows)
    .border(true)
    .render();

  if (Array.isArray(modules) && modules.length) {
    const header = [
      colors.bold("Status"),
      ...modules.map((mod) => colors.bold(colors.cyan(mod))),
    ];
    const enabledRow = [
      colors.bold("Enabled"),
      ...moduleStatus.map(([_, enabled, _r]) =>
        enabled ? colors.green("Yes") : colors.red("No")
      ),
    ];
    const runningStatusRow = [
      colors.bold("Running"),
      ...moduleStatus.map(([_, _e, runningFlag]) =>
        runningFlag ? colors.green("Yes") : colors.red("No")
      ),
    ];
    new Table()
      .header(header)
      .body([
        enabledRow,
        runningStatusRow,
      ])
      .border(true)
      .render();
  }

  if (units.length) {
    const header = units.map((u) => colors.bold(colors.magenta(u.name)));
    const statusRow = units.map((u) =>
      u.active ? colors.green("Yes") : colors.red("No")
    );
    new Table()
      .header(header)
      .body([statusRow])
      .border(true)
      .render();
  }

  console.log(
    colors.gray(
      "Tip: Run 'psh help' for CLI usage and 'psh basics speech' to fetch speech models.",
    ),
  );
}
