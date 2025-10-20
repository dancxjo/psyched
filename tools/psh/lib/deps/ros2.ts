import { join } from "$std/path/mod.ts";
import { ProvisionContext } from "./context.ts";
import {
  detectDebianRelease,
  fetchBinary,
  pathExists,
  safeRemove,
  shouldEnableUniverse,
} from "./os.ts";
import { getRosDomainId } from "../ros_env.ts";
import { repoRoot } from "../paths.ts";

export interface DetermineRosDistroOptions {
  env: Record<string, string | undefined>;
  defaultDistro?: string;
}

export function determineRosDistro(options: DetermineRosDistroOptions): string {
  const { env, defaultDistro = "kilted" } = options;
  const override = env.ROS_DISTRO?.trim();
  return override && override.length ? override : defaultDistro;
}

export function renderRosProfile(distro: string): string {
  const domainId = getRosDomainId();
  return [
    "# ROS 2 defaults provisioned by psh",
    "export LANG=en_US.UTF-8",
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
    `export ROS_DOMAIN_ID=${domainId}`,
    "export ROS_LOCALHOST_ONLY=0",
    "# shellcheck disable=SC1091",
    `. /opt/ros/${distro}/setup.bash`,
    "",
  ].join("\n");
}

export function buildRosBasePackages(distro: string): string[] {
  return [
    `ros-${distro}-ros-base`,
    `ros-${distro}-rmw-cyclonedds-cpp`,
    "python3-rosdep",
  ];
}

export interface ColconProvisionPlan {
  venvPath: string;
  pythonBin: string;
  pipBin: string;
  colconBin: string;
  symlinkPath: string;
  packages: string[];
}

export function buildColconProvisionPlan(distro: string): ColconProvisionPlan {
  const venvPath = `/opt/ros/${distro}/colcon-venv`;
  const binDir = `${venvPath}/bin`;
  return {
    venvPath,
    pythonBin: `${binDir}/python`,
    pipBin: `${binDir}/pip`,
    colconBin: `${binDir}/colcon`,
    symlinkPath: "/usr/local/bin/colcon",
    packages: [
      "colcon-core",
      "colcon-common-extensions",
    ],
  };
}

export type Ros2InstallMode = "native" | "container";

export interface Ros2InstallPlan {
  mode: Ros2InstallMode;
  helperPath: string;
  containerImage?: string;
  containerName?: string;
  networkMode?: string;
  workspaceSource?: string;
  workspaceTarget?: string;
  user?: string;
  volumes: string[];
  passEnv: string[];
  env: Record<string, string>;
}

export interface Ros2ContainerProfileOptions {
  helperPath: string;
  rosDistro: string;
}

type UnknownRecord = Record<string, unknown>;

function isRecord(value: unknown): value is UnknownRecord {
  return !!value && typeof value === "object" && !Array.isArray(value);
}

function readString(
  value: unknown,
  context: string,
): string | undefined {
  if (value === undefined) return undefined;
  if (typeof value !== "string") {
    throw new Error(`Expected ${context} to be a string`);
  }
  const trimmed = value.trim();
  return trimmed.length ? trimmed : undefined;
}

function readStringArray(
  value: unknown,
  context: string,
): string[] | undefined {
  if (value === undefined) return undefined;
  if (!Array.isArray(value)) {
    throw new Error(`Expected ${context} to be an array`);
  }
  const results: string[] = [];
  for (const [index, entry] of value.entries()) {
    const item = readString(entry, `${context}[${index}]`);
    if (!item) {
      throw new Error(`Expected ${context}[${index}] to be non-empty`);
    }
    if (!results.includes(item)) {
      results.push(item);
    }
  }
  return results;
}

function readStringRecord(
  value: unknown,
  context: string,
): Record<string, string> | undefined {
  if (value === undefined) return undefined;
  if (!isRecord(value)) {
    throw new Error(`Expected ${context} to be a table`);
  }
  const result: Record<string, string> = {};
  for (const [key, entry] of Object.entries(value)) {
    const stringKey = readString(key, `${context} key`);
    if (!stringKey) continue;
    const stringValue = readString(entry, `${context}.${key}`);
    if (stringValue === undefined) {
      continue;
    }
    result[stringKey] = stringValue;
  }
  return result;
}

function defaultHelperPath(): string {
  return "/usr/local/bin/ros2-container";
}

export function resolveRos2InstallPlan(
  raw?: UnknownRecord,
): Ros2InstallPlan {
  if (raw !== undefined && !isRecord(raw)) {
    throw new Error("ROS 2 installer config must be a table");
  }

  const helperPath = readString(raw?.helper_path, "ros2.helper_path") ??
    defaultHelperPath();
  const modeText = readString(raw?.mode, "ros2.mode")?.toLowerCase();
  const mode: Ros2InstallMode = (() => {
    if (!modeText || modeText === "container") return "container";
    if (modeText === "native") return "native";
    throw new Error(
      `Unsupported ros2 installer mode '${modeText}'. Expected 'native' or 'container'.`,
    );
  })();

  if (mode === "native") {
    return {
      mode,
      helperPath,
      volumes: [],
      passEnv: [],
      env: {},
    };
  }

  const containerRaw = raw?.container;
  if (containerRaw !== undefined && !isRecord(containerRaw)) {
    throw new Error("ros2.container must be a table when provided");
  }

  const containerImage = readString(
    containerRaw?.image ?? raw?.image ?? raw?.container_image,
    "ros2.container.image",
  ) ?? "osrf/ros:humble-desktop";
  const containerName = readString(
    containerRaw?.name ?? raw?.name ?? raw?.container_name,
    "ros2.container.name",
  ) ?? "psyched-ros2";
  const networkMode = readString(
    containerRaw?.network_mode ?? raw?.network_mode,
    "ros2.container.network_mode",
  ) ?? "host";
  const workspaceSource = readString(
    containerRaw?.workspace_source ?? raw?.workspace_source,
    "ros2.container.workspace_source",
  );
  const workspaceTarget = readString(
    containerRaw?.workspace_target ?? raw?.workspace_target,
    "ros2.container.workspace_target",
  );
  const user = readString(
    containerRaw?.user ?? raw?.user,
    "ros2.container.user",
  );
  const volumeList = readStringArray(
    containerRaw?.volumes ?? raw?.volumes,
    "ros2.container.volumes",
  ) ?? [];
  const passEnv = readStringArray(
    containerRaw?.pass_env ?? raw?.pass_env,
    "ros2.container.pass_env",
  ) ?? [];
  const envRecord = readStringRecord(
    containerRaw?.env ?? raw?.env,
    "ros2.container.env",
  ) ?? {};

  const volumeSet = new Set<string>();
  const volumes: string[] = [];
  const defaults = ["/tmp/.X11-unix:/tmp/.X11-unix:rw"];
  for (const entry of defaults) {
    volumeSet.add(entry);
    volumes.push(entry);
  }
  for (const volume of volumeList) {
    if (!volumeSet.has(volume)) {
      volumeSet.add(volume);
      volumes.push(volume);
    }
  }

  const passEnvSet = new Set<string>();
  const passEnvResult: string[] = [];
  for (const envName of ["DISPLAY", ...passEnv]) {
    if (!envName) continue;
    if (!passEnvSet.has(envName)) {
      passEnvSet.add(envName);
      passEnvResult.push(envName);
    }
  }

  return {
    mode,
    helperPath,
    containerImage,
    containerName,
    networkMode,
    workspaceSource,
    workspaceTarget,
    user,
    volumes,
    passEnv: passEnvResult,
    env: envRecord,
  };
}

export function renderRosContainerProfile(
  options: Ros2ContainerProfileOptions,
): string {
  const domainId = getRosDomainId();
  const helper = shellQuote(options.helperPath);
  return [
    "# ROS 2 container defaults provisioned by psh",
    "export LANG=en_US.UTF-8",
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
    `export ROS_DOMAIN_ID=${domainId}`,
    "export ROS_LOCALHOST_ONLY=0",
    `alias ros2-container=${helper}`,
    "",
  ].join("\n");
}

function shellQuote(value: string): string {
  return `'${value.replace(/'/g, "'\\''")}'`;
}

interface BuildContainerHelperScriptOptions {
  image: string;
  containerName: string;
  networkMode: string;
  workspaceSource: string;
  workspaceTarget: string;
  user?: string;
  volumes: string[];
  passEnv: string[];
  env: Record<string, string>;
}

function buildContainerHelperScript(
  options: BuildContainerHelperScriptOptions,
): string {
  const {
    image,
    containerName,
    networkMode,
    workspaceSource,
    workspaceTarget,
    user,
    volumes,
    passEnv,
    env,
  } = options;

  const volumeLines = volumes.map((volume) =>
    `  ${shellQuote(volume)}`
  ).join("\n");
  const passEnvLines = passEnv.map((name) =>
    `  ${shellQuote(name)}`
  ).join("\n");
  const envLines = Object.entries(env).map(([key, value]) =>
    `  ${shellQuote(`${key}=${value}`)}`
  ).join("\n");

  const script: string[] = [
    "#!/usr/bin/env bash",
    "set -euo pipefail",
    "",
    `readonly image=${shellQuote(image)}`,
    `readonly container_name=${shellQuote(containerName)}`,
    `readonly network_mode=${shellQuote(networkMode)}`,
    `readonly workspace_source=${shellQuote(workspaceSource)}`,
    `readonly workspace_target=${shellQuote(workspaceTarget)}`,
    `readonly custom_user=${shellQuote(user ?? "")}`,
    "",
    "if ! command -v docker >/dev/null 2>&1; then",
    "  echo 'docker is required to launch the ROS 2 container helper' >&2",
    "  exit 1",
    "fi",
    "",
    "user_value=\"$(id -u):$(id -g)\"",
    "if [[ -n \"$custom_user\" ]]; then",
    "  user_value=\"$custom_user\"",
    "fi",
    "",
    "args=(",
    "  \"docker\" \"run\" \"--rm\" \"-it\"",
    "  \"--network\" \"$network_mode\"",
    "  \"--name\" \"$container_name\"",
    "  \"--user\" \"$user_value\"",
    "  \"--workdir\" \"$workspace_target\"",
    "  \"-v\" \"$workspace_source:$workspace_target\"",
    ")",
  ];

  if (volumes.length) {
    script.push(
      "",
      "volume_mounts=(",
      volumeLines,
      ")",
      "for volume in \"${volume_mounts[@]}\"; do",
      "  args+=(\"-v\" \"$volume\")",
      "done",
    );
  }

  if (passEnv.length) {
    script.push(
      "",
      "pass_env=(",
      passEnvLines,
      ")",
      "for env_name in \"${pass_env[@]}\"; do",
      "  args+=(\"-e\" \"$env_name\")",
      "done",
    );
  }

  if (Object.keys(env).length) {
    script.push(
      "",
      "env_pairs=(",
      envLines,
      ")",
      "for pair in \"${env_pairs[@]}\"; do",
      "  args+=(\"-e\" \"$pair\")",
      "done",
    );
  }

  script.push(
    "exec \"${args[@]}\" \"$image\" \"$@\"",
    "",
  );

  return script.join("\n");
}

export async function installRos2(context: ProvisionContext): Promise<void> {
  const rosDistro = determineRosDistro({ env: Deno.env.toObject() });
  const plan = resolveRos2InstallPlan(context.config);

  if (plan.mode === "container") {
    await installRos2Container(context, { rosDistro, plan });
    return;
  }

  const rosRoot = `/opt/ros/${rosDistro}`;
  const rosExists = await pathExists(rosRoot);

  const release = await context.step(
    "Detect Debian/Ubuntu derivative",
    async (step) => {
      const info = await detectDebianRelease(step);
      step.log(`Detected ${info.id} (${info.codename})`);
      return info;
    },
  );

  await context.step(`Detect ROS 2 ${rosDistro} installation`, (step) => {
    if (rosExists) {
      step.log(`Found existing ROS installation under ${rosRoot}`);
    } else {
      step.log(`No existing ROS detected; will install ${rosDistro}.`);
    }
  });

  await context.step("Install ROS 2 prerequisites", async (step) => {
    await step.exec(["apt-get", "update"], {
      sudo: true,
      description: "apt-get update",
    });
    await step.exec(
      [
        "apt-get",
        "install",
        "-y",
        "ca-certificates",
        "curl",
        "gnupg",
        "lsb-release",
        "locales",
        "software-properties-common",
      ],
      { sudo: true, description: "apt-get install prerequisites" },
    );
    if (shouldEnableUniverse(release)) {
      await step.exec(["add-apt-repository", "-y", "universe"], {
        sudo: true,
        description: "enable universe repository",
      });
    } else {
      step.log(
        "Skipping Ubuntu 'universe' repository because the host is not Ubuntu.",
      );
    }
    await step.exec(["locale-gen", "en_US", "en_US.UTF-8"], {
      sudo: true,
      description: "generate locales",
    });
    await step.exec(
      ["update-locale", "LC_ALL=en_US.UTF-8", "LANG=en_US.UTF-8"],
      {
        sudo: true,
        description: "set locale defaults",
      },
    );
  });

  await context.step("Configure ROS 2 apt repository", async (step) => {
    const tmpDir = await Deno.makeTempDir({ prefix: "psh-ros2-" });
    try {
      const keyUrl =
        "https://raw.githubusercontent.com/ros/rosdistro/master/ros.key";
      const keyResponse = await fetchBinary(keyUrl);
      const keyPath = join(tmpDir, "ros.key");
      await Deno.writeFile(keyPath, keyResponse);
      const gpgPath = join(tmpDir, "ros-archive-keyring.gpg");
      await step.exec(["gpg", "--dearmor", "--output", gpgPath, keyPath], {
        description: "convert ROS key to gpg",
      });
      await step.exec(["mkdir", "-p", "/etc/apt/keyrings"], {
        sudo: true,
        description: "create keyring directory",
      });
      await step.exec([
        "install",
        "-m",
        "0644",
        gpgPath,
        "/etc/apt/keyrings/ros-archive-keyring.gpg",
      ], {
        sudo: true,
        description: "install ROS keyring",
      });
      const arch = (await step.exec(["dpkg", "--print-architecture"], {
        description: "detect architecture",
      })).stdout.trim();
      const repoLine =
        `deb [arch=${arch} signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${release.codename} main`;
      const repoPath = join(tmpDir, "ros2.list");
      await Deno.writeTextFile(repoPath, `${repoLine}\n`);
      await step.exec([
        "install",
        "-m",
        "0644",
        repoPath,
        "/etc/apt/sources.list.d/ros2.list",
      ], {
        sudo: true,
        description: "install ROS apt source",
      });
      await step.exec(["apt-get", "update"], {
        sudo: true,
        description: "apt-get update (ros2)",
      });
    } finally {
      await safeRemove(tmpDir);
    }
  });

  await context.step(
    `Install ROS 2 ${rosDistro} base packages`,
    async (step) => {
      const packages = buildRosBasePackages(rosDistro);
      await step.exec(["apt-get", "install", "-y", ...packages], {
        sudo: true,
        description: "apt-get install ros packages",
      });
    },
  );

  const colconPlan = buildColconProvisionPlan(rosDistro);
  await context.step("Provision colcon build toolchain", async (step) => {
    await step.exec(["mkdir", "-p", colconPlan.venvPath], {
      sudo: true,
      description: "prepare colcon venv directory",
    });
    await step.exec([
      "bash",
      "-c",
      `[ -f "${colconPlan.venvPath}/pyvenv.cfg" ] || python3 -m venv "${colconPlan.venvPath}"`,
    ], {
      sudo: true,
      description: "initialise colcon virtualenv",
    });
    await step.exec([
      "bash",
      "-c",
      `[ -x "${colconPlan.pipBin}" ] || "${colconPlan.pythonBin}" -m ensurepip --upgrade`,
    ], {
      sudo: true,
      description: "bootstrap pip for colcon",
    });
    await step.exec([
      colconPlan.pipBin,
      "install",
      "--no-cache-dir",
      "--upgrade",
      "pip",
    ], {
      sudo: true,
      description: "upgrade pip inside colcon venv",
    });
    await step.exec([
      colconPlan.pipBin,
      "install",
      "--no-cache-dir",
      "--upgrade",
      ...colconPlan.packages,
    ], {
      sudo: true,
      description: "install colcon tooling",
    });
    await step.exec([
      "ln",
      "-sf",
      colconPlan.colconBin,
      colconPlan.symlinkPath,
    ], {
      sudo: true,
      description: "link colcon binary",
    });
  });

  await context.step("Initialise rosdep", async (step) => {
    const rosdepSources = "/etc/ros/rosdep/sources.list.d/20-default.list";
    if (!(await pathExists(rosdepSources))) {
      await step.exec(["rosdep", "init"], {
        sudo: true,
        description: "rosdep init",
      });
    } else {
      step.log("rosdep already initialised");
    }
    await step.exec(["rosdep", "update"], {
      description: "rosdep update",
    });
  });

  await context.step("Write ROS environment defaults", async (step) => {
    const profile = renderRosProfile(rosDistro);
    const tmpFile = await Deno.makeTempFile({
      prefix: "psh-ros2-profile-",
      suffix: ".sh",
    });
    try {
      await Deno.writeTextFile(tmpFile, profile);
      await step.exec([
        "install",
        "-m",
        "0644",
        tmpFile,
        "/etc/profile.d/ros2-defaults.sh",
      ], {
        sudo: true,
        description: "install ros2 profile",
      });
    } finally {
      await safeRemove(tmpFile);
    }
  });
}

interface Ros2ContainerInstallOptions {
  rosDistro: string;
  plan: Ros2InstallPlan;
}

async function installRos2Container(
  context: ProvisionContext,
  options: Ros2ContainerInstallOptions,
): Promise<void> {
  const { plan, rosDistro } = options;
  const image = plan.containerImage ?? "osrf/ros:humble-desktop";
  const containerName = plan.containerName ?? "psyched-ros2";
  const networkMode = plan.networkMode ?? "host";
  const helperPath = plan.helperPath ?? defaultHelperPath();
  const workspaceSource = plan.workspaceSource ?? repoRoot();
  const workspaceTarget = plan.workspaceTarget ?? workspaceSource;
  const additionalVolumes = plan.volumes.filter((volume) =>
    volume !== `${workspaceSource}:${workspaceTarget}`
  );
  const passEnv = plan.passEnv.length ? plan.passEnv : ["DISPLAY"];
  const env = plan.env;

  await context.step("Validate Docker availability", async (step) => {
    await step.exec(["docker", "--version"], {
      description: "docker --version",
    });
  });

  await context.step(`Fetch ROS 2 container image (${image})`, async (step) => {
    await step.exec(["docker", "pull", image], {
      description: `docker pull ${image}`,
    });
  });

  await context.step("Install ros2 container helper", async (step) => {
    const script = buildContainerHelperScript({
      image,
      containerName,
      networkMode,
      workspaceSource,
      workspaceTarget,
      user: plan.user,
      volumes: additionalVolumes,
      passEnv,
      env,
    });
    const tmpFile = await Deno.makeTempFile({
      prefix: "psh-ros2-container-",
      suffix: ".sh",
    });
    try {
      await Deno.writeTextFile(tmpFile, script);
      await step.exec([
        "install",
        "-m",
        "0755",
        tmpFile,
        helperPath,
      ], {
        sudo: true,
        description: "install ros2 container helper",
      });
    } finally {
      await safeRemove(tmpFile);
    }
  });

  await context.step("Write ROS container environment defaults", async (step) => {
    const profile = renderRosContainerProfile({
      helperPath,
      rosDistro,
    });
    const tmpFile = await Deno.makeTempFile({
      prefix: "psh-ros2-container-profile-",
      suffix: ".sh",
    });
    try {
      await Deno.writeTextFile(tmpFile, profile);
      await step.exec([
        "install",
        "-m",
        "0644",
        tmpFile,
        "/etc/profile.d/ros2-defaults.sh",
      ], {
        sudo: true,
        description: "install ros2 container profile",
      });
    } finally {
      await safeRemove(tmpFile);
    }
  });

  await context.step("ROS 2 container provisioning summary", (step) => {
    step.log(`Container helper installed at ${helperPath}`);
    step.log(`Use 'sudo ${helperPath}' if your user is not in the docker group.`);
  });
}
