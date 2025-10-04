import { join } from "$std/path/mod.ts";
import { ProvisionContext } from "./context.ts";
import {
  detectUbuntuCodename,
  fetchBinary,
  pathExists,
  safeRemove,
} from "./os.ts";

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
  return [
    "# ROS 2 defaults provisioned by psh",
    "export LANG=en_US.UTF-8",
    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
    "export ROS_DOMAIN_ID=0",
    "export ROS_LOCALHOST_ONLY=0",
    "# shellcheck disable=SC1091",
    `. /opt/ros/${distro}/setup.bash`,
    "",
  ].join("\n");
}

export async function installRos2(context: ProvisionContext): Promise<void> {
  const rosDistro = determineRosDistro({ env: Deno.env.toObject() });
  const rosRoot = `/opt/ros/${rosDistro}`;
  const rosExists = await pathExists(rosRoot);

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
    await step.exec(["add-apt-repository", "-y", "universe"], {
      sudo: true,
      description: "enable universe repository",
    });
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
      const codename = await detectUbuntuCodename(step);
      const repoLine =
        `deb [arch=${arch} signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${codename} main`;
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
      const packages = [
        `ros-${rosDistro}-ros-base`,
        `ros-${rosDistro}-rmw-cyclonedds-cpp`,
        "python3-colcon-common-extensions",
        "python3-rosdep",
      ];
      await step.exec(["apt-get", "install", "-y", ...packages], {
        sudo: true,
        description: "apt-get install ros packages",
      });
    },
  );

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
