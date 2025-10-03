import { join } from "$std/path/mod.ts";
import { ProvisionContext } from "./context.ts";
import { detectUbuntuCodename, fetchBinary, safeRemove } from "./os.ts";

export async function installDocker(context: ProvisionContext): Promise<void> {
  await context.step("Install Docker prerequisites", async (step) => {
    await step.exec(["apt", "update"], {
      sudo: true,
      description: "apt update",
    });
    await step.exec(
      [
        "apt",
        "install",
        "-y",
        "ca-certificates",
        "curl",
        "gnupg",
        "lsb-release",
      ],
      { sudo: true, description: "install docker prerequisites" },
    );
  });

  await context.step("Configure Docker repository", async (step) => {
    const tmpDir = await Deno.makeTempDir({ prefix: "psh-docker-" });
    try {
      const keyData = await fetchBinary(
        "https://download.docker.com/linux/ubuntu/gpg",
      );
      const keyPub = join(tmpDir, "docker.pub");
      const keyGpg = join(tmpDir, "docker.gpg");
      await Deno.writeFile(keyPub, keyData);
      await step.exec(["mkdir", "-p", "/etc/apt/keyrings"], {
        sudo: true,
        description: "create docker keyring dir",
      });
      await step.exec(["gpg", "--dearmor", "--output", keyGpg, keyPub], {
        description: "convert docker key",
      });
      await step.exec([
        "install",
        "-m",
        "0644",
        keyGpg,
        "/etc/apt/keyrings/docker.gpg",
      ], {
        sudo: true,
        description: "install docker keyring",
      });
      const arch = (await step.exec(["dpkg", "--print-architecture"], {
        description: "detect architecture",
      })).stdout.trim();
      const codename = await detectUbuntuCodename(step);
      const repoLine =
        `deb [arch=${arch} signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu ${codename} stable`;
      const repoPath = join(tmpDir, "docker.list");
      await Deno.writeTextFile(repoPath, `${repoLine}\n`);
      await step.exec([
        "install",
        "-m",
        "0644",
        repoPath,
        "/etc/apt/sources.list.d/docker.list",
      ], {
        sudo: true,
        description: "install docker apt source",
      });
      await step.exec(["apt", "update"], {
        sudo: true,
        description: "apt update (docker)",
      });
    } finally {
      await safeRemove(tmpDir);
    }
  });

  await context.step("Install Docker Engine", async (step) => {
    await step.exec(
      [
        "apt",
        "install",
        "-y",
        "docker-ce",
        "docker-ce-cli",
        "containerd.io",
        "docker-buildx-plugin",
        "docker-compose-plugin",
      ],
      {
        sudo: true,
        description: "install docker packages",
      },
    );
  });

  await context.step("Enable Docker service", async (step) => {
    await step.exec(["systemctl", "enable", "--now", "docker"], {
      sudo: true,
      description: "enable docker service",
    });
  });

  await context.step("Manage docker group membership", async (step) => {
    const targetUser = determineTargetUser();
    if (!targetUser) {
      step.log("No non-root user detected; skipping group assignment.");
      return;
    }
    await step.exec(["groupadd", "-f", "docker"], {
      sudo: true,
      description: "ensure docker group",
    });
    await step.exec(["usermod", "-aG", "docker", targetUser], {
      sudo: true,
      description: `add ${targetUser} to docker group`,
    });
    step.log(
      `Added ${targetUser} to docker group. Log out/in or run 'newgrp docker' to refresh.`,
    );
  });
}

function determineTargetUser(): string | undefined {
  const sudoUser = Deno.env.get("SUDO_USER");
  if (sudoUser && sudoUser !== "root") {
    return sudoUser;
  }
  const user = Deno.env.get("USER") ?? Deno.env.get("LOGNAME");
  if (user && user !== "root") {
    return user;
  }
  return undefined;
}
