import { join } from "$std/path/mod.ts";
import { ProvisionContext } from "./context.ts";
import { fetchBinary, safeRemove } from "./os.ts";

export async function installCuda(context: ProvisionContext): Promise<void> {
  const already = await hasCuda();
  await context.step("Check existing CUDA toolkit", (step) => {
    if (already) {
      step.log("Detected CUDA tools; reinstall to ensure latest packages.");
    } else {
      step.log(
        "CUDA toolkit not detected; provisioning repository and packages.",
      );
    }
  });

  await context.step("Install CUDA prerequisites", async (step) => {
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
        "software-properties-common",
      ],
      { sudo: true, description: "install cuda prerequisites" },
    );
  });

  await context.step("Configure NVIDIA CUDA repository", async (step) => {
    const tmpDir = await Deno.makeTempDir({ prefix: "psh-cuda-" });
    try {
      const repo = await determineCudaRepo();
      const keyUrl =
        `https://developer.download.nvidia.com/compute/cuda/repos/${repo}/x86_64/3bf863cc.pub`;
      const keyData = await fetchBinary(keyUrl);
      const keyPath = join(tmpDir, "cuda-key.pub");
      await Deno.writeFile(keyPath, keyData);
      await step.exec(["mkdir", "-p", "/etc/apt/keyrings"], {
        sudo: true,
        description: "create cuda keyring dir",
      });
      const gpgPath = join(tmpDir, "cuda-keyring.gpg");
      await step.exec(["gpg", "--dearmor", "--output", gpgPath, keyPath], {
        description: "convert cuda key",
      });
      await step.exec([
        "install",
        "-m",
        "0644",
        gpgPath,
        "/etc/apt/keyrings/nvidia-cuda-keyring.gpg",
      ], {
        sudo: true,
        description: "install cuda keyring",
      });
      const repoLine =
        `deb [signed-by=/etc/apt/keyrings/nvidia-cuda-keyring.gpg] https://developer.download.nvidia.com/compute/cuda/repos/${repo}/x86_64/ /`;
      const listPath = join(tmpDir, "cuda.list");
      await Deno.writeTextFile(listPath, `${repoLine}\n`);
      await step.exec([
        "install",
        "-m",
        "0644",
        listPath,
        `/etc/apt/sources.list.d/cuda-${repo}.list`,
      ], {
        sudo: true,
        description: "install cuda apt source",
      });
      await step.exec(["apt-get", "update"], {
        sudo: true,
        description: "apt-get update (cuda)",
      });
    } finally {
      await safeRemove(tmpDir);
    }
  });

  await context.step("Install CUDA packages", async (step) => {
    const packages = ["cuda-toolkit-12-4", "nvidia-driver-535"];
    await step.exec(["apt-get", "install", "-y", ...packages], {
      sudo: true,
      description: "install cuda packages",
    });
    step.log("Reboot after installation to load NVIDIA kernel modules.");
  });
}

async function hasCuda(): Promise<boolean> {
  const checks = [
    new Deno.Command("nvidia-smi", { stdout: "null", stderr: "null" }).output(),
    new Deno.Command("nvcc", {
      args: ["--version"],
      stdout: "null",
      stderr: "null",
    }).output(),
  ];
  try {
    const results = await Promise.all(checks);
    return results.every((result) => result.success);
  } catch {
    return false;
  }
}

async function determineCudaRepo(): Promise<string> {
  const osRelease = await Deno.readTextFile("/etc/os-release");
  let versionId = "";
  for (const line of osRelease.split(/\r?\n/)) {
    if (line.startsWith("VERSION_ID=")) {
      const [, value] = line.split("=", 2);
      versionId = value.replace(/\"/g, "").trim();
      break;
    }
  }
  if (!versionId) {
    throw new Error("Unable to determine VERSION_ID from /etc/os-release");
  }
  const suffix = versionId.replace(/\./g, "");
  return `ubuntu${suffix}`;
}
