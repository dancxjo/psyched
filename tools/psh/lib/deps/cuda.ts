import { join } from "$std/path/mod.ts";
import { ProvisionContext } from "./context.ts";
import { fetchBinary, safeRemove } from "./os.ts";

interface CudaRepoSelection {
  repo: string;
  requestedVersion: string;
  matchedVersion: string;
  fallback: boolean;
  message?: string;
  packages: string[];
}

const DEFAULT_CUDA_PACKAGES = ["cuda-toolkit", "cuda-drivers"];

const CUDA_REPOS: Array<
  { version: number; label: string; repo: string; packages?: string[] }
> = [
  { version: 2404, label: "24.04", repo: "ubuntu2404" },
  { version: 2204, label: "22.04", repo: "ubuntu2204" },
  { version: 2004, label: "20.04", repo: "ubuntu2004" },
  { version: 1804, label: "18.04", repo: "ubuntu1804" },
  {
    version: 1604,
    label: "16.04",
    repo: "ubuntu1604",
    packages: [
      "cuda-toolkit-11-3",
      "cuda-drivers",
    ],
  },
];

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

  const repoSelection = await determineCudaRepo();

  await context.step("Configure NVIDIA CUDA repository", async (step) => {
    const tmpDir = await Deno.makeTempDir({ prefix: "psh-cuda-" });
    try {
      if (repoSelection.message) {
        step.log(repoSelection.message);
      }
      const repo = repoSelection.repo;
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
    const packages = repoSelection.packages;
    step.log(`Installing packages: ${packages.join(", ")}`);
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

async function determineCudaRepo(): Promise<CudaRepoSelection> {
  const osRelease = await Deno.readTextFile("/etc/os-release");
  return selectCudaRepo(osRelease);
}

/**
 * Selects the closest matching CUDA repository for the detected Ubuntu release.
 */
export function selectCudaRepo(osRelease: string): CudaRepoSelection {
  let versionId = "";
  let prettyName = "";
  for (const line of osRelease.split(/\r?\n/)) {
    if (line.startsWith("VERSION_ID=")) {
      const [, value] = line.split("=", 2);
      versionId = value.replace(/\"/g, "").trim();
    } else if (line.startsWith("PRETTY_NAME=")) {
      const [, value] = line.split("=", 2);
      prettyName = value.replace(/\"/g, "").trim();
    }
  }

  const requestedVersion = versionId || "(unknown)";
  const targetLabel = prettyName || `VERSION_ID ${requestedVersion}`;
  const versionNumber = parseVersionId(versionId);

  let selection = CUDA_REPOS[0];
  let fallback = true;
  if (versionNumber !== null) {
    for (const candidate of CUDA_REPOS) {
      selection = candidate;
      if (versionNumber >= candidate.version) {
        fallback = versionNumber !== candidate.version;
        break;
      }
    }
  }

  const message = fallback
    ? `No CUDA repository published for ${targetLabel}; using ${selection.repo} (${selection.label}) packages.`
    : undefined;

  return {
    repo: selection.repo,
    requestedVersion,
    matchedVersion: selection.label,
    fallback,
    message,
    packages: selection.packages ?? DEFAULT_CUDA_PACKAGES,
  };
}

function parseVersionId(versionId: string): number | null {
  if (!versionId) {
    return null;
  }
  const match = versionId.match(/^(\d+)\.(\d+)$/);
  if (!match) {
    return null;
  }
  const major = Number(match[1]);
  const minor = Number(match[2]);
  if (Number.isNaN(major) || Number.isNaN(minor)) {
    return null;
  }
  return (major * 100) + minor;
}
