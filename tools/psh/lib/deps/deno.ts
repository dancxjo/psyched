import { join } from "$std/path/mod.ts";
import { ProvisionContext } from "./context.ts";

interface DenoTarget {
  arch: string;
  triple: string;
}

const DENO_TARGETS: DenoTarget[] = [
  { arch: "x86_64", triple: "x86_64-unknown-linux-gnu" },
  { arch: "aarch64", triple: "aarch64-unknown-linux-gnu" },
];

export async function installDeno(context: ProvisionContext): Promise<void> {
  const version = Deno.env.get("DENO_VERSION") ?? "v1.46.3";
  const target = determineTarget();
  const existing = await detectDeno();
  const alreadyMatching = existing?.includes(version);

  await context.step("Check Deno runtime", (step) => {
    if (existing) {
      step.log(existing);
      if (alreadyMatching) {
        step.log("Requested version already installed. Skipping download.");
      } else {
        step.log(`Upgrading to ${version}.`);
      }
    } else {
      step.log("Deno not found; preparing installation.");
    }
  });

  if (existing && alreadyMatching) {
    return;
  }

  await context.step("Ensure unzip dependency", async (step) => {
    if (await commandExists("unzip")) {
      step.log("unzip already available");
      return;
    }
    await step.exec(["apt", "update"], {
      sudo: true,
      description: "apt update",
    });
    await step.exec(["apt", "install", "-y", "unzip"], {
      sudo: true,
      description: "install unzip",
    });
  });

  await context.step(`Download Deno ${version}`, async (step) => {
    const url =
      `https://github.com/denoland/deno/releases/download/${version}/deno-${target.triple}.zip`;
    const tmpDir = await Deno.makeTempDir({ prefix: "psh-deno-" });
    try {
      const archivePath = join(tmpDir, "deno.zip");
      const binaryPath = join(tmpDir, "deno");
      await downloadFile(url, archivePath);
      await step.exec(["unzip", "-o", archivePath, "-d", tmpDir], {
        description: "extract deno archive",
      });
      await step.exec([
        "install",
        "-m",
        "0755",
        binaryPath,
        "/usr/local/bin/deno",
      ], {
        sudo: true,
        description: "install deno binary",
      });
    } finally {
      await Deno.remove(tmpDir, { recursive: true }).catch(() => {});
    }
  });

  await context.step("Verify Deno installation", async (step) => {
    const result = await step.exec(["/usr/local/bin/deno", "--version"], {
      description: "deno --version",
      stdoutOnSuccess: true,
    });
    step.log(result.stdout.split(/\r?\n/)[0]);
  });
}

function determineTarget(): DenoTarget {
  const entry = DENO_TARGETS.find((candidate) =>
    candidate.arch === Deno.build.arch
  );
  if (!entry) {
    throw new Error(
      `Unsupported architecture for Deno installer: ${Deno.build.arch}`,
    );
  }
  return entry;
}

async function detectDeno(): Promise<string | null> {
  try {
    const result = await new Deno.Command("deno", {
      args: ["--version"],
      stdout: "piped",
      stderr: "null",
    }).output();
    if (result.success) {
      return new TextDecoder().decode(result.stdout).trim();
    }
  } catch {
    // ignore
  }
  return null;
}

async function commandExists(name: string): Promise<boolean> {
  try {
    const result = await new Deno.Command("which", {
      args: [name],
      stdout: "null",
      stderr: "null",
    }).output();
    return result.success;
  } catch {
    return false;
  }
}

async function downloadFile(url: string, destination: string): Promise<void> {
  const response = await fetch(url, {
    headers: {
      "User-Agent": "psh-provisioner",
    },
  });
  if (!response.ok) {
    throw new Error(
      `Failed to download ${url}: ${response.status} ${response.statusText}`,
    );
  }
  if (!response.body) {
    throw new Error(`Failed to download ${url}: empty response body`);
  }
  const file = await Deno.open(destination, {
    create: true,
    write: true,
    truncate: true,
  });
  try {
    await response.body.pipeTo(file.writable);
  } finally {
    file.close();
  }
}
