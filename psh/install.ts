// Installer functions for ROS2, Docker, etc.
import { $, repoPath } from "./util.ts";

export async function runInstallRos2(): Promise<void> {
  const script = repoPath("../tools/install_ros2.sh");
  console.log(`Running ROS2 install script: ${script}`);
  const r = await $`bash ${script}`;
  if (r.code !== 0) {
    console.error(r.stderr || r.stdout);
    throw new Error(`ROS2 installer failed (${r.code})`);
  }
}

export async function runInstallDocker(): Promise<void> {
  const script = repoPath("../tools/install_docker.sh");
  console.log(`Running Docker install script: ${script}`);
  const r = await $`bash ${script}`;
  if (r.code !== 0) {
    console.error(r.stderr || r.stdout);
    throw new Error(`Docker installer failed (${r.code})`);
  }
}

export async function installPsh(): Promise<void> {
  const target = "/usr/bin/psh";
  const mainPath = repoPath("./main.ts");
  const shim =
    `#!/usr/bin/env bash\nset -euo pipefail\nexec deno run -A \"${mainPath}\" \"$@\"\n`;
  const tmpFile = await Deno.makeTempFile({
    prefix: "psh-shim-",
    suffix: ".sh",
  });
  try {
    await Deno.writeTextFile(tmpFile, shim);
    await Deno.chmod(tmpFile, 0o755);
    const result = await $`sudo install -m 755 ${tmpFile} ${target}`.noThrow();
    if (result.code !== 0) {
      const message = result.stderr || result.stdout || "unknown error";
      throw new Error(`Failed to install psh shim: ${message}`);
    }
    console.log(`[install] Installed CLI shim at ${target}`);
  } finally {
    await Deno.remove(tmpFile).catch(() => undefined);
  }
}
