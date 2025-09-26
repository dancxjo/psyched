// Installer functions for ROS2, Docker, etc.
import { $, repoPath, runWithStreamingTee } from "./util.ts";
import { join } from "@std/path";
import { colors } from "@cliffy/ansi/colors";

/**
 * Check whether ROS2 appears to be installed on the system.
 * We check for a `ros2` executable in PATH and for an /opt/ros directory.
 */
export async function isRos2Installed(): Promise<boolean> {
  // Check for ros2 executable in PATH without invoking a shell builtin.
  if (await existsInPath("ros2")) return true;

  // Fall back to checking the common installation path
  try {
    const st = await Deno.stat("/opt/ros");
    if (st.isDirectory) return true;
  } catch (_err) {
    // missing /opt/ros is fine
  }
  return false;
}

/**
 * Check whether Docker appears to be installed on the system.
 * We check for a `docker` executable in PATH.
 */
export async function isDockerInstalled(): Promise<boolean> {
  return await existsInPath("docker");
}

async function existsInPath(name: string): Promise<boolean> {
  const pathEnv = Deno.env.get("PATH") || "";
  if (!pathEnv) return false;
  const parts = pathEnv.split(":");
  for (const p of parts) {
    if (!p) continue;
    const fp = join(p, name);
    try {
      const st = await Deno.lstat(fp);
      if (st.isFile || st.isSymlink) return true;
    } catch (_err) {
      // missing file — continue
    }
  }
  return false;
}

export async function runInstallRos2(): Promise<void> {
  if (await isRos2Installed()) {
    console.log("ROS2 appears to be installed already — skipping ROS2 install.");
    return;
  }

  const script = repoPath("../tools/install_ros2.sh");
  console.log(colors.cyan(`Starting ROS2 install: ${script}`));
  // Stream output live but capture for a summarized report after completion.
  const res = await runWithStreamingTee(`bash ${script}`);
  if (res.code !== 0) {
    console.error(colors.red(`ROS2 installer failed (code ${res.code})`));
    if (res.stderr) console.error(colors.red(res.stderr));
    throw new Error(`ROS2 installer failed (${res.code})`);
  }
  // On success, summarize but always print any stderr that occurred.
  if (res.stderr && res.stderr.trim().length > 0) {
    console.log(colors.yellow("ROS2 installer produced stderr (see below):"));
    console.log(res.stderr);
  }
  summarizeTextOutput(res.stdout, "ROS2 installer output");
}

export async function runInstallDocker(): Promise<void> {
  if (await isDockerInstalled()) {
    console.log("Docker appears to be installed already — skipping Docker install.");
    return;
  }

  const script = repoPath("../tools/install_docker.sh");
  console.log(colors.cyan(`Starting Docker install: ${script}`));
  const res = await runWithStreamingTee(`bash ${script}`);
  if (res.code !== 0) {
    console.error(colors.red(`Docker installer failed (code ${res.code})`));
    if (res.stderr) console.error(colors.red(res.stderr));
    throw new Error(`Docker installer failed (${res.code})`);
  }
  if (res.stderr && res.stderr.trim().length > 0) {
    console.log(colors.yellow("Docker installer produced stderr (see below):"));
    console.log(res.stderr);
  }
  summarizeTextOutput(res.stdout, "Docker installer output");
}

function summarizeTextOutput(text: string, label = "output") {
  const t = (text || "").replace(/\r\n/g, "\n").trim();
  if (!t) {
    console.log(colors.gray(`${label}: (no output)`));
    return;
  }
  const lines = t.split(/\n/);
  const head = 20;
  const tail = 10;
  if (lines.length <= head + tail + 5) {
    console.log(colors.gray(`${label}:`));
    console.log(t);
  } else {
    console.log(colors.gray(`${label} (first ${head} lines):`));
    console.log(lines.slice(0, head).join("\n"));
    console.log(colors.gray(`... (${lines.length - head - tail} lines omitted) ...`));
    console.log(colors.gray(`${label} (last ${tail} lines):`));
    console.log(lines.slice(-tail).join("\n"));
  }
  console.log(colors.green(`${label} summarized.`));
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
