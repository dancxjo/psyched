import { join } from "$std/path/mod.ts";
import { StepContext } from "./context.ts";

export async function detectUbuntuCodename(step: StepContext): Promise<string> {
  try {
    const value = (await step.exec(["lsb_release", "-cs"], {
      description: "detect ubuntu codename",
    })).stdout.trim();
    if (value) return value;
  } catch {
    // ignore and fall back to os-release
  }
  const osRelease = await Deno.readTextFile("/etc/os-release");
  for (const line of osRelease.split(/\r?\n/)) {
    if (line.startsWith("VERSION_CODENAME=")) {
      const [, value] = line.split("=", 2);
      if (value) return value.trim();
    }
  }
  return "focal";
}

export async function pathExists(path: string): Promise<boolean> {
  try {
    await Deno.stat(path);
    return true;
  } catch (error) {
    if (error instanceof Deno.errors.NotFound) return false;
    throw error;
  }
}

export async function safeRemove(path: string): Promise<void> {
  try {
    await Deno.remove(path, { recursive: true });
  } catch (error) {
    if (!(error instanceof Deno.errors.NotFound)) {
      throw error;
    }
  }
}

export async function fetchBinary(url: string): Promise<Uint8Array> {
  const response = await fetch(url, {
    headers: {
      "Accept": "application/octet-stream",
      "User-Agent": "psh-provisioner",
    },
  });
  if (!response.ok) {
    throw new Error(
      `Failed to download ${url}: ${response.status} ${response.statusText}`,
    );
  }
  const buffer = await response.arrayBuffer();
  return new Uint8Array(buffer);
}

export async function writeBinaryTemp(
  data: Uint8Array,
  options: { dir?: string; prefix?: string; suffix?: string },
): Promise<string> {
  const dir = options.dir ??
    await Deno.makeTempDir({ prefix: options.prefix ?? "psh-" });
  const path = join(dir, `${crypto.randomUUID()}${options.suffix ?? ""}`);
  await Deno.writeFile(path, data);
  return path;
}
