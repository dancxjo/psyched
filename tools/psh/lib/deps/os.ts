import { join } from "$std/path/mod.ts";
import { StepContext } from "./context.ts";

export interface DebianReleaseInfo {
  /**
   * Normalised identifier from /etc/os-release (for example `ubuntu` or `debian`).
   */
  id: string;
  /**
   * Lowercase identifiers that the current distribution claims compatibility with.
   */
  idLike: string[];
  /**
   * Lowercase Debian/Ubuntu codename (for example `jammy`, `bookworm`).
   */
  codename: string;
}

/**
 * Parse key/value pairs from /etc/os-release content. Quoted values are
 * unwrapped and identifiers are returned verbatim for downstream processing.
 */
export function parseOsRelease(content: string): Record<string, string> {
  const entries: Record<string, string> = {};
  for (const rawLine of content.split(/\r?\n/)) {
    const line = rawLine.trim();
    if (!line || line.startsWith("#")) continue;
    const eqIndex = line.indexOf("=");
    if (eqIndex === -1) continue;
    const key = line.slice(0, eqIndex);
    let value = line.slice(eqIndex + 1);
    if ((value.startsWith('"') && value.endsWith('"')) ||
      (value.startsWith("'") && value.endsWith("'"))) {
      value = value.slice(1, -1);
    }
    entries[key] = value;
  }
  return entries;
}

/**
 * Resolve Debian/Ubuntu derivative metadata from os-release content and an
 * optional lsb_release codename override. The resulting structure normalises
 * identifiers to lowercase for easier downstream comparisons.
 */
export function resolveDebianReleaseInfo(options: {
  osReleaseText?: string;
  lsbCodename?: string;
}): DebianReleaseInfo {
  const osReleaseEntries = options.osReleaseText
    ? parseOsRelease(options.osReleaseText)
    : {};
  const id = (osReleaseEntries.ID ?? "ubuntu").trim().toLowerCase();
  const idLike = (osReleaseEntries.ID_LIKE ?? "")
    .split(/\s+/)
    .map((item) => item.trim().toLowerCase())
    .filter((item) => item.length > 0);

  const lsbCodename = options.lsbCodename?.trim().toLowerCase();
  const versionCodename = (osReleaseEntries.VERSION_CODENAME ?? "")
    .trim().toLowerCase();

  let codename = lsbCodename || versionCodename;
  if (!codename) {
    const versionText = osReleaseEntries.VERSION ?? "";
    const match = versionText.match(/\(([^)]+)\)/);
    if (match) {
      codename = match[1].split(/\s+/)[0]?.toLowerCase() ?? "";
    }
  }
  if (!codename) {
    codename = "focal";
  }

  return { id, idLike, codename };
}

/**
 * Detect Debian/Ubuntu derivative metadata for the current system. The helper
 * prefers the `lsb_release` CLI for codename detection and falls back to
 * parsing `/etc/os-release`.
 */
export async function detectDebianRelease(
  step: StepContext,
): Promise<DebianReleaseInfo> {
  let lsbCodename: string | undefined;
  try {
    const result = await step.exec(["lsb_release", "-cs"], {
      description: "detect debian codename",
    });
    lsbCodename = result.stdout.trim();
  } catch {
    // Ignore failures and fall back to os-release content.
  }

  let osReleaseText: string | undefined;
  try {
    osReleaseText = await Deno.readTextFile("/etc/os-release");
  } catch {
    // Defer to defaults when /etc/os-release is missing.
  }

  return resolveDebianReleaseInfo({ osReleaseText, lsbCodename });
}

/** Determine whether the Ubuntu "universe" repository should be enabled. */
export function shouldEnableUniverse(info: DebianReleaseInfo): boolean {
  return info.id === "ubuntu" || info.idLike.includes("ubuntu");
}

/**
 * Decide whether to use Docker's Ubuntu or Debian repository channel for the
 * current system.
 */
export function determineDockerChannel(
  info: DebianReleaseInfo,
): "ubuntu" | "debian" {
  const hasUbuntuLineage =
    info.id === "ubuntu" || info.idLike.includes("ubuntu");
  if (hasUbuntuLineage) {
    return "ubuntu";
  }
  const hasDebianLineage =
    info.id === "debian" || info.idLike.includes("debian");
  return hasDebianLineage ? "debian" : "ubuntu";
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
