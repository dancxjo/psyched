import { join } from "$std/path/mod.ts";
import { repoRoot } from "./paths.ts";

const DEFAULT_ROS_DOMAIN_ID = "25";
const ROS_DOMAIN_ID_FILE = ["config", "ros_domain_id"];

function sanitizeDomainId(value: string | undefined): string | undefined {
  if (!value) return undefined;
  const trimmed = value.trim();
  if (!trimmed) return undefined;
  const numeric = trimmed.replace(/[^0-9]/g, "");
  return numeric ? numeric : undefined;
}

function readDomainIdFromFile(): string | undefined {
  try {
    const path = join(repoRoot(), ...ROS_DOMAIN_ID_FILE);
    const content = Deno.readTextFileSync(path);
    return sanitizeDomainId(content);
  } catch (_error) {
    return undefined;
  }
}

let cachedDomainId: string | undefined;

export function getRosDomainId(): string {
  if (cachedDomainId !== undefined) {
    return cachedDomainId;
  }

  const envOverride = sanitizeDomainId(Deno.env.get("PSYCHED_ROS_DOMAIN_ID"));
  if (envOverride) {
    cachedDomainId = envOverride;
    return cachedDomainId;
  }

  const fileValue = readDomainIdFromFile();
  if (fileValue) {
    cachedDomainId = fileValue;
    return cachedDomainId;
  }

  cachedDomainId = DEFAULT_ROS_DOMAIN_ID;
  return cachedDomainId;
}

export function buildRosEnv(): Record<string, string> {
  return {
    ROS_DOMAIN_ID: getRosDomainId(),
  };
}

export async function sourcePsychedEnv(): Promise<Record<string, string>> {
  const psychedEnvPath = join(repoRoot(), "env", "psyched_env.sh");
  const command = new Deno.Command("bash", {
    args: ["-c", `source ${psychedEnvPath} && env`],
  });
  const { stdout } = await command.output();
  const output = new TextDecoder().decode(stdout);
  const env: Record<string, string> = {};
  for (const line of output.split("\n")) {
    const [key, ...value] = line.split("=");
    if (key && value.length > 0) {
      env[key] = value.join("=");
    }
  }
  return { ...env, ...buildRosEnv() };
}

export function resetRosDomainCache(): void {
  cachedDomainId = undefined;
}
