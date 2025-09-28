import { ensureDir } from "@std/fs";
import { dirname } from "@std/path";
import { downloadSpeechModels } from "./download_models.ts";
import { repoPath } from "./util.ts";

const SPEECH_SENTINEL_PATH = repoPath("setup/.speech_setup_complete");
const SPEECH_SETUP_VERSION = "1";

function readFlagText(text: string): string {
  return text.replace(/\r\n/g, "\n").trim();
}

export function speechSetupSentinelPath(): string {
  return SPEECH_SENTINEL_PATH;
}

export async function isSpeechSetupComplete(): Promise<boolean> {
  try {
    const text = await Deno.readTextFile(SPEECH_SENTINEL_PATH);
    return readFlagText(text) === SPEECH_SETUP_VERSION;
  } catch (err) {
    if (err instanceof Deno.errors.NotFound) {
      return false;
    }
    console.warn(`Failed to read speech setup sentinel: ${String(err)}`);
    return false;
  }
}

export async function runSetupSpeech(): Promise<void> {
  console.log("[psh] Preparing speech stack assets (models, directories).");
  await downloadSpeechModels();
  const parentDir = dirname(SPEECH_SENTINEL_PATH);
  try {
    // If something exists at the parent path and it's a file, ensureDir will
    // fail. Detect that situation and fall back to writing the sentinel at
    // the repository root to avoid crashing `psh setup` when the repo has a
    // top-level file named `setup` (common on some installs).
    const stat = await Deno.lstat(parentDir).catch(() => null);
    if (stat && stat.isFile) {
      const alt = repoPath(".speech_setup_complete");
      console.warn(
        `[psh] Expected '${parentDir}' to be a directory but found a file. ` +
          `Writing speech sentinel to '${alt}' instead.`,
      );
      await Deno.writeTextFile(alt, `${SPEECH_SETUP_VERSION}\n`);
    } else {
      await ensureDir(parentDir);
      await Deno.writeTextFile(SPEECH_SENTINEL_PATH, `${SPEECH_SETUP_VERSION}\n`);
    }
  } catch (err) {
    // Re-throw unexpected errors so callers can surface them.
    throw err;
  }
  console.log("[psh] Speech stack assets ready.");
}
