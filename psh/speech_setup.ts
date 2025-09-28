import { ensureDir } from "@std/fs";
import { dirname } from "@std/path";
import { downloadSpeechModels } from "./download_models.ts";
import { repoPath } from "./util.ts";

const PREFERRED_SPEECH_SENTINEL = repoPath("setup/.speech_setup_complete");
const ALT_SPEECH_SENTINEL = repoPath(".speech_setup_complete");
const SPEECH_SETUP_VERSION = "1";

/**
 * Return the list of sentinel candidates in preference order. Callers should
 * attempt to read the first existing file; writers should select the most
 * appropriate path (prefer the setup/ directory, but fall back to repo root
 * when a top-level `setup` file exists).
 */
function sentinelCandidates(): string[] {
  return [PREFERRED_SPEECH_SENTINEL, ALT_SPEECH_SENTINEL];
}
function readFlagText(text: string): string {
  return text.replace(/\r\n/g, "\n").trim();
}

export function speechSetupSentinelPath(): string {
  // Return the first candidate that exists; otherwise return the preferred
  // location so callers can predict where the sentinel will be written.
  for (const p of sentinelCandidates()) {
    try {
      const st = Deno.lstatSync(p);
      if (st && st.isFile) return p;
    } catch {
      // ignore
    }
  }
  return PREFERRED_SPEECH_SENTINEL;
}

export async function isSpeechSetupComplete(): Promise<boolean> {
  try {
    // Check both candidate locations and return true if any contains the
    // expected version string.
    for (const p of sentinelCandidates()) {
      try {
        const text = await Deno.readTextFile(p);
        if (readFlagText(text) === SPEECH_SETUP_VERSION) return true;
      } catch (err) {
        // Treat NotFound and NotADirectory as benign — they simply indicate the
        // candidate isn't usable. Other errors are logged but don't abort.
        if (
          err instanceof Deno.errors.NotFound ||
          // Some platforms may raise NotADirectory when attempting to read a
          // file path under a path that's actually a file (e.g. repo 'setup').
          (err instanceof Error && typeof (err as unknown as Error).name === "string" && (err as unknown as Error).name === "NotADirectory")
        ) {
          continue;
        }
        console.warn(`Failed to read speech setup sentinel at ${p}: ${String(err)}`);
        continue;
      }
    }
    return false;
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
  const parentDir = dirname(PREFERRED_SPEECH_SENTINEL);
  try {
    // If something exists at the parent path and it's a file, ensureDir will
    // fail. Detect that situation and fall back to writing the sentinel at
    // the repository root to avoid crashing `psh setup` when the repo has a
    // top-level file named `setup` (common on some installs).
    const stat = await Deno.lstat(parentDir).catch(() => null);
    if (stat && stat.isFile) {
      const alt = ALT_SPEECH_SENTINEL;
      console.warn(
        `[psh] Expected '${parentDir}' to be a directory but found a file. ` +
        `Writing speech sentinel to '${alt}' instead.`,
      );
      await Deno.writeTextFile(alt, `${SPEECH_SETUP_VERSION}\n`);
    } else {
      await ensureDir(parentDir);
      await Deno.writeTextFile(PREFERRED_SPEECH_SENTINEL, `${SPEECH_SETUP_VERSION}\n`);
    }
  } catch (err) {
    // Re-throw unexpected errors so callers can surface them.
    throw err;
  }
  console.log("[psh] Speech stack assets ready.");
}
