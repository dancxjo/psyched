import { ensureDir } from "@std/fs";
import { dirname } from "@std/path";
import { downloadSpeechModels } from "./download_models.ts";
import { repoPath, $, runWithStreamingTee } from "./util.ts";
import { join } from "@std/path";

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

  // Additional forebrain-specific Ollama provisioning:
  // If this host appears to be the forebrain we ensure Ollama is allowed
  // to serve on the LAN (bind to 0.0.0.0) and pull the gpt-oss:20b model so
  // it is available offline.
  try {
    // Determine short hostname (similar to psh/setup.ts behaviour).
    let hostName = Deno.env.get("HOST") || "";
    if (!hostName) {
      try {
        const hn = await $`hostname -s`.stdout("piped").stderr("piped");
        hostName = (hn.stdout || hn.stderr || "").toString().trim();
      } catch {
        hostName = "";
      }
    }

    if (hostName === "forebrain") {
      console.log('[psh] Detected forebrain host — ensuring Ollama is configured for LAN access and pulling model.');

      const home = Deno.env.get("HOME") || "/root";
      const cfgPaths = [
        join(home, ".config", "ollama", "config.json"),
        join(home, ".ollama", "config.json"),
      ];

      // If `ollama` exists on PATH, patch or write config and pull model.
      const whichRes = await $`which ollama`.stdout("piped").stderr("piped").noThrow();
      const whichOut = (whichRes.stdout || whichRes.stderr || "").toString().trim();
      if (!whichOut) {
        console.log('[psh] ollama binary not found on PATH — skipping Ollama provisioning.');
      } else {
        for (const p of cfgPaths) {
          try {
            // Read existing config if present, otherwise create directory and write minimal config.
            let existing: Record<string, unknown> | null = null;
            try {
              const txt = await Deno.readTextFile(p);
              existing = JSON.parse(txt) as Record<string, unknown>;
            } catch {
              existing = null;
            }
            const dir = dirname(p);
            try {
              await ensureDir(dir);
            } catch {
              // ignore
            }

            const merged = existing || {};
            // Ensure merged.http.host = "0.0.0.0"
            if (!merged["http"] || typeof merged["http"] !== "object") {
              // @ts-ignore: merged has unknown index signature in this context
              merged["http"] = { host: "0.0.0.0" };
            } else {
              // @ts-ignore: nested http object may be unknown-typed
              merged["http"]["host"] = "0.0.0.0";
            }

            try {
              await Deno.writeTextFile(p, JSON.stringify(merged, null, 2) + "\n");
              console.log(`[psh] Wrote Ollama config to ${p}`);
            } catch (err) {
              console.warn(`[psh] Failed to write Ollama config to ${p}: ${String(err)}`);
            }
          } catch (err) {
            console.warn(`[psh] Error while preparing Ollama config ${p}: ${String(err)}`);
          }
        }

        // Attempt to pull the requested model. Use the streaming helper so output
        // is visible to the operator.
        try {
          console.log('[psh] Pulling Ollama model gpt-oss:20b (this may take a while)...');
          const res = await runWithStreamingTee("ollama pull gpt-oss:20b");
          if (res.code !== 0) {
            console.warn(`[psh] ollama pull failed (code ${res.code}).`);
            if (res.stderr) console.warn(res.stderr);
          } else {
            console.log('[psh] Successfully pulled gpt-oss:20b.');
          }
        } catch (err) {
          console.warn(`[psh] Failed to run 'ollama pull': ${String(err)}`);
        }
      }
    }
  } catch (err) {
    console.warn(`[psh] Forebrain Ollama provisioning encountered an error: ${String(err)}`);
  }
}
