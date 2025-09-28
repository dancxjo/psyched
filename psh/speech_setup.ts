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
  await ensureDir(dirname(SPEECH_SENTINEL_PATH));
  await Deno.writeTextFile(SPEECH_SENTINEL_PATH, `${SPEECH_SETUP_VERSION}\n`);
  console.log("[psh] Speech stack assets ready.");
}
