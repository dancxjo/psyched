import { repoPath, runWithStreamingTee } from "./util.ts";

export async function downloadSpeechModels(): Promise<void> {
    const script = repoPath("psh/scripts/download_speech_models.sh");
    console.log(`[psh] Downloading speech models using ${script}`);
    const res = await runWithStreamingTee(`bash ${script}`);
    if (res.code !== 0) {
        console.error(`[psh] Model download script failed (code ${res.code})`);
        if (res.stderr) console.error(res.stderr);
        throw new Error(`Model download script failed (${res.code})`);
    }
    if (res.stderr && res.stderr.trim().length > 0) {
        console.log("[psh] Model download script produced stderr (see below):");
        console.log(res.stderr);
    }
    console.log("[psh] Model download complete.");
}
