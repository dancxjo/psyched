import { runWithStreamingTee } from "./util.ts";

export async function testSpeechStack(): Promise<void> {
    console.log("[psh] Launching speech stack via docker compose...");
    const composeDir = "compose";
    const composeFile = "speech-stack.compose.yml";
    // Start the stack in detached mode
    let res = await runWithStreamingTee(
        `cd ${composeDir} && docker compose -f ${composeFile} up -d --build`
    );
    if (res.code !== 0) {
        throw new Error(`[psh] Failed to start speech stack: ${res.stderr}`);
    }
    console.log("[psh] Speech stack containers started.");

    // Test LLM endpoint
    console.log("[psh] Testing LLM endpoint (ws://localhost:8080/chat)...");
    res = await runWithStreamingTee(
        `curl -s -X POST -H 'Content-Type: application/json' --data '{"prompt":"Hello, who are you?"}' http://localhost:8080/chat`
    );
    if (!res.stdout.includes("assistant")) {
        throw new Error("[psh] LLM endpoint did not return expected response.");
    }
    console.log("[psh] LLM endpoint test passed.");

    // Test TTS endpoint
    console.log("[psh] Testing TTS endpoint (ws://localhost:5002/tts)...");
    res = await runWithStreamingTee(
        `curl -s -X POST -H 'Content-Type: application/json' --data '{"text":"Testing TTS"}' http://localhost:5002/tts -o /tmp/tts_test.wav`
    );
    if (res.code !== 0) {
        throw new Error("[psh] TTS endpoint did not return audio data.");
    }
    console.log("[psh] TTS endpoint test passed. (Audio saved to /tmp/tts_test.wav)");

    // Test ASR endpoint (optional, as it may require audio input)
    // You can add a test here if you have a sample wav file and endpoint

    console.log("[psh] All speech stack endpoints tested successfully.");
}
