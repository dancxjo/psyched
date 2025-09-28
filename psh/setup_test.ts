import { assertEquals } from "@std/assert";
import { repoPath } from "./util.ts";
import { setupHosts, type SetupDeps } from "./setup.ts";

function cleanupFiles(paths: string[]) {
  for (const path of paths) {
    try {
      Deno.removeSync(path);
    } catch {
      // ignore
    }
  }
}

Deno.test("setupHosts triggers speech setup when requested", async () => {
  const hostName = `psh-test-${crypto.randomUUID()}`;
  const hostFile = repoPath(`hosts/${hostName}.toml`);
  const created: string[] = [];
  try {
    await Deno.writeTextFile(hostFile, "setup_speech = true\nmodules = []\n");
    created.push(hostFile);

    let speechChecks = 0;
    let speechRuns = 0;
    let rosRuns = 0;
    let dockerRuns = 0;
    const overrides: Partial<SetupDeps> = {
      isRos2Installed: async () => true,
      runInstallRos2: async () => {
        rosRuns++;
      },
      isDockerInstalled: async () => true,
      runInstallDocker: async () => {
        dockerRuns++;
      },
      isSpeechSetupComplete: async () => {
        speechChecks++;
        return false;
      },
      runSetupSpeech: async () => {
        speechRuns++;
      },
    };

    await setupHosts([hostName], overrides);
    assertEquals(speechChecks, 1);
    assertEquals(speechRuns, 1);
    assertEquals(rosRuns, 0);
    assertEquals(dockerRuns, 0);
  } finally {
    cleanupFiles(created);
  }
});

Deno.test("speech setup is skipped when models already prepared", async () => {
  const hostName = `psh-test-${crypto.randomUUID()}`;
  const hostFile = repoPath(`hosts/${hostName}.toml`);
  const sentinel = repoPath("setup/.speech_setup_complete");
  const created: string[] = [];
  try {
    await Deno.writeTextFile(hostFile, "\"setup-speech\" = true\nmodules = []\n");
    created.push(hostFile);

    let speechChecks = 0;
    let speechRuns = 0;
    const overrides: Partial<SetupDeps> = {
      isRos2Installed: async () => true,
      runInstallRos2: async () => {},
      isDockerInstalled: async () => true,
      runInstallDocker: async () => {},
      isSpeechSetupComplete: async () => {
        speechChecks++;
        return true;
      },
      runSetupSpeech: async () => {
        speechRuns++;
      },
    };

    await setupHosts([hostName], overrides);
    assertEquals(speechChecks, 1);
    assertEquals(speechRuns, 0);
  } finally {
    cleanupFiles(created);
    cleanupFiles([sentinel]);
  }
});
