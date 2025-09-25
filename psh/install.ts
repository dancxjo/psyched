// Installer functions for ROS2, Docker, etc.
import { repoPath, $ } from "./util.ts";

export async function runInstallRos2(): Promise<void> {
    const script = repoPath("../tools/install_ros2.sh");
    console.log(`Running ROS2 install script: ${script}`);
    const r = await $`bash ${script}`;
    if (r.code !== 0) {
        console.error(r.stderr || r.stdout);
        throw new Error(`ROS2 installer failed (${r.code})`);
    }
}

export async function runInstallDocker(): Promise<void> {
    const script = repoPath("../tools/install_docker.sh");
    console.log(`Running Docker install script: ${script}`);
    const r = await $`bash ${script}`;
    if (r.code !== 0) {
        console.error(r.stderr || r.stdout);
        throw new Error(`Docker installer failed (${r.code})`);
    }
}
