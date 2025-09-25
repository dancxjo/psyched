// psh_env.ts
// Prints shell code to source ROS2 and workspace setup scripts for use with 'source $(psh env)'

export async function printEnvSource() {
    // Find installed ROS2 setup script (prefer kilted, fallback to any)
    const rosSetup = [
        "/opt/ros/kilted/setup.bash",
        "/opt/ros/humble/setup.bash",
        "/opt/ros/foxy/setup.bash"
    ];
    let foundRos = "";
    for (const path of rosSetup) {
        try {
            Deno.statSync(path);
            foundRos = path;
            break;
        } catch { /* ignore missing file */ }
    }
    if (!foundRos) {
        // Fallback: try glob (sync version)
        try {
            const rosDir = "/opt/ros/";
            for (const dirEntry of Deno.readDirSync(rosDir)) {
                const candidate = `${rosDir}${dirEntry.name}/setup.bash`;
                try {
                    Deno.statSync(candidate);
                    foundRos = candidate;
                    break;
                } catch { /* ignore missing file */ }
            }
        } catch { /* ignore missing dir */ }
    }
    if (!foundRos) {
        console.error("No ROS2 setup.bash found in /opt/ros/*");
        Deno.exit(1);
    }
    // Workspace install setup
    const user = Deno.env.get("USER") ?? "$(whoami)";
    const wsSetup = `/home/${user}/psyched/install/setup.bash`;

    // Add function to ~/.bashrc
    const bashrcPath = `${Deno.env.get("HOME")}/.bashrc`;
    const funcDef = `psyched_env() {\n    source ${foundRos} && source ${wsSetup}\n}`;
    let bashrcContent = "";
    try {
        bashrcContent = await Deno.readTextFile(bashrcPath);
    } catch (_) {
        bashrcContent = "";
    }
    if (!bashrcContent.includes("psyched_env()")) {
        bashrcContent += (bashrcContent.endsWith("\n") ? "" : "\n") + funcDef + "\n" + "psyched_env\n";
        await Deno.writeTextFile(bashrcPath, bashrcContent);
        console.log("Added 'psyched_env' function to ~/.bashrc");
    } else {
        console.log("'psyched_env' function already present in ~/.bashrc");
    }
    // Print instructions
    console.log(`\nTo activate the environment, run:\n\nsource ~/.bashrc\npsyched_env\n`);
}
