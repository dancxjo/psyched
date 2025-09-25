// psh_env.ts
// Prints shell code to source ROS2 and workspace setup scripts for use with 'source $(psh env)'

export function printEnvSource() {
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
    // Print shell code for sourcing both
    console.log(`source ${foundRos} && source ${wsSetup}`);
}
