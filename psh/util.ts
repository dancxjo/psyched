// Shared utilities for psh
// Usage: import { repoPath, $ } from "./util.ts";

// @ts-ignore runtime import via jsr
import $ from "@david/dax";

export function repoPath(relative: string): string {
    // Resolve a path relative to this file (psh/...). Example: '../tools/install_ros2.sh'
    return decodeURIComponent(new URL(relative, import.meta.url).pathname);
}

export { $ };
