// uninstallPsh: Remove /usr/bin/psh
import { $ } from "./util.ts";

export async function uninstallPsh(): Promise<void> {
    console.log('Uninstalling /usr/bin/psh (requires sudo)...');
    const exists = await $`sudo test -f /usr/bin/psh`;
    if (exists.code !== 0) {
        console.log('/usr/bin/psh does not exist; nothing to do.');
        return;
    }
    const rm = await $`sudo rm -f /usr/bin/psh`;
    if (rm.code !== 0) {
        console.error('Failed to remove /usr/bin/psh:', rm.stderr || rm.stdout);
        throw new Error('Failed to remove /usr/bin/psh');
    }
    console.log('Removed /usr/bin/psh');
}
