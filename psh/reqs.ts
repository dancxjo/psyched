export interface Requirement {
    name: string;
    check(): Promise<boolean>;
    install(): Promise<void>;
}

import $ from "@david/dax";

export class AptRequirement implements Requirement {
    constructor(readonly name: string, readonly packageName: string) { }

    async check(): Promise<boolean> {
        // dpkg -s returns 0 when the package is installed
        const r = await $`dpkg -s ${this.packageName} >/dev/null 2>&1`;
        return r.code === 0;
    }

    async install(): Promise<void> {
        console.log(`Installing ${this.packageName} via apt...`);
        // Use apt-get for scripting; combine update+install for simplicity.
        const r = await $`sudo apt-get update && sudo apt-get install -y ${this.packageName}`;
        if (r.code !== 0) {
            throw new Error(`Failed to install ${this.packageName} via apt: ${r.stderr || r.stdout}`);
        }
        console.log(`${this.packageName} installed successfully.`);
    }
}

export class MultipleAptRequirement implements Requirement {
    constructor(readonly name: string, readonly packages: string[]) { }

    async check(): Promise<boolean> {
        for (const pkg of this.packages) {
            const r = await $`dpkg -s ${pkg} >/dev/null 2>&1`;
            if (r.code !== 0) return false;
        }
        return true;
    }

    async install(): Promise<void> {
        console.log(`Installing packages via apt: ${this.packages.join(', ')}`);
        const r = await $`sudo apt-get update && sudo apt-get install -y ${this.packages.join(' ')}`;
        if (r.code !== 0) {
            throw new Error(`Failed to install packages via apt: ${r.stderr || r.stdout}`);
        }
        console.log(`Packages installed: ${this.packages.join(', ')}`);
    }
}

export class CommandRequirement implements Requirement {
    constructor(readonly name: string, readonly command: string) { }

    async check(): Promise<boolean> {
        const r = await $`command -v ${this.command} >/dev/null 2>&1`;
        return r.code === 0;
    }

    install(): Promise<void> {
        return Promise.reject(new Error(`Automatic install not available for command '${this.command}'. Please install it manually.`));
    }
}

export class DebUrlRequirement implements Requirement {
    constructor(readonly name: string, readonly url: string, readonly targetPath = `/tmp/${Date.now()}-package.deb`) { }

    check(): Promise<boolean> {
        // Hard to determine; conservative: not installed.
        return Promise.resolve(false);
    }

    async install(): Promise<void> {
        console.log(`Downloading ${this.url} -> ${this.targetPath}`);
        const dl = await $`curl -fsSL -o ${this.targetPath} ${this.url}`;
        if (dl.code !== 0) {
            throw new Error(`Failed to download ${this.url}: ${dl.stderr || dl.stdout}`);
        }
        const dpkg = await $`sudo dpkg -i ${this.targetPath}`;
        if (dpkg.code !== 0) {
            throw new Error(`Failed to install deb ${this.targetPath}: ${dpkg.stderr || dpkg.stdout}`);
        }
        console.log(`Installed deb from ${this.url}`);
    }
}

export class FileRequirement implements Requirement {
    constructor(readonly name: string, readonly path: string) { }

    async check(): Promise<boolean> {
        const r = await $`test -e ${this.path} >/dev/null 2>&1`;
        return r.code === 0;
    }

    install(): Promise<void> {
        return Promise.reject(new Error(`No automated install for file requirement: ${this.path}`));
    }
}
