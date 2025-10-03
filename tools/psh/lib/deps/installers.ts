import { ProvisionContext, ProvisionContextOptions } from "./context.ts";
import { installCuda } from "./cuda.ts";
import { installDeno } from "./deno.ts";
import { installDocker } from "./docker.ts";
import { installRos2 } from "./ros2.ts";

export interface InstallerDefinition {
  id: string;
  label: string;
  description: string;
  run(context: ProvisionContext): Promise<void>;
}

const REGISTRY: Record<string, InstallerDefinition> = {
  deno: {
    id: "deno",
    label: "Deno runtime",
    description: "Installs the Deno CLI used by psh and module tooling.",
    run: installDeno,
  },
  ros2: {
    id: "ros2",
    label: "ROS 2 base",
    description:
      "Configures the ROS 2 apt repository and installs ros-base packages.",
    run: installRos2,
  },
  docker: {
    id: "docker",
    label: "Docker Engine",
    description:
      "Installs Docker CE and docker compose plugin from Docker's apt repo.",
    run: installDocker,
  },
  cuda: {
    id: "cuda",
    label: "CUDA Toolkit",
    description: "Installs NVIDIA CUDA drivers and toolkit packages.",
    run: installCuda,
  },
};

export function listInstallers(): InstallerDefinition[] {
  return Object.values(REGISTRY);
}

export function getInstaller(id: string): InstallerDefinition | undefined {
  return REGISTRY[id];
}

export async function runInstaller(
  id: string,
  options: ProvisionContextOptions,
): Promise<void> {
  const installer = getInstaller(id);
  if (!installer) {
    throw new Error(`Unknown installer '${id}'`);
  }
  const context = new ProvisionContext(options);
  await installer.run(context);
}
