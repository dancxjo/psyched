import type { ComponentType } from "preact";

import type { Accent } from "@pilot/components/dashboard.tsx";
import { listModules, type ListModulesOptions } from "../server/modules.ts";
import { enabledServicesForHost } from "../server/host_config.ts";

import PilotOverviewIsland from "../../../pilot/islands/PilotOverviewIsland.tsx";
import ChatModulePanelIsland from "../../../../chat/pilot/islands/ChatModulePanelIsland.tsx";
import EarModulePanelIsland from "../../../../ear/pilot/islands/EarModulePanelIsland.tsx";
import ImuTelemetryIsland from "../../../../imu/pilot/islands/ImuTelemetryIsland.tsx";
import FacesModulePanelIsland from "../../../../faces/pilot/islands/FacesModulePanelIsland.tsx";
import FootControlPanelIsland from "../../../../foot/pilot/islands/FootControlPanelIsland.tsx";
import GpsModulePanelIsland from "../../../../gps/pilot/islands/GpsModulePanelIsland.tsx";
import KinectStreamPanelIsland from "../../../../eye/pilot/islands/KinectStreamPanelIsland.tsx";
import MemoryModulePanelIsland from "../../../../memory/pilot/islands/MemoryModulePanelIsland.tsx";
import NavModulePanelIsland from "../../../../nav/pilot/islands/NavModulePanelIsland.tsx";
import VoiceModulePanelIsland from "../../../../voice/pilot/islands/VoiceModulePanelIsland.tsx";
import VisceraModulePanelIsland from "../../../../viscera/pilot/islands/VisceraModulePanelIsland.tsx";
import WifiModulePanelIsland from "../../../../wifi/pilot/islands/WifiModulePanelIsland.tsx";
import WillModulePanelIsland from "../../../../will/pilot/islands/WillModulePanelIsland.tsx";
import AsrServicePanelIsland from "../../../../../services/asr/pilot/islands/AsrServicePanelIsland.tsx";
import GraphsServicePanelIsland from "../../../../../services/graphs/pilot/islands/GraphsServicePanelIsland.tsx";
import LlmServicePanelIsland from "../../../../../services/llm/pilot/islands/LlmServicePanelIsland.tsx";
import Ros2ServicePanelIsland from "../../../../../services/ros2/pilot/islands/Ros2ServicePanelIsland.tsx";
import TtsServicePanelIsland from "../../../../../services/tts/pilot/islands/TtsServicePanelIsland.tsx";
import VectorsServicePanelIsland from "../../../../../services/vectors/pilot/islands/VectorsServicePanelIsland.tsx";

export type TileKind = "module" | "service";

export interface DashboardTileDefinition {
  name: string;
  title: string;
  description: string;
  accent: Accent;
  kind: TileKind;
  href: string;
  overlay: ComponentType<unknown>;
  overlayProps?: Record<string, unknown>;
  ctaLabel?: string;
}

/** Tiles rendered for each ROS module managed by the cockpit. */
export const moduleTiles: ReadonlyArray<DashboardTileDefinition> = [
  {
    name: "pilot",
    title: "Pilot module",
    description:
      "Websocket bridge health, cockpit version, and operator pings.",
    accent: "amber",
    kind: "module",
    href: "/modules/pilot",
    overlay: PilotOverviewIsland,
    overlayProps: {
      version: "dev",
      description:
        "Home for the cockpit websocket bridge and Fresh frontend. Monitor connectivity and broadcast heartbeats.",
    },
  },
  {
    name: "chat",
    title: "Chat module",
    description: "Conversational agent orchestrating Pete's dialogue skills.",
    accent: "violet",
    kind: "module",
    href: "/modules/chat",
    overlay: ChatModulePanelIsland,
  },
  {
    name: "ear",
    title: "Ear module",
    description: "Microphone ingestion and speech recognition interface.",
    accent: "cyan",
    kind: "module",
    href: "/modules/ear",
    overlay: EarModulePanelIsland,
  },
  {
    name: "imu",
    title: "IMU module",
    description: "Orientation, acceleration, and angular velocity telemetry.",
    accent: "magenta",
    kind: "module",
    href: "/modules/imu",
    overlay: ImuTelemetryIsland,
  },
  {
    name: "faces",
    title: "Faces module",
    description: "Face detection pipeline for presence and identity cues.",
    accent: "magenta",
    kind: "module",
    href: "/modules/faces",
    overlay: FacesModulePanelIsland,
  },
  {
    name: "foot",
    title: "Foot module",
    description:
      "Create drive base command console with live telemetry overlays.",
    accent: "teal",
    kind: "module",
    href: "/modules/foot",
    overlay: FootControlPanelIsland,
  },
  {
    name: "gps",
    title: "GPS module",
    description: "GNSS receiver integration for Pete's localization stack.",
    accent: "amber",
    kind: "module",
    href: "/modules/gps",
    overlay: GpsModulePanelIsland,
  },
  {
    name: "eye",
    title: "Eye module",
    description: "Kinect RGB-D stream with payload diagnostics.",
    accent: "cyan",
    kind: "module",
    href: "/modules/eye",
    overlay: KinectStreamPanelIsland,
  },
  {
    name: "memory",
    title: "Memory module",
    description: "Semantic memory pipelines bridging vectors and graph stores.",
    accent: "teal",
    kind: "module",
    href: "/modules/memory",
    overlay: MemoryModulePanelIsland,
  },
  {
    name: "nav",
    title: "Nav module",
    description:
      "Navigation stack coordinating localization and path planning.",
    accent: "violet",
    kind: "module",
    href: "/modules/nav",
    overlay: NavModulePanelIsland,
  },
  {
    name: "voice",
    title: "Voice module",
    description: "Speech synthesis bridge coordinating the TTS service.",
    accent: "cyan",
    kind: "module",
    href: "/modules/voice",
    overlay: VoiceModulePanelIsland,
  },
  {
    name: "viscera",
    title: "Viscera module",
    description: "System health monitors and feelers for onboard diagnostics.",
    accent: "amber",
    kind: "module",
    href: "/modules/viscera",
    overlay: VisceraModulePanelIsland,
  },
  {
    name: "wifi",
    title: "Wi-Fi module",
    description: "Wireless connectivity tooling and diagnostics for Pete.",
    accent: "teal",
    kind: "module",
    href: "/modules/wifi",
    overlay: WifiModulePanelIsland,
  },
  {
    name: "will",
    title: "Will module",
    description: "High-level intent planner orchestrating Pete's behaviours.",
    accent: "magenta",
    kind: "module",
    href: "/modules/will",
    overlay: WillModulePanelIsland,
  },
] as const;

/** Tiles rendered for dockerised services bootstrapped through psh. */
export const serviceTiles: ReadonlyArray<DashboardTileDefinition> = [
  {
    name: "asr",
    title: "ASR service",
    description: "Streaming Whisper server for voice transcription.",
    accent: "teal",
    kind: "service",
    href: "/psh/srv",
    overlay: AsrServicePanelIsland,
    ctaLabel: "Manage ASR",
  },
  {
    name: "graphs",
    title: "Graphs service",
    description: "Neo4j + Memgraph orchestration for semantic maps.",
    accent: "magenta",
    kind: "service",
    href: "/psh/srv",
    overlay: GraphsServicePanelIsland,
    ctaLabel: "Manage graphs",
  },
  {
    name: "llm",
    title: "LLM service",
    description: "Language model runtime powering conversational skills.",
    accent: "violet",
    kind: "service",
    href: "/psh/srv",
    overlay: LlmServicePanelIsland,
    ctaLabel: "Manage LLM",
  },
  {
    name: "ros2",
    title: "ROS 2 bridge",
    description: "Dockerised ROS 2 workspace helpers and tooling.",
    accent: "amber",
    kind: "service",
    href: "/psh/srv",
    overlay: Ros2ServicePanelIsland,
    ctaLabel: "Manage ROS 2",
  },
  {
    name: "tts",
    title: "TTS service",
    description: "Text-to-speech stack for the voice module.",
    accent: "cyan",
    kind: "service",
    href: "/psh/srv",
    overlay: TtsServicePanelIsland,
    ctaLabel: "Manage TTS",
  },
  {
    name: "vectors",
    title: "Vector store",
    description: "Embedding database for semantic memory and retrieval.",
    accent: "teal",
    kind: "service",
    href: "/psh/srv",
    overlay: VectorsServicePanelIsland,
    ctaLabel: "Manage vectors",
  },
] as const;

export const dashboardTiles: ReadonlyArray<DashboardTileDefinition> = [
  ...moduleTiles,
  ...serviceTiles,
];

export type ModuleTileFilterOptions = ListModulesOptions;

export interface ServiceTileFilterOptions {
  hostname?: string;
  hostsDir?: string;
}

export function moduleTilesForHost(
  options: ModuleTileFilterOptions = {},
): DashboardTileDefinition[] {
  const enabled = new Set(listModules(options));
  if (enabled.size === 0) {
    return [];
  }
  return moduleTiles.filter((tile) => enabled.has(tile.name));
}

export function serviceTilesForHost(
  options: ServiceTileFilterOptions = {},
): DashboardTileDefinition[] {
  const { services } = enabledServicesForHost({
    hostname: options.hostname,
    hostsDir: options.hostsDir,
  });
  if (services.length === 0) {
    return [];
  }
  const enabled = new Set(services);
  return serviceTiles.filter((tile) => enabled.has(tile.name));
}

export function dashboardTilesForHost(
  options: ModuleTileFilterOptions = {},
): DashboardTileDefinition[] {
  const serviceOptions: ServiceTileFilterOptions = {
    hostname: options.hostname,
    hostsDir: options.hostsDir,
  };
  return [
    ...moduleTilesForHost(options),
    ...serviceTilesForHost(serviceOptions),
  ];
}
