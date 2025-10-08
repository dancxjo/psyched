import type { ComponentType } from "preact";

import type { Accent } from "@pilot/components/dashboard.tsx";

import PilotOverviewIsland from "../../../pilot/islands/PilotOverviewIsland.tsx";
import ImuTelemetryIsland from "../../../../imu/pilot/islands/ImuTelemetryIsland.tsx";
import FootControlPanelIsland from "../../../../foot/pilot/islands/FootControlPanelIsland.tsx";
import KinectStreamPanelIsland from "../../../../eye/pilot/islands/KinectStreamPanelIsland.tsx";
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
    name: "imu",
    title: "IMU module",
    description: "Orientation, acceleration, and angular velocity telemetry.",
    accent: "magenta",
    kind: "module",
    href: "/modules/imu",
    overlay: ImuTelemetryIsland,
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
    name: "eye",
    title: "Eye module",
    description: "Kinect RGB-D stream with payload diagnostics.",
    accent: "cyan",
    kind: "module",
    href: "/modules/eye",
    overlay: KinectStreamPanelIsland,
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
