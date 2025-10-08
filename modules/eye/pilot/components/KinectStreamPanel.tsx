import { useMemo } from "preact/hooks";
import { useCockpitTopic } from "@pilot/lib/cockpit.ts";

import {
  CONNECTION_STATUS_LABELS,
  LcarsCard,
  LcarsPanel,
  toneFromConnection,
} from "../../../pilot/frontend/components/lcars.tsx";
import {
  formatBytes,
  formatRelativeTime,
} from "../../../pilot/frontend/lib/format.ts";

export type ImageStatistics = {
  mean?: number;
  min?: number;
  max?: number;
};

export type EncodedImageMessage = {
  data?: string;
  encoding?: string;
  format?: string;
  frame_id?: string;
  height?: number;
  is_depth?: boolean;
  received_at?: string;
  size_bytes?: number;
  statistics?: ImageStatistics;
  step?: number;
  timestamp?: string;
  width?: number;
};

export interface KinectStreamPanelProps {
  colorTopic?: string;
  depthTopic?: string;
}

const DEFAULT_IMAGE: EncodedImageMessage = {
  encoding: "rgb8",
  width: 0,
  height: 0,
};

const DEFAULT_DEPTH: EncodedImageMessage = {
  encoding: "16UC1",
  width: 0,
  height: 0,
  is_depth: true,
};

function toDataUrl(message: EncodedImageMessage | undefined) {
  if (!message?.data || !message?.format) {
    return null;
  }
  const mime = message.format === "jpeg"
    ? "image/jpeg"
    : message.format === "png"
    ? "image/png"
    : "application/octet-stream";
  return `data:${mime};base64,${message.data}`;
}

function formatStatistics(stats?: ImageStatistics) {
  if (!stats) return "—";
  const items: string[] = [];
  if (typeof stats.min === "number" && Number.isFinite(stats.min)) {
    items.push(`min ${stats.min.toFixed(2)}`);
  }
  if (typeof stats.mean === "number" && Number.isFinite(stats.mean)) {
    items.push(`mean ${stats.mean.toFixed(2)}`);
  }
  if (typeof stats.max === "number" && Number.isFinite(stats.max)) {
    items.push(`max ${stats.max.toFixed(2)}`);
  }
  return items.length > 0 ? items.join(", ") : "—";
}

function parseTimestamp(value?: string) {
  if (!value) return undefined;
  const parsed = Date.parse(value);
  return Number.isNaN(parsed) ? undefined : parsed;
}

export default function KinectStreamPanel({
  colorTopic = "/image_raw",
  depthTopic = "/camera/depth/image_raw",
}: KinectStreamPanelProps) {
  const color = useCockpitTopic<EncodedImageMessage>(colorTopic, {
    replay: true,
    initialValue: DEFAULT_IMAGE,
  });
  const depth = useCockpitTopic<EncodedImageMessage>(depthTopic, {
    replay: true,
    initialValue: DEFAULT_DEPTH,
  });

  const colorUrl = useMemo(() => toDataUrl(color.data), [
    color.data?.data,
    color.data?.format,
  ]);
  const depthUrl = useMemo(() => toDataUrl(depth.data), [
    depth.data?.data,
    depth.data?.format,
  ]);

  const colorUpdated = formatRelativeTime(parseTimestamp(
    color.data?.timestamp ?? color.data?.received_at,
  ));
  const depthUpdated = formatRelativeTime(parseTimestamp(
    depth.data?.timestamp ?? depth.data?.received_at,
  ));

  return (
    <LcarsPanel
      title="Kinect Eye"
      subtitle="RGB-D telemetry stream"
      accent="cyan"
      badges={[
        {
          label: `RGB ${CONNECTION_STATUS_LABELS[color.status] ?? "Unknown"}`,
          tone: toneFromConnection(color.status),
        },
        {
          label: `Depth ${CONNECTION_STATUS_LABELS[depth.status] ?? "Unknown"}`,
          tone: toneFromConnection(depth.status),
        },
      ]}
    >
      <div class="lcars-grid lcars-grid--stretch">
        <LcarsCard title="RGB stream" tone="cyan">
          <dl class="lcars-list">
            <div class="lcars-list__item">
              <dt>Frame</dt>
              <dd>{color.data?.frame_id ?? "—"}</dd>
            </div>
            <div class="lcars-list__item">
              <dt>Resolution</dt>
              <dd>
                {color.data?.width ?? 0}×{color.data?.height ?? 0} ·{" "}
                {color.data?.encoding ?? "unknown"}
              </dd>
            </div>
            <div class="lcars-list__item">
              <dt>Payload</dt>
              <dd>{formatBytes(color.data?.size_bytes)}</dd>
            </div>
            <div class="lcars-list__item">
              <dt>Updated</dt>
              <dd>{colorUpdated}</dd>
            </div>
          </dl>
          <div class="lcars-frame">
            {colorUrl
              ? <img alt="Kinect RGB" src={colorUrl} />
              : <p class="lcars-placeholder">Waiting for RGB frames…</p>}
          </div>
        </LcarsCard>

        <LcarsCard title="Depth stream" tone="violet">
          <dl class="lcars-list">
            <div class="lcars-list__item">
              <dt>Frame</dt>
              <dd>{depth.data?.frame_id ?? "—"}</dd>
            </div>
            <div class="lcars-list__item">
              <dt>Resolution</dt>
              <dd>
                {depth.data?.width ?? 0}×{depth.data?.height ?? 0} ·{" "}
                {depth.data?.encoding ?? "unknown"}
              </dd>
            </div>
            <div class="lcars-list__item">
              <dt>Payload</dt>
              <dd>{formatBytes(depth.data?.size_bytes)}</dd>
            </div>
            <div class="lcars-list__item">
              <dt>Statistics</dt>
              <dd>{formatStatistics(depth.data?.statistics)}</dd>
            </div>
            <div class="lcars-list__item">
              <dt>Updated</dt>
              <dd>{depthUpdated}</dd>
            </div>
          </dl>
          <div class="lcars-frame">
            {depthUrl
              ? <img alt="Kinect depth" src={depthUrl} />
              : <p class="lcars-placeholder">Waiting for depth frames…</p>}
          </div>
        </LcarsCard>
      </div>
    </LcarsPanel>
  );
}
