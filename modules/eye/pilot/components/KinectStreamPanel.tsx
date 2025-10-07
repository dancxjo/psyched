import { useMemo } from "preact/hooks";
import { type ConnectionStatus, useCockpitTopic } from "@pilot/lib/cockpit.ts";

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

const STATUS_LABELS: Record<ConnectionStatus, string> = {
  idle: "Idle",
  connecting: "Connecting",
  open: "Connected",
  closed: "Disconnected",
  error: "Error",
};

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

function formatTimestamp(value?: string) {
  if (!value) return "—";
  const date = new Date(value);
  if (Number.isNaN(date.getTime())) return value;
  return date.toLocaleString();
}

function formatBytes(bytes?: number) {
  if (typeof bytes !== "number" || !Number.isFinite(bytes) || bytes <= 0) {
    return "—";
  }
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KiB`;
  return `${(bytes / (1024 * 1024)).toFixed(1)} MiB`;
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

  return (
    <section class="kinect-stream">
      <header class="kinect-stream__header">
        <h2>Kinect Eye</h2>
        <ul class="kinect-stream__status">
          <li>
            RGB: {STATUS_LABELS[color.status] ?? "Unknown"}
          </li>
          <li>
            Depth: {STATUS_LABELS[depth.status] ?? "Unknown"}
          </li>
        </ul>
      </header>

      <div class="kinect-stream__grid">
        <figure class="kinect-stream__frame">
          <header>
            <h3>RGB</h3>
            <dl>
              <div>
                <dt>Frame</dt>
                <dd>{color.data?.frame_id ?? "—"}</dd>
              </div>
              <div>
                <dt>Size</dt>
                <dd>
                  {color.data?.width ?? 0}×{color.data?.height ?? 0} ·{" "}
                  {color.data?.encoding ?? "unknown"}
                </dd>
              </div>
              <div>
                <dt>Payload</dt>
                <dd>{formatBytes(color.data?.size_bytes)}</dd>
              </div>
              <div>
                <dt>Updated</dt>
                <dd>
                  {formatTimestamp(
                    color.data?.timestamp ?? color.data?.received_at,
                  )}
                </dd>
              </div>
            </dl>
          </header>
          <div class="kinect-stream__image">
            {colorUrl
              ? <img alt="Kinect RGB" src={colorUrl} />
              : (
                <p class="kinect-stream__placeholder">
                  Waiting for RGB frames…
                </p>
              )}
          </div>
        </figure>

        <figure class="kinect-stream__frame">
          <header>
            <h3>Depth</h3>
            <dl>
              <div>
                <dt>Frame</dt>
                <dd>{depth.data?.frame_id ?? "—"}</dd>
              </div>
              <div>
                <dt>Size</dt>
                <dd>
                  {depth.data?.width ?? 0}×{depth.data?.height ?? 0} ·{" "}
                  {depth.data?.encoding ?? "unknown"}
                </dd>
              </div>
              <div>
                <dt>Payload</dt>
                <dd>{formatBytes(depth.data?.size_bytes)}</dd>
              </div>
              <div>
                <dt>Statistics</dt>
                <dd>{formatStatistics(depth.data?.statistics)}</dd>
              </div>
              <div>
                <dt>Updated</dt>
                <dd>
                  {formatTimestamp(
                    depth.data?.timestamp ?? depth.data?.received_at,
                  )}
                </dd>
              </div>
            </dl>
          </header>
          <div class="kinect-stream__image">
            {depthUrl
              ? <img alt="Kinect depth" src={depthUrl} />
              : (
                <p class="kinect-stream__placeholder">
                  Waiting for depth frames…
                </p>
              )}
          </div>
        </figure>
      </div>
    </section>
  );
}
