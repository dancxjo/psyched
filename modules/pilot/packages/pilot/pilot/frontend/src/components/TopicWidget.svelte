<script>
  import JoystickControl from './JoystickControl.svelte';
  import ImuPanel from './ImuPanel.svelte';

  export let record;
  export let topic;

  const MAX_PREVIEW = 1200;

  function formatNumber(value, digits = 2) {
    return typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : '--';
  }

  function describeImage(data) {
    if (!data) return 'Awaiting frame…';
    const width = data.width ?? data.info?.width;
    const height = data.height ?? data.info?.height;
    const encoding = data.encoding ?? data.info?.encoding ?? 'unknown';
    const step = data.step ?? data.info?.step;
    return `Resolution: ${width} × ${height}\nEncoding: ${encoding}\nStep: ${step ?? '--'}`;
  }

  function describeMap(data) {
    if (!data) return 'Awaiting occupancy grid…';
    const width = data.info?.width ?? data.width;
    const height = data.info?.height ?? data.height;
    const resolution = data.info?.resolution;
    return `Size: ${width} × ${height}\nResolution: ${formatNumber(resolution, 3)} m/px`;
  }

  function describePath(data) {
    if (!data) return 'Awaiting path…';
    const poses = data.poses ?? [];
    return `Waypoints: ${poses.length}`;
  }

  function describePose(data) {
    if (!data) return 'Awaiting pose…';
    const position = data.pose?.pose?.position ?? data.pose?.position ?? data.position ?? {};
    const orientation = data.pose?.pose?.orientation ?? data.pose?.orientation ?? data.orientation ?? {};
    return `Position: (${formatNumber(position.x)}, ${formatNumber(position.y)}, ${formatNumber(position.z)})\nOrientation: (${formatNumber(orientation.x)}, ${formatNumber(orientation.y)}, ${formatNumber(orientation.z)}, ${formatNumber(orientation.w)})`;
  }

  function describeVector(data) {
    if (!data) return 'Awaiting twist…';
    const linear = data.twist?.twist?.linear ?? data.twist?.linear ?? data.linear ?? {};
    const angular = data.twist?.twist?.angular ?? data.twist?.angular ?? data.angular ?? {};
    return `Linear: (${formatNumber(linear.x)}, ${formatNumber(linear.y)}, ${formatNumber(linear.z)})\nAngular: (${formatNumber(angular.x)}, ${formatNumber(angular.y)}, ${formatNumber(angular.z)})`;
  }

  function describeText(data) {
    if (data == null) return 'Awaiting message…';
    if (typeof data === 'string') return data;
    if (typeof data.data === 'string') return data.data;
    return JSON.stringify(data, null, 2);
  }

  function describeGeneric(data) {
    if (data == null) return 'Awaiting data…';
    const text = typeof data === 'string' ? data : JSON.stringify(data, null, 2);
    return text.length > MAX_PREVIEW ? `${text.slice(0, MAX_PREVIEW)}…` : text;
  }

  $: status = record ? record.state : 'idle';
  $: display = (() => {
    if (!record) return null;
    const presentation = topic.presentation || '';
    if (presentation === 'image' || presentation === 'depth') return describeImage(record.last);
    if (presentation === 'map') return describeMap(record.last);
    if (presentation === 'path') return describePath(record.last);
    if (presentation === 'pose') return describePose(record.last);
    if (presentation === 'vector') return describeVector(record.last);
    if (presentation === 'status' || presentation === 'text' || presentation === 'log') return describeText(record.last);
    return describeGeneric(record.last);
  })();
</script>

<div class="topic-widget">
  <div class="topic-state">
    <span class={`state ${status}`}>{status}</span>
    {#if record?.paused}
      <span class="paused">Paused</span>
    {/if}
    {#if record?.error}
      <span class="error">{record.error}</span>
    {/if}
  </div>

  {#if record}
    {#if topic.presentation === 'joystick'}
      <JoystickControl record={record} />
    {:else if topic.presentation === 'imu'}
      <ImuPanel data={record.last} />
    {:else}
      <pre class="topic-payload">{display}</pre>
    {/if}
  {:else}
    <div class="inactive">Not subscribed</div>
  {/if}
</div>
