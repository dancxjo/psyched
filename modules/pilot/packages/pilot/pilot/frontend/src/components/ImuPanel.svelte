<script>
  export let data;

  function format(value, digits = 2) {
    return typeof value === 'number' && Number.isFinite(value) ? value.toFixed(digits) : '--';
  }

  const empty = {
    acceleration: { x: '--', y: '--', z: '--' },
    angular: { x: '--', y: '--', z: '--' },
    orientation: { x: '--', y: '--', z: '--', w: '--' },
  };

  $: summary = (() => {
    if (!data) return empty;
    const linear = data.linear_acceleration ?? data.linear ?? data.acceleration ?? {};
    const angular = data.angular_velocity ?? data.angular ?? {};
    const orientation = data.orientation ?? {};
    return {
      acceleration: {
        x: format(linear.x),
        y: format(linear.y),
        z: format(linear.z),
      },
      angular: {
        x: format(angular.x),
        y: format(angular.y),
        z: format(angular.z),
      },
      orientation: {
        x: format(orientation.x),
        y: format(orientation.y),
        z: format(orientation.z),
        w: format(orientation.w),
      },
    };
  })();
</script>

<div class="imu-panel">
  <div>
    <h5>Linear Acceleration (m/s²)</h5>
    <ul>
      <li>X: {summary.acceleration.x}</li>
      <li>Y: {summary.acceleration.y}</li>
      <li>Z: {summary.acceleration.z}</li>
    </ul>
  </div>
  <div>
    <h5>Angular Velocity (rad/s)</h5>
    <ul>
      <li>X: {summary.angular.x}</li>
      <li>Y: {summary.angular.y}</li>
      <li>Z: {summary.angular.z}</li>
    </ul>
  </div>
  <div>
    <h5>Orientation (quaternion)</h5>
    <ul>
      <li>X: {summary.orientation.x}</li>
      <li>Y: {summary.orientation.y}</li>
      <li>Z: {summary.orientation.z}</li>
      <li>W: {summary.orientation.w}</li>
    </ul>
  </div>
</div>
