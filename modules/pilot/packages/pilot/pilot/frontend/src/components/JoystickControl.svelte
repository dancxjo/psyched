<script>
  import { onMount, onDestroy } from 'svelte';

  export let record;

  let container;
  let knob;
  let active = false;
  let lastSend = 0;
  let displayLinear = 0;
  let displayAngular = 0;

  const maxLinear = 0.7;
  const maxAngular = 2.0;
  const deadZone = 0.06;
  const sendIntervalMs = 60;

  function clamp(value, min, max) {
    return Math.min(Math.max(value, min), max);
  }

  function sendVelocity(linearX, angularZ) {
    const now = Date.now();
    if (now - lastSend < sendIntervalMs) {
      return;
    }
    lastSend = now;
    displayLinear = linearX;
    displayAngular = angularZ;
    record.send({
      linear: { x: linearX, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: angularZ },
    });
  }

  function resetKnob() {
    if (knob) {
      knob.style.transform = 'translate(-50%, -50%)';
    }
    displayLinear = 0;
    displayAngular = 0;
    record.send({
      linear: { x: 0.0, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: 0.0 },
    });
  }

  function handlePointerDown(event) {
    if (event.pointerType === 'mouse' && event.button !== 0) {
      return;
    }
    active = true;
    container.setPointerCapture(event.pointerId);
    updateFromEvent(event);
  }

  function handlePointerMove(event) {
    if (!active) return;
    updateFromEvent(event);
  }

  function handlePointerUp(event) {
    if (!active) return;
    active = false;
    try {
      container.releasePointerCapture(event.pointerId);
    } catch (error) {
      // ignore capture errors
    }
    resetKnob();
  }

  function updateFromEvent(event) {
    if (!container || !knob) return;
    const rect = container.getBoundingClientRect();
    const centerX = rect.left + rect.width / 2;
    const centerY = rect.top + rect.height / 2;
    const radius = Math.min(rect.width, rect.height) / 2;
    const knobRadius = knob.offsetWidth / 2;
    const maxDistance = radius - knobRadius;

    const offsetX = event.clientX - centerX;
    const offsetY = event.clientY - centerY;

    const distance = Math.min(Math.hypot(offsetX, offsetY), maxDistance);
    const angle = Math.atan2(offsetY, offsetX);

    const knobX = Math.cos(angle) * distance;
    const knobY = Math.sin(angle) * distance;

    knob.style.transform = `translate(calc(-50% + ${knobX}px), calc(-50% + ${knobY}px))`;

    const nx = knobX / maxDistance;
    const ny = knobY / maxDistance;
    const filteredX = Math.abs(nx) < deadZone ? 0 : clamp(nx, -1, 1);
    const filteredY = Math.abs(ny) < deadZone ? 0 : clamp(ny, -1, 1);

    const linearX = -filteredY * maxLinear;
    const angularZ = -filteredX * maxAngular;
    sendVelocity(linearX, angularZ);
  }

  onMount(() => {
    window.addEventListener('pointerup', handlePointerUp);
    window.addEventListener('pointermove', handlePointerMove);
  });

  onDestroy(() => {
    window.removeEventListener('pointerup', handlePointerUp);
    window.removeEventListener('pointermove', handlePointerMove);
  });
</script>

<div class="joystick" bind:this={container} on:pointerdown={handlePointerDown}>
  <div class="joystick-grid">
    <div class="joystick-knob" bind:this={knob}></div>
  </div>
  <div class="joystick-readout">
    <span>Linear X: {displayLinear.toFixed(2)} m/s</span>
    <span>Angular Z: {displayAngular.toFixed(2)} rad/s</span>
  </div>
</div>
