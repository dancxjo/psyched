<script>
  import { createEventDispatcher } from 'svelte';
  import TopicWidget from './TopicWidget.svelte';

  export let module;
  export let activeTopics;

  const dispatch = createEventDispatcher();
  const defaultRegime = 'general';
  const regimeLabels = {
    system: 'System',
    audio: 'Audio',
    conversation: 'Conversation',
    navigation: 'Navigation',
    behavior: 'Behavior',
    perception: 'Perception',
    mobility: 'Mobility',
    general: 'General',
  };

  function parseRegimes(value) {
    if (Array.isArray(value)) {
      const cleaned = value.map((item) => `${item}`.trim()).filter(Boolean);
      return cleaned.length ? cleaned : [defaultRegime];
    }
    if (typeof value === 'string') {
      const trimmed = value.trim();
      return trimmed ? [trimmed] : [defaultRegime];
    }
    if (value == null) {
      return [defaultRegime];
    }
    const coerced = `${value}`.trim();
    return coerced ? [coerced] : [defaultRegime];
  }

  function formatRegimeName(slug) {
    if (slug in regimeLabels) {
      return regimeLabels[slug];
    }
    return slug
      .split(/[-_\s]+/)
      .filter(Boolean)
      .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
      .join(' ');
  }

  $: moduleRegimes = parseRegimes(module?.regimes);

  function command(scope, command) {
    dispatch('runCommand', { scope, command });
  }

  function startTopic(topic) {
    dispatch('startTopic', { module: module.name, topic });
  }

  function stopTopic(topic) {
    dispatch('stopTopic', { module: module.name, topic });
  }

  function togglePause(topic, paused) {
    dispatch('pauseTopic', { module: module.name, topic, paused });
  }

  function topicKey(topic) {
    return `${module.name}:${topic.name || topic.topic}`;
  }

  function topicRecord(topic) {
    return activeTopics.get(topicKey(topic));
  }
</script>

<section class="module-section" id={`module-${module.name}`}>
  <header class="module-header">
    <h2>{module.display_name || module.name}</h2>
    {#if module.description}
      <p>{module.description}</p>
    {/if}
    {#if moduleRegimes.length}
      <ul class="regime-tags">
        {#each moduleRegimes as regime}
          <li>{formatRegimeName(regime)}</li>
        {/each}
      </ul>
    {/if}
    <div class="command-groups">
      <div class="command-set">
        <h3>Module Commands</h3>
        <div class="button-row">
          {#each module.commands?.mod ?? [] as commandName}
            <button type="button" on:click={() => command('mod', commandName)}>{commandName}</button>
          {/each}
        </div>
      </div>
      <div class="command-set">
        <h3>System Commands</h3>
        <div class="button-row">
          {#each module.commands?.system ?? [] as commandName}
            <button type="button" on:click={() => command('sys', commandName)}>{commandName}</button>
          {/each}
        </div>
      </div>
    </div>
  </header>

  <div class="topics-grid">
    {#each module.topics as topic (topic.name || topic.topic)}
      {#if topic}
        {#key topicKey(topic)}
          <article class="topic-card">
            <div class="topic-header">
              <div>
                <h4>{topic.name || topic.topic}</h4>
                <small>{topic.type}</small>
              </div>
              {#if topicRecord(topic)}
                <div class="topic-actions">
                  <button type="button" on:click={() => stopTopic(topic)}>Disconnect</button>
                  <button type="button" on:click={() => togglePause(topic, !topicRecord(topic).paused)}>
                    {topicRecord(topic).paused ? 'Resume' : 'Pause'}
                  </button>
                </div>
              {:else}
                <div class="topic-actions">
                  <button type="button" on:click={() => startTopic(topic)}>Connect</button>
                </div>
              {/if}
            </div>
            <TopicWidget record={topicRecord(topic)} topic={topic} />
          </article>
        {/key}
      {/if}
    {/each}
  </div>
</section>

<style>
  .regime-tags {
    display: flex;
    gap: 0.5rem;
    flex-wrap: wrap;
    list-style: none;
    padding: 0;
    margin: 0;
  }

  .regime-tags li {
    background: rgba(79, 70, 229, 0.12);
    color: #4338ca;
    border-radius: 9999px;
    padding: 0.25rem 0.65rem;
    font-size: 0.7rem;
    font-weight: 600;
    letter-spacing: 0.05em;
    text-transform: uppercase;
  }
</style>
