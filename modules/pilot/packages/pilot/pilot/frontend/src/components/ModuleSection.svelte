<script>
  import { createEventDispatcher } from 'svelte';
  import TopicWidget from './TopicWidget.svelte';

  export let module;
  export let activeTopics;

  const dispatch = createEventDispatcher();

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
