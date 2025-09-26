# Chat Package

Chat node that listens to `/conversation` for `psyched_msgs/Message` messages. When a new `role: user` arrives, it sends the conversation (with a system prompt at the top) to Ollama (default model `gemma3`), truncates the answer to a single sentence, publishes it to `/voice` for TTS, and only after the corresponding `voice_done` signal is received does it publish the assistant message back to `/conversation`.

The node always records the spoken payload, even if the acknowledgement text differs slightly (for example, because the synthesizer collapses whitespace). Mismatches are logged for debugging, but the conversation remains aligned with what the user actually heard.

## Parameters
- `system_prompt` (string, env `CHAT_SYSTEM_PROMPT`): top system message
- `conversation_topic` (default `/conversation`)
- `voice_topic` (default `/voice`)
- `transcript_topic` (default `/audio/transcription`)
- `model` (env `CHAT_MODEL`, default `gemma3`)
- `ollama_host` (env `OLLAMA_HOST`, default `http://localhost:11434`)
- `max_history` (int, default 20)

## Run
```
psh provision <host>
source tools/setup_env.sh
colcon build --symlink-install --base-paths src
psh mod chat launch
```

Ensure Ollama is installed and `ollama pull gemma3` completed. When
`transcript_topic` is populated, user turns are sourced from
`psyched_msgs/Transcript` messages so metadata such as speaker and confidence
propagates through the conversation history.
