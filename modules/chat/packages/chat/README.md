# Chat Package

Chat node that listens to `/conversation` for `psyched_msgs/Message` messages. When a new `role: user` arrives, it sends the conversation (with a system prompt at the top) to Ollama (default model `gemma3`), truncates the answer to a single sentence, publishes it to `/voice` for TTS, and only after a matching `voice_done` signal is received does it publish the assistant message back to `/conversation`.

## Parameters
- `system_prompt` (string, env `CHAT_SYSTEM_PROMPT`): top system message
- `conversation_topic` (default `/conversation`)
- `voice_topic` (default `/voice`)
- `model` (env `CHAT_MODEL`, default `gemma3`)
- `ollama_host` (env `OLLAMA_HOST`, default `http://localhost:11434`)
- `max_history` (int, default 20)

## Run
```
./modules/chat/setup.sh
make build
ros2 launch chat chat.launch.py
```

Ensure Ollama is installed and `ollama pull gemma3` completed.
