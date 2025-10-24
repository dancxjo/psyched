#!/usr/bin/env bash
# services/llm/setup.sh
# Ensures the Ollama runtime has the requested lightweight model available.
set -euo pipefail

MODEL_NAME=${OLLAMA_MODEL:-gemma3:latest}
OLLAMA_IMAGE=${OLLAMA_IMAGE:-ollama/ollama:latest}
OLLAMA_DATA_DIR=${OLLAMA_DATA_DIR:-/usr/share/ollama/.ollama}

if ! command -v docker >/dev/null 2>&1; then
	echo "[ERROR] docker command not found. Install Docker before running services/llm/setup.sh" >&2
	exit 1
fi

if ! sudo mkdir -p "${OLLAMA_DATA_DIR}"; then
	echo "[ERROR] Failed to create ${OLLAMA_DATA_DIR}. Try running with elevated permissions." >&2
	exit 1
fi

echo "[INFO] Preparing Ollama model directory at ${OLLAMA_DATA_DIR}."

if docker run --rm \
	-v "${OLLAMA_DATA_DIR}:/root/.ollama" \
	"${OLLAMA_IMAGE}" \
	ollama show "${MODEL_NAME}" >/dev/null 2>&1;
then
	echo "[INFO] Model ${MODEL_NAME} already available in Ollama cache."
else
	echo "[INFO] Pulling Ollama model ${MODEL_NAME}..."
	docker run --rm \
		-v "${OLLAMA_DATA_DIR}:/root/.ollama" \
		"${OLLAMA_IMAGE}" \
		ollama pull "${MODEL_NAME}"
fi

echo "[SUCCESS] Ollama model ${MODEL_NAME} ready under ${OLLAMA_DATA_DIR}"