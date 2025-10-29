#!/usr/bin/env bash
# services/llm/setup.sh
# Ensures the Ollama runtime has the requested lightweight model available.
set -euo pipefail

MODEL_NAME=${OLLAMA_MODEL:-gemma3:latest}
SERVICE_NAME=${OLLAMA_SERVICE_NAME:-llm}
COMPOSE_FILE=${OLLAMA_COMPOSE_FILE:-docker-compose.yml}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

if ! command -v docker >/dev/null 2>&1; then
	echo "[ERROR] docker command not found. Install Docker before running services/llm/setup.sh" >&2
	exit 1
fi

if docker compose version >/dev/null 2>&1; then
	DOCKER_COMPOSE=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
	DOCKER_COMPOSE=(docker-compose)
else
	echo "[ERROR] docker compose plugin not available. Install it before running services/llm/setup.sh" >&2
	exit 1
fi

COMPOSE_FILE_PATH="${SCRIPT_DIR}/${COMPOSE_FILE}"
if [[ ! -f "${COMPOSE_FILE_PATH}" ]]; then
	echo "[ERROR] Compose file ${COMPOSE_FILE_PATH} not found." >&2
	exit 1
fi

compose_cmd() {
	"${DOCKER_COMPOSE[@]}" -f "${COMPOSE_FILE_PATH}" "$@"
}

container_exists() {
	docker ps --format '{{.Names}}' | grep -Fxq "${SERVICE_NAME}"
}

wait_for_server() {
	local retries=30
	local delay=2
	local attempt
	for ((attempt = 1; attempt <= retries; attempt++)); do
		if docker exec "${SERVICE_NAME}" ollama list >/dev/null 2>&1; then
			return 0
		fi
		sleep "${delay}"
	done
	echo "[ERROR] Timed out waiting for Ollama server inside container ${SERVICE_NAME}." >&2
	return 1
}

started_container=false
if container_exists; then
	echo "[INFO] Using running container ${SERVICE_NAME}."
else
	echo "[INFO] Starting container ${SERVICE_NAME} to prepare the model cache."
	compose_cmd up -d "${SERVICE_NAME}"
	started_container=true
fi

cleanup() {
	if [[ "${started_container}" == true ]]; then
		echo "[INFO] Stopping temporary container ${SERVICE_NAME}."
		compose_cmd stop "${SERVICE_NAME}" >/dev/null 2>&1 || true
		compose_cmd rm -f "${SERVICE_NAME}" >/dev/null 2>&1 || true
	fi
}
trap cleanup EXIT

wait_for_server

if docker exec "${SERVICE_NAME}" ollama show "${MODEL_NAME}" >/dev/null 2>&1; then
	echo "[INFO] Model ${MODEL_NAME} already available in Ollama cache."
else
	echo "[INFO] Pulling Ollama model ${MODEL_NAME}..."
	docker exec "${SERVICE_NAME}" ollama pull "${MODEL_NAME}"
fi

echo "[SUCCESS] Ollama model ${MODEL_NAME} ready inside container ${SERVICE_NAME}"
