#!/usr/bin/env bash
set -euo pipefail

QDRANT_URL=${QDRANT_URL:-http://localhost:6333}
QDRANT_API_KEY=${QDRANT_API_KEY:-}
NEO4J_URI=${NEO4J_URI:-bolt://localhost:7687}
NEO4J_USER=${NEO4J_USER:-neo4j}
NEO4J_PASSWORD=${NEO4J_PASSWORD:-test}
MEMORY_BATCH_SIZE=${MEMORY_BATCH_SIZE:-10}

ros2 run memory memory_node \
  --ros-args \
    -p qdrant.url:="${QDRANT_URL}" \
    -p qdrant.api_key:="${QDRANT_API_KEY}" \
    -p neo4j.uri:="${NEO4J_URI}" \
    -p neo4j.user:="${NEO4J_USER}" \
    -p neo4j.password:="${NEO4J_PASSWORD}" \
    -p memory.batch_size:=${MEMORY_BATCH_SIZE} &

wait -n
