## 2025-02-14 - Prevent Hardcoded Credentials Failing Open

**Vulnerability:** A hardcoded `neo4j/password` for `NEO4J_AUTH` existed in `services/graphs/docker-compose.yml`. Even worse, simply removing it without enforcing its existence could cause the database authentication to silently fail open or use insecure defaults.
**Learning:** Hardcoded credentials should be removed without completely disabling the underlying authentication mechanisms (e.g., hardcoding `NEO4J_AUTH: none`) or using insecure environment fallbacks like `${VAR:-none}`.
**Prevention:** Always enforce the presence of the environment variable (e.g., `${NEO4J_AUTH?must be set}`) in Docker Compose to ensure the system fails securely rather than failing open.