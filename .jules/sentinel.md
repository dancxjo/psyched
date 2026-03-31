## 2025-02-23 - Hardcoded Neo4j Passwords
**Vulnerability:** Default credentials for Neo4j database were hardcoded in multiple places.
**Learning:** Disabling auth completely via code (e.g. auth=None instead of dynamic setup) can cause connection issues if the driver expects None instead of empty credentials when auth is none. Also, docker-compose defaults must safely fallback to none when no env vars exist.
**Prevention:** Always use environment variables for authentication and test database connections with empty credentials to ensure local development works.
