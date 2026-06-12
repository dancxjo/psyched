## 2024-04-14 - Hardcoded Neo4j Credentials in docker-compose.yml
**Vulnerability:** A hardcoded default password (`neo4j/password`) was embedded in `services/graphs/docker-compose.yml` via the `NEO4J_AUTH` environment variable. This fails open, allowing unrestricted access to the database using default credentials if deployed without overriding the value.
**Learning:** Default configurations in Docker Compose files often prioritize ease of local setup over secure-by-default practices.
**Prevention:** Use the `?must be set` bash substitution syntax (e.g., `NEO4J_AUTH: ${NEO4J_AUTH?must be set}`) in docker-compose files. This enforces a secure fail-safe, preventing the container from starting if the credential is not explicitly provided in the environment.
