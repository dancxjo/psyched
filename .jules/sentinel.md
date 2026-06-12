## 2024-04-10 - Hardcoded Database Credentials in Docker Compose

**Vulnerability:** A hardcoded Neo4j password (`NEO4J_AUTH: neo4j/password`) was discovered in `services/graphs/docker-compose.yml`. This exposes database credentials in version control and allows anyone with access to the codebase or container to authenticate to the graph database.

**Learning:** Development environments often use hardcoded passwords for convenience during initial setup. However, these often leak into production environments or are committed to repositories, creating a critical vulnerability where sensitive data can be accessed or modified by unauthorized individuals.

**Prevention:** Never hardcode secrets or passwords in configuration files or code. Use environment variables with validation (e.g., `${NEO4J_AUTH?must be set}`) to enforce that credentials are provided securely at runtime. This approach ensures the application "fails securely" (refuses to start) rather than "failing open" with default or empty credentials if the environment variable is missing.
