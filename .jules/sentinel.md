## 2024-05-15 - Fail Securely with Docker Compose Variables
**Vulnerability:** Hardcoded database credentials (`NEO4J_AUTH: neo4j/password`) in `docker-compose.yml`.
**Learning:** Services that depend on environment variables for authentication shouldn't use hardcoded values or fallbacks (e.g. `${VAR:-none}`) which cause the system to fail open and run insecurely.
**Prevention:** Use bash-style parameter expansion with error messages like `${VAR?must be set}` to enforce required secrets and fail securely before the service even starts.
