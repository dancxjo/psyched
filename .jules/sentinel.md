
## 2024-05-18 - Hardcoded Database Credentials in Docker Compose
**Vulnerability:** Hardcoded database credentials (`NEO4J_AUTH: neo4j/password`) were found in `services/graphs/docker-compose.yml`. This exposes sensitive database access information in version control.
**Learning:** When trying to make development easy, we often hardcode credentials. However, removing them and replacing them with standard fallback environments (like `${NEO4J_AUTH:-none}`) is insecure because it fails open, completely disabling authentication mechanisms.
**Prevention:** Always use `${VAR?must be set}` syntax for critical authentication environment variables in `docker-compose.yml`. This ensures the application "fails securely" (refuses to start) rather than "failing open" if the credential is forgotten.
