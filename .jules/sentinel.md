## 2025-04-19 - Hardcoded Neo4j Credentials
**Vulnerability:** Default passwords like 'neo4j/password' and 'test' were hardcoded as insecure fallbacks in docker-compose.yml and launch scripts.
**Learning:** Default fallbacks for environment variables (`${VAR:-default}`) frequently lead to insecure open defaults if the application is run without explicit configuration.
**Prevention:** Always enforce the presence of sensitive environment variables using bash/docker parameter expansion syntax `${VAR?error message}` so that the app 'fails securely' rather than failing open.
