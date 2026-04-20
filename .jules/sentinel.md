## 2024-04-20 - Enforcing Environment Variables to Fail Securely
**Vulnerability:** Hardcoded credentials (e.g., `NEO4J_PASSWORD="test"`) and insecure fallbacks (e.g., `NEO4J_PASSWORD=${NEO4J_PASSWORD:-test}`) were used in launch scripts and configuration files, leaving services vulnerable if secrets were not explicitly provided.
**Learning:** Services must fail securely. Relying on default insecure fallback passwords (`neo4j/password`) or omitting environment validations leads to "failing open" or relying on known default credentials, risking unauthorized access.
**Prevention:** Always enforce required sensitive environment variables using shell parameter expansion (e.g., `${VAR?must be set}`) and never hardcode secrets in configuration files like `hosts/*.toml` or `docker-compose.yml`.
