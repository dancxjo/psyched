## 2025-02-23 - Enforce Secure Defaults with Variable Expansion
**Vulnerability:** A hardcoded password (`neo4j/password` and `"test"`) was used in Docker Compose files, shell scripts, and ROS 2 node parameters for Neo4j authentication.
**Learning:** Default values like `NEO4J_PASSWORD=${NEO4J_PASSWORD:-test}` or `neo4j/password` lead to insecure deployments because the system silently fails open.
**Prevention:** Use bash and Docker variable expansion with the syntax `${VAR?must be set}` to ensure the configuration explicitly requires secure credentials to be provided and fails securely if they are missing. Also avoid setting fallback values in application code parameter declarations (e.g. `""` instead of `"test"`).
