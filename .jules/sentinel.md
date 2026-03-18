## 2024-03-18 - Hardcoded Database Passwords

**Vulnerability:** Default database passwords (like "test" for Neo4j) were hardcoded across multiple configuration files, including ROS parameter fallbacks, launch scripts (`launch_unit.sh`), and host manifests (`hosts/motherbrain.toml`), and local docker-compose environments (`NEO4J_AUTH: neo4j/password`).

**Learning:** Hardcoding passwords simplifies local setup but introduces severe security risks when configurations are deployed or if local environments are exposed to unauthorized access. In systems with distributed orchestration like ROS 2, these default configurations can easily leak into production.

**Prevention:** Always use empty strings `""` for password fallbacks and configuration files in source control. For local environments without authentication, explicitly disable it (e.g. `NEO4J_AUTH: none`). Ensure database drivers handle empty passwords correctly (e.g., using `auth=None` in Neo4j's Python driver) to prevent connection failures when authentication is disabled.
