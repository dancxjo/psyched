## 2024-XX-XX - [CRITICAL] Avoid Hardcoded Database Passwords
**Vulnerability:** Default passwords for databases (e.g., Neo4j, Qdrant) were hardcoded in ROS parameter declarations, launch scripts, and host configuration files.
**Learning:** Default passwords expose local services to potential unauthorized access or exploitation, even in isolated local network environments.
**Prevention:** Always use empty strings as fallback defaults for database passwords in configurations instead of providing guessable default passwords like "test" or "password". When using Neo4j driver without a password, ensure `auth=None` is explicitly provided instead of `auth=(user, "")`.
