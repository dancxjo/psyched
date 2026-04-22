## 2025-02-14 - Removed hardcoded Neo4j database credentials
**Vulnerability:** Hardcoded fallback credentials (like `neo4j/password` and `test`) were present in `docker-compose.yml`, shell launch scripts, and Python default ROS parameters.
**Learning:** This existed to allow easy local environment bootstrapping without explicit `.env` configurations, but creating an insecure default allows these credentials to potentially leak into production if variables aren't properly passed.
**Prevention:** Instead of insecure defaults, enforce environment variable presence using Bash strict checks `${VAR?must be set}` and `docker compose` syntax, ensuring the system fails securely rather than falling back to known passwords.
