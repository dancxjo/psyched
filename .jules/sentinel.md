## 2024-05-24 - Hardcoded Default Passwords in Graph Database Integrations

**Vulnerability:** Found hardcoded "test" passwords for the Neo4j graph database embedded across multiple files including ROS 2 launch scripts (`launch_unit.sh`), host configuration (`hosts/motherbrain.toml`), Python node parameters (`memory/node.py`), and a Docker compose file (`docker-compose.yml`).

**Learning:** When falling back to an unauthenticated/empty password setup in a local environment, Neo4j drivers in Python require `auth=None` instead of passing `auth=("user", "")`. This was causing problems when we just removed the hardcoded password string. Furthermore, we must explicitly set `NEO4J_AUTH: none` in the `docker-compose.yml` to disable the default password constraint on the Docker image.

**Prevention:** Ensure configuration parameters default to empty strings rather than dummy passwords like "test" or "password", and ensure underlying database integrations handle empty credentials (by omitting the `auth` parameter entirely) properly.
