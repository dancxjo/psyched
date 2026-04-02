## 2025-04-02 - Sentinel Hardcoded Credentials
**Vulnerability:** Found hardcoded credentials for Neo4j (`neo4j/password` in docker-compose.yml and `test` in Python configs/scripts).
**Learning:** Hardcoding credentials creates critical security risks and prevents users from configuring their environments dynamically. When disabling auth in Neo4j (using `NEO4J_AUTH: none`), the Python driver expects `auth=None`, not a tuple with an empty string, otherwise it will fail to connect.
**Prevention:** Use environment variables with secure fallbacks (e.g., `NEO4J_AUTH: ${NEO4J_AUTH:-none}`) for configuration. When instantiating the neo4j Python driver, handle empty/none passwords by setting `auth=None`.
