## 2024-05-18 - Hardcoded Database Credentials in Docker Compose

**Vulnerability:** A hardcoded password (`NEO4J_AUTH: neo4j/password`) was present in the `docker-compose.yml` file for the graph database service.

**Learning:** Hardcoding secrets in infrastructure definitions (like `docker-compose.yml` or Kubernetes manifests) makes it easy to accidentally commit credentials to version control and promotes deploying services with default, insecure passwords, violating the principle of secure defaults.

**Prevention:** Use environment variable interpolation (`${VAR?must be set}`) in infrastructure configurations to ensure the system fails securely (refuses to start) if the required credentials are not explicitly provided by the deployment environment, thus preventing unintentional exposure and enforcing proper secret management.
