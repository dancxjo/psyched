## 2024-05-18 - Hardcoded Password Default Fix
**Vulnerability:** Hardcoded database password in `services/graphs/docker-compose.yml`.
**Learning:** Hardcoded credentials should be replaced with environment variables.
**Prevention:** Do not hardcode passwords in config files. Use environment variables with appropriate defaults or enforcement.
