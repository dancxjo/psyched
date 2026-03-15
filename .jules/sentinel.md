## 2024-05-24 - Default passwords in host config and launch scripts
**Vulnerability:** Default passwords like `test` for Neo4j are hardcoded in `hosts/motherbrain.toml`, `modules/memory/launch_unit.sh`, and `modules/memory/packages/memory/memory/node.py`.
**Learning:** Default passwords can lead to unauthorized access to databases in local or shared environments. Defaulting to an empty string and requiring an explicit configuration or secret is safer.
**Prevention:** Always default to an empty string (`""`) for passwords in configuration files, launch scripts, and node parameter defaults. Ensure database clients handle empty passwords correctly (e.g. by setting auth=None).
