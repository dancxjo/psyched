import re

file = "modules/pilot/cockpit/components/pilot-dashboard.helpers.js"
with open(file, "r") as f:
    content = f.read()

# Replace global ignore with specific local ignores
content = content.replace(
    'if (typeof Buffer !== "undefined") {',
    '// deno-lint-ignore no-node-globals\n  if (typeof Buffer !== "undefined") {'
)
content = content.replace(
    'return Buffer.from(binary, "binary").toString("base64");',
    '// deno-lint-ignore no-node-globals\n    return Buffer.from(binary, "binary").toString("base64");'
)

with open(file, "w") as f:
    f.write(content)
