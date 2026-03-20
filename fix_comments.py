import re

files = [
    "modules/pilot/cockpit/components/pilot-dashboard.helpers.js",
    "services/asr/app/static/harness.html",
    "modules/eye/cockpit/browser-eye.html",
    "modules/ear/cockpit/browser-ear.html"
]

for file in files:
    with open(file, "r") as f:
        content = f.read()

    # Add explanatory comment above chunkSize
    content = content.replace(
        "const chunkSize = 8192;",
        "// Process in chunks to avoid Maximum call stack size exceeded and O(N^2) string concatenation overhead\n        const chunkSize = 8192;"
    )

    with open(file, "w") as f:
        f.write(content)
