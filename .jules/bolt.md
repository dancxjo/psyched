## 2024-05-24 - Efficient base64 encoding
**Learning:** O(N^2) string concatenation performance bottlenecks and 'Maximum call stack size exceeded' errors occur in JavaScript when converting large byte arrays to Base64 byte-by-byte or via `String.fromCharCode.apply(null, array)` for huge arrays.
**Action:** Use chunked processing (e.g., 8192-byte chunks) with `String.fromCharCode.apply(null, chunk)` for large arrays.
