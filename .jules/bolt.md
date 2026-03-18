## 2024-05-24 - Base64 Conversion Bottleneck
**Learning:** Converting typed arrays (like `Uint8Array`) to strings byte-by-byte using string concatenation (`+= String.fromCharCode(view[i])`) creates an O(N^2) performance bottleneck due to constant string reallocation. This is especially slow for large payloads like images or audio.
**Action:** Use chunked processing (e.g., 8192-byte chunks) with `String.fromCharCode.apply(null, chunk)` to convert typed arrays to strings efficiently.
