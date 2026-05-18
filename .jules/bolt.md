## 2024-05-24 - Base64 Encoding Bottlenecks
**Learning:** In JavaScript, converting large byte arrays to Base64 using byte-by-byte string concatenation (`binary += String.fromCharCode(view[i])`) scales O(N^2) and becomes a significant performance bottleneck for large payloads.
**Action:** Replace byte-by-byte loops with chunked string generation (e.g., using `String.fromCharCode.apply(null, subarray)`) to drastically improve performance while avoiding "Maximum call stack size exceeded" errors.
